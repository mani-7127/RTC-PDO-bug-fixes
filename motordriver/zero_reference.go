package motordriver

import (
	channels "EtherCAT/channels"
	logger   "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"fmt"
	"math"
	"time"
)

// moveToZero moves the motor to the customer-defined physical zero.
//
// Rules:
//   - HomingOffset is the customer's mechanical calibration — NEVER touched here.
//   - HomingApos is saved as the RAW pdoFbActual value (not getRawApos).
//     InitAposCorrection also reads raw pdoFbActual, so both sides of the
//     comparison must be in the same raw sign space.
//   - Saving getRawApos() instead would cause an infinite flip loop:
//     next boot sees opposite signs → flips → display=0 but shaft is wrong.
//
// BUG FIX: live position is now read directly from the encoder via
// getRawApos() + currentPosition() rather than from the cached
// driverStatus.currentPosition. The cache defaults to 0 at boot and is
// only populated after the first poll tick (~50ms). If zero-ref is
// triggered before that tick, getPos(0, 0, ...) returns 0 and the
// motor never moves. Reading live from the encoder is always correct
// regardless of timing.
func moveToZero(device MasterDevice) error {
	logger.Debug("move to zero started")
	driverSettings := settings.GetDriverSettings(device.Device.Name)
	_ = driverSettings

	var targetPosition = 0.0
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.00), device)
	notifier.NotifyDestinationPosition(device.Name, float32(targetPosition))
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", targetPosition), device)

	// Read live position directly from encoder — do NOT use the cached
	// driverStatus.currentPosition which may be stale (zero) at boot.
	rawPos := getRawApos()
	livePos, _ := currentPosition(rawPos, device.Device.DriveXRatio, device.Device.Name)
	logger.Info("moveToZero: live position from encoder:", livePos, "° for driver:", device.Device.Name)

	// Shortest path to zero using the live encoder position.
	cwPosition  := getPos(livePos, 0, true)
	ccwPosition := getPos(livePos, 0, false)

	position := ccwPosition
	if math.Abs(cwPosition) < math.Abs(ccwPosition) {
		position = cwPosition
	}

	logger.Debug("Position Values:", cwPosition, ccwPosition)
	logger.Debug("zero ref position:", position)

	freeRotate(device, position)

	// Wait for physical standstill.
	// Save HomingApos as a NORMALIZED negative value regardless of
	// what signFlipActive is at this moment.
	//
	// WHY: if zero-ref is done on a flipped boot, pdoFbActual is positive
	// (+520984704). On the next normal boot pdoFbActual is negative
	// (-520984704). InitAposCorrection sees opposite signs and wrongly
	// applies correction — display shows 3.174° instead of 0.000°.
	//
	// By always saving HomingApos as negative (abs value negated), both
	// sides of the InitAposCorrection comparison are always in the same
	// space: bootApos negative (no flip) matches, bootApos positive (flip)
	// gets corrected to negative — either way currentPosition() sees the
	// same value every single boot.
	rawAtZero := waitForMotorSettle(10*time.Second, 50*time.Millisecond, 3)
	var aposAtZero int32
	if rawAtZero > 0 {
		aposAtZero = -rawAtZero // normalize to negative
	} else {
		aposAtZero = rawAtZero
	}

	logger.Info("HomingApos captured after motor settled:",
		aposAtZero, "(normalized negative) for driver:", device.Device.Name)

	if err := settings.SaveHomingReference(device.Device.Name, aposAtZero); err != nil {
		logger.Error("Failed to save HomingApos after zero ref:", err)
	} else {
		logger.Info("HomingApos saved:", aposAtZero, "driver:", device.Device.Name)
	}

	channels.DestinationReached()
	notifier.SocketMessage("gotozero_done", "goto zero completed")
	sendECSFinSignal(device)
	return nil
}

// waitForMotorSettle polls raw pdoFbActual until stable for stableCount
// consecutive reads spaced interval apart, or until timeout elapses.
// Returns the RAW encoder value.
func waitForMotorSettle(timeout, interval time.Duration, stableCount int) int32 {
	deadline := time.Now().Add(timeout)
	prev  := pdoFbActual.Load()
	streak := 1

	for time.Now().Before(deadline) {
		time.Sleep(interval)
		cur := pdoFbActual.Load()
		if cur == prev {
			streak++
			if streak >= stableCount {
				return cur
			}
		} else {
			streak = 1
			prev = cur
		}
	}

	logger.Warn("waitForMotorSettle: timed out, using last known apos:", prev)
	return prev
}

func getPos(currentPos float64, targetPos float64, clockwise bool) float64 {
	currentPos = math.Mod(currentPos, 360)
	modeDiff   := math.Mod((currentPos - targetPos), 360)
	if clockwise {
		return math.Mod((360 - modeDiff), 360)
	}
	return math.Mod((modeDiff * -1), 360)
}