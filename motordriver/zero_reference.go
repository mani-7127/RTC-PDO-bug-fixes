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
func moveToZero(device MasterDevice) error {
	logger.Debug("move to zero started")
	driverStatus := getCurrentDriverStatus(device.Device.Name)
	driverSettings := settings.GetDriverSettings(device.Device.Name)
	_ = driverSettings

	var targetPosition = 0.0
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.00), device)
	notifier.NotifyDestinationPosition(device.Name, float32(targetPosition))
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", targetPosition), device)

	// Shortest path to zero.
	cwPosition  := getPos(driverStatus.currentPosition, 0, true)
	ccwPosition := getPos(driverStatus.currentPosition, 0, false)

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