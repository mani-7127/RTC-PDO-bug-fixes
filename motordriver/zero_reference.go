package motordriver

import (
	"EtherCAT/channels"
	"EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	"EtherCAT/settings"
	"fmt"
	"math"
	"time"
)

// moveToZero moves the motor to the customer-defined physical zero.
//
// The overall logic (shortest-path calculation, freeRotate, notifications)
// is preserved exactly from the original uploaded version.
//
// PDO-specific additions:
//   - After freeRotate, waits for motor to physically settle using PDO apos.
//   - Saves HomingApos as a NORMALIZED NEGATIVE value so InitAposCorrection
//     works correctly on every subsequent boot regardless of sign-flip state.
//   - sendECSFinSignal uses PDO 60FE atomics.
//   - configureDriver call removed — was redundant; original uploaded version
//     had it before freeRotate but it is already called during InitMaster.
//
// WHY normalize HomingApos to negative:
//   If zero-ref is done on a "flipped" boot (pdoFbActual positive), saving
//   raw pdoFbActual would give a positive HomingApos. On the next normal boot
//   (pdoFbActual negative), InitAposCorrection sees opposite signs and wrongly
//   applies correction — display shows ~3° instead of 0.000°. By always saving
//   negative, both sides of the comparison are always in the same space.
func moveToZero(device MasterDevice) error {
	logger.Debug("move to zero started")
	driverStatus := getCurrentDriverStatus(device.Device.Name)
	driverSettings := settings.GetDriverSettings(device.Device.Name)
	_ = driverSettings

	var targetPosition = 0.0
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.00), device)
	notifier.NotifyDestinationPosition(device.Name, float32(targetPosition))
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", targetPosition), device)

	// Shortest-path calculation — identical to original uploaded version.
	position := 0.00
	var cwPosition  = getPos(driverStatus.currentPosition, 0, true)
	var ccwPosition = getPos(driverStatus.currentPosition, 0, false)
	if math.Abs(cwPosition) < math.Abs(ccwPosition) {
		position = cwPosition
	} else {
		position = ccwPosition
	}
	logger.Debug("Position Values:", cwPosition, ccwPosition)
	logger.Debug("zero ref position: ", position)

	// freeRotate is unchanged — it calls doRotate (PDO PP mode) internally.
	freeRotate(device, position)

	// Wait for the physical shaft to settle before capturing HomingApos.
	// Without this, the encoder value captured immediately after bit10 fires
	// may still be oscillating by 1-2 pulses as deceleration completes.
	rawAtZero := waitForMotorSettle(10*time.Second, 50*time.Millisecond, 3)

	// Normalize HomingApos to negative regardless of current sign-flip state.
	// See header comment for the reason.
	var aposAtZero int32
	if rawAtZero > 0 {
		aposAtZero = -rawAtZero
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
// Returns the RAW encoder value (before sign-flip correction) so it can be
// saved directly as HomingApos in the same space as InitAposCorrection reads it.
func waitForMotorSettle(timeout, interval time.Duration, stableCount int) int32 {
	deadline := time.Now().Add(timeout)
	prev   := pdoFbActual.Load()
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

// getPos calculates the angular distance to move to reach targetPos from
// currentPos in the specified direction. Unchanged from original uploaded version.
func getPos(currentPos float64, targetPos float64, clockwise bool) float64 {
	currentPos = math.Mod(currentPos, 360)
	modeDiff   := math.Mod((currentPos - targetPos), 360)

	if clockwise {
		tomove := math.Mod(360-modeDiff, 360)
		return tomove
	}

	return math.Mod(modeDiff*-1, 360)
}
