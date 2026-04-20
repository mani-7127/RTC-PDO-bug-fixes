package motordriver

import (
	channels "EtherCAT/channels"
	logger "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"fmt"
	"math"
)

// moveToZero moves the motor to the zero/home position.
func moveToZero(device MasterDevice) error {
	logger.Debug("move to zero started")
	driverStatus := getCurrentDriverStatus(device.Device.Name)
	driverSettings := settings.GetDriverSettings(device.Device.Name)
	_ = driverSettings

	var targetPosition = 0.0
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.00), device)
	notifier.NotifyDestinationPosition(device.Name, float32(targetPosition))
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", targetPosition), device)

	// Use shortest path to zero, same behavior as previous version.
	cwPosition := getPos(driverStatus.currentPosition, 0, true)
	ccwPosition := getPos(driverStatus.currentPosition, 0, false)

	position := 0.00
	if math.Abs(cwPosition) < math.Abs(ccwPosition) {
		position = cwPosition
	} else {
		position = ccwPosition
	}

	logger.Debug("Position Values:", cwPosition, ccwPosition)
	logger.Debug("zero ref position: ", position)

	// configureDriver removed — parameters are already written at InitMaster time.
	// Calling it here added ~600ms of SDO latency before every zero reference
	// (6 SDO writes × ~100ms each, visible in logs at 08:32:44.579–08:32:45.179).

	freeRotate(device, position)

	// Save the encoder apos at this exact zero position.
	// This reference point allows InitMaster to reanchor HomingOffset on the
	// next boot if the Panasonic A6B encoder multi-turn counter flips sign
	// during drive power cycle (e.g. -7561 revolutions → +7561 for the same
	// physical shaft position). Without this, every drive power cycle causes
	// a 2-4° display error that accumulates each time the user re-zeros.
	//
	// HomingOffset itself is saved by the REST API when the user sets zero.
	// HomingApos is saved here because this is the only place in motordriver
	// where the exact encoder value at physical zero is known precisely.
	aposAtZero := int32(pdoFbActual.Load())
	if err := settings.SaveHomingReference(device.Device.Name, aposAtZero); err != nil {
		logger.Error("Failed to save HomingApos after zero ref:", err)
	} else {
		logger.Info("HomingApos saved after zero ref:", aposAtZero, "for driver:", device.Device.Name)
	}

	channels.DestinationReached()
	notifier.SocketMessage("gotozero_done", "goto zero completed")
	sendECSFinSignal(device)
	return nil
}

func getPos(currentPos float64, targetPos float64, clockwise bool) float64 {
	currentPos = math.Mod(currentPos, 360)
	modeDiff := math.Mod((currentPos - targetPos), 360)

	if clockwise {
		tomove := math.Mod((360 - modeDiff), 360)
		return tomove
	}

	return math.Mod((modeDiff * -1), 360)
}