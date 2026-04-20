package motordriver

import (
	channels "EtherCAT/channels"
	logger "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/google/uuid"
)

const (
	positionToleranceFastPath = 0.02
	positionToleranceFinal    = 0.02
)

func clearTargetReached(device MasterDevice) {
	logger.Debug("Clearing 'target reached' status for device:", device.Name)
}

func sleepMs(ms int) {
	time.Sleep(time.Duration(ms) * time.Millisecond)
}

func moveMotorToDegree(device MasterDevice, degreeToRotate float64) error {
	driverStatus := getCurrentDriverStatus(device.Device.Name)
	if driverStatus.potNotExceeded {
		logger.Error("pot/not exceeded, exiting from move command")
		return errors.New("pot/not exceeded, exiting from move command")
	}

	uid := uuid.New()
	logger.Info(
		"rotate motor to", degreeToRotate,
		"current:", driverStatus.currentPosition,
		"prev dest:", driverStatus.destinationPosition,
		"workoffset", driverStatus.workOffset,
		"id:", uid,
	)

	var moveToPos, destination float64
	if driverStatus.mode == "ABS" {
		moveToPos, destination = getAbsolutePosition(
			driverStatus.currentPosition,
			degreeToRotate,
			driverStatus.shortestPathEnabled,
		)
	} else {
		moveToPos, destination = getRelativePosition(
			driverStatus.currentPosition,
			degreeToRotate,
			driverStatus.destinationPosition,
		)
	}

	notifier.NotifyDestinationPosition(device.Name, float32(destination-driverStatus.workOffset))

	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcs := doECSCheck(device, destination)
	if gotEcs == 0 || gotEcs == 2 {
		return fmt.Errorf("failed to receive ECS, program stopped/reset event received")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	driverStatusAfterECS := getCurrentDriverStatus(device.Device.Name)

	if driverStatusAfterECS.mode == "ABS" &&
		math.Abs(driverStatusAfterECS.currentPosition-destination) <= positionToleranceFastPath {
		recheck := getCurrentDriverStatus(device.Device.Name)
		if math.Abs(recheck.currentPosition-destination) <= positionToleranceFastPath {
			logger.Info("Already at target — marking line complete without FIN", "id:", uid)
			refreshCurrentPositionForDevice(device)
			doneDriverAction()
			channels.DestinationReached()
			return nil
		}
	}

	setDirection(device, driverStatusAfterECS, moveToPos)
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", destination), device)

	backlash := driverStatusAfterECS.backlash
	pitchErr := getPitchError(device.Name, destination)
	moveToWithComp := moveToPos + pitchErr - backlash

	if err := doRotate(device, moveToWithComp); err != nil {
		return err
	}

	if err := sendECSFinSignal(device); err != nil {
		return err
	}

	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcsZero := doECSCheckZero(device, destination)
	if gotEcsZero == 0 || gotEcsZero == 2 {
		return fmt.Errorf("failed to receive ECS zero")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	doneDriverAction()
	channels.DestinationReached()
	refreshCurrentPositionForDevice(device)
	logger.Info("move to position completed", "driver", device.Name, "id", uid)
	return nil
}

func stepMode(masterDevice MasterDevice, valueInDegreeToAdd float64) error {
	logger.Debug("step mode moving to position: ", valueInDegreeToAdd)
	driverStatus := getCurrentDriverStatus(masterDevice.Device.Name)
	if driverStatus.potNotExceeded {
		logger.Error("pot/not exceeded, exiting from move command")
		channels.StepModeComplete()
		return errors.New("pot/not exceeded, exiting from move command")
	}
	err := freeRotate(masterDevice, valueInDegreeToAdd)
	channels.StepModeComplete()
	return err
}

func setDirection(device MasterDevice, driverStatus driverCurrentStatus, degreeToRotate float64) {
	if !driverStatus.shortestPathEnabled {
		notifyDriverStatusWithWait("rotation_direction", "1", device)
		return
	}
	if degreeToRotate > 0 {
		notifyDriverStatusWithWait("rotation_direction", "1", device)
	} else {
		notifyDriverStatusWithWait("rotation_direction", "-1", device)
	}
}

func RefreshCurrentPosition() {
	for _, device := range masterDevices {
		refreshCurrentPositionForDevice(device)
	}
}

func refreshCurrentPositionForDevice(device MasterDevice) {
	rawPos := pdoFbActual.Load()
	curPos, _ := currentPosition(int32(rawPos), device.Device.DriveXRatio, device.Name)
	status := getCurrentDriverStatus(device.Name)
	status.currentPosition = curPos
	status.destinationPosition = curPos
	setCurrentDriverStatus(device.Name, status)
	logger.Info("[SYNC] Updated current position to %.3f° from drive encoder for drive %s", curPos, device.Name)
}

func ReadActualPositionFromDrive(device MasterDevice) float64 {
	rawPos := pdoFbActual.Load()
	curPos, _ := currentPosition(int32(rawPos), device.Device.DriveXRatio, device.Name)
	return curPos
}
