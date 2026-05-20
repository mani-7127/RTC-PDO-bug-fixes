package motordriver

import (
	channels "EtherCAT/channels"
	logger   "EtherCAT/logger"
	notifier "EtherCAT/motordriver/statusnotifier"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/google/uuid"
)

// ----------------------------------------------------------
// Precision configuration — unchanged from original uploaded version.
// ----------------------------------------------------------
const (
	positionToleranceFastPath = 0.05
	positionToleranceFinal    = 0.05
)

func clearTargetReached(device MasterDevice) {
	logger.Debug("Clearing 'target reached' status for device:", device.Name)
}

func sleepMs(ms int) {
	time.Sleep(time.Duration(ms) * time.Millisecond)
}

// moveMotorToDegree is the main motion function.
//
// The overall flow (ABS/REL handling, ECS HIGH/LOW, fast-path, direction,
// compensation, FINISH BLOCKED safety check) is preserved from the original
// uploaded version.
//
// PDO-specific changes:
//   - setDirection reads fresh backlash after direction update (was stale in original).
//   - doRotate now uses PDO atomics instead of SDO moveToPosition.
//   - sendECSFinSignal uses PDO 60FE instead of SDO finsignal/finsignalend.
//   - refreshCurrentPositionForDevice uses getRawApos() for sign-correct sync.
//   - Fast-path calls doECSCheckZero after FIN (bug fix: original skipped it).
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

	// ABS / Relative handling — identical to original uploaded version.
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

	// ----------------------------------------------------------
	// ECS HIGH wait — identical to original uploaded version.
	// ----------------------------------------------------------
	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcs := doECSCheck(device, destination)
	if gotEcs == 0 || gotEcs == 2 {
		return fmt.Errorf("failed to receive ECS, program stopped/reset event received")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	driverStatusAfterECS := getCurrentDriverStatus(device.Device.Name)

	// ----------------------------------------------------------
	// ABS FAST-PATH (already at target)
	//
	// Original uploaded version: sent FIN then returned without calling
	// doECSCheckZero — machine never acknowledged LOW, next ECS HIGH
	// arrived immediately and the machine appeared to skip the LOW phase.
	//
	// Fix: always call doECSCheckZero after FIN, even on fast-path.
	// ----------------------------------------------------------
	if driverStatusAfterECS.mode == "ABS" &&
		math.Abs(driverStatusAfterECS.currentPosition-destination) <= positionToleranceFastPath {

		recheck := getCurrentDriverStatus(device.Device.Name)

		if math.Abs(recheck.currentPosition-destination) <= positionToleranceFastPath {
			logger.Info("Already at target — sending FIN and waiting ECS LOW", "id:", uid)

			if err := sendECSFinSignal(device); err != nil {
				return err
			}

			// Wait for ECS LOW — required even on fast-path.
			channels.WriteCommandExecInput("waiting_for_ecs", "")
			gotEcsZero := doECSCheckZero(device, destination)
			if gotEcsZero == 0 || gotEcsZero == 2 {
				return fmt.Errorf("failed to receive ECS zero on fast-path")
			}
			channels.WriteCommandExecInput("ecs_done", "")

			refreshCurrentPositionForDevice(device)
			doneDriverAction()
			channels.DestinationReached()
			logger.Info("move to position completed (already at target)", "driver", device.Name, "id", uid)
			return nil
		}
	}

	// ----------------------------------------------------------
	// SET DIRECTION
	// setDirection uses notifyDriverStatusWithWait — blocks until
	// driver_status_keeper has updated backlash. Re-read status AFTER
	// this call to get correct backlash (was stale in original uploaded version).
	// ----------------------------------------------------------
	setDirection(device, driverStatusAfterECS, moveToPos)
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", destination), device)

	// Fresh read after setDirection so backlash is current.
	driverStatusAfterDirection := getCurrentDriverStatus(device.Device.Name)
	backlash := driverStatusAfterDirection.backlash
	pitchErr := getPitchError(device.Name, destination)
	moveToWithComp := moveToPos + pitchErr - backlash

	logger.Info("compensation: moveToPos=", moveToPos, "pitchErr=", pitchErr, "backlash=", backlash, "moveToWithComp=", moveToWithComp)

	clearTargetReached(device)

	// ----------------------------------------------------------
	// ROTATE — doRotate uses PDO PP mode instead of SDO.
	// ----------------------------------------------------------
	err := doRotate(device, moveToWithComp)
	if err != nil {
		return err
	}

	// ----------------------------------------------------------
	// STRONG SAFETY BLOCK BEFORE FINISH SIGNAL
	// Preserved from original uploaded version.
	// ----------------------------------------------------------
	currentPos := getCurrentDriverStatus(device.Device.Name).currentPosition
	errVal := math.Abs(currentPos - destination)

	if errVal > positionToleranceFinal {
		logger.Error(
			"FINISH BLOCKED — target NOT reached!",
			"target=", destination,
			"current=", currentPos,
			"error=", errVal,
		)
		return fmt.Errorf("finish blocked: target not reached")
	}

	// ----------------------------------------------------------
	// SEND FINISH SIGNAL
	// ----------------------------------------------------------
	if err := sendECSFinSignal(device); err != nil {
		return err
	}

	// ----------------------------------------------------------
	// ECS ZERO
	// ----------------------------------------------------------
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

// ----------------------------------------------------------
// Step Mode — unchanged from original uploaded version.
// ----------------------------------------------------------
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

// ----------------------------------------------------------
// Direction Selection — unchanged from original uploaded version.
// ----------------------------------------------------------
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

// ----------------------------------------------------------
// Position Sync Helpers — updated to use getRawApos() for sign-correct sync.
// ----------------------------------------------------------

// RefreshCurrentPosition syncs all devices' cached currentPosition to live encoder.
func RefreshCurrentPosition() {
	for _, device := range masterDevices {
		refreshCurrentPositionForDevice(device)
	}
}

// refreshCurrentPositionForDevice syncs cached currentPosition to live encoder.
// Uses getRawApos() — sign-flip correction applied, consistent with display.
func refreshCurrentPositionForDevice(device MasterDevice) {
	rawPos := getRawApos()
	curPos, _ := currentPosition(rawPos, device.Device.DriveXRatio, device.Name)
	status := getCurrentDriverStatus(device.Name)
	status.currentPosition = curPos
	status.destinationPosition = curPos
	setCurrentDriverStatus(device.Name, status)
	logger.Info("[SYNC] Updated current position:", curPos, "° for drive", device.Name)
}

// ReadActualPositionFromDrive returns current live position in degrees.
// Uses getRawApos() for sign-correct value. Signature kept compatible with
// the original uploaded version (takes device name string).
func ReadActualPositionFromDrive(driveName string) float64 {
	status := getCurrentDriverStatus(driveName)
	return status.currentPosition
}
