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

	// --- ECS HIGH wait ---
	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcs := doECSCheck(device, destination)
	if gotEcs == 0 || gotEcs == 2 {
		return fmt.Errorf("failed to receive ECS, program stopped/reset event received")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	driverStatusAfterECS := getCurrentDriverStatus(device.Device.Name)

	// --- ABS fast-path: already at target ---
	//
	// When the motor is already at the destination (within tolerance), we skip
	// the physical move but MUST still complete the full ECS handshake:
	//   FIN → wait ECS LOW
	//
	// BUG FIX: The previous fast-path returned after sending FIN without
	// calling doECSCheckZero. The machine sends ECS LOW only after seeing FIN.
	// Skipping doECSCheckZero caused the program to race to the next line
	// before the machine had acknowledged completion — the next ECS HIGH
	// arrived immediately because the machine hadn't had time to go LOW first,
	// making it appear the machine skipped the LOW phase entirely.
	if driverStatusAfterECS.mode == "ABS" &&
		math.Abs(driverStatusAfterECS.currentPosition-destination) <= positionToleranceFastPath {
		recheck := getCurrentDriverStatus(device.Device.Name)
		if math.Abs(recheck.currentPosition-destination) <= positionToleranceFastPath {
			logger.Info("Already at target — sending FIN and waiting ECS LOW", "id:", uid)

			// Send FIN — machine needs this to know we acknowledged the position.
			if err := sendECSFinSignal(device); err != nil {
				return err
			}

			// Wait for ECS LOW — required even on fast-path.
			// Without this, the next command's ECS HIGH check sees the ECS
			// signal that is still HIGH from this command, and proceeds
			// immediately without the machine having reset its state.
			channels.WriteCommandExecInput("waiting_for_ecs", "")
			gotEcsZero := doECSCheckZero(device, destination)
			if gotEcsZero == 0 || gotEcsZero == 2 {
				return fmt.Errorf("failed to receive ECS zero on fast-path")
			}
			channels.WriteCommandExecInput("ecs_done", "")

			// BUG FIX: Use getRawApos() not pdoFbActual.Load().
			refreshCurrentPositionForDevice(device)
			doneDriverAction()
			channels.DestinationReached()
			logger.Info("move to position completed (already at target)", "driver", device.Name, "id", uid)
			return nil
		}
	}

	// setDirection uses notifyDriverStatusWithWait — it blocks until the
	// driver_status_keeper listener has updated backlash in the map.
	// We must re-read status AFTER this call to get the correct backlash value
	// (either backlashInSetting for CCW, or 0 for CW).
	// Reading from driverStatusAfterECS here would use a stale snapshot and
	// backlash compensation would never apply correctly on direction changes.
	setDirection(device, driverStatusAfterECS, moveToPos)
	notifyDriverStatus("destination_position", fmt.Sprintf("%f", destination), device)

	driverStatusAfterDirection := getCurrentDriverStatus(device.Device.Name) // fresh read — backlash now updated by setDirection
	backlash := driverStatusAfterDirection.backlash
	pitchErr := getPitchError(device.Name, destination)
	moveToWithComp := moveToPos + pitchErr - backlash

	logger.Info("compensation: moveToPos=", moveToPos, "pitchErr=", pitchErr, "backlash=", backlash, "moveToWithComp=", moveToWithComp)

	// Execute the physical move.
	// doRotate internally: declamp → move → wait bit10 → clamp
	if err := doRotate(device, moveToWithComp); err != nil {
		return err
	}

	// Send FIN only after clamp confirmed (doRotate returned successfully).
	if err := sendECSFinSignal(device); err != nil {
		return err
	}

	// Wait for ECS LOW before advancing to next line.
	channels.WriteCommandExecInput("waiting_for_ecs", "")
	gotEcsZero := doECSCheckZero(device, destination)
	if gotEcsZero == 0 || gotEcsZero == 2 {
		return fmt.Errorf("failed to receive ECS zero")
	}
	channels.WriteCommandExecInput("ecs_done", "")

	doneDriverAction()
	channels.DestinationReached()
	// BUG FIX: Use getRawApos() not pdoFbActual.Load().
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

// refreshCurrentPositionForDevice syncs cached currentPosition to live encoder.
// BUG FIX: Uses getRawApos() not pdoFbActual.Load() — sign-flip correction applied.
func refreshCurrentPositionForDevice(device MasterDevice) {
	rawPos := getRawApos()
	curPos, _ := currentPosition(rawPos, device.Device.DriveXRatio, device.Name)
	status := getCurrentDriverStatus(device.Name)
	status.currentPosition = curPos
	status.destinationPosition = curPos
	setCurrentDriverStatus(device.Name, status)
	logger.Info("[SYNC] Updated current position:", curPos, "° for drive", device.Name)
}

func ReadActualPositionFromDrive(device MasterDevice) float64 {
	rawPos := getRawApos()
	curPos, _ := currentPosition(rawPos, device.Device.DriveXRatio, device.Name)
	return curPos
}