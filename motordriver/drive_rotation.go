package motordriver

import (
	helper "EtherCAT/helper"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"errors"
	"fmt"
	"math"
	"time"
)

func reverseDir(masterDevice MasterDevice) error {
	logger.Trace("Reverse direction activated for driver", masterDevice.Name)
	operation, err := GetEtherCATOperation("reverse", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		_ = SDODownload(masterDevice.Master, masterDevice.Position, step)
	}
	return nil
}

func nonReverseDir(masterDevice MasterDevice) error {
	logger.Trace("Non reverse direction activated for driver", masterDevice.Name)
	operation, err := GetEtherCATOperation("nonreverse", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		_ = SDODownload(masterDevice.Master, masterDevice.Position, step)
	}
	return nil
}

func ManualJog(masterDevice MasterDevice, direction int) error {
	driverStatus := getCurrentDriverStatus(masterDevice.Device.Name)
	if driverStatus.potNotExceeded {
		logger.Error("pot/not exceeded, exiting from jog")
		return errors.New("pot/not exceeded, exiting from jog")
	}
	if (pdoFbStatus.Load() & 0x0008) != 0 {
		logger.Error("ManualJog blocked — drive is faulted (stw bit3=1), reset first")
		return errors.New("drive is faulted, reset before jogging")
	}
	logger.Trace("manual jog (PDO velocity), driver:", masterDevice.Name)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	notifyDriverStatus("set_backlash", fmt.Sprintf("%f", 0.00), masterDevice)
	FastPowerOn(masterDevice)
	_, declampErr := hasDeclamped(masterDevice, envSettings)
	if declampErr != nil {
		logger.Error(declampErr)
		statusnotifier.Alarm(declampErr.Error())
		return declampErr
	}
	notifyDriverStatus("motor_running", "true", masterDevice)
	rpm := direction * int(masterDevice.Device.RPMConst*envSettings.JogFeed)
	pdoJogActive.Store(false)
	pdoJogStep.Store(0)
	pdoCmdMode.Store(cia402ModeProfileVelocity)
	pdoEnableRequested.Store(true)
	cmd := int64(rpm)
	if cmd > math.MaxInt32 {
		cmd = math.MaxInt32
	}
	if cmd < math.MinInt32 {
		cmd = math.MinInt32
	}
	pdoCmdVelocity.Store(int32(cmd))
	// Seed target to current corrected position so PP hold after jog stops is correct.
	pdoCmdTarget.Store(getRawApos())
	logger.Info("ManualJog PV started. rpm=", rpm, " 60FF=", int32(cmd))
	return nil
}

func StopJog(masterDevice MasterDevice) error {
	logger.Trace("stop jog (PDO velocity fast-stop), driver:", masterDevice.Name)
	pdoJogActive.Store(false)
	pdoJogStep.Store(0)
	pdoCmdVelocity.Store(0)
	if int8(pdoFbMode.Load()) == cia402ModeProfileVelocity {
		pdoEnableRequested.Store(false)
		go func() {
			time.Sleep(120 * time.Millisecond)
			// Re-enable holding at corrected current position.
			pdoCmdTarget.Store(getRawApos())
			pdoEnableRequested.Store(true)
		}()
	} else {
		// Already in PP — hold at corrected current position.
		pdoCmdTarget.Store(getRawApos())
	}
	notifyDriverStatus("motor_running", "false", masterDevice)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	_, clampErr := hasClamped(masterDevice, envSettings)
	if clampErr != nil {
		logger.Error(clampErr)
		statusnotifier.Alarm(clampErr.Error())
		return clampErr
	}
	logger.Info("StopJog PV done. 60FF=0, enable dropped briefly for fast stop.")
	return nil
}

func hasTargetReached(masterDevice MasterDevice) error {
	operation, err := GetEtherCATOperation("jog", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	driver := GetMotorDriver()
	driver.hasTargetReached(masterDevice, 0, 0, operation)
	return nil
}

func freeRotate(masterDevice MasterDevice, valueinDegree float64) error {
	logger.Trace("freeRotate to position, driver", masterDevice.Name, "degree:", valueinDegree)
	err := doRotate(masterDevice, valueinDegree)
	if err != nil {
		return err
	}
	doneDriverAction()
	logger.Trace("freeRotate to position completed, driver", masterDevice.Name)
	return nil
}

func doRotate(masterDevice MasterDevice, valueinDegree float64) error {
	FastPowerOn(masterDevice)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	_, declampErr := hasDeclamped(masterDevice, envSettings)
	if declampErr != nil {
		logger.Error(declampErr)
		statusnotifier.Alarm(declampErr.Error())
		return declampErr
	}

	notifyDriverStatus("motor_running", "true", masterDevice)
	deltaPulses64 := getPulsesFromDegree(masterDevice, helper.RoundFloatTo3(valueinDegree))
	if deltaPulses64 == 0 {
		notifyDriverStatus("motor_running", "false", masterDevice)
		_, clampErr := hasClamped(masterDevice, envSettings)
		return clampErr
	}

	// Use getRawApos() so the move target is computed from the sign-corrected
	// position. If a sign flip occurred at boot and we used pdoFbActual.Load()
	// directly here, the target pulse would be wrong by ~2× the position value,
	// sending the motor to a completely incorrect angle.
	currentActual := getRawApos()
	delta := deltaPulses64
	if delta > math.MaxInt32 {
		delta = math.MaxInt32
	}
	if delta < math.MinInt32 {
		delta = math.MinInt32
	}
	target := int32(int64(currentActual) + delta)

	pdoCmdVelocity.Store(0)
	pdoJogActive.Store(false)
	pdoJogStep.Store(0)
	pdoCmdMode.Store(cia402ModeProfilePosition)
	pdoCmdTarget.Store(target)
	pdoEnableRequested.Store(true)

	const (
		timeout  = 5 * time.Second
		poll     = 2 * time.Millisecond
		modeWait = 200 * time.Millisecond
	)
	modeStart := time.Now()
	for time.Since(modeStart) < modeWait {
		if int8(pdoFbMode.Load()) == cia402ModeProfilePosition {
			break
		}
		time.Sleep(poll)
	}
	if int8(pdoFbMode.Load()) != cia402ModeProfilePosition {
		logger.Info("Warning: drive did not confirm PP mode within 200ms, proceeding anyway")
	}

	pdoNewSetPoint.Store(true)
	logger.Info("PDO PP move issued. deg=", helper.RoundFloatTo3(valueinDegree), " target=", target)

	ackStart := time.Now()
	for time.Since(ackStart) < 2*time.Second {
		if !pdoNewSetPoint.Load() {
			break
		}
		time.Sleep(poll)
	}
	if pdoNewSetPoint.Load() {
		pdoNewSetPoint.Store(false)
		logger.Info("Warning: drive did not acknowledge new set-point (bit12 timeout) — move may not execute")
	}

	// Phase 2a: wait for stale target-reached bit to clear.
	clearStart := time.Now()
	for time.Since(clearStart) < timeout {
		if pdoStopRequest.Load() {
			return errors.New("move aborted: stop request active")
		}
		if (pdoFbStatus.Load() & (1 << 10)) == 0 {
			break
		}
		time.Sleep(poll)
	}

	// Phase 2b: wait for the next real target-reached edge.
	start := time.Now()
	for time.Since(start) < timeout {
		if pdoStopRequest.Load() {
			return errors.New("move aborted: stop request active")
		}
		if (pdoFbStatus.Load() & (1 << 10)) != 0 {
			break
		}
		time.Sleep(poll)
	}
	if time.Since(start) >= timeout {
		return fmt.Errorf("timeout waiting for target reached (bit10)")
	}

	logger.Info("Target reached (bit10) for driver:", masterDevice.Name)

	// Resync tpos = actual settled apos.
	// After a move, bit10 fires when the drive declares the target reached,
	// but apos may have settled 1-2 pulses away from the commanded tpos due
	// to deceleration overshoot or encoder quantisation. If we leave tpos
	// pointing at the old commanded value, the next move computes:
	//   newTarget = getRawApos() + delta
	// which is correct, but the drive's internal reference (tpos) disagrees
	// by those residual pulses. Over many moves this accumulates as drift —
	// visible as the motor making a small extra rotation at the start of each
	// new move to "close" the tpos gap before executing the real move.
	// Resyncing tpos to the actual apos here eliminates that drift entirely.
	pdoCmdTarget.Store(getRawApos())

	notifyDriverStatus("motor_running", "false", masterDevice)
	_, clampErr := hasClamped(masterDevice, envSettings)
	if clampErr != nil {
		logger.Error(clampErr)
		statusnotifier.Alarm(clampErr.Error())
		return clampErr
	}
	return nil
}