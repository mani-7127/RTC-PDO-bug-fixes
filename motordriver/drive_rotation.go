package motordriver

import (
	helper "EtherCAT/helper"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"errors"
	"fmt"
	"math"
	"sync/atomic"
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

// ManualJog starts jogging using PDO velocity mode (Profile Velocity, mode=3).
// Direction param: 1 = clockwise, -1 = counter-clockwise.
// pot/not exceeded guard: blocks motion INTO the limit, but allows escape
// in the opposite (safe) direction so the operator can recover without a full reset.
func ManualJog(masterDevice MasterDevice, direction int) error {
	driverStatus := getCurrentDriverStatus(masterDevice.Device.Name)
	if driverStatus.potNotExceeded {
		// When signFlipActive, command direction is inverted in position space.
		// Use effectiveDirection for limit gating so escape works correctly.
		effectiveDirection := direction
		if signFlipActive.Load() {
			effectiveDirection = -effectiveDirection
		}
		// POT (CW limit in position space) — block if moving toward POT
		if driverStatus.potExceeded && effectiveDirection == 1 {
			logger.Error("POT limit active, cannot jog toward POT. Jog away to escape.")
			return errors.New("pot/not exceeded, exiting from jog")
		}
		// NOT (CCW limit in position space) — block if moving toward NOT
		if driverStatus.notExceeded && effectiveDirection == -1 {
			logger.Error("NOT limit active, cannot jog toward NOT. Jog away to escape.")
			return errors.New("pot/not exceeded, exiting from jog")
		}
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

	// RPM calculation is identical to the original uploaded version.
	rpm := direction * int(masterDevice.Device.RPMConst*envSettings.JogFeed)

	// Switch to PDO velocity mode; invalidate any pending StopJog re-enable goroutine.
	pdoJogActive.Store(false)
	pdoJogStep.Store(0)
	pdoCmdMode.Store(cia402ModeProfileVelocity)
	pdoEnableRequested.Store(true)
	jogStopSeq.Add(1)

	cmd := int64(rpm)
	if cmd > math.MaxInt32 {
		cmd = math.MaxInt32
	}
	if cmd < math.MinInt32 {
		cmd = math.MinInt32
	}
	pdoCmdVelocity.Store(int32(cmd))
	// Seed target to current corrected position so PP hold after jog stops is correct.
	pdoCmdTarget.Store(correctedToCmdTarget(getRawApos()))
	logger.Info("ManualJog PV started. rpm=", rpm, " 60FF=", int32(cmd))
	return nil
}

// jogStopSeq is incremented each time StopJog is called.
// The re-enable goroutine checks it before writing — if it changed,
// another operation already took over and the goroutine must not overwrite
// the new target (fixes motor-spinning-endlessly-after-jog bug).
var jogStopSeq atomic.Int64

// StopJog stops jogging and restores Profile Position mode.
// Clamp logic from the original uploaded version is preserved.
func StopJog(masterDevice MasterDevice) error {
	logger.Trace("stop jog (PDO velocity fast-stop), driver:", masterDevice.Name)
	pdoJogActive.Store(false)
	pdoJogStep.Store(0)
	pdoCmdVelocity.Store(0)
	if int8(pdoFbMode.Load()) == cia402ModeProfileVelocity {
		pdoEnableRequested.Store(false)

		// BUG FIX: The re-enable goroutine previously fired unconditionally
		// after 120ms and overwrote pdoCmdTarget with getRawApos(). If zero-ref
		// or a program move started within that 120ms window, the goroutine
		// silently corrupted the new move target mid-flight, causing the motor
		// to chase a moving target and appear to rotate endlessly.
		//
		// Fix: capture the sequence number before sleeping. If it changed by
		// the time we wake up, another operation owns the drive — abort.
		seq := jogStopSeq.Add(1)
		go func() {
			time.Sleep(120 * time.Millisecond)
			if jogStopSeq.Load() != seq {
				logger.Debug("StopJog re-enable goroutine cancelled — drive already claimed")
				return
			}
			pdoCmdTarget.Store(correctedToCmdTarget(getRawApos()))
			pdoEnableRequested.Store(true)
		}()
	} else {
		// Already in PP — hold at corrected current position.
		pdoCmdTarget.Store(correctedToCmdTarget(getRawApos()))
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

// hasTargetReached calls the specialized driver module to check whether the
// intended target position has been reached. Behaviour unchanged.
func hasTargetReached(masterDevice MasterDevice) error {
	operation, err := GetEtherCATOperation("jog", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	driver := GetMotorDriver()
	driver.hasTargetReached(masterDevice, 0, 0, operation)
	return nil
}

// freeRotate rotates without waiting for ECS or sending a fin signal.
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

// doRotate powers on and rotates by the given degree, waiting for declamp
// before moving and clamp after. Uses PDO Profile Position mode.
//
// Key changes from the original uploaded version:
//   - Checks AL state before issuing any move (prevents infinite spin on SAFEOP).
//   - Uses PDO atomics (pdoCmdTarget, pdoNewSetPoint) instead of SDO moveToPosition.
//   - Waits for CiA402 bit10 (target reached) instead of calling hasTargetReached().
//   - Resyncs tpos to actual apos after bit10 to eliminate cumulative drift.
//   - Invalidates the StopJog re-enable goroutine before starting.
//
// All declamp/clamp, motor_running notifications, and pulse calculation via
// getPulsesFromDegree are unchanged from the original uploaded version.
func doRotate(masterDevice MasterDevice, valueinDegree float64) error {
	// Guard: reject move if EtherCAT is not in OP state.
	// When al_states=4 (SAFEOP), PDO outputs are zeroed — the stop command
	// never reaches the drive and any residual velocity keeps the motor spinning.
	alState := pdoAlState.Load()
	if alState != 0x08 {
		return fmt.Errorf("doRotate rejected: EtherCAT not in OP state (al_states=0x%02X) — wait for AL=OP before moving", alState)
	}

	// Invalidate any pending StopJog re-enable goroutine so it cannot
	// overwrite our move target mid-flight.
	jogStopSeq.Add(1)

	FastPowerOn(masterDevice)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	_, declampErr := hasDeclamped(masterDevice, envSettings)
	if declampErr != nil {
		logger.Error(declampErr)
		statusnotifier.Alarm(declampErr.Error())
		return declampErr
	}

	notifyDriverStatus("motor_running", "true", masterDevice)

	// Pulse calculation — identical to original uploaded version.
	deltaPulses64 := getPulsesFromDegree(masterDevice, helper.RoundFloatTo3(valueinDegree))
	if deltaPulses64 == 0 {
		notifyDriverStatus("motor_running", "false", masterDevice)
		_, clampErr := hasClamped(masterDevice, envSettings)
		return clampErr
	}

	// Compute move target in the sign-corrected coordinate space so delta
	// calculations are consistent with currentPosition() and display.
	// Then convert back to drive raw space before writing pdoCmdTarget,
	// because the drive hardware operates in its own uncorrected space.
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
	pdoCmdTarget.Store(correctedToCmdTarget(target))
	pdoEnableRequested.Store(true)

	const (
		timeout  = 5 * time.Second
		poll     = 2 * time.Millisecond
		modeWait = 200 * time.Millisecond
	)

	// Wait for drive to confirm PP mode before asserting new set-point.
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

	// Trigger the PP new-set-point handshake.
	// The cyclic loop holds bit4 until the drive acknowledges with stw bit12.
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

	// Phase 2a: wait for stale target-reached bit (bit10) to clear.
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

	// Phase 2b: wait for the next real target-reached edge (bit10 set).
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
	// After a move, bit10 fires when the drive declares target reached, but
	// apos may have settled 1-2 pulses away from commanded tpos due to
	// deceleration overshoot. Resyncing tpos here eliminates cumulative
	// drift — visible as a small extra rotation at the start of each new move.
	pdoCmdTarget.Store(correctedToCmdTarget(getRawApos()))

	notifyDriverStatus("motor_running", "false", masterDevice)
	_, clampErr := hasClamped(masterDevice, envSettings)
	if clampErr != nil {
		logger.Error(clampErr)
		statusnotifier.Alarm(clampErr.Error())
		return clampErr
	}
	return nil
}