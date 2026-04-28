package motordriver

import (
	logger   "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"errors"
)

// hasDeclamped verifies declamp before a move.
// If ClampDeclamp=0 (disabled), returns true immediately — no action.
func hasDeclamped(masterDevice MasterDevice, envSettings settings.DriverSettings) (bool, error) {
	driver := GetMotorDriver()
	if envSettings.ClampDeclamp == 1 {
		breakOff(masterDevice)
		logger.Debug("Clamp declamp enabled, checking declamp status")
		isDeclamped, clampErr := driver.readDeclampSignal(masterDevice, envSettings.ClampDeclampTiming)
		if clampErr != nil {
			msg := "DECLAMP FAILED — table did not release within timeout. " +
				"Check pneumatic/hydraulic supply and declamp sensor. " +
				"Do NOT run program until resolved."
			logger.Error(msg)
			statusnotifier.Alarm(msg)
		}
		return isDeclamped, clampErr
	}
	return true, nil
}

// hasClamped verifies clamp after a move.
// If ClampDeclamp=0 (disabled), returns false, nil — no clamp, no error.
//
// Sequence is always: breakOn → FastPowerOff → readClampSignal
//   - breakOn:        engage mechanical brake
//   - FastPowerOff:   remove drive enable so motor cannot move even if brake fails
//   - readClampSignal: confirm brake is physically locked before continuing
//
// If clamp confirmation times out, the brake was commanded ON and the drive
// is powered off — table is physically stopped. But the sensor did not confirm.
// Operator must inspect before resuming. A clear alarm is sent.
func hasClamped(masterDevice MasterDevice, envSettings settings.DriverSettings) (bool, error) {
	driver := GetMotorDriver()
	if envSettings.ClampDeclamp == 1 {
		breakOn(masterDevice)
		FastPowerOff(masterDevice)

		logger.Debug("Clamp declamp enabled, checking clamp status")
		isClamped, clampErr := driver.readClampSignal(masterDevice, envSettings.ClampDeclampTiming)
		if clampErr != nil {
			msg := "CLAMP FAILED — table did not lock after move. " +
				"Table is at an intermediate angle. " +
				"Check clamp sensor and pneumatic/hydraulic supply. " +
				"Run ZERO REF before resuming program."
			logger.Error(msg)
			statusnotifier.Alarm(msg)
			return false, errors.New(msg)
		}
		if !isClamped {
			msg := "CLAMP ERROR — clamp signal not confirmed. " +
				"Run ZERO REF before resuming program."
			logger.Error(msg)
			statusnotifier.Alarm(msg)
			return false, errors.New(msg)
		}
		return true, nil
	}
	return false, nil
}

// applyClampIfSettingsChanged applies clamp state when settings change at runtime.
func applyClampIfSettingsChanged() {
	for _, dev := range masterDevices {
		devSetting := settings.GetDriverSettings(dev.Name)
		stat := getCurrentDriverStatus(dev.Name)
		if stat.isMotorRunning {
			return
		}
		if devSetting.ClampDeclamp == 1 {
			logger.Debug("CL/DL enabled power off driver", dev.Name)
			FastPowerOff(dev)
		} else {
			logger.Debug("CL/DL disabled power on driver", dev.Name)
			FastPowerOn(dev)
		}
	}
}