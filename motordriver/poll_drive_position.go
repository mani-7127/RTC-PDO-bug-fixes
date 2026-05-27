package motordriver

/**
Position polling uses PDO feedback updated by the cyclic loop.
getRawApos() is used instead of pdoFbActual.Load() so that boot-time
encoder sign-flip correction is applied transparently.
**/

import (
	channels "EtherCAT/channels"
	logger   "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	notifier "EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"fmt"
	"time"
)

// stopChans is a per-device slice of stop channels.
//
// BUG FIX: Was a single shared chan bool. pollDrivePosition spawns one
// goroutine per device, but stopDriverPolling only sent one message —
// with 2+ devices, only the first goroutine stopped. The rest leaked,
// keeping ghost position polling loops running after reset and causing
// stale position broadcasts that interfered with the fresh poll loop.
var stopChans []chan bool

func pollDrivePosition(availableDevices []MasterDevice) error {
	logger.Debug("starting driver position listener")
	stopChans = make([]chan bool, 0, len(availableDevices))
	for _, device := range availableDevices {
		ch := make(chan bool)
		stopChans = append(stopChans, ch)
		go pollDrivePositionProcess(device, ch)
	}
	return nil
}

func stopDriverPolling() {
	for _, ch := range stopChans {
		if ch != nil {
			ch <- true
		}
	}
	stopChans = nil
}

func pollDrivePositionProcess(device MasterDevice, stopChan chan bool) {
	logger.Info("polling status of driver:", device.Name)

	for {
		select {
		default:
			driverSettings := settings.GetDriverSettings(device.Name)

			// Wait for first valid PDO frame before reading position.
			if !pdoDomainValid.Load() {
				time.Sleep(10 * time.Millisecond)
				continue
			}

			// getRawApos() applies sign-flip correction transparently.
			rawPosition := getRawApos()

			driveStatus := getCurrentDriverStatus(device.Name)

			curPos, posWithErrCorrection :=
				currentPosition(rawPosition, device.Device.DriveXRatio, device.Name)

			// FIX: inline call (not goroutine) so it reads fresh driveStatus,
			// and passes device instead of hardcoded masterDevices[0].
			checkPotNotLimit(curPos, device, driverSettings, driveStatus)

			pitchError := getPitchError(device.Name, driveStatus.destinationPosition)

			withErrCorr :=
				(posWithErrCorrection - pitchError) +
					driveStatus.backlash -
					driveStatus.workOffset

			if withErrCorr >= 359.999 {
				withErrCorr = 0
			}

			notifier.NotifyCurrentPosition(device.Name, withErrCorr)
			currentDriverPosition(device, curPos)

			dir := "CW"
			if driveStatus.direction <= 0 {
				dir = "CCW"
			}

			logger.PrintOnSameLine(
				"Current position:",
				"corrected:", fmt.Sprintf("%.3f", posWithErrCorrection),
				"actual:", fmt.Sprintf("%.3f", curPos),
				"pitch err:", fmt.Sprintf("%.3f", pitchError),
				"backlash:", fmt.Sprintf("%.3f", driveStatus.backlash),
				"workoffset:", fmt.Sprintf("%.3f", driveStatus.workOffset),
				"dir:", dir,
			)

			time.Sleep(50 * time.Millisecond)

		case <-stopChan:
			logger.Debug("stopping driver position listener")
			return
		}
	}
}

func checkPotNotLimit(
	currentPosition float64,
	device MasterDevice,
	driverSettings settings.DriverSettings,
	driverStatus driverCurrentStatus,
) (exceeded bool) {

	// FIX: was OR (||) — skipped limit checks whenever either limit was zero.
	// Corrected to AND (&&): only skip when BOTH limits are unset (0).
	if driverSettings.NOT >= 0 && driverSettings.POT <= 0 {
		return false
	}

	// FIX: removed re-alarm block — was re-firing POT/NOT alarm every 50ms
	// while motor was stopped at limit, causing alarm flood on UI.
	// The alarm is sent once when the limit is first detected below.

	// Only check limits while motor is moving.
	// This prevents false triggers when motor is stationary at or near
	// the limit position (e.g. after escape jog stops just inside zone).
	if !driverStatus.isMotorRunning {
		return false
	}

	threshold := device.Device.PotNotThreshold
	exceeded = false

	pot := float64(driverSettings.POT)
	not := float64(driverSettings.NOT)
	not = 360 + not

	// POT (positive/CW limit): only trigger when moving CW (direction == 1).
	// When direction == -1 (CCW/escape), motor is moving away — do not fire.
	if driverStatus.direction == 1 && pot > 0 {
		if currentPosition >= (pot-threshold) &&
			currentPosition <= pot+(threshold*10) {

			FastPowerOff(device)
			StopJog(device)
			logger.Error("POT limit exceeded at", currentPosition)
			channels.WriteCommandExecInput("stop_prog_exec", "")
			statusnotifier.Alarm("POT Limit Exceeded")
			exceeded = true
			notifyDriverStatus("pot_not_exceeded", "POT", device)
		}
	}

	// NOT (negative/CCW limit): only trigger when moving CCW (direction == -1).
	// When direction == 1 (CW/escape), motor is moving away — do not fire.
	if driverStatus.direction == -1 && not > 0 {
		if currentPosition <= (not+threshold) &&
			currentPosition >= not-(threshold*10) {

			FastPowerOff(device)
			StopJog(device)
			logger.Error("NOT limit exceeded at", currentPosition)
			channels.WriteCommandExecInput("stop_prog_exec", "")
			statusnotifier.Alarm("NOT Limit Exceeded")
			notifyDriverStatus("pot_not_exceeded", "NOT", device)
			exceeded = true
		}
	}

	return
}