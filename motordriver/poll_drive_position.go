package motordriver

/**
Position polling uses PDO feedback updated by the cyclic loop.
getRawApos() is used instead of pdoFbActual.Load() directly so that the
boot-time encoder sign-flip correction is applied transparently here.
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

var stopChan chan bool

func pollDrivePosition(availableDevices []MasterDevice) error {
	logger.Debug("starting driver position listener")
	stopChan = make(chan bool)
	for _, device := range availableDevices {
		go pollDrivePositionProcess(device)
	}
	return nil
}

func stopDriverPolling() {
	if stopChan != nil {
		stopChan <- true
	}
}

func pollDrivePositionProcess(device MasterDevice) {
	logger.Info("polling status of driver:", device.Name)

	for {
		select {
		default:
			driverSettings := settings.GetDriverSettings(device.Name)

			// Wait until the cyclic loop has received at least one valid PDO frame
			// (WC=COMPLETE) before reading position. Without this guard, the first
			// broadcast to the UI uses a zero or stale value — showing wrong
			// position on first boot.
			if !pdoDomainValid.Load() {
				time.Sleep(10 * time.Millisecond)
				continue
			}

			// getRawApos() applies sign-flip correction transparently.
			// HomingOffset, pitch error, work offsets are all applied downstream
			// in currentPosition() and the withErrCorr calculation — unchanged.
			rawPosition := getRawApos()

			driveStatus := getCurrentDriverStatus(device.Name)

			curPos, posWithErrCorrection :=
				currentPosition(rawPosition, device.Device.DriveXRatio, device.Name)

			go checkPotNotLimit(curPos, device, driverSettings, driveStatus)

			pitchError := getPitchError(device.Name, driveStatus.destinationPosition)

			withErrCorr :=
				(posWithErrCorrection - pitchError) +
					driveStatus.backlash -
					driveStatus.workOffset

			if withErrCorr >= 359.999 {
				withErrCorr = 0
			}

			// Rate-limit HMI broadcasts to 50ms (20Hz).
			// The human eye cannot perceive updates faster than ~24Hz, and
			// socket.io + JSON encode at 100Hz floods the Pi's CPU and SD card,
			// causing HMI lag. Motor control is unaffected — it runs via PDO
			// atomics at 2ms cycle, independent of this display loop.
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

	if driverSettings.NOT >= 0 || driverSettings.POT <= 0 {
		return false
	}

	if driverStatus.potNotExceeded {
		if driverStatus.potExceeded {
			statusnotifier.Alarm("POT Limit Exceeded")
		} else {
			statusnotifier.Alarm("NOT Limit Exceeded")
		}
		return true
	}

	threshold := device.Device.PotNotThreshold
	exceeded = false

	pot := float64(driverSettings.POT)
	not := float64(driverSettings.NOT)
	not = 360 + not

	if driverStatus.direction == 1 && pot > 0 {
		if currentPosition >= (pot-threshold) &&
			currentPosition <= pot+(threshold*10) {

			FastPowerOff(masterDevices[0])
			StopJog(masterDevices[0])
			logger.Trace("POT limit exceeded")
			channels.WriteCommandExecInput("stop_prog_exec", "")
			statusnotifier.Alarm("POT Limit Exceeded")
			exceeded = true
			notifyDriverStatus("pot_not_exceeded", "POT", device)
		}
	}

	if driverStatus.direction == -1 && not > 0 {
		if currentPosition <= (not+threshold) &&
			currentPosition >= not-(threshold*10) {

			FastPowerOff(masterDevices[0])
			StopJog(masterDevices[0])
			logger.Trace("NOT limit exceeded")
			channels.WriteCommandExecInput("stop_prog_exec", "")
			statusnotifier.Alarm("NOT Limit Exceeded")
			notifyDriverStatus("pot_not_exceeded", "NOT", device)
			exceeded = true
		}
	}

	return
}