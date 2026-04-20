package motordriver

/**
Error polling now uses pdoFbErrCode atomic (atomic.Uint32) updated by the
PDO cyclic loop every 1ms from 0x603F.

Previously used SDOUpload2("readError") every 100ms — this caused
"Failed to execute SDO upload: Input/output error" storms whenever the
drive was in fault state, since EtherCAT rejects SDO requests from
a faulted drive. PDO reads are unaffected by drive fault state.
**/
import (
	parser "EtherCAT/configparser"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"errors"
	"fmt"
	"strconv"
	"time"
)

var stopErrPollingChan chan bool

// pollDriveError starts error polling goroutines for each device.
func pollDriveError(avilableDevices []MasterDevice) error {
	logger.Debug("starting driver error listener")
	if len(avilableDevices) <= 0 {
		return errors.New("no driver found")
	}
	stopErrPollingChan = make(chan bool)
	for _, device := range avilableDevices {
		go pollDriveErrWorker(device)
	}
	return nil
}

func stopErrorPolling() {
	stopErrPollingChan <- true
}

// pollDriveErrWorker reads 0x603F error code from pdoFbErrCode atomic.
// Only notifies statusnotifier on change to avoid flooding on persistent faults.
func pollDriveErrWorker(device MasterDevice) {
	logger.Info("polling error of driver: ", device.Name)

	// Wait for cyclic loop to populate pdoFbErrCode before first read.
	time.Sleep(300 * time.Millisecond)

	const noCode = uint32(0xFFFFFFFF) // sentinel: force first notification
	lastErrCode := noCode
	lastNotify  := time.Now()

	for {
		select {
		default:
			errCode := pdoFbErrCode.Load()

			// Notify on change OR re-notify every 2s if fault is active.
			// Re-notification ensures newly connected UI clients see the fault
			// even if the socketserver sent "No Alarms" on their connect event.
			changed := errCode != lastErrCode

			if changed {
				// State change: full DriverError notification (logs + alarm + any side effects)
				if errCode != 0 {
					logger.Error("Drive error code: 0x", fmt.Sprintf("%04X", errCode))
				}
				statusnotifier.DriverError(int(errCode))
				lastErrCode = errCode
				lastNotify  = time.Now()
			} else if errCode != 0 && time.Since(lastNotify) >= 2*time.Second {
				// Persistent fault: re-broadcast alarm text only so newly connected UI
				// clients see the error. Do NOT call DriverError again — that would
				// trigger stop_prog_exec via UI JS every 2s (storm).
				errID := int(errCode) - 65280
				if errID > 0 {
					errString := parser.GetErrorString(strconv.Itoa(errID))
					statusnotifier.Alarm(errString)
				}
				lastNotify = time.Now()
			}

			time.Sleep(100 * time.Millisecond)

		case <-stopErrPollingChan:
			logger.Debug("stopping driver error listener")
			return
		}
	}
}