package motordriver

/**
Specialized implementation of Panasonic A6 Minas motor driver. All the functions in this file is
specifically designed for A6 Minas. In case to run against a different driver then implement the
same functions in another file.
**/
import (
	ethercatDevice "EtherCAT/ethercatdevicedatatypes"
	helper          "EtherCAT/helper"
	logger          "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"errors"
	"fmt"
	"time"
)

// A6Minas implementation for Panasonic A6 Minas driver
type A6Minas struct{}

// hasTargetReached waits for CiA402 bit10 (target reached) via PDO status word.
// No SDO or mutex needed — reads pdoFbStatus atomic directly.
func (a6 A6Minas) hasTargetReached(masterDevice MasterDevice, action int, immediate int, operation ethercatDevice.Operation) error {
	logger.Trace("A6Minas waiting for target reached via PDO stw")
	for {
		stw := uint16(pdoFbStatus.Load())
		if (stw & 0x0400) != 0 {
			logger.Trace("A6Minas target reached stw=0x", fmt.Sprintf("%04X", stw))
			return nil
		}
		time.Sleep(5 * time.Millisecond)
	}
}

// Not used
func (a6 A6Minas) potNotEnabled(masterDevice MasterDevice) (bool, error) {
	inputStatus, err := readInputSignal(masterDevice)
	if err != nil {
		return false, err
	}
	logger.Debug("Input signal value to check pot/not status", inputStatus)
	toBinary := helper.IntToBinary(inputStatus)
	if toBinary[0] == '1' || toBinary[1] == '1' {
		return true, nil
	}
	return false, nil
}

// readDeclampSignal polls the DCL bit (bit 7 of 4F25) until the table
// confirms it has declamped, or the timeout expires.
//
// FIX: The old implementation had two conflicting exit conditions —
// a loop counter (i < declampTiming) and an elapsed-ms check. With PDO
// reads being ~0ms (atomic), the loop ran only 3ms total (3000 × 1µs)
// instead of the intended 3000ms, causing false timeouts even when the
// clamp had already released.
//
// New implementation: single deadline-based loop, polls every 10ms,
// true timeout = declampTiming milliseconds from start.
func (a6 A6Minas) readDeclampSignal(masterDevice MasterDevice, declampTiming int) (bool, error) {
	logger.Debug("waiting for declamp status, timeout:", declampTiming, "ms")
	deadline := time.Now().Add(time.Duration(declampTiming) * time.Millisecond)

	for time.Now().Before(deadline) {
		// DCL = bit 7 of 4F25, active high — table has declamped
		val := GetDigitalInputs4F25()
		toBinary := helper.IntToBinary(int(val))
		if len(toBinary) > 7 && toBinary[7] == '1' {
			logger.Debug("declamp confirmed after", time.Duration(declampTiming)*time.Millisecond-time.Until(deadline))
			return true, nil
		}
		time.Sleep(10 * time.Millisecond)
	}

	logger.Error("declamp timeout after", declampTiming, "ms — DCL signal never went high")
	statusnotifier.DriverError(65379)
	return false, errors.New("declamp failed")
}

// readClampSignal polls the CL bit (bit 6 of 4F25) until the table
// confirms it has clamped, or the timeout expires.
//
// FIX: Same dual-exit bug as readDeclampSignal — see comment above.
// New implementation: single deadline-based loop, polls every 10ms,
// true timeout = clampTiming milliseconds from start.
func (a6 A6Minas) readClampSignal(masterDevice MasterDevice, clampTiming int) (bool, error) {
	logger.Debug("waiting for clamp status, timeout:", clampTiming, "ms")
	deadline := time.Now().Add(time.Duration(clampTiming) * time.Millisecond)

	for time.Now().Before(deadline) {
		// CL = bit 6 of 4F25, active high — table has clamped
		val := GetDigitalInputs4F25()
		toBinary := helper.IntToBinary(int(val))
		if len(toBinary) > 6 && toBinary[6] == '1' {
			logger.Debug("clamp confirmed after", time.Duration(clampTiming)*time.Millisecond-time.Until(deadline))
			return true, nil
		}
		time.Sleep(10 * time.Millisecond)
	}

	logger.Error("clamp timeout after", clampTiming, "ms — CL signal never went high")
	statusnotifier.DriverError(65378)
	return false, errors.New("clamp failed")
}

// receivedECS polls the ECS bit (bit 0 of 4F25) until it goes high.
// Returns 1=received, 0=not received, 2=stop requested.
func (a6 A6Minas) receivedECS(masterDevice MasterDevice, operation ethercatDevice.Operation, stopECSChan chan bool) int {
	const ecsMask = uint32(1 << 0) // bit 0 = ECS, active high
	for {
		select {
		default:
			if (GetDigitalInputs4F25() & ecsMask) != 0 {
				return 1
			}
			time.Sleep(50 * time.Microsecond)
		case <-stopECSChan:
			logger.Debug("stopping ECS Polling")
			return 2
		}
	}
}

func (a6 A6Minas) receivedECSZero(masterDevice MasterDevice, operation ethercatDevice.Operation, stopECSChan chan bool) int {
	const ecsMask = uint32(1 << 0)
	for {
		select {
		default:
			if (GetDigitalInputs4F25() & ecsMask) == 0 {
				return 1
			}
			time.Sleep(50 * time.Microsecond)
		case <-stopECSChan:
			logger.Debug("stopping ECS zero Polling")
			return 2
		}
	}
}

func (a6 A6Minas) sendFinishSignal(masterDevice MasterDevice, operation ethercatDevice.Operation) error {
	return nil
}

var stopIOStatChan chan bool

func (a6 A6Minas) pollIOStat(avilableDevices []MasterDevice) {
	logger.Debug("starting A6Minas io status listener")
	stopIOStatChan = make(chan bool)
	for _, d := range avilableDevices {
		go a6.ioStatusListener(d)
	}
}

func (a6 A6Minas) stopPollIOStat() {
	stopIOStatChan <- true
}

func (a6 A6Minas) ioStatusListener(masterDev MasterDevice) {
	// Edge detectors — only act on rising edge to prevent storm on sustained signal.
	// POT/NOT bounce at the limit: without edge detection, every poll cycle while
	// the limit is active fires FastPowerOff+StopJog, flooding the system.
	lastHardReset := false
	lastPOT       := false
	lastNOT       := false

	for {
		select {
		default:
			var ioStat statusnotifier.IOStatus

			// All signals come from the PDO-fed 4F25 atomic — no SDO calls.
			val      := GetDigitalInputs4F25()
			toBinary := helper.IntToBinary(int(val))

			// ECS (bit 0, active high)
			ioStat.ECS = a6.isIOOn(toBinary, 0, '1')

			// Hard reset (bit 5, active high) — rising edge only
			hardResetNow := a6.isIOOn(toBinary, 5, '1')
			if hardResetNow && !lastHardReset {
				logger.Info("io status listener received hard reset input and performing system reset.")
				go performSysReset(true)
			}
			lastHardReset = hardResetNow

			// IO signals
			ioStat.NOT   = a6.isIOOn(toBinary, 2, '0')
			ioStat.POT   = a6.isIOOn(toBinary, 1, '0')
			ioStat.HOME  = a6.isIOOn(toBinary, 3, '0')
			ioStat.CL    = a6.isIOOn(toBinary, 6, '1')
			ioStat.DCL   = a6.isIOOn(toBinary, 7, '1')
			ioStat.ALMIN = a6.isIOOn(toBinary, 4, '0')

			driverState  := getCurrentDriverStatus(masterDev.Name)
			ioStat.FIN   = driverState.isSendingFinSignal
			ioStat.SOLOP = driverState.isDriverOnOff
			// Drive fault state from stw bit3 — drives the FAULT LED in the UI
			ioStat.Fault = (pdoFbStatus.Load() & 0x0008) != 0

			statusnotifier.NotifyIOStatus(ioStat)

			if masterDev.Device.StopWhenHWPOTNOT {
				// Rising edge only — do not repeat FastPowerOff/StopJog while
				// limit remains active. Signal bounces at the mechanical limit
				// causing repeated triggers every poll cycle without this guard.
				if ioStat.NOT && !lastNOT {
					statusnotifier.Alarm("NOT Limit Exceeded")
					logger.Error("hardware NOT activated")
					FastPowerOff(masterDev)
					StopJog(masterDev)
				}
				if ioStat.POT && !lastPOT {
					statusnotifier.Alarm("POT Limit Exceeded")
					logger.Error("hardware POT activated")
					FastPowerOff(masterDev)
					StopJog(masterDev)
				}
			}
			lastNOT = ioStat.NOT
			lastPOT = ioStat.POT

			interval := 1000
			if masterDev.Device.IOPollingInterval > 0 {
				interval = masterDev.Device.IOPollingInterval
			}
			time.Sleep(time.Duration(interval) * time.Microsecond)

		case <-stopIOStatChan:
			logger.Debug("stopping A6Minas io status listener")
			return
		}
	}
}

func (a6 A6Minas) isIOOn(binary string, binpos int, isVal rune) bool {
	if len(binary) <= 0 {
		return false
	}
	if binary[binpos] == byte(isVal) {
		return true
	}
	return false
}