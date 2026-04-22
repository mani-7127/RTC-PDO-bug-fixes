package motordriver

import (
	channels "EtherCAT/channels"
	logger   "EtherCAT/logger"
	"EtherCAT/settings"
	"time"
)

var stopECSCheckChan chan bool
var isECSCheckInProgress bool

func init() {
	isECSCheckInProgress = false
	stopECSCheckChan = make(chan bool)
}

func stopECSCheck() {
	if isECSCheckInProgress {
		stopECSCheckChan <- true
	}
	isECSCheckInProgress = false
}

// -------------------------------------------------------------------
// ECS "GO-HIGH" Phase — wait until ECS goes high (ready to move)
// -------------------------------------------------------------------
func doECSCheck(masterDevice MasterDevice, degree float64) int {
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	if envSettings.ECS == 1 {
		logger.Debug("driver", masterDevice.Name, "waiting for ECS HIGH. rotate to", degree)
		ecsRec, _ := waitForECS(masterDevice)
		if ecsRec == 1 {
			logger.Debug("driver", masterDevice.Name, "received ECS HIGH")
		} else if ecsRec == 0 {
			logger.Error("driver", masterDevice.Name, "NOT received ECS HIGH")
			channels.SendAlarm("Not received ECS")
		} else {
			logger.Info("exiting ECS check due to stop/reset event")
		}
		return ecsRec
	}
	return 1
}

// -------------------------------------------------------------------
// ECS "GO-LOW" Phase — wait until ECS goes low again
// -------------------------------------------------------------------
func doECSCheckZero(masterDevice MasterDevice, degree float64) int {
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	if envSettings.ECS == 1 {
		logger.Debug("driver", masterDevice.Name, "waiting for ECS LOW. rotate to", degree)
		ecsRec, _ := waitForECSZero(masterDevice)
		if ecsRec == 1 {
			logger.Debug("driver", masterDevice.Name, "ECS went LOW")
		} else if ecsRec == 0 {
			logger.Error("driver", masterDevice.Name, "ECS zero NOT received")
			channels.SendAlarm("ECS did not go LOW")
		} else {
			logger.Info("exiting ECS zero check due to stop/reset event")
		}
		return ecsRec
	}
	return 1
}

// -------------------------------------------------------------------
// Wait for ECS HIGH — delegates to driver.receivedECS (PDO-based)
// -------------------------------------------------------------------
func waitForECS(masterDevice MasterDevice) (int, error) {
	// operation is passed for interface compatibility only —
	// receivedECS reads from GetDigitalInputs4F25() PDO atomic, not SDO.
	operation, err := GetEtherCATOperation("ecs", masterDevice.Device.AddressConfigName)
	if err != nil {
		return 0, err
	}
	driver := GetMotorDriver()
	isECSCheckInProgress = true
	ecsStat := driver.receivedECS(masterDevice, operation, stopECSCheckChan)
	isECSCheckInProgress = false
	return ecsStat, nil
}

// -------------------------------------------------------------------
// Read ECS bit from PDO atomic — no SDO, no YAML operation needed.
// Returns 1 = high, 0 = low.
// -------------------------------------------------------------------
func readECSInput(_ MasterDevice) int {
	const ecsBitMask = uint32(0x01) // bit 0 = ECS in 4F25
	if GetDigitalInputs4F25()&ecsBitMask == 0 {
		return 0
	}
	return 1
}

// -------------------------------------------------------------------
// Wait for ECS LOW — polls PDO with debounce, no timeout.
// -------------------------------------------------------------------
func waitForECSZero(masterDevice MasterDevice) (int, error) {
	isECSCheckInProgress = true
	defer func() { isECSCheckInProgress = false }()

	logger.Info("Waiting for ECS to go LOW...")

	for {
		if readECSInput(masterDevice) == 0 {
			// Debounce: confirm stable LOW for 5 consecutive reads × 20ms = 100ms
			stable := true
			for i := 0; i < 5; i++ {
				time.Sleep(20 * time.Millisecond)
				if readECSInput(masterDevice) != 0 {
					stable = false
					break
				}
			}
			if stable {
				logger.Info("ECS signal stable LOW — continuing")
				return 1, nil
			}
		}

		select {
		case <-stopECSCheckChan:
			logger.Info("ECS zero wait interrupted by stop/reset")
			return 2, nil
		default:
		}

		time.Sleep(50 * time.Millisecond)
	}
}

// -------------------------------------------------------------------
// Send finish signal via PDO (60FE:01 and 60FE:02).
// Value 65536 = 1<<16, same bit written via SDO before.
//
// Waits for apos to fully settle before asserting 60FE.
// Uses getRawApos() so the settle check uses the sign-corrected value —
// consistent with everything else in the position pipeline.
// -------------------------------------------------------------------
func sendECSFinSignal(device MasterDevice) error {
	envSettings := settings.GetDriverSettings(device.Name)
	if envSettings.FinishSignal == 0 {
		return nil
	}

	// Wait for apos to stabilise before asserting 60FE.
	const (
		settleWindow  = 5
		settlePoll    = 20 * time.Millisecond
		settleTimeout = 2 * time.Second
	)
	lastPos := getRawApos()
	stableCount := 0
	settleStart := time.Now()
	for time.Since(settleStart) < settleTimeout {
		time.Sleep(settlePoll)
		currentPos := getRawApos()
		if currentPos == lastPos {
			stableCount++
			if stableCount >= settleWindow {
				break
			}
		} else {
			stableCount = 0
			lastPos = currentPos
		}
	}
	if stableCount < settleWindow {
		logger.Info("apos settle timeout — proceeding with fin signal anyway", device.Name)
	} else {
		logger.Trace("apos settled, asserting fin signal", device.Name)
	}

	notifyDriverStatusWithWait("fin_signal", "true", device)
	logger.Trace("start sending ECS fin signal (PDO)", device.Name)

	pdoFinishSub1.Store(65536)
	pdoFinishSub2.Store(65536)

	time.Sleep(time.Duration(envSettings.ECSFinTiming) * time.Millisecond)

	pdoFinishSub1.Store(0)
	pdoFinishSub2.Store(0)

	notifyDriverStatusWithWait("fin_signal", "false", device)
	logger.Trace("finish sending ECS fin signal (PDO)", device.Name)

	return nil
}