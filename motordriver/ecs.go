package motordriver

import (
	channels "EtherCAT/channels"
	logger   "EtherCAT/logger"
	"EtherCAT/settings"
	"sync/atomic"
	"time"
)

var stopECSCheckChan chan bool

// isECSCheckInProgress is atomic to prevent a data race between
// stopECSCheck (writer, reset goroutine) and waitForECS/waitForECSZero
// (writer+reader, motion goroutine).
var isECSCheckInProgress atomic.Bool

func init() {
	isECSCheckInProgress.Store(false)
	// Buffered so stopECSCheck never blocks if receiver already exited.
	stopECSCheckChan = make(chan bool, 1)
}

func stopECSCheck() {
	if isECSCheckInProgress.Load() {
		select {
		case stopECSCheckChan <- true:
		default:
		}
	}
	isECSCheckInProgress.Store(false)
}

// -------------------------------------------------------------------
// ECS "GO-HIGH" Phase — wait until ECS goes high (ready to move)
// Logic unchanged from original uploaded version.
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
// ECS "GO-LOW" Phase — wait until ECS goes low again.
// debugECSOperation call from original uploaded version preserved.
// -------------------------------------------------------------------
func doECSCheckZero(masterDevice MasterDevice, degree float64) int {
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	if envSettings.ECS == 1 {
		logger.Debug("driver", masterDevice.Name, "waiting for ECS LOW (zero). rotate to", degree)
		debugECSOperation(masterDevice)
		ecsRec, _ := waitForECSZero(masterDevice)
		if ecsRec == 1 {
			logger.Debug("driver", masterDevice.Name, "ECS went LOW (zero phase done)")
		} else if ecsRec == 0 {
			logger.Error("driver", masterDevice.Name, "ECS zero NOT received")
			channels.SendAlarm("ECS did not go LOW")
		} else {
			logger.Info("exiting ECS zero check as program stop/reset event received")
		}
		return ecsRec
	}
	return 1
}

// -------------------------------------------------------------------
// Wait for ECS HIGH — delegates to driver.receivedECS (PDO-based).
// operation is passed for interface compatibility only —
// receivedECS reads from GetDigitalInputs4F25() PDO atomic, not SDO.
// -------------------------------------------------------------------
func waitForECS(masterDevice MasterDevice) (int, error) {
	operation, err := GetEtherCATOperation("ecs", masterDevice.Device.AddressConfigName)
	if err != nil {
		return 0, err
	}
	driver := GetMotorDriver()
	isECSCheckInProgress.Store(true)
	ecsStat := driver.receivedECS(masterDevice, operation, stopECSCheckChan)
	isECSCheckInProgress.Store(false)
	return ecsStat, nil
}

// -------------------------------------------------------------------
// Read ECS bit from PDO atomic — no SDO, no YAML operation needed.
// Returns 1 = high, 0 = low.
//
// Replaces the SDO-based readECSInput from the original uploaded version.
// The original called SDOUpload2 which fails under fault conditions and
// adds latency in tight ECS polling loops.
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
// Debounce logic (5 stable reads × 20ms) is identical to the original
// uploaded version.
// -------------------------------------------------------------------
func waitForECSZero(masterDevice MasterDevice) (int, error) {
	isECSCheckInProgress.Store(true)
	defer func() { isECSCheckInProgress.Store(false) }()

	logger.Info("Waiting indefinitely for ECS to go LOW...")

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
				logger.Info("ECS signal is LOW – continuing execution")
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
// sendECSFinSignal sends the finish signal via PDO (60FE:01 and 60FE:02).
//
// Key changes from original uploaded version:
//   - Uses pdoFinishSub1/pdoFinishSub2 atomics instead of SDO finsignal/finsignalend.
//   - Waits for apos to fully settle before asserting 60FE (2s timeout,
//     5 stable reads × 20ms each).
//   - Uses getRawApos() for settle check — sign-corrected, consistent with
//     everything else in the position pipeline.
//
// fin_signal notify and ECSFinTiming sleep are unchanged from original.
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

	pdoFinishSub1.Store(65536) // bit16 = 1<<16, same value as original SDO
	pdoFinishSub2.Store(65536)

	time.Sleep(time.Duration(envSettings.ECSFinTiming) * time.Millisecond)

	pdoFinishSub1.Store(0)
	pdoFinishSub2.Store(0)

	notifyDriverStatusWithWait("fin_signal", "false", device)
	logger.Trace("finish sending ECS fin signal (PDO)", device.Name)

	return nil
}

// -------------------------------------------------------------------
// debugECSOperation prints the ECS EtherCAT operation mapping details.
// Preserved exactly from the original uploaded version.
// -------------------------------------------------------------------
func debugECSOperation(masterDevice MasterDevice) {
	op, err := GetEtherCATOperation("ecs", masterDevice.Device.AddressConfigName)
	if err != nil {
		logger.Error("debugECSOperation: failed to get ECS operation:", err)
		return
	}

	logger.Info("----- ECS Operation Mapping -----")
	logger.Info("Device:", masterDevice.Device.Name)
	logger.Info("Operation Name:", op.Name)
	logger.Info("Number of Steps:", len(op.Steps))
	for i, step := range op.Steps {
		logger.Info(
			"Step", i,
			"| Action:", step.Action,
			"| DataType:", step.DataType,
			"| Value:", step.Value,
		)
		func() {
			defer func() { _ = recover() }()
			logger.Info("   (extra fields) Step struct:", step)
		}()
	}
	logger.Info("--------------------------------")
}