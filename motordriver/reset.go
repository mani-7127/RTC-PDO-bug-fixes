package motordriver

import (
	"fmt"
	logger "EtherCAT/logger"
	"sync"
	"time"
)

// resetMu ensures only one ResetDriver executes at a time.
// Without this, rapid button presses spawn concurrent goroutines that
// each call pdoResetState.Store(1), racing with the state machine as it
// progresses 1→2→3→4 and resetting the counter mid-sequence → timeout.
var resetMu sync.Mutex

// ResetDriver performs a CiA402 fault reset entirely via PDO.
// Safe to call concurrently — only one reset runs at a time, others are
// dropped (not queued) to prevent the reset storm seen in production.
//
// Sequence matches YAML faultReset:
//   state 1-2: send 0x0080 pulse (bit7) for 150ms
//   state 3:   release to cwShutdown 0x0006
//   state 4:   wait for fault bit (stw bit3) to clear
func ResetDriver(avilableDevices []MasterDevice) error {

	// Drop concurrent calls immediately — don't queue them.
	// If reset is already running, the drive is already being fixed.
	if !resetMu.TryLock() {
		logger.Info("ResetDriver: reset already in progress, skipping duplicate call")
		return nil
	}
	defer resetMu.Unlock()

	for _, device := range avilableDevices {
		logger.Info("PDO fault reset: ", device.Name)

		// Zero finish signal outputs before reset
		pdoFinishSub1.Store(0)
		pdoFinishSub2.Store(0)

		// Drop enable so drive can accept fault reset controlword
		pdoEnableRequested.Store(false)
		time.Sleep(50 * time.Millisecond)

		// Check there is actually a fault before starting state machine.
		// Avoids unnecessary resets when called speculatively.
		stw := uint16(pdoFbStatus.Load())
		if (stw & 0x0008) == 0 {
			logger.Info("PDO fault reset: no fault present (stw=0x",
				fmt.Sprintf("%04X", stw), "), re-enabling")
			pdoEnableRequested.Store(true)
			continue
		}

		// Trigger the reset state machine in the cyclic goroutine.
		// Only store(1) if currently idle (0) to avoid overwriting
		// a state machine already in progress from a previous call.
		if !pdoResetState.CompareAndSwap(0, 1) {
			logger.Info("PDO fault reset: state machine already running, waiting...")
		} else {
			pdoResetCycles.Store(0)
		}

		// Wait for state machine to reach 0 (done), max 2 seconds
		for i := 0; i < 200; i++ {
			if pdoResetState.Load() == 0 {
				break
			}
			time.Sleep(10 * time.Millisecond)
		}

		if pdoResetState.Load() != 0 {
			logger.Error("PDO fault reset timeout for ", device.Name)
			pdoResetState.Store(0) // force clear so system is not stuck
			pdoEnableRequested.Store(true)
			continue
		}

		// Restore PP mode and re-enable
		pdoCmdMode.Store(cia402ModeProfilePosition)
		pdoEnableRequested.Store(true)

		logger.Info("PDO fault reset completed: ", device.Name)
	}
	return nil
}

// resetMultiTurn resets the absolute encoder multi-turn data via SDO.
//
// The PDO cyclic loop owns 0x6040 (controlword) every 1ms cycle.
// We must drop pdoEnableRequested and then WAIT for the drive STW to
// confirm switch-on disabled (bit6=1) before issuing any SDO writes.
// A fixed sleep is not reliable — when the drive is in operation enabled
// state it takes several CiA402 state machine steps to reach switch-on
// disabled, and the time varies. Polling STW guarantees the drive is
// actually ready before the 0x6040 and 0x4D01/0x4D00 SDO writes arrive.
//
// After the operation the cyclic loop re-enables the drive automatically
// via the normal CiA402 state machine — no restart required.
func resetMultiTurn(avilableDevices []MasterDevice) error {
	for _, device := range avilableDevices {
		operation, _ := GetEtherCATOperation("resetMultiTurn", device.Device.AddressConfigName)
		logger.Info("reset multi turn: ", device.Name)

		// Step 1: drop PDO enable — cyclic loop will write cwShutdown to 0x6040
		pdoEnableRequested.Store(false)

		// Step 2: poll STW until drive reaches switch-on disabled (bit6=1).
		// CiA402 switch-on disabled state: stw & 0x004F == 0x0040
		// Timeout after 1 second — if drive doesn't reach disabled state,
		// log and proceed anyway (SDO may still work if drive is faulted).
		logger.Info("reset multi turn: waiting for switch-on disabled (bit6)...")
		switchOnDisabled := false
		for i := 0; i < 100; i++ {
			stw := uint16(pdoFbStatus.Load())
			if (stw & 0x004F) == 0x0040 || (stw & 0x006F) == 0x0060 {
				switchOnDisabled = true
				logger.Info("reset multi turn: drive in switch-on disabled (stw=0x",
					fmt.Sprintf("%04X", stw), ") after ", i*10, "ms")
				break
			}
			time.Sleep(10 * time.Millisecond)
		}
		if !switchOnDisabled {
			stw := uint16(pdoFbStatus.Load())
			logger.Info("reset multi turn: timeout waiting for switch-on disabled (stw=0x",
				fmt.Sprintf("%04X", stw), "), proceeding anyway")
		}

		// Extra 50ms settling time after STW confirmed
		time.Sleep(50 * time.Millisecond)

		// Step 3: execute SDO sequence
		// YAML order: 0x6040=0x07, 0x4D01=0x31 (200ms delay in YAML),
		//             0x4D00:1=0x200, 0x4D00:1=0x0
		for _, step := range operation.Steps {
			if step.Action == "read" {
				val, _ := SDOUpload2(device.Master, device.Position, step)
				logger.Debug("val", val)
			} else {
				SDODownload(device.Master, device.Position, step)
			}
		}

		// Step 4: wait for drive to process the multiturn reset internally
		time.Sleep(200 * time.Millisecond)

		// Step 5: re-enable — cyclic loop resumes normal CiA402 state machine
		// and brings the drive back to operation enabled automatically.
		pdoEnableRequested.Store(true)
		logger.Info("reset multi turn completed: ", device.Name)
	}
	return nil
}