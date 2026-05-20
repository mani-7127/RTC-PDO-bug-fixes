package motordriver

import (
	channels "EtherCAT/channels"
	"EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"sync/atomic"
	"time"
)

// systemResetInProgress prevents the infinite reset storm.
// When ioStatusListener fires performSysReset() rapidly, the buffered channel
// serialises them, but resetSystemWorker must finish before accepting the next.
// This flag lets us drop extras fast without blocking.
var systemResetInProgress atomic.Bool

func listenSystemReset() {
	channels.ResetDriverSystem = make(chan bool, 1) // buffered=1, extras dropped
	go resetSystemWorker()
}

func performSysReset(checkMotorRunning bool) {
	if checkMotorRunning {
		for _, dev := range masterDevices {
			stat := getCurrentDriverStatus(dev.Name)
			if stat.isMotorRunning {
				logger.Info("Unable to do system reset, motor", dev.Name, "is running")
				statusnotifier.Alarm("Unable to do system reset, motor " + dev.Name + " is running")
				statusnotifier.SocketMessage("reset_done", "reset completed")
				return
			}
		}
	}

	// If a reset is already running, drop this request.
	// The running reset will fix the problem — no need to queue another.
	if systemResetInProgress.Load() {
		logger.Info("System reset already in progress, dropping duplicate request")
		return
	}

	// Non-blocking send — if channel already has a pending reset, drop this one.
	select {
	case channels.ResetDriverSystem <- true:
	default:
		logger.Info("System reset already queued, dropping duplicate request")
	}
}

func resetSystemWorker() {
	for {
		msg := <-channels.ResetDriverSystem
		if !msg {
			continue
		}

		// Guard: mark reset in progress so performSysReset drops concurrent calls.
		if !systemResetInProgress.CompareAndSwap(false, true) {
			logger.Info("Reset already in progress (guard), skipping")
			continue
		}

		logger.Info("===== SYSTEM RESET STARTED =====")

		// Stop background workers — PDO cyclic stays running the whole time
		// so the drive never loses its keepalive frames during reset.
		stopDriverPolling()
		stopDriverActionListener()
		stopDriveStatusListener()
		stopErrorPolling()
		stopPollIOStat()
		stopECSCheck()

		doneDriverAction()
		channels.WriteCommandExecInput("reset", "")
		channels.NotifyCmdComplete()

		// PDO fault reset — mutex inside ResetDriver prevents concurrent races.
		ResetDriver(masterDevices)

		// Drain any reset requests that piled up while we were resetting.
		for {
			select {
			case <-channels.ResetDriverSystem:
				logger.Info("Draining queued reset request")
			default:
				goto drained
			}
		}
	drained:

		time.Sleep(300 * time.Millisecond)

		// Restart background workers.
		pollDrivePosition(masterDevices)
		pollDriveError(masterDevices)
		pollIOStat(masterDevices)
		initDriverActionListener()
		startDriverStatusListener()

		systemResetInProgress.Store(false)

		logger.Info("===== SYSTEM RESET COMPLETED =====")
		statusnotifier.Alarm("No Alarms")
		statusnotifier.SocketMessage("reset_done", "reset completed")
	}
}

// StopSystem stops all channel listeners and the motor driver cleanly.
// Includes stopPdoCyclic() to gracefully release the EtherCAT master.
func StopSystem() {
	if !HasDriverConnected() {
		return
	}
	logger.Info("Stopping system...")

	stopDriverPolling()
	stopDriverActionListener()
	stopDriveStatusListener()
	stopErrorPolling()
	stopPollIOStat()
	stopECSCheck()

	// Stop PDO cyclic before releasing master — prevents watchdog fault on next boot.
	stopPdoCyclic()
	time.Sleep(20 * time.Millisecond)

	channels.WriteCommandExecInput("reset", "")
	channels.NotifyCmdComplete()
	ShutdownMasters()

	logger.Info("System stopped.")
}
