package motordriver

import (
	"EtherCAT/ethercatdevicedatatypes"
	logger "EtherCAT/logger"
)

// PowerOn drives the CiA402 state machine to Operation Enabled via PDO.
// Replaces SDO sequence: write 6040=0x0006, write 6040=0x0007.
// With PDO running, the cyclic state machine handles 6040 automatically
// once pdoEnableRequested=true. We just seed target to current position
// so the drive does not jump on enable.
//
// Function signature and behaviour are unchanged from the original —
// callers such as InitMaster, setupDrivers, and PowerOnMasters still work.
func PowerOn(masterDevice MasterDevice) error {
	logger.Trace("poweron driver: ", masterDevice.Name)
	pdoCmdTarget.Store(pdoFbActual.Load())
	pdoCmdMode.Store(cia402ModeProfilePosition)
	pdoEnableRequested.Store(true)
	return nil
}

// FastPowerOn is the same as PowerOn — the PDO state machine handles 6040.
// Replaces SDO: write 6040 = 0x000F.
// All code that previously called FastPowerOn (doRotate, ManualJog, etc.)
// continues to work without changes.
func FastPowerOn(masterDevice MasterDevice) error {
	logger.Trace("fast poweron driver: ", masterDevice.Name)
	pdoCmdTarget.Store(pdoFbActual.Load())
	pdoCmdMode.Store(cia402ModeProfilePosition)
	pdoEnableRequested.Store(true)
	notifyDriverStatus("driver_on_off", "1", masterDevice)
	return nil
}

// PowerOffAll powers off all connected drives.
func PowerOffAll(masterDevices []MasterDevice) error {
	for _, d := range masterDevices {
		err := PowerOff(d)
		if err != nil {
			return err
		}
	}
	return nil
}

// powerOff cached operation — kept for interface compatibility,
// not used in PDO mode.
var powerOff = ethercatdevicedatatypes.Operation{}

// PowerOff commands the drive to Shutdown state via PDO.
// Replaces SDO sequence: write 6040=0xFF then write 6040=0x06.
// With PDO: set pdoEnableRequested=false — the cyclic state machine
// sends cwShutdown (0x0006) automatically.
func PowerOff(masterDevice MasterDevice) error {
	logger.Trace("poweroff driver: ", masterDevice.Name)
	pdoEnableRequested.Store(false)
	notifyDriverStatus("driver_on_off", "0", masterDevice)
	notifyDriverStatus("motor_running", "false", masterDevice)
	return nil
}

// FastPowerOff immediately commands Shutdown via PDO.
// Replaces SDO fastPowerOff: read 6040, write 6040=0x0006.
// The PDO cyclic sends cwShutdown as soon as pdoEnableRequested=false.
func FastPowerOff(masterDevice MasterDevice) error {
	logger.Trace("fast poweroff driver: ", masterDevice.Name)
	pdoEnableRequested.Store(false)
	notifyDriverStatus("driver_on_off", "0", masterDevice)
	notifyDriverStatus("motor_running", "false", masterDevice)
	return nil
}

// emergency triggers a quick-stop via PDO by zeroing velocity and dropping enable.
// The old SDO emergency was commented out in the YAML so this is the correct path.
func emergency(masterDevice MasterDevice) error {
	logger.Trace("emergency activated ", masterDevice.Name)
	pdoCmdVelocity.Store(0)
	pdoEnableRequested.Store(false)
	return nil
}
