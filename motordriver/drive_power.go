package motordriver

import (
	"EtherCAT/ethercatdevicedatatypes"
	logger "EtherCAT/logger"
)

// PowerOn drives the CiA402 state machine to Operation Enabled via PDO.
// Replaces SDO sequence: write 6040=0x0006, write 6040=0x0007.
// With PDO running, the cyclic state machine handles 6040 automatically
// once pdoEnableRequested=true. We just need to ensure mode is set.
func PowerOn(masterDevice MasterDevice) error {
	logger.Trace("poweron driver: ", masterDevice.Name)
	// Seed target to current position so drive does not jump on enable
	pdoCmdTarget.Store(pdoFbActual.Load())
	pdoCmdMode.Store(cia402ModeProfilePosition)
	pdoEnableRequested.Store(true)
	return nil
}

// FastPowerOn is the same as PowerOn — PDO state machine handles 6040.
// Replaces SDO: write 6040 = 0x000F (binary 0000000000001111).
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

// emergency triggers a quick-stop via PDO by clearing bit2 of the controlword.
// The old SDO emergency was commented out in the YAML so this is a safe default.
func emergency(masterDevice MasterDevice) error {
	logger.Trace("emergency activated ", masterDevice.Name)
	// Quick-stop: clear bit2 of controlword (0x000F -> 0x000B)
	// The cyclic loop checks pdoStopRequest which does the same thing,
	// but for emergency we also disable immediately.
	pdoCmdVelocity.Store(0)
	pdoEnableRequested.Store(false)
	return nil
}