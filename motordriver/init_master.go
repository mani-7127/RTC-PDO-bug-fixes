package motordriver

import (
	parser "EtherCAT/configparser"
	ethercatDevice "EtherCAT/ethercatdevicedatatypes"
	"EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"time"
	"fmt"
)

var driverConnectionStatus = "SUCCESS"
var masterDevices []MasterDevice
var ethercatAddressMapping map[string]ethercatDevice.Ethercat

func InitMaster() error {

	ethercatAddressMapping = make(map[string]ethercatDevice.Ethercat)
	position := 0

	devices, err := parser.ParseDeviceConfig()
	if err != nil {
		return err
	}

	for _, dev := range devices.Device {
		address, parseErr := parser.ParseEthercatAddressConfig(dev.AddressConfigFile)
		if parseErr != nil {
			return parseErr
		}
		ethercatAddressMapping[dev.AddressConfigName] = address
	}

	for _, dev := range devices.Device {

		master, err := RequestMaster(dev)
		if err != nil {
			logger.Error(err)
			continue
		}
		if master == nil {
			continue
		}

		// IMPORTANT: activate PDO and start cyclic loop
		if err := ActivatePdoForMaster(master); err != nil {
			logger.Error("PDO activation failed:", err)
			driverConnectionStatus = "ERROR"
			statusnotifier.DriverStatus(dev.Name, "0")
			return err
		}

		masterDevice := MasterDevice{
			Master:   master,
			Position: position,
			Name:     dev.Name,
			Device:   dev,
		}

		masterDevices = append(masterDevices, masterDevice)

		// Wait for domain to be valid (WC complete) AND no fault before SDO config.
// pdoDomainValid is set only after first valid PDO frame — guarantees PREOP+ 
// and that stw is real, not zero/stale.
for i := 0; i < 50; i++ {
	if pdoDomainValid.Load() {
			break
	}
	logger.Info("Waiting for PDO domain valid before configuring driver:", dev.Name)
	time.Sleep(100 * time.Millisecond)
}
for i := 0; i < 30; i++ {
	if (pdoFbStatus.Load() & 0x0008) == 0 {
			break
	}
	logger.Info("Waiting for drive fault to clear before configuring driver:", dev.Name)
	time.Sleep(100 * time.Millisecond)
}

		configErr := configureDriver(masterDevice)
		if configErr != nil {
			driverConnectionStatus = "ERROR"
			statusnotifier.DriverStatus(dev.Name, "0")
			return configErr
		}

		ds := settings.GetDriverSettings(masterDevice.Name)
		if ds.MotorDirection == 1 {
			nonReverseDir(masterDevice)
		} else {
			reverseDir(masterDevice)
		}

		statusnotifier.DriverStatus(dev.Name, "1")
		time.Sleep(500 * time.Millisecond)

		PowerOn(masterDevice)
		time.Sleep(500 * time.Millisecond)

		// Auto-reset any fault retained from previous session.
		// Panasonic A6 keeps the last fault in non-volatile memory — if the
		// app was killed while faulted (FF50, etc.) the drive boots faulted.
		// Clearing it here means the UI starts clean with no alarm shown.
		bootFaultPresent := (pdoFbStatus.Load() & 0x0008) != 0
		if bootFaultPresent {
			logger.Info("Boot fault detected on", dev.Name, "— auto-resetting")
			ResetDriver([]MasterDevice{masterDevice})
			time.Sleep(300 * time.Millisecond)
		}

		// InitAposCorrection computes a boot-time offset that absorbs any
		// encoder sign flip transparently inside currentPosition().
		// HomingOffset (user-configured) is never modified by this.
		InitAposCorrection(dev.Name)

		position++
	}

	initListeners(masterDevices)
	setupDrivers(masterDevices)

	time.Sleep(500 * time.Millisecond)
	if len(masterDevices) > 0 {
		driverConnectionStatus = "SUCCESS"
	}
	return nil
}

func setupDrivers(masterDevices []MasterDevice) {
	for _, dev := range masterDevices {
		ds := settings.GetDriverSettings(dev.Name)
		if ds.ClampDeclamp == 1 {
			FastPowerOff(dev)
		} else {
			FastPowerOn(dev)
		}
	}
}

func initListeners(masterDevices []MasterDevice) {
	if len(masterDevices) <= 0 {
		return
	}
	initDriverActionListener()
	initDriverStatusKeeperListener()
	listenSystemReset()
	pollDrivePosition(masterDevices)
	pollDriveError(masterDevices)
	pollIOStat(masterDevices)
}

func ShutdownMasters() {
	logger.Info("ShutdownMasters: sending cwShutdown to drive")
	for _, device := range masterDevices {
			FastPowerOff(device)
	}
	// Wait for drive to leave Operation Enabled state
	for i := 0; i < 500; i++ {
			stw := uint16(pdoFbStatus.Load())
			state := stw & 0x006F
			if (stw&0x0008) == 0 && state != 0x0027 && state != 0x0023 {
					logger.Info("ShutdownMasters: drive safe stw=0x",
							fmt.Sprintf("%04X", stw), "after", i*2, "ms")
					break
			}
			if i == 499 {
					logger.Warn("ShutdownMasters: timeout stw=0x", fmt.Sprintf("%04X", stw))
			}
			time.Sleep(2 * time.Millisecond)
	}
	stopPdoCyclic()
        time.Sleep(50 * time.Millisecond)
        for _, device := range masterDevices {
                ReleaseMaster(device.Master)
        }
}

func PowerOnMasters() {
	for _, device := range masterDevices {
		PowerOn(device)
	}
}

func GetEtherCATOperation(operation string, deviceAddressConfigName string) (ethercatDevice.Operation, error) {
	addressMapping := getEtherCATAddress(deviceAddressConfigName)
	return addressMapping.GetOperation(operation)
}

func HasDriverConnected() bool {
	return driverConnectionStatus == "SUCCESS"
}

func getMasterDevices() []MasterDevice {
	return masterDevices
}

func getEtherCATAddress(deviceAddressConfigName string) ethercatDevice.Ethercat {
	return ethercatAddressMapping[deviceAddressConfigName]
}