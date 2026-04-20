package motordriver

import (
	logger "EtherCAT/logger"
	"strconv"
)

// setRpm sets the profile velocity (0x6081) of the driver via SDO.
// 0x6081 is a configuration register, not PDO-mapped — SDO is correct here.
// Called once per SET_RPM command, not in a hot loop.
func setRpm(device MasterDevice, rpm int) error {
	operation, err := GetEtherCATOperation("setRPM", device.Device.AddressConfigName)
	if err != nil {
		return err
	}

	logger.Trace("set RPM of driver: ", device.Name)
	for _, step := range operation.Steps {
		if step.Value == "rpm_val" {
			step.Value = strconv.Itoa(device.Device.RPMConst * rpm)
		}
		SDODownload(device.Master, device.Position, step)
	}
	doneDriverAction()
	return nil
}