package motordriver

import (
	logger "EtherCAT/logger"
	"strconv"
)

//setRpm sets the rpm of the specified driver
func setRpm(device MasterDevice, rpm int) error {
	operation, err := GetEtherCATOperation("setRPM", device.Device.AddressConfigName)
	if err != nil {
		return err
	}

	logger.Trace("set RPM of driver: ", device.Name)
	for _, step := range operation.Steps {
		if step.Action == "read" {
			val, _ := SDOUpload2(device.Master, device.Position, step)
			logger.Debug("val", val)
		} else {
			if step.Value == "rpm_val" {
				rpm = int(device.Device.RPMConst * rpm)
				step.Value = strconv.Itoa(rpm)
			}
			SDODownload(device.Master, device.Position, step)
		}
	}
	doneDriverAction()
	return nil
}