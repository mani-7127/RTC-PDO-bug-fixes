package motordriver

import "EtherCAT/logger"

//breakOn switch on the solenoid, this is an output signal
func breakOn(masterDevice MasterDevice) error {
	logger.Debug("break on")
	operation, err := GetEtherCATOperation("break_on", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		SDODownload(masterDevice.Master, masterDevice.Position, step)
	}

	return nil
}

//breakOff switch off the solenoid
func breakOff(masterDevice MasterDevice) error {
	logger.Debug("break off")
	operation, err := GetEtherCATOperation("break_off", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	for _, step := range operation.Steps {
		SDODownload(masterDevice.Master, masterDevice.Position, step)
	}

	return nil
}
