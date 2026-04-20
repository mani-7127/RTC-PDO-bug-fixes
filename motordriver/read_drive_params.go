package motordriver

func readInputSignal(masterDevice MasterDevice) (int, error) {
    return int(GetDigitalInputs4F25()), nil
}
func readECSSignal(masterDevice MasterDevice) (int, error) {
    // ECS is part of 4F25 PDO
    return int(GetDigitalInputs4F25()), nil
}


func hardResetInput(masterDevice MasterDevice) (int, error) {
    // Hard reset input also from 4F25 PDO
    return int(GetDigitalInputs4F25()), nil
}
