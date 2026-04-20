package motordriver

//Listen for any action request on Motor to perform from the client. For e.g. reset, zero reference, etc

import (
    settings "EtherCAT/settings"
    channels "EtherCAT/channels"
    logger "EtherCAT/logger"
    "EtherCAT/motordriver/statusnotifier"
    //"EtherCAT/ren"
    //"time"
    //"fmt"
    "strconv"
)

// rotation_direction is a package-level variable used to store the last jog direction.
// This declaration resolves the "syntax error: unexpected keyword else" issue.
var rotation_direction int 

func initDriverActionListener() {
    logger.Debug("starting driver action listener")
    channels.DriverActionChannel = make(chan channels.DriverAction, 100)
    channels.DriveActionChannelReady()
    //listen for any action to takes on driver.
    go listenDriverAction()
}

func stopDriverActionListener() {
    channels.NotifyMotorDriver("EXIT_DRIVE_LISTENER", "", "", 0)
}

/**
function listen to channel DriverActionChannel. Any one can write actions to this channel and this function
will channel the action to the motor driver.
**/
func listenDriverAction() {
    masterDevices := getMasterDevices()
    device := masterDevices[0]
    for {
        msg := <-channels.DriverActionChannel

        switch msg.Action {
        case channels.RESET:
            performSysReset(true)
            // if !HasDriverConnected() {
            //  InitMaster()
            // }
            // ResetDriver(masterDevices)
        case channels.MANUAL_JOG:
            // Drain any queued jog events — keep only the latest direction.
            // Rapid clicks queue up in the 100-slot buffer causing the motor
            // to keep running long after the user stopped clicking.
            latestDir := msg.Direction
        drainJog:
            for {
                select {
                case queued := <-channels.DriverActionChannel:
                    if queued.Action == channels.MANUAL_JOG {
                        latestDir = queued.Direction
                    } else if queued.Action == channels.STOP_JOG {
                        // STOP found in queue — stop immediately, don't jog
                        StopJog(device)
                        break drainJog
                    }
                default:
                    // Queue empty — proceed with latest direction
                    rotation_direction = latestDir
                    notifyDriverStatusWithWait("rotation_direction", strconv.Itoa(latestDir), device)
                    ManualJog(device, latestDir)
                    break drainJog
                }
            }
        case channels.STOP_JOG:
            // Drain any remaining queued jog events before stopping
            drained:
            for {
                select {
                case <-channels.DriverActionChannel:
                    // discard
                default:
                    break drained
                }
            }
            StopJog(device)
            //time.Sleep(500 * time.Millisecond)
            //if rotation_direction == 1 { // 'if' statement is now valid
            //  pos := ReadActualPositionFromDrive("A") // 1. Read RTC position from motor driver
            //  ren.UpdateJogPositionToPC(fmt.Sprintf("%.3f", pos))
            //} else { // 'else' block is now correctly associated with the 'if'
            //  pos := ReadActualPositionFromDrive("A") // 1. Read RTC position from motor driver
            //  ren.UpdateJogPositionToPC1(fmt.Sprintf("%.3f", pos))
            //}
           // ResetDriver(masterDevices)
        case channels.ZERO_REF:
            go moveToZero(device)
        case channels.STEP_MODE_ENABLE:
            configureDriver(device)
        case channels.STEP_MODE:
            pos, _ := strconv.ParseFloat(msg.Value, 64)
            notifyDriverStatusWithWait("rotation_direction", strconv.Itoa(msg.Direction), device)
            stepMode(device, pos)
        case channels.SET_RPM:
            rpm, _ := strconv.ParseInt(msg.Value, 0, 32)
            setRpm(device, int(rpm))
        case channels.MOVE_TO_POSITION:
            degree, _ := strconv.ParseFloat(msg.Value, 64)
            //run moveMotorToDegree as go routine, so that this listener can continue listening other events
            //such as emergency etc.
            go moveMotorToDegree(device, degree)
        case channels.START_EXECUTION:
            logger.Trace("program exec started")
            notifyDriverStatus("reset", "", device)
        case channels.POSITION_MODE:
            notifyDriverStatus("mode", msg.Value, device)
        case channels.SHORTEST_PATH_ENABLED:
            notifyDriverStatus("shortest_path_enable", msg.Value, device)
            channels.NotifyCmdComplete() // FIX: G68 blocks on WaitTillCmdComplete() after sending
                                         // this action. Without this call it blocked ~27s per cycle.
        case channels.EMERGENCY:
            stopECSCheck()
            emergency(device)
            PowerOffAll(masterDevices)
            statusnotifier.SocketMessage("emergency_done", "emergency completed")
            statusnotifier.Alarm("Software Emergency pressed")
        case channels.PROGRAM_EXEC_COMPLETED:
            logger.Trace("program exec completed")
            notifyDriverStatus("reset", "", device)
        case channels.FAST_POWER_OFF:
            FastPowerOff(device)
        case channels.RESET_MULTI_TURN:
            resetMultiTurn(masterDevices)
        case channels.SET_WORK_OFFSET:
            notifyDriverStatusWithWait("workoffset", msg.Value, device)
        case channels.SETTINGS_CHANGED:
            // Reload settings from disk so HomingOffset, WorkOffset, and all
            // other values are fresh. Without this, UI changes to settings.json
            // are never picked up in-memory until the next full app restart.
            if err := settings.LoadDriverSettings(); err != nil {
                logger.Error("SETTINGS_CHANGED: failed to reload settings:", err)
            }
            applyClampIfSettingsChanged()
        case channels.STOP_PROGRAM_EXECUTION:
            stopECSCheck()
        case channels.EXIT_DRIVE_LISTENER:
            logger.Debug("stoping driver action listener")
            break
        default:
            logger.Error("listenDriverAction->unrecognized driver action type passed", msg.Action)
        }
    }
}

//doneDriverAction will feedback the command handlers about the completion of a command
//for eg. move to 90 degree, once move completed will feed back the handler that its done.
//So than command handlers can execute the next line of command
func doneDriverAction() {
    channels.NotifyCmdComplete()
}