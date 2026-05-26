package executors

import (
	channels "EtherCAT/channels"
	h "EtherCAT/commands"
	parsers "EtherCAT/configparser"
	dt "EtherCAT/datatypes"
	"EtherCAT/licensechecker"
	logger "EtherCAT/logger"
	motor "EtherCAT/motordriver"
	"EtherCAT/settings"
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"sync/atomic"
	"time"
)

var funcMap map[string]h.Handler
var yamlConfig dt.YamlConfig
var hasYmlLoaded bool
var execContext dt.ExecutionContext
var RS232Enabled atomic.Bool

const lastLineFile = "/mnt/app/jamun/settings/last_line.txt"

// lastProgramFile persists the name of the program that was last running
// when execution stopped. Used to detect program switches — if the operator
// selects a different program, the saved line number must be ignored and
// execution starts from line 0. This matches industrial CNC behaviour:
// resume only applies to the same program.
const lastProgramFile = "/mnt/app/jamun/settings/last_program.txt"

func SetRS232Enabled(enable bool) {
	RS232Enabled.Store(enable)
}

func IsRS232Enabled() bool {
	return RS232Enabled.Load()
}

func Initialize() error {
	var err error

	funcMap, err = loadCommandPlugins()
	if err != nil {
		return err
	}

	_, err = loadYmlConfig()
	if err != nil {
		channels.SendAlarm(err.Error())
		return err
	}

	execContext.CommandMaps = yamlConfig.Execution
	execContext.ExecutionMode = "continuous"
	listenCommandExecInput(&execContext)

	channels.SendAlarm("No Alarms")

	execContext.NextLineWhenStopped = 0
	execContext.NextCmdLineToExec = 0

	execContext.Reset()
	return nil
}

func loadYmlConfig() (dt.YamlConfig, error) {
	if hasYmlLoaded == false {
		var err error
		yamlConfig, err = parsers.ParseExececutionConfigYML()
		if err != nil {
			return yamlConfig, err
		}
		hasYmlLoaded = true
	}
	return yamlConfig, nil
}

func validateLicenseBeforeRun() error {
	licErr := licensechecker.CheckLicense(false)
	if licErr != nil {
		channels.SendAlarm(licErr.Error())
		return licErr
	}
	return nil
}

func setDriveSettings() {
	allSettings := settings.GetAllSettings()
	execdriveSettings := make(map[string]dt.DriveSetting)

	for k, v := range allSettings {
		setting := dt.DriveSetting{
			POTLimit:             v.POT,
			NOTLimit:             v.NOT,
			ConfiguredWorkOffset: v.GetWorkOffset(),
		}
		execdriveSettings[k] = setting
	}

	execContext.DriveSettings = execdriveSettings
}

func CompileProgram(fileName string) error {
	commands, err := createCommands(fileName)
	if err != nil {
		return err
	}

	errVerify := canExecuteGivenCommands(commands)
	if errVerify != nil {
		return errVerify
	}

	return nil
}

func clearLastLine() {
	err := os.WriteFile(lastLineFile, []byte("0"), 0644)
	if err != nil {
		logger.Error("Failed to clear last_line.txt:", err)
	}
	// Also clear the program name so a fresh boot starts from line 0
	// for whichever program is selected first.
	_ = os.WriteFile(lastProgramFile, []byte(""), 0644)
}

func saveLastLine(lineNumber int) {
	lineStr := strconv.Itoa(lineNumber)
	err := os.WriteFile(lastLineFile, []byte(lineStr), 0644)
	if err != nil {
		logger.Error("Failed to save execution state to last_line.txt:", err)
	}
}

func saveLastProgram(fileName string) {
	name := filepath.Base(fileName)
	_ = os.WriteFile(lastProgramFile, []byte(name), 0644)
}

func readLastProgram() string {
	content, err := os.ReadFile(lastProgramFile)
	if err != nil {
		return ""
	}
	return string(content)
}

func readLastLine() (int, error) {
	content, err := os.ReadFile(lastLineFile)
	if err != nil {
		if os.IsNotExist(err) {
			return 0, nil
		}
		return 0, err
	}
	lineNumber, err := strconv.Atoi(string(content))
	if err != nil {
		return 0, err
	}
	return lineNumber, nil
}

// resolveStartLine decides where execution should begin.
// Priority:
// 1. explicit user-selected line (one-shot, UI is 1-based)
// 2. stopped/resume line — BUT ONLY if the same program is being executed
// 3. beginning of program (line 0)
//
// INDUSTRIAL CNC RULE: resume line is forgotten when a different program
// is selected. If you stopped at line 8 of program LONG and now execute
// program SHORT, SHORT starts at line 0. This prevents the line-8 saved
// state from SHORT's program which only has 5 lines — causing an
// out-of-bounds resume or skipping setup commands entirely.
func resolveStartLine(currentFile string) int {
	currentName := filepath.Base(currentFile)

	// Priority 1: explicit user line selection from UI (one-shot).
	userLine, err := settings.LoadLineNumberAsInt()
	if err == nil && userLine > 0 {
		startIdx := userLine - 1
		if startIdx < 0 {
			startIdx = 0
		}
		logger.Info("Starting from user-selected line:", userLine, "-> internal index:", startIdx)
		settings.ClearLineNumber()
		return startIdx
	}

	// Priority 2: resume from last stop — only if same program.
	lastProgram := readLastProgram()
	if lastProgram != currentName {
		// Different program selected — forget the saved line.
		// Clear it now so a subsequent stop saves correctly for this program.
		if lastProgram != "" {
			logger.Info("Program changed from", lastProgram, "to", currentName,
				"— starting from line 0 (resume cleared)")
		}
		clearLastLine()
		return 0
	}

	lastLine, err := readLastLine()
	if err == nil && lastLine >= 0 {
		if lastLine > 0 {
			logger.Info("Resuming from last stopped internal index:", lastLine,
				"program:", currentName)
		}
		return lastLine
	}

	return 0
}

// replaySetupCommands silently re-executes all non-motion setup commands
// from lines 0..startLine-1 before resuming at startLine.
//
// WHY THIS IS NEEDED:
// When a program is stopped mid-way (e.g. at line 6) and the operator runs
// zero-ref or jogs before pressing Execute again, the drive's runtime state
// (feedrate, ABS/INC mode, shortest path, workoffset) has been reset to
// defaults. Resuming at line 6 without replaying lines 0-5 means the motor
// runs at zero-ref speed instead of the programmed feedrate, and may use
// the wrong mode (INC instead of ABS) if the program switches modes.
//
// Setup commands are identified by ConsiderInBlockExecution == 0 — these
// are G01 F**, G90, G91, G68, G69, G17, workoffset commands, etc.
// Motion commands (A**, B**) have ConsiderInBlockExecution == 1 and are
// skipped — we do NOT want to re-execute any physical moves.
//
// This replicates what "reset + execute from beginning" does for the drive
// context, without actually moving the table.
func replaySetupCommands(commands []dt.Command, startLine int) {
	if startLine <= 0 {
		return
	}

	logger.Info("[RESUME] Replaying setup commands from lines 0 to", startLine-1,
		"to restore feedrate/mode before resuming at line", startLine)

	// Run in trial-mode equivalent: no ECS waits, no position moves,
	// just the state changes (RPM, mode, workoffset, shortest path).
	for i := 0; i < startLine && i < len(commands); i++ {
		cmd := commands[i]
		commandToExec := yamlConfig.Execution.GetCommand(cmd.Cmd)

		// Skip motion commands entirely — we must not physically move the table.
		if commandToExec.ConsiderInBlockExecution == 1 {
			logger.Debug("[RESUME] Skipping motion command at line", i, ":", cmd.Cmd)
			continue
		}

		// Skip M99/M30 loop/end markers — these affect execution flow.
		if cmd.Cmd == "M99" || cmd.Cmd == "M30" {
			logger.Debug("[RESUME] Skipping program flow command at line", i, ":", cmd.Cmd)
			continue
		}

		funcToExec := funcMap[commandToExec.Func]
		if funcToExec == nil {
			continue
		}
		if reflect.ValueOf(funcToExec).Kind() == reflect.Ptr &&
			reflect.ValueOf(funcToExec).IsNil() {
			continue
		}

		cmd.ConsiderInBlockExecution = commandToExec.ConsiderInBlockExecution
		logger.Debug("[RESUME] Replaying setup line", i, ":", cmd.Cmd)
		funcToExec.Handle(cmd, &execContext)

		// Allow async state changes (mode, shortest path) to propagate.
		// These are sent via channels to driver_status_keeper, so a small
		// sleep ensures they're applied before the first motion command runs.
		time.Sleep(5 * time.Millisecond)
	}

	logger.Info("[RESUME] Setup replay complete — drive context restored for line", startLine)
}

func RunCodeFile(fileName string) error {
	file := filepath.Base(fileName)
	logger.Info("executing program file", file)

	channels.SendAlarm("No Alarms")

	if licErr := validateLicenseBeforeRun(); licErr != nil {
		return licErr
	}

	execContext.PrepareExecutingFile(fileName)
	execContext.ExecutingFilePath = fileName

	commands, err := createCommands(fileName)
	if err != nil {
		return err
	}

	execContext.Commands = commands

	setDriveSettings()

	drvSettings := settings.GetDriverSettings("A")
	execContext.ECSEnabled = drvSettings.ECS

	execContext.StopExecution = false
	panicAfter = 0

	channels.NotifyMotorDriver(channels.START_EXECUTION, "", "", 0)

	// IMPORTANT: Do NOT call ResetExecutingProgram() here.
	// That clears last_line.txt and userline.json before resolveStartLine() can use them.
	execContext.Reset()
	execContext.ExecutingFilePath = fileName
	execContext.Commands = commands
	execContext.ECSEnabled = drvSettings.ECS
	execContext.StopExecution = false

	startLine := resolveStartLine(fileName)
	execContext.NextLineWhenStopped = startLine
	execContext.NextCmdLineToExec = startLine

	errVerify := canExecuteGivenCommands(commands)
	if errVerify != nil {
		channels.SendAlarm(errVerify.Error())
		return errVerify
	}

	// Trial mode reset clears runtime state, so restore the intended start line again.
	execContext.NextLineWhenStopped = startLine
	execContext.NextCmdLineToExec = startLine
	execContext.StopExecution = false

	// Save which program is running so resume logic knows whether to honour
	// the saved line number or reset to 0 on next Execute.
	saveLastProgram(fileName)

	// If resuming mid-program, silently replay setup commands (feedrate, mode,
	// shortest path) from lines before the resume point so the drive context
	// is identical to what it would have been if the program ran from line 0.
	//
	// This fixes the issue where after zero-ref or jog, the motor resumes at
	// zero-ref speed instead of the programmed feedrate because G01 F20 (line 0)
	// was never executed for this resume cycle.
	if startLine > 0 {
		replaySetupCommands(commands, startLine)
	}

	logger.Trace("executing commands from", execContext.NextLineWhenStopped)

	execErr := executeCommands(commands, execContext.NextLineWhenStopped)

	if !execContext.StopExecution {
		channels.NotifyMotorDriver(channels.PROGRAM_EXEC_COMPLETED, "", "A", 0)
		ResetExecutingProgram()
	}

	channels.NotifyUIProgramCompleted()

	return execErr
}

func ResetExecutingProgram() {
	execContext.StopExecution = true
	execContext.Reset()
	clearLastLine()
	settings.ClearLineNumber()
	logger.Info("PROGRAM RESET IS INITIATED.")
}

func canExecuteGivenCommands(commands []dt.Command) error {
	execContext.Reset()
	execContext.ActivateTrialMode()

	logger.Info("running in trial mode to verify the commands")

	execVerifyErr := executeCommands(commands, 0)
	if execVerifyErr != nil {
		return execVerifyErr
	}

	logger.Info("trial run completed, commands ok.")

	execContext.DeActivateTrialMode()
	execContext.Reset()

	return nil
}

func executeCommands(commands []dt.Command, nextCommandIndex int) error {
	if execContext.StopExecution {
		logger.Info("Execution stopped. Unwinding command calls...")
		return nil
	}

	if nextCommandIndex < 0 || nextCommandIndex >= len(commands) {
		logger.Trace("command execution completed", nextCommandIndex)
		return nil
	}

	if !execContext.TrialModeActive {
		saveLastLine(nextCommandIndex)
	}

	previousNextIdx := execContext.NextCmdLineToExec

	waitForNextBlock, err := executeCommand(commands[nextCommandIndex], &execContext, nextCommandIndex)
	if err != nil {
		return err
	}

	// True if the command completed its motion naturally before any stop signal arrived.
	// Used below to decide whether resume should replay the same line or advance past it.
	commandCompletedNaturally := waitForNextBlock && !execContext.StopExecution

	// RS232 file reload support
	if IsRS232Enabled() && execContext.ExecutingFilePath != "" {
		updatedCommands, err := createCommands(execContext.ExecutingFilePath)
		if err == nil {
			commands = updatedCommands
			execContext.Commands = updatedCommands
			logger.Debug("Program file reloaded, total commands:", len(commands))
		} else {
			logger.Warn("Could not reload updated program file:", err)
		}
	}

	if waitForNextBlock && !execContext.TrialModeActive {
		if IsRS232Enabled() {
			nextIdx := nextCommandIndex + 1
			execContext.NextLineWhenStopped = nextIdx
			saveLastLine(nextIdx)

			logger.Info("[RS232] Blocking execution: waiting for file update...")

			waitForProgramFileUpdate(execContext.ExecutingFilePath)

			updatedCommands, err := createCommands(execContext.ExecutingFilePath)
			if err == nil {
				commands = updatedCommands
				execContext.Commands = updatedCommands
				execContext.NextCmdLineToExec = nextCommandIndex + 1
				logger.Debug("Program file reloaded after RS232 update")
			}
		} else {
			logger.Debug("[EXECUTOR] RS232 disabled -> standard wait")
			motor.RefreshCurrentPosition()

			// Do not advance resume line here.
			// If stop happens now, we must resume the same not-yet-completed command.
			execContext.NextLineWhenStopped = nextCommandIndex

			execContext.WaitExecuteNextCommand()
		}
	}

	if execContext.StopExecution {
		if execContext.HasResetted {
			execContext.NextLineWhenStopped = 0
		} else if commandCompletedNaturally {
			// Motion finished before stop arrived — resume from the next line.
			execContext.NextLineWhenStopped = nextCommandIndex + 1
		} else {
			// Stop interrupted mid-command — replay the same line on resume.
			execContext.NextLineWhenStopped = nextCommandIndex
		}

		logger.Debug("Stopping execution, next resume line will be #", execContext.NextLineWhenStopped)

		if !execContext.TrialModeActive {
			saveLastLine(execContext.NextLineWhenStopped)
		}

		return nil
	}

	nextIdx := execContext.NextCmdLineToExec

	if nextIdx == previousNextIdx || nextIdx == nextCommandIndex {
		nextIdx = nextCommandIndex + 1
		execContext.NextCmdLineToExec = nextIdx
	}

	return executeCommands(commands, nextIdx)
}

func executeCommand(cmd dt.Command, execContext *dt.ExecutionContext, currentCmdIndex int) (bool, error) {
	commandToExec := yamlConfig.Execution.GetCommand(cmd.Cmd)

	cmd.Description = commandToExec.Description
	execContext.CurrentExecCommandLine = currentCmdIndex

	funcToExec := funcMap[commandToExec.Func]

	err := isAValidHandler(funcToExec, cmd.Cmd)
	if err != nil {
		return false, err
	}

	if funcToExec != nil {
		if !execContext.TrialModeActive {
			logger.Info("executing line:", cmd.CodeLineNumber, "command:", cmd.Cmd)

			for i := 0; i < 4; i++ {
				channels.SendLineNumber(cmd.CodeLineNumber)
				time.Sleep(time.Duration(10) * time.Millisecond)
			}
		}

		cmd.ConsiderInBlockExecution = commandToExec.ConsiderInBlockExecution

		results := funcToExec.Handle(cmd, execContext)

		if execContext.Err != nil {
			logger.Error("Error executing command " + cmd.Cmd)
			logger.Error(execContext.Err)
			return false, execContext.Err
		}

		execLineErr := executeInLineParamFunction(funcToExec, cmd, execContext, results)
		if execLineErr != nil {
			return false, execLineErr
		}
	}

	waitForNextBlock := false
	if commandToExec.ConsiderInBlockExecution == 1 {
		waitForNextBlock = true
	}

	return waitForNextBlock, nil
}

func isAValidHandler(funcToExec h.Handler, funcName string) error {
	if funcName == "invalidCommand" {
		return errors.New("Unable to find command processor for " + funcName)
	}

	if funcToExec == nil || (reflect.ValueOf(funcToExec).Kind() == reflect.Ptr && reflect.ValueOf(funcToExec).IsNil()) {
		return errors.New("Unable to find command processor for " + funcName)
	}

	return nil
}

var panicAfter int

func executeInLineParamFunction(funcToExec h.Handler, cmd dt.Command, execContext *dt.ExecutionContext, returnedResults []dt.ExecutionResult) error {
	if len(returnedResults) <= 0 {
		return nil
	}

	for _, result := range returnedResults {
		if result.ShouldExecute {
			paramToExec := funcMap[result.Cmd.Func]

			if paramToExec != nil {
				childResults := paramToExec.Handle(result.Cmd, execContext)

				if execContext.Err != nil {
					logger.Error("Error executing command " + cmd.Cmd)
					logger.Error(execContext.Err)
					return execContext.Err
				}

				if result.Cmd.ConsiderInBlockExecution == 1 {
					// inline motion completed
					_ = childResults
				}
			}
		}
	}
	return nil
}

func waitForProgramFileUpdate(filePath string) {
	info, err := os.Stat(filePath)
	if err != nil {
		logger.Warn("Could not stat file for updates:", err)
		return
	}

	lastMod := info.ModTime()

	for {
		time.Sleep(20 * time.Millisecond)

		info, err := os.Stat(filePath)
		if err != nil {
			logger.Warn("Could not stat file for updates:", err)
			return
		}

		if info.ModTime().After(lastMod) {
			logger.Info("Detected program file update:", filePath)
			break
		}
	}
}

func UpdateLastLineFromJSON() {
	userLine, err := settings.LoadLineNumberAsInt()
	if err != nil {
		logger.Error("UpdateLastLineFromJSON: failed to load userline.json:", err)
		return
	}

	if userLine <= 0 {
		logger.Info("UpdateLastLineFromJSON: no valid user-selected line found")
		return
	}

	startIdx := userLine - 1
	if startIdx < 0 {
		startIdx = 0
	}

	execContext.NextLineWhenStopped = startIdx

	// Keep compatibility — last_line.txt reflects the selected restart line.
	saveLastLine(startIdx)
}