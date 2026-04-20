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

func SetRS232Enabled(enable bool) {
	RS232Enabled.Store(enable)
}

func IsRS232Enabled() bool {
	return RS232Enabled.Load()
}

// Initialize executors to load plugins and parse execution yaml file.
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

	// Boot must always start from beginning.
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
}

func saveLastLine(lineNumber int) {
	lineStr := strconv.Itoa(lineNumber)

	err := os.WriteFile(lastLineFile, []byte(lineStr), 0644)
	if err != nil {
		logger.Error("Failed to save execution state to last_line.txt:", err)
	}
}

func readLastLine() (int, error) {
	content, err := os.ReadFile(lastLineFile)
	if err != nil {
		if os.IsNotExist(err) {
			return 0, nil
		}
		return 0, err
	}

	lineStr := string(content)

	lineNumber, err := strconv.Atoi(lineStr)
	if err != nil {
		return 0, err
	}

	return lineNumber, nil
}

// resolveStartLine decides where execution should begin.
// Priority:
// 1. explicit user-selected line (one-shot, UI is 1-based)
// 2. stopped/resume line (internal index)
// 3. beginning of program
func resolveStartLine() int {
	userLine, err := settings.LoadLineNumberAsInt()
	if err == nil && userLine > 0 {
		startIdx := userLine - 1
		if startIdx < 0 {
			startIdx = 0
		}
		logger.Info("Starting from user-selected line:", userLine, " -> internal index:", startIdx)
		settings.ClearLineNumber() // one-shot behavior
		return startIdx
	}

	lastLine, err := readLastLine()
	if err == nil && lastLine >= 0 {
		if lastLine > 0 {
			logger.Info("Resuming from last stopped internal index:", lastLine)
		}
		return lastLine
	}

	return 0
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

	// IMPORTANT:
	// Do NOT call ResetExecutingProgram() here.
	// That clears last_line.txt and userline.json before resolveStartLine() can use them.
	// We only want to reset in-memory execution state for a fresh run attempt.
	execContext.Reset()
	execContext.ExecutingFilePath = fileName
	execContext.Commands = commands
	execContext.ECSEnabled = drvSettings.ECS
	execContext.StopExecution = false

	// Decide run start point only at execution time.
	startLine := resolveStartLine()
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
		} else {
			// Resume the current command index.
			// Prevent skipping a line that did not fully execute.
			execContext.NextLineWhenStopped = nextCommandIndex
		}

		logger.Debug("Stopping execution, next resume line will be #", execContext.NextLineWhenStopped)

		if !execContext.TrialModeActive {
			saveLastLine(execContext.NextLineWhenStopped)
		}

		return nil
	}

	// Respect handler-driven control flow (e.g. M99 loops).
	// If the handler changed NextCmdLineToExec, keep it.
	nextIdx := execContext.NextCmdLineToExec

	// If handler did not change next index, advance normally.
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
					execContext.WaitExecuteNextCommand()
				}

				return executeInLineParamFunction(paramToExec, result.Cmd, execContext, childResults)
			}
		}
	}

	return nil
}

func ResumeExecution() error {
	startLine := resolveStartLine()

	execContext.StopExecution = false
	execContext.NextLineWhenStopped = startLine
	execContext.NextCmdLineToExec = startLine

	logger.Info("resuming execution from line ", startLine)

	return executeCommands(execContext.Commands, startLine)
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

	// Keep compatibility with older callers that expect last_line.txt to reflect
	// the selected restart line, but store internal zero-based index.
	saveLastLine(startIdx)

	logger.Info("Updated execContext.NextLineWhenStopped from userline.json:",
		userLine, " -> internal index:", startIdx)
}