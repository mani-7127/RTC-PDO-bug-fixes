package datatypes

import (
	"EtherCAT/channels"
	"regexp"
	"strconv"
)

type DriveSetting struct {
	POTLimit             int
	NOTLimit             int
	ConfiguredWorkOffset map[string]float64
	DestinationPosition  float64
}

// ExecutionContext keeps track of execution context of the commands.
type ExecutionContext struct {
	Commands                  []Command
	Divide360On               int
	CommandMaps               Execution
	CurrentExecCommandLine    int
	LoopCount                 int
	WhereLoopStarted          int
	RestartExecFromBegining   bool
	LinearInterpolationEnabled bool
	NextCmdLineToExec         int
	Err                       error
	TrialModeActive           bool
	RunMode                   string
	CurrentLoopCounter        int
	ExecutionMode             string
	EmergencyActivated        bool
	StopExecution             bool
	NextLineWhenStopped       int
	CommandFileName           string
	HasResetted               bool
	ECSEnabled                int
	DriveSettings             map[string]DriveSetting
	CurrentWorkOffSet         string
	WaitingForECS             bool
	ExecutingFilePath         string
	ResumeConsumed            bool
}

// MoveNextLine updates the next line to execute.
// This must always advance exactly one line unless a command handler explicitly
// overrides NextCmdLineToExec (for example M99 / loop handlers).
func (e *ExecutionContext) MoveNextLine() {
	e.WaitingForECS = false
	next := e.CurrentExecCommandLine + 1
	if next == e.CurrentExecCommandLine {
		next++
	}
	e.NextCmdLineToExec = next
}

// EndExecution will set NextCmdLineToExec to -1 and thus end execution.
func (e *ExecutionContext) EndExecution() {
	e.Reset()
	e.NextCmdLineToExec = -1
}

// MoveToStart moves execution to line 0 and resets transient state.
func (e *ExecutionContext) MoveToStart() {
	e.NextCmdLineToExec = 0
	e.Reset()
}

// Reset resets runtime state to initial values.
func (e *ExecutionContext) Reset() {
	e.CurrentExecCommandLine = 0
	e.RestartExecFromBegining = false
	e.LinearInterpolationEnabled = false
	e.NextCmdLineToExec = 0
	e.Err = nil
	e.StopExecution = false
	e.Divide360On = 0
	e.LoopCount = 0
	e.CurrentLoopCounter = 0
	e.WhereLoopStarted = 0
	e.NextLineWhenStopped = 0
	e.RunMode = "ABS"
	e.CurrentWorkOffSet = "G53"
	e.WaitingForECS = false
	e.ResumeConsumed = false
	for k, v := range e.DriveSettings {
		v.DestinationPosition = 0
		e.DriveSettings[k] = v
	}
}

// PrepareExecutingFile compares the current executing file name and the passed one.
func (e *ExecutionContext) PrepareExecutingFile(fileName string) {
	e.StopExecution = false
	e.HasResetted = false
	if e.CommandFileName != fileName {
		e.Reset()
		e.CommandFileName = fileName
	}
}

func (e *ExecutionContext) ActivateTrialMode() {
	e.TrialModeActive = true
}

// WaitExecuteNextCommand blocks only in single mode.
func (e *ExecutionContext) WaitExecuteNextCommand() {
	if e.TrialModeActive {
		return
	}
	if e.ExecutionMode == "continuous" {
		return
	}
	channels.OpenSingleModeChannel()
	channels.WaitTillSingleModeComplete()
}

func (e *ExecutionContext) DeActivateTrialMode() {
	e.TrialModeActive = false
}

func (e *ExecutionContext) ExtractString(str string) string {
	re := regexp.MustCompile("[A-Z]+")
	return re.FindString(str)
}

func (e *ExecutionContext) ExtractNumeric(str string) string {
	re := regexp.MustCompile("[-]?[0-9.]+")
	return re.FindString(str)
}

func (e *ExecutionContext) ExtractNumericAsFloat(str string) (float64, error) {
	numericStr := e.ExtractNumeric(str)
	return strconv.ParseFloat(numericStr, 32)
}

func (e *ExecutionContext) ExtractNumericAsInt(str string) (int, error) {
	numericStr := e.ExtractNumeric(str)
	return strconv.Atoi(numericStr)
}
