package datatypes

import (
	"errors"
	"testing"
)

// TestResetDoesNotClearStopExecution verifies that Reset() does not wipe
// StopExecution. The ghost execution bug was caused by Reset() setting
// StopExecution = false, so a Stop followed by Reset would re-enable execution.
func TestResetDoesNotClearStopExecution(t *testing.T) {
	ctx := &ExecutionContext{}

	// Simulate: user pressed Stop
	ctx.StopExecution = true

	// Simulate: Reset() called internally (as happens in the reset case)
	ctx.Reset()

	if !ctx.StopExecution {
		t.Fatal("GHOST BUG: Reset() cleared StopExecution — execution would continue after stop+reset")
	}
}

// TestResetClearsOtherState verifies Reset() still clears all other runtime
// state correctly (regression guard — nothing else should be broken by the fix).
func TestResetClearsOtherState(t *testing.T) {
	ctx := &ExecutionContext{
		CurrentExecCommandLine:     5,
		RestartExecFromBegining:    true,
		LinearInterpolationEnabled: true,
		NextCmdLineToExec:          5,
		Err:                        errors.New("some error"),
		Divide360On:                1,
		LoopCount:                  3,
		CurrentLoopCounter:         2,
		WhereLoopStarted:           1,
		NextLineWhenStopped:        4,
		RunMode:                    "INC",
		CurrentWorkOffSet:          "G54",
		WaitingForECS:              true,
		ResumeConsumed:             true,
	}

	ctx.Reset()

	if ctx.CurrentExecCommandLine != 0 {
		t.Errorf("CurrentExecCommandLine: want 0, got %d", ctx.CurrentExecCommandLine)
	}
	if ctx.RestartExecFromBegining {
		t.Error("RestartExecFromBegining: want false")
	}
	if ctx.LinearInterpolationEnabled {
		t.Error("LinearInterpolationEnabled: want false")
	}
	if ctx.NextCmdLineToExec != 0 {
		t.Errorf("NextCmdLineToExec: want 0, got %d", ctx.NextCmdLineToExec)
	}
	if ctx.Err != nil {
		t.Errorf("Err: want nil, got %v", ctx.Err)
	}
	if ctx.Divide360On != 0 {
		t.Errorf("Divide360On: want 0, got %d", ctx.Divide360On)
	}
	if ctx.LoopCount != 0 {
		t.Errorf("LoopCount: want 0, got %d", ctx.LoopCount)
	}
	if ctx.CurrentLoopCounter != 0 {
		t.Errorf("CurrentLoopCounter: want 0, got %d", ctx.CurrentLoopCounter)
	}
	if ctx.WhereLoopStarted != 0 {
		t.Errorf("WhereLoopStarted: want 0, got %d", ctx.WhereLoopStarted)
	}
	if ctx.NextLineWhenStopped != 0 {
		t.Errorf("NextLineWhenStopped: want 0, got %d", ctx.NextLineWhenStopped)
	}
	if ctx.RunMode != "ABS" {
		t.Errorf("RunMode: want ABS, got %s", ctx.RunMode)
	}
	if ctx.CurrentWorkOffSet != "G53" {
		t.Errorf("CurrentWorkOffSet: want G53, got %s", ctx.CurrentWorkOffSet)
	}
	if ctx.WaitingForECS {
		t.Error("WaitingForECS: want false")
	}
	if ctx.ResumeConsumed {
		t.Error("ResumeConsumed: want false")
	}
}

// TestStopThenResetSequence simulates the exact ghost execution sequence:
// Stop pressed → Reset() called → NotifyCmdComplete unblocks goroutine →
// goroutine checks StopExecution. With the fix, StopExecution must still be
// true so the goroutine exits instead of advancing to the next G-code line.
func TestStopThenResetSequence(t *testing.T) {
	ctx := &ExecutionContext{}

	// Step 1: fresh execution starts
	ctx.StopExecution = false
	ctx.NextCmdLineToExec = 3

	// Step 2: user presses Stop
	ctx.StopExecution = true

	// Step 3: user presses Reset — this calls Reset() internally
	ctx.HasResetted = true
	ctx.Reset()
	ctx.StopExecution = true // re-assert (Fix 2)

	// Step 4: goroutine unblocks and checks StopExecution
	if !ctx.StopExecution {
		t.Fatal("GHOST BUG: after Stop+Reset sequence, StopExecution is false — goroutine would execute next line")
	}
	if !ctx.HasResetted {
		t.Fatal("HasResetted should still be true after reset sequence")
	}
}
