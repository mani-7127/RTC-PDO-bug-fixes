package serialtest

import (
	"fmt"
	"os"
	"strconv"
	"strings"
	"sync/atomic"
	"time"

	"github.com/tarm/serial"

	"EtherCAT/channels"
	executors "EtherCAT/executors"
	"EtherCAT/helper"
	"EtherCAT/logger"
	motor "EtherCAT/motordriver"
)

const (
	serialDevice = "/dev/ttyUSB0"
	baudRate     = 9600
)

func getFilesPath() string {
	return helper.GetCodeFilePath() + "/FILES"
}

var executionInProgress atomic.Bool
var fileUpdatedDuringExecution atomic.Bool

// pendingCommand holds the last RS232 command received while execution was
// in progress. Only the most recent command matters — older ones are superseded.
var pendingCommand atomic.Value // stores string

// lastReceivedCmd and lastReceivedAt are used to debounce the RS232 input.
// The CNC machine often sends the same command string multiple times in rapid
// succession (retransmit / polling behaviour). We drop duplicates received
// within debounceWindow to avoid flooding the executor with redundant interrupts.
var lastReceivedCmd atomic.Value  // stores string
var lastReceivedAt  atomic.Int64  // stores UnixNano

const debounceWindow = 200 * time.Millisecond

// waitForProgramFileUpdateTimeout is the maximum time to wait for the CNC to
// send the next command after a motion completes. If the machine goes silent
// (cable fault, program end, operator stop), the executor unblocks and exits
// cleanly rather than hanging forever.
const waitForProgramFileUpdateTimeout = 60 * time.Second

func StartSerialListener() {
	cfg := &serial.Config{Name: serialDevice, Baud: baudRate, ReadTimeout: time.Millisecond * 200}
	port, err := serial.OpenPort(cfg)
	if err != nil {
		logger.Error("Failed to open RS-232:", err)
		return
	}
	defer port.Close()

	logger.Info("RS-232 FILES updater + deferred executor started")
	buf := make([]byte, 256)
	frame := make([]byte, 0, 128)

	for {
		n, err := port.Read(buf)
		if err != nil || n == 0 {
			continue
		}
		for i := 0; i < n; i++ {
			b := buf[i]
			if b == 0x12 {
				continue
			}
			clean := b & 0x7F
			if clean == '\n' || clean == '\r' || clean == 0x14 {
				cmd := strings.TrimSpace(string(frame))
				frame = frame[:0]
				if cmd != "" {
					handleRS232Command(cmd)
				}
				continue
			}
			frame = append(frame, clean)
		}
	}
}

func handleRS232Command(cmd string) {
	cmd = strings.TrimSpace(cmd)
	if cmd == "" {
		return
	}

	// Debounce: drop identical commands arriving within debounceWindow.
	// The CNC often retransmits the same string several times in <10ms.
	// Without this, each copy sends a stop_prog_exec, thrashes FILES, and
	// interrupts the executor unnecessarily.
	now := time.Now().UnixNano()
	prev, _ := lastReceivedCmd.Load().(string)
	prevAt := time.Duration(lastReceivedAt.Load())
	if cmd == prev && time.Duration(now)-prevAt < debounceWindow {
		logger.Debug("[RS232] Debounced duplicate command:", cmd)
		return
	}
	lastReceivedCmd.Store(cmd)
	lastReceivedAt.Store(now)

	parts := parseBatchCommands(cmd)
	logger.Info("[RS232] Batch received:", cmd)
	logger.Info("[RS232] Parsed commands:", strings.Join(parts, " | "))

	// Validate every token BEFORE writing anything to FILES.
	// If any token is invalid, reject the entire batch.
	normalized := make([]string, 0, len(parts))
	for _, p := range parts {
		p = strings.TrimSpace(p)
		if p == "" {
			continue
		}
		norm := normalizeControllerToken(p)
		if norm == "" {
			logger.Warn("[RS232] Invalid token in batch, rejecting entire command:", p, "from:", cmd)
			return
		}
		normalized = append(normalized, norm)
	}

	if len(normalized) == 0 {
		logger.Warn("[RS232] No valid commands after normalization, ignoring:", cmd)
		return
	}

	// All tokens valid — rebuild FILES atomically from the full batch.
	if err := rebuildFILES(normalized); err != nil {
		logger.Error("[RS232] rebuildFILES failed, falling back to token update:", err)
		for _, norm := range normalized {
			if err2 := updateFILESFromRS232(norm); err2 != nil {
				logger.Error("[RS232] FILES update failed for token:", norm, "error:", err2)
			} else {
				logger.Info("[RS232] FILES updated with:", norm)
			}
		}
	} else {
		logger.Info("[RS232] FILES rebuilt successfully from batch:", cmd)
	}

	if executionInProgress.Load() {
		// Interrupt the currently running executor so it exits RunCodeFile.
		// The deferred command will then execute cleanly on re-launch.
		channels.WriteCommandExecInput("stop_prog_exec", "")
		fileUpdatedDuringExecution.Store(true)
		pendingCommand.Store(cmd)
		logger.Warn("[RS232] Execution interrupted + queued:", cmd)
		return
	}

	executionInProgress.Store(true)
	go executeFilesProgramOnce()
}

// ------------------------------------------------------------
// FILES REBUILD
// Constructs FILES from scratch with strict line ordering:
//
//   G01 F<n>;   — feedrate (always first)
//   G90; / G91; — absolute / incremental mode
//   [G68;]      — shortest path (G90 only, omitted in G91)
//   A<deg>;     — target axis position
//   M99;        — loop back
//
// G68 persistence rules:
//   - If the incoming batch contains G68 explicitly  → include G68
//   - If the incoming batch contains G69 explicitly  → omit G68
//   - If neither G68 nor G69 in the batch           → preserve
//     whatever G68 state is in the existing FILES
//   - G68 is always removed when mode is G91
// ------------------------------------------------------------

func rebuildFILES(normalized []string) error {
	var feedrate, modeCmd, axisCmd string
	g68Explicit := false
	g69Explicit := false

	for _, tok := range normalized {
		u := upperTrim(tok)
		switch {
		case isFeedrate(u):
			feedrate = tok
		case u == "G90":
			modeCmd = "G90;"
		case u == "G91":
			modeCmd = "G91;"
		case u == "G68":
			g68Explicit = true
		case u == "G69":
			g69Explicit = true
			if modeCmd == "" {
				modeCmd = "G90;"
			}
		case isAxisKey(u):
			axisCmd = tok
		}
	}

	// Fall back to existing FILES content for any field not supplied.
	existingFeedrate, existingAxis, existingMode, existingHasG68 := readExistingFILES()
	if feedrate == "" {
		feedrate = firstNonEmpty(existingFeedrate, "G01 F20;")
	}
	if axisCmd == "" {
		axisCmd = firstNonEmpty(existingAxis, "A0;")
	}
	if modeCmd == "" {
		modeCmd = firstNonEmpty(existingMode, "G90;")
	}

	// Determine final G68 state:
	//   explicit G68 in batch → true
	//   explicit G69 in batch → false
	//   neither               → inherit from existing FILES
	var hasG68 bool
	switch {
	case g68Explicit:
		hasG68 = true
	case g69Explicit:
		hasG68 = false
	default:
		hasG68 = existingHasG68
	}

	lines := []string{feedrate, modeCmd}

	if strings.HasPrefix(modeCmd, "G90") {
		if hasG68 {
			lines = append(lines, "G68;")
			logger.Info("[RS232] G68 added — absolute mode with shortest path")
		} else {
			logger.Info("[RS232] G68 omitted — absolute mode without shortest path")
		}
	} else if hasG68 {
		logger.Info("[RS232] G68 removed — not valid in incremental (G91) mode")
	}

	lines = append(lines, axisCmd, "M99;")

	logger.Info("[RS232] Rebuilding FILES:")
	for i, l := range lines {
		logger.Info(fmt.Sprintf("[RS232]   line %d: %s", i, l))
	}

	return writeFILESAtomic(lines)
}

// readExistingFILES reads the current FILES content and returns its
// feedrate, axis, mode, and G68 state for use as fallback values when
// a new command does not supply those fields.
func readExistingFILES() (feedrate, axis, mode string, hasG68 bool) {
	content, err := os.ReadFile(getFilesPath())
	if err != nil {
		return
	}
	for _, line := range strings.Split(string(content), "\n") {
		lt := strings.TrimSpace(line)
		if lt == "" {
			continue
		}
		u := upperTrim(lt)
		switch {
		case isFeedrate(u):
			feedrate = lt
		case u == "G90" || u == "G91":
			mode = lt
		case u == "G68":
			hasG68 = true
		case u == "G69":
			hasG68 = false
		case isAxisKey(u):
			axis = lt
		}
	}
	return
}

func upperTrim(s string) string {
	return strings.ToUpper(strings.TrimSpace(strings.TrimSuffix(strings.TrimSpace(s), ";")))
}

func isFeedrate(upper string) bool {
	return strings.HasPrefix(upper, "G01") || strings.HasPrefix(upper, "G1 ") ||
		(strings.HasPrefix(upper, "G0 ") && strings.Contains(upper, "F"))
}

func firstNonEmpty(vals ...string) string {
	for _, v := range vals {
		if v != "" {
			return v
		}
	}
	return ""
}

func writeFILESAtomic(lines []string) error {
	data := strings.Join(lines, "\n")
	if !strings.HasSuffix(data, "\n") {
		data += "\n"
	}
	tmp := getFilesPath() + ".tmp"
	if err := os.WriteFile(tmp, []byte(data), 0644); err != nil {
		return fmt.Errorf("failed to write temp FILES: %w", err)
	}
	if err := os.Rename(tmp, getFilesPath()); err != nil {
		return fmt.Errorf("failed to rename FILES: %w", err)
	}
	return nil
}

// ------------------------------------------------------------
// EXECUTION LOGIC
// ------------------------------------------------------------

func executeFilesProgramOnce() {
	// NOTE: executionInProgress.Store(false) is NOT deferred here.
	// It is called explicitly only at the non-re-execute exit path below.
	// When a deferred re-execution is triggered, we return without clearing
	// the flag so handleRS232Command cannot race in between the defer and
	// the new goroutine launch, which was causing concurrent executor runs.

	logger.Info("[RS232] Preparing program execution")

	channels.WriteCommandExecInput("stop_prog_exec", "")
	executors.ResetExecutingProgram()
	time.Sleep(200 * time.Millisecond)

	motor.RefreshCurrentPosition()

	// Short settle wait — enough for the PDO cyclic to stabilise after reset.
	// The previous 2s unconditional sleep added 2s of latency to every
	// RS232-triggered re-execution. 400ms is sufficient for the drive to
	// reach a stable state when no physical motion was happening.
	time.Sleep(400 * time.Millisecond)

	file := getFilesPath()

	if _, err := os.Stat(file); err != nil {
		logger.Error("[RS232] Program file not found or not accessible:", file, "error:", err)
		fileUpdatedDuringExecution.Store(false)
		return
	}

	if content, err := os.ReadFile(file); err == nil {
		logger.Info("[RS232] Executing file content:\n", string(content))
	}

	logger.Info("[RS232] EXECUTING Program:", file)

	executors.SetRS232Enabled(true)
	defer executors.SetRS232Enabled(false)

	if err := executors.RunCodeFile(file); err != nil {
		logger.Error("[RS232] Program execution failed:", err)
	} else {
		logger.Info("[RS232] Program execution completed successfully")
	}

	if fileUpdatedDuringExecution.Load() {
		pending, _ := pendingCommand.Load().(string)
		logger.Info("[RS232] Deferred update detected, re-executing latest program. Triggered by:", pending)
		fileUpdatedDuringExecution.Store(false)
		pendingCommand.Store("")
		// executionInProgress is already true — do NOT clear it before launching
		// the next goroutine. This prevents handleRS232Command from racing in
		// between and launching a competing goroutine.
		go executeFilesProgramOnce()
		return // exit without calling Store(false) — next goroutine owns the flag
	}

	// Normal exit: no pending re-execution. Release the lock.
	executionInProgress.Store(false)
}

// WaitForProgramFileUpdate polls for the FILES mod time to change.
// Returns true if a file update was detected, false if timeout elapsed.
// This is called by command_executor.go via the RS232 blocking path.
// A timeout prevents a permanent hang if the CNC goes silent.
func WaitForProgramFileUpdate(filePath string) bool {
	info, err := os.Stat(filePath)
	if err != nil {
		logger.Warn("[RS232] Could not stat file for updates:", err)
		return false
	}

	lastMod := info.ModTime()
	deadline := time.Now().Add(waitForProgramFileUpdateTimeout)

	for time.Now().Before(deadline) {
		time.Sleep(20 * time.Millisecond)

		info, err := os.Stat(filePath)
		if err != nil {
			logger.Warn("[RS232] Could not stat file for updates:", err)
			return false
		}

		if info.ModTime().After(lastMod) {
			logger.Info("[RS232] Detected program file update:", filePath)
			return true
		}
	}

	logger.Warn("[RS232] waitForProgramFileUpdate: timed out after", waitForProgramFileUpdateTimeout,
		"— no new command received. Exiting RS232 wait loop.")
	return false
}

// ------------------------------------------------------------
// PARSER
// ------------------------------------------------------------

func parseBatchCommands(s string) []string {
	s = strings.TrimSpace(s)
	if s == "" {
		return nil
	}

	if strings.Count(s, ";") > 1 {
		raw := strings.Split(s, ";")
		out := make([]string, 0, len(raw))
		for _, r := range raw {
			r = strings.TrimSpace(r)
			if r != "" {
				out = append(out, r)
			}
		}
		return out
	}

	s = strings.TrimSuffix(s, ";")
	s = strings.TrimSpace(s)

	if strings.Contains(s, ",") {
		raw := strings.Split(s, ",")
		out := make([]string, 0, len(raw))
		for _, r := range raw {
			r = strings.TrimSpace(r)
			if r != "" {
				out = append(out, r)
			}
		}
		return out
	}

	if strings.Contains(s, " ") || strings.Contains(s, "\t") {
		toks := strings.Fields(s)
		if len(toks) == 0 {
			return nil
		}

		isCmdStart := func(t string) bool {
			if t == "" {
				return false
			}
			u := strings.ToUpper(strings.TrimSuffix(t, ";"))
			switch u[0] {
			case 'G', 'M', 'A', 'B', 'X', 'Y', 'Z', 'D':
				return true
			default:
				return false
			}
		}

		isParam := func(t string) bool {
			if t == "" {
				return false
			}
			u := strings.ToUpper(strings.TrimSuffix(t, ";"))
			switch u[0] {
			case 'F', 'P', 'I', 'J', 'K', 'R', 'X', 'Y', 'Z':
				return true
			default:
				return false
			}
		}

		var out []string
		var cur []string
		for _, t := range toks {
			if isCmdStart(t) {
				if len(cur) > 0 {
					out = append(out, strings.Join(cur, " "))
					cur = cur[:0]
				}
				cur = append(cur, t)
				continue
			}
			if len(cur) > 0 && isParam(t) {
				cur = append(cur, t)
				continue
			}
			if len(cur) > 0 {
				cur = append(cur, t)
			}
		}
		if len(cur) > 0 {
			out = append(out, strings.Join(cur, " "))
		}
		return out
	}

	return splitConcatenatedBatch(s)
}

func splitConcatenatedBatch(s string) []string {
	s = strings.TrimSpace(s)
	if s == "" {
		return nil
	}

	u := strings.ToUpper(s)

	isStart := func(c byte) bool {
		switch c {
		case 'G', 'M', 'A', 'B', 'D':
			return true
		default:
			return false
		}
	}

	var out []string
	i := 0
	for i < len(u) {
		if u[i] == ' ' || u[i] == '\t' {
			i++
			continue
		}
		if !isStart(u[i]) {
			i++
			continue
		}
		start := i
		i++
		if u[start] == 'G' || u[start] == 'M' {
			for i < len(u) && u[i] >= '0' && u[i] <= '9' {
				i++
			}
			for i < len(u) && !isStart(u[i]) {
				i++
			}
		} else {
			for i < len(u) && !isStart(u[i]) {
				i++
			}
		}
		if token := strings.TrimSpace(s[start:i]); token != "" {
			out = append(out, token)
		}
	}
	return out
}

func normalizeControllerToken(s string) string {
	s = strings.TrimSpace(s)
	if s == "" {
		return ""
	}

	if v, err := strconv.ParseFloat(s, 64); err == nil {
		return fmt.Sprintf("A%g;", v)
	}

	s = strings.TrimSuffix(s, ";")
	s = strings.ReplaceAll(s, ",", " ")
	s = strings.ToUpper(s)

	// Fix GO1 -> G01 (letter O instead of zero, common typo)
	if strings.HasPrefix(s, "GO") && len(s) >= 3 {
		s = "G0" + s[2:]
	}

	if len(s) == 0 {
		return ""
	}
	switch s[0] {
	case 'G', 'M', 'A', 'B', 'X', 'Y', 'Z', 'D', 'F':
	default:
		logger.Warn("[RS232] Unrecognized command token rejected:", s)
		return ""
	}

	if s[0] == 'G' || s[0] == 'M' {
		s = insertSpacesBeforeLetters(s)
	}

	s = strings.Join(strings.Fields(s), " ")

	if strings.HasPrefix(s, "G0 F") {
		s = "G01" + s[2:]
	}

	if !strings.HasSuffix(s, ";") {
		s += ";"
	}
	return s
}

func insertSpacesBeforeLetters(s string) string {
	var out []rune
	var prev rune
	for i, r := range s {
		if i > 0 && (r >= 'A' && r <= 'Z') && (prev >= '0' && prev <= '9') {
			out = append(out, ' ')
		}
		out = append(out, r)
		prev = r
	}
	return string(out)
}

// ------------------------------------------------------------
// FILE UPDATE HELPERS
// ------------------------------------------------------------

func isAxisKey(k string) bool {
	if k == "" {
		return false
	}
	switch k[0] {
	case 'A', 'B', 'X', 'Y', 'Z', 'D':
		return true
	default:
		return false
	}
}

func findInsertBeforeEnd(lines []string) int {
	for i, line := range lines {
		lt := strings.TrimSpace(line)
		if strings.HasPrefix(lt, "M99") || strings.HasPrefix(lt, "M30") {
			return i
		}
	}
	return len(lines)
}

// updateFILESFromRS232 is the fallback token-by-token updater used when
// rebuildFILES fails. Should rarely be reached.
func updateFILESFromRS232(oneCmd string) error {
	oneCmd = strings.TrimSpace(oneCmd)
	if oneCmd == "" {
		return fmt.Errorf("empty command")
	}
	if !strings.HasSuffix(oneCmd, ";") {
		oneCmd += ";"
	}

	content, err := os.ReadFile(getFilesPath())
	if err != nil {
		return fmt.Errorf("failed to read FILES: %w", err)
	}
	lines := strings.Split(string(content), "\n")

	cmdNoSemi := strings.TrimSuffix(oneCmd, ";")
	toks := strings.Fields(cmdNoSemi)
	if len(toks) == 0 {
		return fmt.Errorf("invalid command: %q", oneCmd)
	}
	cmdKey := toks[0]
	replaced := false

	for i, line := range lines {
		lt := strings.TrimSpace(line)
		if lt == "" {
			continue
		}
		if isAxisKey(cmdKey) {
			if strings.HasPrefix(lt, string(cmdKey[0])) {
				lines[i] = oneCmd
				replaced = true
				break
			}
			continue
		}
		if cmdKey == "G90" || cmdKey == "G91" {
			if strings.HasPrefix(lt, "G90") || strings.HasPrefix(lt, "G91") {
				lines[i] = oneCmd
				replaced = true
				break
			}
			continue
		}
		if strings.HasPrefix(lt, cmdKey) {
			lines[i] = oneCmd
			replaced = true
			break
		}
	}

	if !replaced {
		idx := findInsertBeforeEnd(lines)
		lines = append(lines[:idx], append([]string{oneCmd}, lines[idx:]...)...)
	}

	return writeFILESAtomic(lines)
}