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
	filesPath    = "/mnt/app/jamun/gm_codes/FILES"
)

var executionInProgress atomic.Bool
var fileUpdatedDuringExecution atomic.Bool

// pendingCommand holds the last RS232 command received while execution was
// in progress. Only the most recent command matters — older ones are
// superseded. This prevents a queue of stale commands building up.
var pendingCommand atomic.Value // stores string

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

	parts := parseBatchCommands(cmd)
	logger.Info("[RS232] Batch received:", cmd)
	logger.Info("[RS232] Parsed commands:", strings.Join(parts, " | "))

	// Validate every token BEFORE writing anything to FILES.
	// If any token is invalid, reject the entire batch and log it.
	// This prevents partial file updates from corrupt serial data.
	normalized := make([]string, 0, len(parts))
	for _, p := range parts {
		p = strings.TrimSpace(p)
		if p == "" {
			continue
		}
		norm := normalizeControllerToken(p)
		if norm == "" {
			logger.Warn("[RS232] Invalid token in batch, rejecting entire command:", p, "from:", cmd)
			return // reject the whole batch
		}
		normalized = append(normalized, norm)
	}

	if len(normalized) == 0 {
		logger.Warn("[RS232] No valid commands after normalization, ignoring:", cmd)
		return
	}

	// All tokens valid — now update FILES atomically.
	for _, norm := range normalized {
		if err := updateFILESFromRS232(norm); err != nil {
			logger.Error("[RS232] FILES update failed for token:", norm, "error:", err)
			// Still continue with remaining valid tokens.
		} else {
			logger.Info("[RS232] FILES updated with:", norm)
		}
	}

	if executionInProgress.Load() {
		// Execution is running — mark that the file was updated so
		// executeFilesProgramOnce will re-run after the current execution ends.
		fileUpdatedDuringExecution.Store(true)
		// Store the raw command so we can log what triggered the re-run.
		pendingCommand.Store(cmd)
		logger.Warn("[RS232] Execution in progress, update deferred for next cycle")
		return
	}

	// No execution running — start one.
	executionInProgress.Store(true)
	go executeFilesProgramOnce()
}

// ------------------------------------------------------------
// EXECUTION LOGIC
// ------------------------------------------------------------

func executeFilesProgramOnce() {
	defer executionInProgress.Store(false)

	logger.Info("[RS232] Preparing program execution")

	// Stop any partially running program and reset state cleanly.
	channels.WriteCommandExecInput("stop_prog_exec", "")
	executors.ResetExecutingProgram()
	time.Sleep(200 * time.Millisecond)

	// Sync position so move calculations start from the real encoder value.
	motor.RefreshCurrentPosition()

	// Wait for the drive to settle after reset before issuing moves.
	// 2 seconds gives the PDO cyclic loop time to re-establish valid frames.
	time.Sleep(2 * time.Second)

	file := helper.GetCodeFilePath() + "/FILES"

	// Verify the file exists and is readable before attempting execution.
	// If it doesn't exist, log and return — don't crash or hang.
	if _, err := os.Stat(file); err != nil {
		logger.Error("[RS232] Program file not found or not accessible:", file, "error:", err)
		// Drain any deferred update flag — no point re-running if file is missing.
		fileUpdatedDuringExecution.Store(false)
		return
	}

	// Read and log the file content before execution so we have an audit trail
	// of exactly what program was run from this RS232 command.
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

	// If the file was updated while we were executing, run again immediately
	// with the latest content. This handles the case where the machine sends
	// a new position command before the previous program cycle finishes.
	if fileUpdatedDuringExecution.Load() {
		pending, _ := pendingCommand.Load().(string)
		logger.Info("[RS232] Deferred update detected, re-executing latest program. Triggered by:", pending)
		fileUpdatedDuringExecution.Store(false)
		pendingCommand.Store("")
		executionInProgress.Store(true)
		go executeFilesProgramOnce()
	}
}

// ------------------------------------------------------------
// PARSER: parseBatchCommands
// Supports these controller styles:
//
//  1. Concatenated: "G01F20G91G68A90"       -> ["G01F20","G91","G68","A90"]
//  2. Space-batch:  "G01 F20 G91 G68 A90;"  -> ["G01 F20","G91","G68","A90"]
//  3. Comma-batch:  "go1f20,g91,g68,a90"    -> ["go1f20","g91","g68","a90"]
//  4. Multi-;:      "G01 F20;G91;A90;"      -> ["G01 F20","G91","A90"]
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
		token := strings.TrimSpace(s[start:i])
		if token != "" {
			out = append(out, token)
		}
	}

	return out
}

// normalizeControllerToken converts a parsed token into a proper FILES line.
// Returns empty string if the token is not recognizable — caller must reject.
//
// Examples:
//
//	"G01F20" -> "G01 F20;"
//	"G91"    -> "G91;"
//	"A90"    -> "A90;"
//	"45"     -> "A45;"   (legacy numeric-only)
func normalizeControllerToken(s string) string {
	s = strings.TrimSpace(s)
	if s == "" {
		return ""
	}

	// Legacy numeric-only => A<number>;
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

	// Validate: must start with a known command letter.
	// Reject anything that doesn't — prevents garbage from corrupting FILES.
	if len(s) == 0 {
		return ""
	}
	switch s[0] {
	case 'G', 'M', 'A', 'B', 'X', 'Y', 'Z', 'D', 'F':
		// valid
	default:
		logger.Warn("[RS232] Unrecognized command token rejected:", s)
		return ""
	}

	if s[0] == 'G' || s[0] == 'M' {
		s = insertSpacesBeforeLetters(s)
	}

	s = strings.Join(strings.Fields(s), " ")

	if !strings.HasSuffix(s, ";") {
		s += ";"
	}
	return s
}

func insertSpacesBeforeLetters(s string) string {
	var out []rune
	var prev rune
	for i, r := range s {
		if i > 0 {
			isLetter := (r >= 'A' && r <= 'Z')
			isPrevDigit := (prev >= '0' && prev <= '9')
			if isLetter && isPrevDigit {
				out = append(out, ' ')
			}
		}
		out = append(out, r)
		prev = r
	}
	return string(out)
}

// ------------------------------------------------------------
// FILE UPDATE LOGIC
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

// updateFILESFromRS232 updates gm_codes/FILES atomically.
// Writes via temp file + rename to prevent partial writes on power loss.
func updateFILESFromRS232(oneCmd string) error {
	oneCmd = strings.TrimSpace(oneCmd)
	if oneCmd == "" {
		return fmt.Errorf("empty command")
	}
	if !strings.HasSuffix(oneCmd, ";") {
		oneCmd += ";"
	}

	content, err := os.ReadFile(filesPath)
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

		// Axis: match by letter only
		if isAxisKey(cmdKey) {
			if strings.HasPrefix(lt, string(cmdKey[0])) {
				lines[i] = oneCmd
				replaced = true
				break
			}
			continue
		}

		// Special modal: G90/G91 replace each other
		if cmdKey == "G90" || cmdKey == "G91" {
			if strings.HasPrefix(lt, "G90") || strings.HasPrefix(lt, "G91") {
				lines[i] = oneCmd
				replaced = true
				break
			}
			continue
		}

		// Normal: exact token match
		if strings.HasPrefix(lt, cmdKey) {
			lines[i] = oneCmd
			replaced = true
			break
		}
	}

	// Not found: insert before M99/M30
	if !replaced {
		idx := findInsertBeforeEnd(lines)
		lines = append(lines[:idx], append([]string{oneCmd}, lines[idx:]...)...)
	}

	// Atomic write via temp file
	tmpPath := filesPath + ".tmp"
	data := strings.Join(lines, "\n")
	if !strings.HasSuffix(data, "\n") {
		data += "\n"
	}

	if err := os.WriteFile(tmpPath, []byte(data), 0644); err != nil {
		return fmt.Errorf("failed to write temp FILES: %w", err)
	}
	if err := os.Rename(tmpPath, filesPath); err != nil {
		return fmt.Errorf("failed to replace FILES: %w", err)
	}

	return nil
}