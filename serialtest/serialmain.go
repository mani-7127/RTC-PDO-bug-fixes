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

// getFilesPath returns the correct FILES path regardless of run location.
// Using helper.GetCodeFilePath() avoids the hardcoded /mnt/app/jamun path
// that broke execution when running from the dev directory.
func getFilesPath() string {
	return helper.GetCodeFilePath() + "/FILES"
}

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
	// rebuildFILES enforces strict line ordering and G68 rules.
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
		// Interrupt the currently running executor immediately so it exits
		// RunCodeFile and returns. The deferred command will then execute
		// cleanly in the re-launch of executeFilesProgramOnce.
		// Without this, the old execution stays blocked in doECSCheck or
		// waitForProgramFileUpdate indefinitely.
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
//   G01 F<n>;     — feedrate (always first)
//   G90; / G91;   — absolute / incremental mode
//   [G68;]        — shortest path (G90 only, omitted in G91)
//   A<deg>;       — target axis position
//   M99;          — loop back
//
// G68 rules:
//   G91 sent       → G68 removed (not valid in incremental mode)
//   G90 + G68 sent → G68 added after G90 line
//   G90 no G68     → G68 omitted
// ------------------------------------------------------------

func rebuildFILES(normalized []string) error {
	var feedrate, modeCmd, axisCmd string
	var hasG68 bool

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
			hasG68 = true
		case u == "G69":
			hasG68 = false
			if modeCmd == "" {
				modeCmd = "G90;"
			}
		case isAxisKey(u):
			axisCmd = tok
		}
	}

	// Fall back to existing FILES content for any field not supplied.
	existingFeedrate, existingAxis, existingMode := readExistingFILES()
	if feedrate == "" {
		feedrate = firstNonEmpty(existingFeedrate, "G01 F20;")
	}
	if axisCmd == "" {
		axisCmd = firstNonEmpty(existingAxis, "A0;")
	}
	if modeCmd == "" {
		modeCmd = firstNonEmpty(existingMode, "G90;")
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

func readExistingFILES() (feedrate, axis, mode string) {
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
	defer executionInProgress.Store(false)

	logger.Info("[RS232] Preparing program execution")

	channels.WriteCommandExecInput("stop_prog_exec", "")
	executors.ResetExecutingProgram()
	time.Sleep(200 * time.Millisecond)

	motor.RefreshCurrentPosition()
	time.Sleep(2 * time.Second)

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
		executionInProgress.Store(true)
		go executeFilesProgramOnce()
	}
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
// rebuildFILES fails. Kept for safety — should rarely be reached.
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