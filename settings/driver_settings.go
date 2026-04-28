package settings

import (
	"EtherCAT/channels"
	"EtherCAT/helper"
	"encoding/json"
	"io/ioutil"
	"os"
	"strconv"
	"sync"
)

type DriverSettings struct {
	FinishSignal       int             `json:"fin_signal"`
	WorkOffSet         float64         `json:"work_offset,string"`
	JogFeed            int             `json:"jog_feed,string"`
	HomingOffset       float32         `json:"homing_offset,string"`
	// HomingApos is the raw encoder apos saved at zero-ref time.
	// Written only by zero_reference.go — never by boot logic.
	// Used at boot to detect encoder sign flips transparently.
	// HomingOffset (user-configured) is NEVER modified by the system.
	HomingApos         int32           `json:"homing_apos,string"`
	HomeDirection      int             `json:"home_dir"`
	GearRation         string          `json:"gear_ratio"`
	ECSFinTiming       int             `json:"timing,string"`
	BackLash           float64         `json:"back_lash,string"`
	ECS                int             `json:"ecs"`
	ClampDeclamp       int             `json:"cl_dl"`
	G55                float64         `json:"g55,string"`
	G54                float64         `json:"g54,string"`
	NOT                int             `json:"not,string"`
	G57                float64         `json:"g57,string"`
	MotorDirection     int             `json:"motor_dir"`
	G58                float64         `json:"g58,string"`
	ClampDeclampTiming int             `json:"cldl_timing,string"`
	POT                int             `json:"pot,string"`
	G56                float64         `json:"g56,string"`
	PitchError         []Float64Str    `json:"pitch_error"`
	Mode               string
	FactorBacklash     int
	BinaryPosFeeds     []BinaryPosFeed `json:"binary_pos_feed"`
	LineNumber         string          `json:"line_number,omitempty"`
}

type BinaryPosFeed struct {
	Binary    string `json:"binary"`
	Position  string `json:"pos"`
	Direction int16  `json:"dir"`
	FeedRate  int32  `json:"feed_rate"`
}

type Float64Str float64

type TextProgramConfig struct {
	IP                      string `json:"ip"`
	User                    string `json:"user"`
	Password                string `json:"password"`
	Path                    string `json:"path"`
	JogClockwisePath        string `json:"jogClockwisePath"`
	JogCounterClockwisePath string `json:"jogCounterClockwisePath"`
}

func SaveTextProgramConfig(config TextProgramConfig) error {
	fileData, err := json.MarshalIndent(config, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile("/mnt/app/jamun/settings/textprogram.json", fileData, 0644)
}

func LoadTextProgramConfig() (TextProgramConfig, error) {
	var config TextProgramConfig
	fileData, err := os.ReadFile("/mnt/app/jamun/settings/textprogram.json")
	if err != nil {
		return config, err
	}
	err = json.Unmarshal(fileData, &config)
	return config, err
}

func (i Float64Str) MarshalJSON() ([]byte, error) {
	return json.Marshal(strconv.FormatFloat(float64(i), 'f', 3, 64))
}

func (i *Float64Str) UnmarshalJSON(b []byte) error {
	var s string
	if err := json.Unmarshal(b, &s); err == nil {
		value, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return err
		}
		*i = Float64Str(value)
		return nil
	}
	return json.Unmarshal(b, (*float64)(i))
}

type SettingsRoot map[string]DriverSettings

var settingsRoot SettingsRoot

// settingsMu protects settingsRoot from concurrent read/write panics.
//
// settingsRoot is accessed from multiple goroutines simultaneously:
//   - poll_drive_position reads GetDriverSettings every 50ms
//   - move_to_degree reads on every move
//   - ecs.go reads on every ECS check
//   - zero_reference writes SaveHomingReference after zero-ref
//   - REST API writes on settings save
//
// Without a mutex, concurrent map access causes a runtime panic:
//   "concurrent map read and map write"
//
// RWMutex: multiple readers never block each other, only writes are exclusive.
// This keeps the 50ms poll loop fast under normal operation.
var settingsMu sync.RWMutex

// LoadDriverSettings loads settings.json into memory.
func LoadDriverSettings() error {
	path := helper.AppendWDPath("/settings/settings.json")
	settingsFile, err := ioutil.ReadFile(path)
	if err != nil {
		return err
	}
	settingsMu.Lock()
	defer settingsMu.Unlock()
	return json.Unmarshal(settingsFile, &settingsRoot)
}

// LoadAndNotifyDriverSettings loads settings then notifies motordriver.
func LoadAndNotifyDriverSettings() error {
	if err := LoadDriverSettings(); err != nil {
		return err
	}
	channels.NotifyMotorDriver("SETTINGS_CHANGED", "", "", 0)
	return nil
}

// SaveDriverSettings writes a single driver's settings back to settings.json.
// Atomic write via temp file prevents corruption on power loss mid-write.
func SaveDriverSettings(driverName string, ds DriverSettings) error {
	settingsMu.Lock()
	if settingsRoot == nil {
		settingsRoot = make(SettingsRoot)
	}
	settingsRoot[driverName] = ds
	// Marshal while holding the lock so we snapshot a consistent state.
	data, err := json.MarshalIndent(settingsRoot, "", "  ")
	settingsMu.Unlock()

	if err != nil {
		return err
	}

	path := helper.AppendWDPath("/settings/settings.json")
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, data, 0644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// SaveHomingReference saves the raw encoder apos at zero-ref time.
// Called only by zero_reference.go after the motor has physically settled.
func SaveHomingReference(driverName string, apos int32) error {
	ds := GetDriverSettings(driverName)
	ds.HomingApos = apos
	return SaveDriverSettings(driverName, ds)
}

func GetAllSettings() map[string]DriverSettings {
	settingsMu.RLock()
	defer settingsMu.RUnlock()
	copy := make(map[string]DriverSettings, len(settingsRoot))
	for k, v := range settingsRoot {
		copy[k] = v
	}
	return copy
}

func GetDriverSettings(driverName string) DriverSettings {
	settingsMu.RLock()
	defer settingsMu.RUnlock()
	return settingsRoot[driverName]
}

func (ds *DriverSettings) GetWorkOffset() map[string]float64 {
	wrkOffset := make(map[string]float64)
	wrkOffset["G53"] = 0
	wrkOffset["G54"] = ds.G54
	wrkOffset["G55"] = ds.G55
	wrkOffset["G56"] = ds.G56
	wrkOffset["G57"] = ds.G57
	wrkOffset["G58"] = ds.G58
	return wrkOffset
}