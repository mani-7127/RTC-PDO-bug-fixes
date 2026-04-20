package settings

import (
	"EtherCAT/channels"
	"EtherCAT/helper"
	"encoding/json"
	"io/ioutil"
	"os"
	"strconv"
)

type DriverSettings struct {
	FinishSignal       int             `json:"fin_signal"`
	WorkOffSet         float64         `json:"work_offset,string"`
	JogFeed            int             `json:"jog_feed,string"`
	HomingOffset       float32         `json:"homing_offset,string"`
	// HomingApos is the raw encoder apos saved at zero-ref time.
	// Written only by zero_reference.go — never by boot logic.
	// Used at boot to compute an in-memory aposCorrection so that
	// currentPosition() gives the correct display after an encoder sign flip.
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

// LoadDriverSettings loads settings.json into memory.
// It does NOT send SETTINGS_CHANGED — callers that want to notify motordriver
// must do so explicitly. This prevents a feedback loop where the
// SETTINGS_CHANGED handler calls LoadDriverSettings which re-fires the channel.
func LoadDriverSettings() error {
	path := helper.AppendWDPath("/settings/settings.json")
	settingsFile, err := ioutil.ReadFile(path)
	if err != nil {
		return err
	}
	return json.Unmarshal(settingsFile, &settingsRoot)
}

// LoadAndNotifyDriverSettings loads settings then notifies motordriver.
// Call this from the REST API after a settings save — NOT from inside
// a SETTINGS_CHANGED handler (that would create an infinite loop).
func LoadAndNotifyDriverSettings() error {
	if err := LoadDriverSettings(); err != nil {
		return err
	}
	channels.NotifyMotorDriver("SETTINGS_CHANGED", "", "", 0)
	return nil
}

// SaveDriverSettings writes a single driver's settings back to settings.json.
// Used by InitMaster to reanchor HomingOffset after encoder sign flip on power cycle.
func SaveDriverSettings(driverName string, ds DriverSettings) error {
	if settingsRoot == nil {
		settingsRoot = make(SettingsRoot)
	}
	settingsRoot[driverName] = ds

	path := helper.AppendWDPath("/settings/settings.json")
	data, err := json.MarshalIndent(settingsRoot, "", "  ")
	if err != nil {
		return err
	}
	// Write atomically via temp file to avoid corrupting settings.json on power loss
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, data, 0644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// SaveHomingReference saves the encoder apos at zero-ref time.
// This is the reference used by InitAposCorrection on each boot to compute
// a transparent position correction for encoder sign flips.
// HomingOffset (user setting) is never modified by the system.
func SaveHomingReference(driverName string, apos int32) error {
	ds := GetDriverSettings(driverName)
	ds.HomingApos = apos
	return SaveDriverSettings(driverName, ds)
}

func GetAllSettings() map[string]DriverSettings {
	return settingsRoot
}

func GetDriverSettings(driverName string) DriverSettings {
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