package restapi

import (
	"EtherCAT/helper"
	"EtherCAT/logger"
	"EtherCAT/settings"
	"encoding/json"
	"io/ioutil"
	"net/http"
	"os"
)

//SettingsResponse struct to hold the settings data for the ui
type SettingsResponse struct {
	Status   string          `json:"status"`
	Response json.RawMessage `json:"resp"`
}

func manipulateSettings(w http.ResponseWriter, r *http.Request) {
	setupCorsResponse(&w, r)
	if (*r).Method == "OPTIONS" {
		return
	}
	if (*r).Method == "GET" {
		settings, _ := readSettingsFile()
		resp := SettingsResponse{Status: "success", Response: json.RawMessage(settings)}
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(resp)
	} else {
		body, err := ioutil.ReadAll(r.Body)
		if err != nil {
			panic(err)
		}

		jsonRaw := json.RawMessage(body)
		toWriteJSON, _ := json.MarshalIndent(jsonRaw, "", "\t")
		errWrite := ioutil.WriteFile(helper.AppendWDPath("/settings/settings.json"), toWriteJSON, 0777)
		if errWrite != nil {
			json.NewEncoder(w).Encode("{\"status\":\"error\"}")
		} else {
			json.NewEncoder(w).Encode("{\"status\":\"success\"}")
		}

		// Reload settings and notify motordriver. Using LoadAndNotifyDriverSettings
		// instead of LoadDriverSettings to avoid the feedback loop where the
		// SETTINGS_CHANGED handler calls LoadDriverSettings which re-fires the
		// channel indefinitely.
		if err := settings.LoadAndNotifyDriverSettings(); err != nil {
			logger.Error("Failed to reload settings after save:", err)
		}
		logger.Debug("settings updated...")
	}
}

func readSettingsFile() ([]byte, error) {
	jsonFile, err := os.Open(helper.AppendWDPath("/settings/settings.json"))
	if err != nil {
		return nil, err
	}
	defer jsonFile.Close()
	byteValue, _ := ioutil.ReadAll(jsonFile)
	return byteValue, nil
}