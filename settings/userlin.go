package settings

import (
	"encoding/json"
	"io/ioutil"
	"strconv"
)

type UserLineSettings struct {
	UserLine string `json:"user_line"`
}

var userLineFile = "/mnt/app/jamun/settings/userline.json"

// SaveLineNumber saves the user line into a separate JSON file
func SaveLineNumber(line string) error {
	s := UserLineSettings{UserLine: line}
	data, err := json.MarshalIndent(s, "", "  ")
	if err != nil {
		return err
	}
	return ioutil.WriteFile(userLineFile, data, 0644)
}

// LoadLineNumber loads the user line back from file
func LoadLineNumber() (string, error) {
	data, err := ioutil.ReadFile(userLineFile)
	if err != nil {
		return "", err
	}
	var s UserLineSettings
	if err := json.Unmarshal(data, &s); err != nil {
		return "", err
	}
	return s.UserLine, nil
}

// LoadLineNumberAsInt returns the selected line as int.
// Invalid / empty values are treated as zero.
func LoadLineNumberAsInt() (int, error) {
	line, err := LoadLineNumber()
	if err != nil {
		return 0, err
	}
	if line == "" {
		return 0, nil
	}
	n, err := strconv.Atoi(line)
	if err != nil {
		return 0, err
	}
	return n, nil
}

// ClearLineNumber clears the user selected restart line.
func ClearLineNumber() {
	_ = SaveLineNumber("0")
}