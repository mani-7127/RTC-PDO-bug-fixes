package motordriver

import (
	helper "EtherCAT/helper"
	"EtherCAT/logger"
	settings "EtherCAT/settings"
	"fmt"
	"math"
	"sync/atomic"
)

// aposCorrection absorbs the Panasonic A6B encoder sign flip that can occur
// when the drive is power-cycled. The multi-turn counter may read as the
// arithmetic complement of the previous value for the same physical position.
//
// Computed ONCE at boot in InitAposCorrection() by comparing the current apos
// against HomingApos (saved at zero-ref time). Applied inside currentPosition()
// so the display is always correct WITHOUT ever modifying HomingOffset.
//
// HomingOffset is a USER-configured value — the system must never write it.
// This correction lives only in memory and is recomputed each boot.
//
//   aposCorrection = boot_apos - HomingApos
//   corrected_apos = live_apos - aposCorrection
//
// With correction: corrected_apos always behaves as if the encoder origin
// equals HomingApos, regardless of sign flips.
var aposCorrection atomic.Int32

// InitAposCorrection detects and corrects a Panasonic A6B encoder sign flip
// at boot time. It sets aposCorrection ONLY when a genuine sign flip is
// detected — never for legitimate position changes.
//
// A sign flip is identified by two conditions both being true:
//   1. bootApos and HomingApos have OPPOSITE signs
//   2. Their magnitudes are within 0.5% of each other
//      (same physical position, just counter wrapped to the other side)
//
// If the table physically moved while powered off, the magnitudes differ
// by more than 0.5% → no correction → correct position displayed.
//
// If HomingApos=0 (zero ref never performed) → no correction.
func InitAposCorrection(driveName string) {
	ds := settings.GetDriverSettings(driveName)
	if ds.HomingApos == 0 {
		aposCorrection.Store(0)
		logger.Info("InitAposCorrection: HomingApos not set — no correction (perform zero ref first)")
		return
	}

	bootApos  := pdoFbActual.Load()
	homingApos := ds.HomingApos

	// Check for sign flip: opposite signs AND same magnitude (within 0.5%)
	signFlip := false
	if (bootApos > 0) != (homingApos > 0) { // opposite signs
		magRatio := float64(bootApos) / float64(homingApos) // both non-zero here
		if magRatio < 0 {
			magRatio = -magRatio // abs of ratio
		}
		if math.Abs(magRatio-1.0) < 0.005 { // within 0.5%
			signFlip = true
		}
	}

	if signFlip {
		aposCorrection.Store(bootApos - homingApos)
		logger.Info("InitAposCorrection: sign flip detected — correction applied:",
			"bootApos=", bootApos,
			"HomingApos=", homingApos,
			"correction=", aposCorrection,
		)
	} else {
		aposCorrection.Store(0)
		logger.Info("InitAposCorrection: no sign flip — showing real position:",
			"bootApos=", bootApos,
			"HomingApos=", homingApos,
		)
	}
}

// Functions related to driver angle manipulation.

//getPulsesFromDegree returns the value to send to driver based on the degree passed by the client
func getPulsesFromDegree(masterDevice MasterDevice, degree float64) int64 {

	// driverSettings := settings.GetDriverSettings(masterDevice.Name)
	pulse := float64(masterDevice.Device.DriveXRatio) * degree
	// pulse = helper.RoundFloatTo3(pulse + float64(driverSettings.BackLash*float64((settings.BacklashRequired(int(pulse))*masterDevice.Device.DriveXRatio))))

	//this rounding is commented as it cause wrong value if the value is big and thus the rotation is always wrong
	// pulse = helper.RoundFloatTo3(pulse)
	pulseForDisp := fmt.Sprintf("%f", pulse)
	logger.Debug("degree: ", degree, " pulse: ", pulseForDisp, " integer: ", int32(pulse), "drive x ratio:", masterDevice.Device.DriveXRatio)
	//tcpserver.js line# 1307
	// if pulse < 0 {
	// 	pulse = pulse * -1
	// }
	return int64(pulse)
}

/*
currentPosition returns the angle in degree based on the postion value received from driver.
 driveXRatio: is a config value configured along with the master device in configs/device-configuration.yml
 pos: position received from driver, this will be in pulses
*/
func currentPosition(pos int32, driveXRatio int, driveName string) (float64, float64) {
	driverSettings := settings.GetDriverSettings(driveName)
	// angleWithPitchError := float32(0.00)

	driveOffset := driverSettings.HomingOffset * float32(driveXRatio)
	// Apply aposCorrection to neutralise encoder sign flip on power cycle.
	// HomingOffset is a user value and is never modified — the correction is
	// applied here in the calculation only, invisibly to the user.
	correctedPos := pos - aposCorrection.Load()
	drivePosition := float64(correctedPos) - float64(driveOffset)
	drivePosition = drivePosition / float64(driveXRatio)
	// Proper positive modulo — works correctly for any sign of apos.
	// Go's math.Mod preserves the sign of the dividend, so math.Mod(-64.6, 360) = -64.6
	// Adding 360 once only works when the result is in [-360, 0).
	// For large negative apos (many CCW rotations), the result can be << -360,
	// causing the position to be wrong by multiples of 360° — seen as 60°+ zero drift
	// between boots when the encoder multi-turn counter has opposite sign.
	drivePosition = math.Mod(math.Mod(float64(drivePosition), 360)+360, 360)
	if drivePosition >= 359.999 {
		drivePosition = 0
	}
	drivePositionWithErrorCorrection := (drivePosition) //+ (float64(driverSettings.FactorBacklash) * driverSettings.BackLash)
	return helper.RoundFloat(drivePosition, 3), helper.RoundFloat(drivePositionWithErrorCorrection, 3)
}

//TODO add function to find pitch error like below, this is the value sending to ui
// drivePosition = drivePosition + float32((settings.FactorBacklash() * driverSettings.BackLash)) - angleWithPitchError

/*
getAbsolutePosition move the motor to absolute degree

returns
	move to position for driver
	destination position to communicate to clients to display the destination postion

definition

target position is 30 then motor will rotate to 30degree
irrespective where the current position is
based on getDestinationAngle() in machine_parser.js line# 420
*/
func getAbsolutePosition(currentPos float64, targetPos float64, useShortestPath bool) (float64, float64) {
	return helper.GetAbsolutePosition(currentPos, targetPos, useShortestPath)
}

// /*
// getAbsolutePath get the angle to rotate in absolute path

// for e.g. if current position is 50 and target position is 10 then rotate all the way
// to 360 and go to 10
// */
// func getAbsolutePath(currentPos float64, targetPos float64) float64 {
// 	toMove := math.Mod(currentPos, 360)
// 	toMove = toMove - targetPos
// 	toMove = 360 - toMove
// 	if math.Abs(currentPos) > 360 {
// 		toMove = math.Mod(toMove, 360)
// 	}

// 	if targetPos < 0 {
// 		toMove = toMove - 360
// 		if toMove <= -360 {
// 			toMove = toMove + 360
// 		}
// 	}

// 	return toMove
// }

// /*
// getAbsolutePositionWithShortestPath move to absolute postion but uses shortest path

// for eg. if current position is 50 and target position is 40 then will -10 and move to 40
// */
// func getAbsolutePositionWithShortestPath(currentPos float64, targetPos float64) float64 {
// 	currentPos = math.Mod(currentPos, 360)
// 	modeDiff := math.Mod((targetPos - currentPos), 360)
// 	shortestDistance := 180 - math.Abs(math.Abs(modeDiff)-180)

// 	test := math.Mod((modeDiff + 360), 360)
// 	if test < 180 {
// 		return shortestDistance * 1
// 	}
// 	return shortestDistance * -1
// }

/*
getRelativePosition returns the relative position based on the current position

for e.g. if the current position is 10 degree and ordered 20 degree then
motor will move to 30degree (10+20=30)
based on getDestinationAngle() in machine_parser.js line# 420
*/
func getRelativePosition(currentPos float64, targetPos float64, prevDestinationAngle float64) (float64, float64) {
	// If previous destination is too far from current, use current position instead.
	if math.Abs(currentPos-prevDestinationAngle) > 1.0 { // 1° threshold, tune as needed
		prevDestinationAngle = currentPos
	}
	return helper.GetRelativePosition(currentPos, targetPos, prevDestinationAngle)
}

/*
getPitchError returns the configured pitch error

It’s position calibration done to compensate mechanical error.
The mechanical systems shall be manufactured and assembled to the closest precision possible
before entering the pitch error compensation. Pitch error is a position error measured at equal
intervals of rotary table. In our case we shall keep interval for every 10 degrees. That is 0 degrees
till 360 Degrees shall have a total of 36 intervals. The pitch error on the rotary table shall be
measured on the final output shaft / Rotary table top. The error at different intervals is measured
by external fine measuring equipment.

Whenever the controller is moving or commanded to any position. The commanded value will be
added or subtracted with error defined in the interval chart. The user display of position data will
always show the original position command, all the error data shall be acting only in the backend
calculation. When the controller is being operated the user display shall not physically show
the compensated error.
*/
func getPitchError(driveName string, targetPos float64) float64 {
	index := int(math.Abs(targetPos / 10))
	index = index - 1
	if index < 0 {
		return 0
	}
	if index > 35 {
		return 0
	}
	driverSettings := settings.GetDriverSettings(driveName)
	return float64(driverSettings.PitchError[index])
}

// func reversePitchError(currentPos float64) float64 {
// 	index := int(math.Abs(currentPos / 10))
// 	index = index - 1
// 	if index < 0 {
// 		return 0
// 	}
// 	// if index > 35 {
// 	// 	return 0
// 	// }
// 	driverSettings := settings.GetDriverSettings()
// 	return float64(driverSettings.PitchError[index] * -1)
// }