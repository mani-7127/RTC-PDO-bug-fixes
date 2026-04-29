package motordriver

import (
	helper   "EtherCAT/helper"
	"EtherCAT/logger"
	settings "EtherCAT/settings"
	"fmt"
	"math"
	"sync/atomic"
)

// signFlipActive is set true at boot when a full-power-cycle encoder sign flip
// is detected. When true, getRawApos() negates the raw hardware value so every
// consumer (position display, move targets, ECS settle) sees a sign-consistent
// value matching what was present when zero-ref was last performed.
//
// HomingOffset, pitch error, work offsets — all user-configured values —
// are NEVER modified. The correction is invisible to the customer and is
// recomputed from scratch on every boot.
var signFlipActive atomic.Bool

// getRawApos returns the live encoder position with sign-flip correction
// already applied. This is the ONLY place pdoFbActual is consumed in the
// entire codebase. All other files call this instead of pdoFbActual.Load().
func getRawApos() int32 {
	v := pdoFbActual.Load()
	if signFlipActive.Load() {
		return -v
	}
	return v
}

// correctedToCmdTarget converts a corrected position back to the drive raw
// coordinate space for writing to pdoCmdTarget (0x607A).
//
// getRawApos() negates pdoFbActual when signFlipActive=true so all code sees
// consistent positive values. But pdoCmdTarget is sent to the drive hardware
// which operates in its own raw space. Storing a corrected (+) value when the
// drive apos is negative makes the drive travel ~2x the position — hundreds of
// rotations. This function converts back so pdoCmdTarget is always in raw space.
func correctedToCmdTarget(correctedPos int32) int32 {
	if signFlipActive.Load() {
		return -correctedPos
	}
	return correctedPos
}

// InitAposCorrection is called once per boot from InitMaster, after the drive
// is powered and the first valid PDO frame has been received.
//
// It sets signFlipActive=true when bootApos is POSITIVE and HomingApos is
// NEGATIVE (the only valid sign combination indicating a flip).
//
// HomingApos is always saved as a normalized negative value by zero_reference.go.
// So the sign of bootApos alone is sufficient to detect a flip — no magnitude
// comparison is needed or wanted. The old 15% magnitude gate was incorrectly
// blocking the correction when the operator stopped the table at a different
// position before powering off (which caused bootApos magnitude to differ from
// HomingApos magnitude even though a sign flip had still occurred).
func InitAposCorrection(driveName string) {
	signFlipActive.Store(false)

	ds := settings.GetDriverSettings(driveName)
	homingApos := ds.HomingApos

	if homingApos == 0 {
		logger.Info("InitAposCorrection: HomingApos not set — no correction",
			"drive=", driveName,
			"HomingOffset=", ds.HomingOffset,
		)
		return
	}

	bootApos := pdoFbActual.Load() // raw, before any correction

	bootMag := math.Abs(float64(bootApos))
	homeMag := math.Abs(float64(homingApos))

	if homeMag < 1 {
		logger.Info("InitAposCorrection: HomingApos magnitude too small — no correction",
			"drive=", driveName,
		)
		return
	}

	// Sign-only check. HomingApos is always saved as negative (zero_reference.go
	// normalizes it). bootApos positive = sign flip occurred. No magnitude gate —
	// the table may have physically moved between power cycles AND the sign may
	// have flipped simultaneously. The old 15% magnitude gate blocked the
	// correction in exactly that case (operator parks table at any position).
	ratioDiff := math.Abs((bootMag/homeMag) - 1.0)
	logger.Info("InitAposCorrection: checking sign",
		"drive=", driveName,
		"bootApos=", bootApos,
		"HomingApos=", homingApos,
		"ratioDiff%=", fmt.Sprintf("%.2f%%", ratioDiff*100),
	)

	if bootApos > 0 {
		signFlipActive.Store(true)
		logger.Info("InitAposCorrection: sign flip confirmed — negating apos transparently",
			"drive=", driveName,
			"bootApos=", bootApos,
			"HomingApos=", homingApos,
			"ratioDiff%=", fmt.Sprintf("%.2f%%", ratioDiff*100),
		)
	} else {
		logger.Info("InitAposCorrection: same sign — no correction needed",
			"drive=", driveName,
			"bootApos=", bootApos,
			"HomingApos=", homingApos,
		)
	}
}

// getPulsesFromDegree returns the pulse count to send to the drive.
func getPulsesFromDegree(masterDevice MasterDevice, degree float64) int64 {
	pulse := float64(masterDevice.Device.DriveXRatio) * degree
	pulseForDisp := fmt.Sprintf("%f", pulse)
	logger.Debug("degree:", degree, "pulse:", pulseForDisp,
		"integer:", int32(pulse), "driveXRatio:", masterDevice.Device.DriveXRatio)
	return int64(pulse)
}

// currentPosition converts encoder pulses → degrees [0, 360).
// pos must come from getRawApos() — sign-flip correction is applied there.
//
// After our sign-flip architecture change, getRawApos() ALWAYS returns a
// value in the NEGATIVE coordinate space when the table is near zero
// (e.g. -554431729). This is consistent across all boots.
//
// However, the customer's HomingOffset (e.g. 1.587°) was originally set
// when the old code returned POSITIVE values for the same position. So the
// stored HomingOffset has the opposite sign for our new coordinate space.
// We negate it here to compensate. The customer never needs to change their
// HomingOffset setting.
func currentPosition(pos int32, driveXRatio int, driveName string) (float64, float64) {
	driverSettings := settings.GetDriverSettings(driveName)

	// Negate HomingOffset because getRawApos() now returns normalized
	// negative values, but the customer calibrated when pos was positive.
	homingOffset := -driverSettings.HomingOffset

	driveOffset := homingOffset * float32(driveXRatio)
	drivePosition := float64(pos) - float64(driveOffset)
	drivePosition = drivePosition / float64(driveXRatio)

	// Double mod guarantees [0, 360) for any sign/magnitude of input.
	drivePosition = math.Mod(math.Mod(drivePosition, 360)+360, 360)
	if drivePosition >= 359.999 {
		drivePosition = 0
	}

	drivePositionWithErrorCorrection := drivePosition
	return helper.RoundFloat(drivePosition, 3),
		helper.RoundFloat(drivePositionWithErrorCorrection, 3)
}

func getAbsolutePosition(currentPos float64, targetPos float64, useShortestPath bool) (float64, float64) {
	return helper.GetAbsolutePosition(currentPos, targetPos, useShortestPath)
}

func getRelativePosition(currentPos float64, targetPos float64, prevDestinationAngle float64) (float64, float64) {
	if math.Abs(currentPos-prevDestinationAngle) > 1.0 {
		prevDestinationAngle = currentPos
	}
	return helper.GetRelativePosition(currentPos, targetPos, prevDestinationAngle)
}

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