package motordriver

/*
#cgo CFLAGS: -g -Wall -I/opt/etherlab/include -I/home/pi/gosrc/src/EtherCAT
#cgo LDFLAGS: -L/home/pi/gosrc/src/EtherCAT -L/opt/etherlab/lib/ -lethercatinterface -lethercat
#include "ecrt.h"
#include "ethercatinterface.h"

// ---------------- PDO CONFIG ----------------
// RxPDO 0x1600 (6 entries): 6040, 6060, 607A, 60FF, 60FE:01, 60FE:02
// TxPDO 0x1A00 (8 entries): 603F, 6041, 6061, 6064, 60B9, 60BA, 60F4, 4F25

static ec_pdo_entry_info_t slave_pdo_entries[] = {

    // ---- RxPDO 0x1600 ----
    {0x6040, 0x00, 16},
    {0x6060, 0x00,  8},
    {0x607A, 0x00, 32},
    {0x60FF, 0x00, 32},
    {0x60FE, 0x01, 32},
    {0x60FE, 0x02, 32},

    // ---- TxPDO 0x1A00 ----
    {0x603F, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6061, 0x00,  8},
    {0x6064, 0x00, 32},
    {0x60B9, 0x00, 16},
    {0x60BA, 0x00, 32},
    {0x60F4, 0x00, 32},
    {0x4F25, 0x00, 32},
};

static ec_pdo_info_t slave_pdos[] = {
    {0x1600, 6, slave_pdo_entries + 0},
    {0x1A00, 8, slave_pdo_entries + 6},
};

static ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,            EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,            EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0,  EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, slave_pdos + 1,  EC_WD_DISABLE},
    {0xff}
};

static inline const ec_sync_info_t* get_slave_syncs(void) { return slave_syncs; }

// ---------------- PDO OFFSETS ----------------

// RxPDO — master → drive
unsigned int offControl     = 0;   // 6040:00  Controlword        (U16)
unsigned int offMode        = 0;   // 6060:00  Mode of operation  (S8)
unsigned int offModeDisp    = 0;   // 6061:00  Mode of operation display (S8, TxPDO feedback)
unsigned int offTarget      = 0;   // 607A:00  Target position    (S32)
unsigned int offVelocity    = 0;   // 60FF:00  Target velocity    (S32)
unsigned int offFinishSub1  = 0;   // 60FE:01  Digital outputs physical (U32)
unsigned int offFinishSub2  = 0;   // 60FE:02  Digital outputs bitmask  (U32)

// TxPDO — drive → master
unsigned int offErrCode     = 0;   // 603F:00  Error code         (U16)
unsigned int offStatus      = 0;   // 6041:00  Status word        (U16)
unsigned int offActual      = 0;   // 6064:00  Actual position    (S32)
unsigned int offFbDigInputs = 0;   // 4F25:00  Mfg digital inputs (U32) ECS/POT/NOT/HOME/ALMIN/HRes/CL/DCL

const ec_pdo_entry_reg_t domain_regs[] = {
    {0,0,0x0000066f,0x60380008,0x6040,0x00,&offControl},
    {0,0,0x0000066f,0x60380008,0x6060,0x00,&offMode},
    {0,0,0x0000066f,0x60380008,0x6061,0x00,&offModeDisp},
    {0,0,0x0000066f,0x60380008,0x607A,0x00,&offTarget},
    {0,0,0x0000066f,0x60380008,0x60FF,0x00,&offVelocity},
    {0,0,0x0000066f,0x60380008,0x60FE,0x01,&offFinishSub1},
    {0,0,0x0000066f,0x60380008,0x60FE,0x02,&offFinishSub2},
    {0,0,0x0000066f,0x60380008,0x603F,0x00,&offErrCode},
    {0,0,0x0000066f,0x60380008,0x6041,0x00,&offStatus},
    {0,0,0x0000066f,0x60380008,0x6064,0x00,&offActual},
    {0,0,0x0000066f,0x60380008,0x4F25,0x00,&offFbDigInputs},
    {}
};

static inline int32_t  rd_s32(uint8_t *pd,unsigned int off){return EC_READ_S32(pd+off);}
static inline uint32_t rd_u32(uint8_t *pd,unsigned int off){return EC_READ_U32(pd+off);}
static inline uint16_t rd_u16(uint8_t *pd,unsigned int off){return EC_READ_U16(pd+off);}
static inline void wr_s32(uint8_t *pd,unsigned int off,int32_t  v){EC_WRITE_S32(pd+off,v);}
static inline void wr_u32(uint8_t *pd,unsigned int off,uint32_t v){EC_WRITE_U32(pd+off,v);}
static inline void wr_u16(uint8_t *pd,unsigned int off,uint16_t v){EC_WRITE_U16(pd+off,v);}
static inline void wr_s8 (uint8_t *pd,unsigned int off,int8_t   v){EC_WRITE_S8 (pd+off,v);}
static inline uint8_t  rd_u8 (uint8_t *pd,unsigned int off){return EC_READ_U8(pd+off);}
static inline uint8_t  get_al_states(const ec_master_state_t *ms){return ms->al_states;}

// get_domain_wc returns the domain working counter state.
// wc_state == EC_WC_COMPLETE (2) means all slaves responded with valid data.
// Reading PDO data before wc_state is COMPLETE gives stale/zero values —
// this is the root cause of wrong position on first boot of the day.
static inline int get_domain_wc_state(ec_domain_t *d) {
    ec_domain_state_t ds;
    ecrt_domain_state(d, &ds);
    return (int)ds.wc_state;
}

*/
import "C"

import (
	ethercatDevice "EtherCAT/ethercatdevicedatatypes"
	"EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"errors"
	"fmt"
	"runtime"
	"sync"
	"time"
	"unsafe"
)

var domain   *C.ec_domain_t
var domainPD *C.uint8_t
var sdoMutex sync.Mutex

// --------------------------------------------------------------------
// MasterDevice STRUCT
// --------------------------------------------------------------------
type MasterDevice struct {
	Master   *C.ec_master_t
	Position int
	Name     string
	Device   ethercatDevice.Device
}

// PDO atomics declared in pdo_atomics.go (no cgo dependency).

var pdoStopChan chan struct{}
var pdoLogChan  chan string // non-blocking log relay from cyclic to disk writer

const (
	// CiA402 modes
	cia402ModeProfilePosition = 1
	cia402ModeProfileVelocity = 3

	// CiA402 controlwords
	cwFaultReset = 0x0086   // bit7(fault reset) + bits1,2(shutdown) — Panasonic A6 requires lower bits maintained during fault reset pulse
	cwShutdown   = 0x0006
	cwSwitchOn   = 0x0007
	cwEnableOp   = 0x000F

	// PP helpers
	cwNewSetPoint = 1 << 4

	// 4F25 digital input bit positions
	// Bit 0: ECS       (active high)
	// Bit 1: POT       (active low)
	// Bit 2: NOT       (active low)
	// Bit 3: HOME      (active low)
	// Bit 4: ALMIN     (active low)
	// Bit 5: HardReset (active high)
	// Bit 6: CL        (active high)
	// Bit 7: DCL       (active high)
	digInECSBit      = 0
	digInPOTBit      = 1
	digInNOTBit      = 2
	digInHOMEBit     = 3
	digInALMINBit    = 4
	digInHardResBit  = 5
	digInCLBit       = 6
	digInDCLBit      = 7

	// 60FE:01 finish signal bit (bit 16 = output pin 17)
	finSignalBit = uint32(1 << 16)
)

// --------------------------------------------------------------------
// RequestMaster — configure PDO mapping and register offsets
// --------------------------------------------------------------------
func RequestMaster(device ethercatDevice.Device) (*C.ec_master_t, error) {

	master := C.ecrt_request_master(C.uint(device.ID))
	if master == nil {
		return nil, errors.New("unable to find master")
	}

	domain = C.ecrt_master_create_domain(master)
	if domain == nil {
		return nil, errors.New("failed to create PDO domain")
	}

	sc := C.ecrt_master_slave_config(
		master,
		C.ushort(device.Alias),
		C.ushort(device.ID),
		C.uint(device.VendorID),
		C.uint(device.ProductCode),
	)
	if sc == nil {
		return nil, errors.New("failed to get slave config")
	}
	if C.ecrt_slave_config_pdos(sc, C.EC_END, C.get_slave_syncs()) != 0 {
		return nil, errors.New("failed to configure slave PDOs (sync managers)")
	}

	if C.ecrt_domain_reg_pdo_entry_list(domain, &C.domain_regs[0]) != 0 {
		return nil, errors.New("PDO entry registration failed")
	}

	logger.Info("PDO configured for ", device.Name)
	return master, nil
}

// --------------------------------------------------------------------
// ActivatePdoForMaster — activate master and start cyclic loop
// --------------------------------------------------------------------
func ActivatePdoForMaster(master *C.ec_master_t) error {

	if C.ecrt_master_activate(master) != 0 {
		return errors.New("failed to activate master for PDO")
	}

	domainPD = C.ecrt_domain_data(domain)
	if domainPD == nil {
		return errors.New("failed to get domain process data pointer")
	}

	pdoCmdMode.Store(cia402ModeProfilePosition)
	pdoCmdTarget.Store(0)
	pdoCmdVelocity.Store(0)
	pdoEnableRequested.Store(true)

	startPdoCyclic(master, 2*time.Millisecond)

	// Wait for slave to reach OP state before returning.
	// After ecrt_master_activate the EtherLab kernel module negotiates
	// INIT→PREOP→SAFEOP→OP with the slave. On a Pi this takes 200-800ms.
	// If we return immediately, the AL monitor fires at t+2s and sees PREOP,
	// then spams "re-requesting OP" every 2s with no effect.
	// Waiting here ensures the boot fault check and SDO config run only
	// after the slave is ready to accept SDO commands (requires PREOP+).
	logger.Info("PDO activated + cyclic started — waiting for AL=OP")
	for i := 0; i < 500; i++ {
		time.Sleep(20 * time.Millisecond)
		var ms C.ec_master_state_t
		C.ecrt_master_state(master, &ms)
		alState := uint8(C.get_al_states(&ms))
		if alState == 0x08 {
			logger.Info("AL reached OP state after ", (i+1)*20, "ms")
			break
		}
		if i > 0 && i%100 == 0 {
			logger.Info("AL still not OP after ", (i+1)*20, "ms — al_states=", alState, " waiting...")
		}
		if i == 499 {
			logger.Error("AL did not reach OP within 10s — al_states=", alState)
		}
	}
	
	return nil
}

// --------------------------------------------------------------------
// CiA402 state machine helper
// --------------------------------------------------------------------
func cia402ControlwordFromStatus(stw uint16, enable bool) (uint16, bool) {
	// Mask bits 0-5 and bit6 only — ignore bit12 (set-point ack) and other
	// status-only bits that do not affect PDS state classification.
	// CiA402 PDS state is encoded in bits: 0,1,2,3,5,6 only.
	state := stw & 0x006F

	if !enable {
		return cwShutdown, false
	}

	switch state {
	case 0x0040, 0x0060: // switch on disabled (bit6=1; bit5 may vary)
		// 0x0060 = bit5+bit6 set — same PDS state, bit12 or other flags set in stw
		// This was the unhandled case causing permanent cwShutdown loop (stw=0x1670)
		return cwShutdown, false
	case 0x0021: // ready to switch on
		return cwSwitchOn, false
	case 0x0023: // switched on
		return cwEnableOp, false
	case 0x0027: // operation enabled
		return cwEnableOp, true
	default:
		// Unknown/transitional state — send shutdown and log via PDO status log
		return cwShutdown, false
	}
}

// --------------------------------------------------------------------
// PDO cyclic goroutine — 1ms tick
// --------------------------------------------------------------------
func startPdoCyclic(master *C.ec_master_t, cycle time.Duration) {

	stopPdoCyclic()
	pdoStopChan = make(chan struct{})
	pdoLogChan  = make(chan string, 64) // buffer 64 lines; cyclic never blocks
	go startPdoLogger(pdoLogChan, pdoStopChan)

	go func() {
		runtime.LockOSThread() // pin this goroutine to its OS thread for stable scheduling

		seeded   := false
		toggle   := false
		lastLog  := time.Now()
		lastAL   := time.Now()

		for {
			cycleStart := time.Now()

			// Check stop channel without blocking
			select {
			case <-pdoStopChan:
				logger.Info("Stopping PDO cyclic")
				return
			default:
			}

			C.ecrt_master_receive(master)
			C.ecrt_domain_process(domain)

			// AL state check every 2s — detect slave dropping out of OP due to timing jitter.
			// On non-RT Linux (Pi), scheduler pauses can cause frame loss → slave leaves OP.
			// Re-requesting OP here allows self-recovery without a full system reset.
			if time.Since(lastAL) >= 2*time.Second {
				lastAL = time.Now()
				var ms C.ec_master_state_t
				C.ecrt_master_state(master, &ms)
				alStates := uint8(C.get_al_states(&ms))
				pdoAlState.Store(uint32(alStates))
				if alStates != 0x08 { // 0x08 = EC_AL_STATE_OP
					// LOG ONLY. ecrt_master_activate is only valid before activation —
					// calling it post-activation does nothing and spams the log.
					// Runtime ESM recovery (SAFEOP/PREOP) requires the EtherLab kernel
					// module to re-negotiate the slave state — this happens automatically
					// when the master keeps sending frames. The PDS state machine will
					// re-enable the drive once AL returns to OP.
					logger.Error("AL state not OP, al_states=", alStates)
					// Ensure re-enable is armed so when AL recovers the drive proceeds.
					pdoEnableRequested.Store(true)
				}
			}

				// Guard: only trust PDO data when domain working counter is COMPLETE.
				// EC_WC_COMPLETE=2 means all slaves responded with valid data this cycle.
				// On first boot of the day, the EtherCAT link takes several cycles to
				// reach OP — during that time domainPD contains zeros or stale values
				// from the previous session. Reading position before WC is complete
				// is the root cause of wrong zero position shown on UI at first boot.
				wcState := int(C.get_domain_wc_state(domain))
				if wcState != 2 { // 2 = EC_WC_COMPLETE
					// Domain not ready — skip this cycle, do not update atomics.
					// Position listeners will hold their last value (or zero if first boot).
					C.ecrt_domain_queue(domain)
					C.ecrt_master_send(master)
					continue
				}

				// Read TxPDO feedback — domain is valid this cycle
				actual    := int32(C.rd_s32(domainPD, C.offActual))
				stw       := uint16(C.rd_u16(domainPD, C.offStatus))
				errCode   := uint16(C.rd_u16(domainPD, C.offErrCode))
				modeDisp  := int8(C.rd_u8(domainPD, C.offModeDisp))  // 6061 is S8 (8-bit PDO entry)
				digInputs := uint32(C.rd_u32(domainPD, C.offFbDigInputs))

				// Store feedback — safe for any goroutine to read
				pdoFbActual.Store(actual)
				pdoFbStatus.Store(uint32(stw))
				pdoFbErrCode.Store(uint32(errCode))
				pdoFbMode.Store(int32(modeDisp))
				pdoFbDigInputs.Store(digInputs)

				// Seed position target from actual on first valid cycle only
				if !seeded {
					pdoCmdTarget.Store(actual)
					seeded = true
					pdoDomainValid.Store(true) // signal poll_drive_position it is safe to read
					pdoAlState.Store(0x08)      // assume OP on first valid frame
					// Log raw apos AND the settings used for position calculation.
					// If HomingOffset is wrong at this moment, the displayed position
					// will be wrong. Check this log line when field issue occurs.
					for _, dev := range masterDevices {
						ds := settings.GetDriverSettings(dev.Name)
						logger.Info("PDO domain valid — first valid position frame:",
							"raw_apos=", actual,
							"HomingOffset=", ds.HomingOffset,
							"DriveXRatio=", dev.Device.DriveXRatio,
						)
					}
				}

				// ---- FAULT RESET STATE MACHINE ----
				// Triggered by pdoResetState.CompareAndSwap(0,1) from ResetDriver().
				// The CAS in ResetDriver() prevents concurrent goroutines from
				// overwriting state mid-sequence (the bug that caused timeout loops).
				// States progress 1→2→3→4→0; only state=0 can be set externally.
				resetState := pdoResetState.Load()
				if resetState > 0 {
					switch resetState {
					case 1:
						C.wr_u16(domainPD, C.offControl, C.uint16_t(cwFaultReset))
						C.ecrt_domain_queue(domain)
						C.ecrt_master_send(master)
						pdoResetCycles.Store(0)
						pdoResetState.Store(2)

					case 2:
						// Hold fault reset pulse for 150ms (150 × 1ms cycles)
						C.wr_u16(domainPD, C.offControl, C.uint16_t(cwFaultReset))
						C.ecrt_domain_queue(domain)
						C.ecrt_master_send(master)
						if pdoResetCycles.Add(1) >= 75 {  // 75 × 2ms = 150ms pulse
							pdoResetState.Store(3)
						}

					case 3:
						// Release fault reset bit — send Shutdown
						C.wr_u16(domainPD, C.offControl, C.uint16_t(cwShutdown))
						C.ecrt_domain_queue(domain)
						C.ecrt_master_send(master)
						pdoResetCycles.Store(0) // reset counter so state4 gets a fresh 500ms window
						pdoResetState.Store(4)

					case 4:
						// Wait for fault bit (stw bit3) to clear — counter reset in state3.
						if (stw & 0x0008) == 0 {
							logger.Info("Fault cleared via PDO reset")
							pdoEnableRequested.Store(true)
							pdoResetState.Store(0)
						} else if pdoResetCycles.Add(1) >= 250 { // 250 × 2ms = 500ms
							logger.Info("State4 timeout: fault bit still set after 500ms, forcing exit")
							pdoEnableRequested.Store(true)
							pdoResetState.Store(0)
						} else {
							C.wr_u16(domainPD, C.offControl, C.uint16_t(cwShutdown))
							C.ecrt_domain_queue(domain)
							C.ecrt_master_send(master)
						}
					}
					continue // skip normal outputs while reset is in progress
				}

				// ---- NORMAL CiA402 STATE MACHINE ----
				baseCw, opEnabled := cia402ControlwordFromStatus(stw, pdoEnableRequested.Load())

				// Immediate stop: clear bit2 (QuickStop) without dropping OP
				now := time.Now().UnixNano()
				if pdoStopRequest.Load() && now < pdoStopUntil.Load() {
					baseCw &^= 0x0004
					pdoCmdVelocity.Store(0)
				} else if pdoStopRequest.Load() && now >= pdoStopUntil.Load() {
					pdoStopRequest.Store(false)
					pdoCmdVelocity.Store(0)
				}

				mode := int8(pdoCmdMode.Load())

				// PP continuous jog stepping (pdoJogActive)
				if opEnabled && pdoJogActive.Load() && mode == cia402ModeProfilePosition {
					pdoCmdTarget.Store(pdoCmdTarget.Load() + pdoJogStep.Load())
					toggle = !toggle
				}

				target := int32(pdoCmdTarget.Load())
				vel    := int32(pdoCmdVelocity.Load())

				cw := baseCw

				// PP continuous jog: toggle bit4 every cycle to step the drive
				if opEnabled && pdoJogActive.Load() && mode == cia402ModeProfilePosition {
					if toggle {
						cw |= cwNewSetPoint
					} else {
						cw &^= cwNewSetPoint
					}
				}

				// PP new set-point handshake (CiA402 §6.3.2):
				//   1. Hold bit4=1 until drive acknowledges with stw bit12=1
				//   2. Clear bit4 — drive then clears bit12, drops bit10, starts moving
				// One-shot (fire for 1ms only) is unreliable — a single dropped EtherCAT
				// frame causes the drive to miss the set-point entirely (bit10 stays set,
				// drive never moves). Holding until bit12 guarantees acknowledgement.
				// Only assert when opEnabled — bit4 on a non-OP controlword is invalid.
				const stwSetPointAck = uint16(1 << 12)
				if pdoNewSetPoint.Load() {
					if opEnabled {
						if (stw & stwSetPointAck) != 0 {
							// Drive acknowledged — clear bit4, handshake complete
							pdoNewSetPoint.Store(false)
							// cw bit4 left clear this cycle to complete the handshake
						} else {
							// Still waiting for ack — hold bit4
							cw |= cwNewSetPoint
						}
					}
					// else not opEnabled yet — leave flag set, retry next cycle
				}

				// Write RxPDO
				C.wr_u16(domainPD, C.offControl,    C.uint16_t(cw))
				C.wr_s8 (domainPD, C.offMode,        C.int8_t(mode))
				C.wr_s32(domainPD, C.offTarget,      C.int32_t(target))
				C.wr_s32(domainPD, C.offVelocity,    C.int32_t(vel))
				C.wr_u32(domainPD, C.offFinishSub1,  C.uint32_t(pdoFinishSub1.Load()))
				C.wr_u32(domainPD, C.offFinishSub2,  C.uint32_t(pdoFinishSub2.Load()))

				C.ecrt_domain_queue(domain)
				C.ecrt_master_send(master)

				// 500ms diagnostic log — sent via non-blocking channel to avoid
				// SD card write latency inside the RT cyclic goroutine.
				// A dedicated goroutine (startPdoLogger) drains this channel and
				// writes to disk. If the channel is full the log line is dropped
				// (non-blocking select) — acceptable for diagnostics.
				if time.Since(lastLog) >= 500*time.Millisecond {
					lastLog = time.Now()
					msg := fmt.Sprintf(
						"[PDO stw=0x %04X  err=0x %04X  digIn=0x %08X  opEn= %v  apos= %d  tpos= %d  vel= %d  cw=0x %04X  mode= %d  jogPP= %v  step= %d]",
						stw, errCode, digInputs, opEnabled, actual, target, vel, cw, int(mode),
						pdoJogActive.Load(), pdoJogStep.Load(),
					)
					select {
					case pdoLogChan <- msg:
					default: // channel full — drop, never block cyclic
					}
				}

				// Jitter monitor — sent via channel too so overrun log itself
				// cannot cause the next overrun.
				elapsed := time.Since(cycleStart)
				if elapsed > 3*cycle {
					select {
					case pdoLogChan <- fmt.Sprintf("[PDO overrun: %v]", elapsed.Round(time.Millisecond)):
					default:
					}
				}

				// Compensated sleep — accounts for time spent in this iteration.
				// Avoids ticker drift/bunching on non-RT Linux.
				if sleep := cycle - elapsed; sleep > 0 {
					time.Sleep(sleep)
				}
		}
	}()
}

func stopPdoCyclic() {
	if pdoStopChan != nil {
		select {
		case <-pdoStopChan:
		default:
			close(pdoStopChan)
		}
		pdoStopChan = nil
	}
}

// --------------------------------------------------------------------
// PDO helper API — called by other modules
// --------------------------------------------------------------------

// GetDigitalInputs4F25 returns the latest 4F25 manufacturer input word.
// Bit layout: 0=ECS, 1=POT, 2=NOT, 3=HOME, 4=ALMIN, 5=HardReset, 6=CL, 7=DCL
func GetDigitalInputs4F25() uint32 {
	return pdoFbDigInputs.Load()
}

// GetPdoErrCode returns the latest 603F error code read via PDO.
func GetPdoErrCode() uint32 {
	return pdoFbErrCode.Load()
}

// PdoSetFinishSignal asserts the finish signal on 60FE:01/02 (bit 16).
// Takes effect on the next 1ms cycle.
func PdoSetFinishSignal() {
	pdoFinishSub1.Store(finSignalBit)
	pdoFinishSub2.Store(finSignalBit)
}

// PdoClearFinishSignal de-asserts the finish signal on 60FE:01/02.
func PdoClearFinishSignal() {
	pdoFinishSub1.Store(0)
	pdoFinishSub2.Store(0)
}

// PdoResetIfFault checks for a fault condition and starts the PDO
// fault reset state machine if one is present.
func PdoResetIfFault() {
	stw := uint16(pdoFbStatus.Load())
	if (stw & 0x0008) != 0 {
		logger.Info("Fault detected — starting PDO reset state machine")
		pdoEnableRequested.Store(false)
		// CAS: only start if idle, don't interrupt a running sequence
		if pdoResetState.CompareAndSwap(0, 1) {
			pdoResetCycles.Store(0)
		}
	} else {
		logger.Info("Reset requested but no fault present (stw=0x", fmt.Sprintf("%04X", stw), ")")
	}
}

func SDODownload(master *C.ec_master_t, position int, step ethercatDevice.Step) error {

	sdoMutex.Lock()
	defer sdoMutex.Unlock()

	valueToWrite, errValue := step.GetValue()
	if errValue != nil {
		return errValue
	}

	abortCode := 0
	dataSize := getSize(step)

	logger.Trace("ethercatGateway->", step.Name, "Addr:", fmt.Sprintf("%X", step.Address),
		"Indx:", fmt.Sprintf("%X", step.SubIndex), "Val:", valueToWrite)

	var uploadErr error

	for retry := 0; retry < 5; retry++ {
		result := C.sdo_download(
			master,
			(C.uint16_t)(position),
			(C.uint16_t)(step.Address),
			(C.uint8_t)(step.SubIndex),
			(*C.uint8_t)(unsafe.Pointer(&valueToWrite)),
			dataSize,
			(*C.uint32_t)(unsafe.Pointer(&abortCode)),
		)

		if result >= 0 {
			uploadErr = nil
			break
		}

		uploadErr = errors.New("error sending command to driver, step " + step.Name)
		time.Sleep(2 * time.Millisecond)
		logger.Info("retry sending command", step.Name, "retry", retry)
	}

	if step.Delay > 0 {
		time.Sleep(time.Duration(step.Delay) * time.Microsecond)
	}

	if uploadErr != nil {
		logger.Error(uploadErr)
		statusnotifier.Alarm("Error sending command to driver, step " + step.Name)
	}

	return uploadErr
}

func SDOUpload(master *C.ec_master_t, position int, step ethercatDevice.Step) (int32, error) {

	sdoMutex.Lock()
	defer sdoMutex.Unlock()

	valueToWrite, errValue := step.GetValue()
	if errValue != nil {
		return 0, errValue
	}

	abortCode := 0
	dataSize  := getSize(step)
	toPass    := C.uint8_t(valueToWrite)

	C.sdo_upload(
		master,
		(C.uint16_t)(position),
		(C.uint16_t)(step.Address),
		(C.uint8_t)(step.SubIndex),
		(*C.uint8_t)(&toPass),
		dataSize,
		(*C.uint32_t)(unsafe.Pointer(&abortCode)),
	)

	if step.Delay > 0 {
		time.Sleep(time.Duration(step.Delay) * time.Microsecond)
	}

	return int32(toPass), nil
}

func DrivePosition(master *C.ec_master_t, position int, step ethercatDevice.Step) (int32, error) {

	sdoMutex.Lock()
	defer sdoMutex.Unlock()

	valueToWrite, errValue := step.GetValue()
	if errValue != nil {
		return 0, errValue
	}

	toPass := C.uint8_t(valueToWrite)
	pos := C.drivePosition(
		master,
		(C.uint16_t)(position),
		(C.uint16_t)(step.Address),
		(C.uint8_t)(step.SubIndex),
		(C.uint8_t)(toPass),
	)

	return int32(pos), nil
}

func SDOUpload2(master *C.ec_master_t, position int, step ethercatDevice.Step) (int, error) {

	sdoMutex.Lock()
	defer sdoMutex.Unlock()

	valueToWrite, errValue := step.GetValue()
	if errValue != nil {
		return 0, errValue
	}

	toPass   := C.uint8_t(valueToWrite)
	dataSize := getSize(step)

	pos := C.sdo_upload2(
		master,
		(C.uint16_t)(position),
		(C.uint16_t)(step.Address),
		(C.uint8_t)(step.SubIndex),
		(C.uint8_t)(toPass),
		dataSize,
	)

	if step.Delay > 0 {
		time.Sleep(time.Duration(step.Delay) * time.Microsecond)
	}

	return int(pos), nil
}

func getSize(step ethercatDevice.Step) C.size_t {
	switch step.DataType {
	case "U32":  return C.uint32Size()
	case "U8":   return C.uint8Size()
	case "U16":  return C.uint16Size()
	case "UINT": return C.unintSize()
	case "I16":  return C.int16Size()
	case "I32":  return C.int32Size()
	case "I8":   return C.int8Size()
	}
	return C.uint8Size()
}

// startPdoLogger drains pdoLogChan and writes each message via logger.Info.
// Running in a separate goroutine keeps all disk I/O (SD card writes) off the
// RT cyclic goroutine, eliminating the main source of PDO watchdog timeouts.
// It exits when pdoStopChan is closed.
func startPdoLogger(ch <-chan string, stop <-chan struct{}) {
	for {
		select {
		case msg, ok := <-ch:
			if !ok {
				return
			}
			logger.Info(msg)
		case <-stop:
			// Drain remaining messages before exit
			for {
				select {
				case msg := <-ch:
					logger.Info(msg)
				default:
					return
				}
			}
		}
	}

}
	// ReleaseMaster calls ecrt_release_master to gracefully return the slave
// to PREOP/INIT state. Must be called after stopPdoCyclic().
// This prevents the drive from latching a watchdog fault on next boot.
func ReleaseMaster(master *C.ec_master_t) {
	if master != nil {
			C.ecrt_release_master(master)
			logger.Info("EtherCAT master released")
	}
}
