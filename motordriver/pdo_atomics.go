package motordriver

import "sync/atomic"

// pdo_atomics.go — all PDO atomic variables shared across the motordriver package.
// Declared here (no cgo) so all files can reference them without depending
// on ether_cat_gateway.go's cgo build.
//
// Written exclusively by the PDO cyclic goroutine in ether_cat_gateway.go.
// Read by drive_rotation.go, reset.go, poll_drive_position.go, ecs.go, etc.

// --- Feedback (written by cyclic, read by callers) ---
var pdoFbActual    atomic.Int32   // 0x6064 actual position
var pdoFbStatus    atomic.Uint32  // 0x6041 status word
var pdoFbErrCode   atomic.Uint32  // 0x603F error code
var pdoFbDigInputs atomic.Uint32  // 0x4F25 manufacturer digital inputs
var pdoFbMode      atomic.Int32   // 0x6061 mode of operation display

// --- Commands (written by callers, read by cyclic) ---
var pdoCmdMode     atomic.Int32   // 0x6060 mode of operation (1=PP, 3=PV)
var pdoCmdTarget   atomic.Int32   // 0x607A target position (PP mode)
var pdoCmdVelocity atomic.Int32   // 0x60FF target velocity (PV mode)
var pdoFinishSub1  atomic.Uint32  // 0x60FE:01 digital output sub1
var pdoFinishSub2  atomic.Uint32  // 0x60FE:02 digital output sub2

// --- CiA402 control ---
var pdoEnableRequested atomic.Bool  // true = request Operation Enabled
var pdoJogActive       atomic.Bool  // true = PP jog toggle running
var pdoJogStep         atomic.Int32 // PP jog step counter
var pdoNewSetPoint     atomic.Bool  // one-shot: assert bit4 next cycle

// --- Fault reset state machine ---
var pdoResetState  atomic.Int32 // 0=idle, 1-4=in progress
var pdoResetCycles atomic.Int32 // cycle counter for reset pulse timing
var pdoDomainValid atomic.Bool  // true once first valid WC=COMPLETE frame received

// --- EtherCAT AL state ---
var pdoAlState atomic.Uint32 // current AL state: 1=INIT 2=PREOP 4=SAFEOP 8=OP

// --- Immediate stop ---
var pdoStopRequest atomic.Bool
var pdoStopUntil   atomic.Int64