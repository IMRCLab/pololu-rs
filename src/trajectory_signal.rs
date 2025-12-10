//! Trajectory signals and events
//!
//! Note: WheelCmd and WHEEL_CMD_CH are now in robotstate.rs
//! This module only contains trajectory-specific signals and events.

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as Raw;
use embassy_sync::signal::Signal;

// Re-export from robotstate for backwards compatibility
pub use crate::robotstate::{Pose, WHEEL_CMD_CH, WheelCmd, send_wheel_cmd, try_recv_wheel_cmd};

// ====== LEGACY TYPE ALIAS ======
// PoseAbs is now just an alias to Pose from robotstate
pub type PoseAbs = crate::robotstate::Pose;

// ====== EVENT DEFINITIONS ========
pub static FIRST_MESSAGE: Signal<Raw, ()> = Signal::new();
pub static START_EVENT: Signal<Raw, ()> = Signal::new();
pub static TRAJECTORY_CONTROL_EVENT: Signal<Raw, bool> = Signal::new(); // true = start, false = stop

// ====== STATE SIGNAL (PUSHED EACH TIME A NEW FRAME COMES IN) ======
pub static STATE_SIG: Signal<Raw, PoseAbs> = Signal::new();

// ====== LAST STATE (legacy, use robotstate::POSE instead) ======
// Kept for backwards compatibility during migration
use embassy_sync::mutex::Mutex;
pub static LAST_STATE: Mutex<Raw, PoseAbs> = Mutex::new(PoseAbs::DEFAULT);
