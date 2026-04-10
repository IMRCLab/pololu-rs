use embassy_sync::channel::Channel;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Raw, signal::Signal};
use embassy_time::Instant;

use crate::robotstate::MocapPose;

// ====== EVENT DEFINITIONS ========
pub static FIRST_MESSAGE: Signal<Raw, ()> = Signal::new();
pub static START_EVENT: Signal<Raw, ()> = Signal::new();
pub static TRAJECTORY_CONTROL_EVENT: Signal<Raw, bool> = Signal::new(); // true = start, false = stop

// ====== STATE (PUSHED EACH TIME A NEW FRAME COMES IN) ======
pub static STATE_SIG: Signal<Raw, MocapPose> = Signal::new();

// ====== WHEEL COMMAND =======
#[derive(Copy, Clone)]
pub struct WheelCmd {
    pub omega_l: f32,
    pub omega_r: f32,
    pub stamp: Instant,
}

pub static WHEEL_CMD_CH: Channel<Raw, WheelCmd, 4> = Channel::new();
