use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Raw, mutex::Mutex, signal::Signal};

#[derive(Clone, Copy, Debug, Default)]
pub struct PoseAbs {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub roll: f32,  // rad
    pub pitch: f32, // rad
    pub yaw: f32,   // rad
}

// ====== EVENT DEFINITIONS ========
pub static FIRST_MESSAGE: Signal<Raw, ()> = Signal::new();
pub static START_EVENT: Signal<Raw, ()> = Signal::new();

// ====== STATE (PUSHED EACH TIME A NEW FRAME COMES IN) ======
pub static STATE_SIG: Signal<Raw, PoseAbs> = Signal::new();

// ====== LAST STATE =======
pub static LAST_STATE: Mutex<Raw, PoseAbs> = Mutex::new(PoseAbs {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
});
