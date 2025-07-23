#![allow(static_mut_refs)]

use embassy_rp::peripherals::{PIN_12, PIN_13, PIN_8, PIN_9, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

pub struct EncoderPair<'a> {
    pub encoder_left: PioEncoder<'a, PIO0, 0>,
    pub encoder_right: PioEncoder<'a, PIO0, 1>,
}

// Global Counters
static LEFT_CELL: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();
static RIGHT_CELL: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();
static mut ENCODER_LEFT: Option<&'static Mutex<NoopRawMutex, i32>> = None;
static mut ENCODER_RIGHT: Option<&'static Mutex<NoopRawMutex, i32>> = None;

impl<'a> EncoderPair<'a> {
    pub fn new(
        common: &mut Common<'a, PIO0>,
        sm0: StateMachine<'a, PIO0, 0>,
        sm1: StateMachine<'a, PIO0, 1>,
        pin_left_a: embassy_rp::Peri<'a, PIN_12>,
        pin_left_b: embassy_rp::Peri<'a, PIN_13>,
        pin_right_a: embassy_rp::Peri<'a, PIN_8>,
        pin_right_b: embassy_rp::Peri<'a, PIN_9>,
    ) -> Self {
        let prg = PioEncoderProgram::new(common);
        let encoder_left = PioEncoder::new(common, sm0, pin_left_a, pin_left_b, &prg);
        let encoder_right = PioEncoder::new(common, sm1, pin_right_a, pin_right_b, &prg);
        Self {
            encoder_left,
            encoder_right,
        }
    }
}

// Initialize encoder counters
pub fn init_encoder_counts() {
    let left = LEFT_CELL.init(Mutex::new(0));
    let right = RIGHT_CELL.init(Mutex::new(0));
    unsafe {
        ENCODER_LEFT = Some(left);
        ENCODER_RIGHT = Some(right);
    }
}

// Left Encoder Task
#[embassy_executor::task]
pub async fn encoder_left_task(mut encoder: PioEncoder<'static, PIO0, 0>) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        if let Some(enc) = unsafe { ENCODER_LEFT } {
            let mut guard = enc.lock().await;
            *guard += delta;
        }
    }
}

#[embassy_executor::task]
pub async fn encoder_right_task(mut encoder: PioEncoder<'static, PIO0, 1>) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        if let Some(enc) = unsafe { ENCODER_RIGHT } {
            let mut guard = enc.lock().await;
            *guard += delta;
        }
    }
}

// get RPM of the left motor
#[allow(unused_assignments)]
pub async fn get_left_rpm(ppr: u32, interval_ms: u64) -> Option<f32> {
    let mut last = 0;
    Timer::after(Duration::from_millis(interval_ms)).await;

    if let Some(enc) = unsafe { ENCODER_LEFT } {
        let guard = enc.lock().await;
        let now = *guard;
        let delta = now - last;
        last = now;

        let rps = delta as f32 / (ppr as f32 * 4.0) / (interval_ms as f32 / 1000.0);
        Some(rps * 60.0)
    } else {
        None
    }
}

// get RPM of the right motor
#[allow(unused_assignments)]
pub async fn get_right_rpm(ppr: u32, interval_ms: u64) -> Option<f32> {
    let mut last = 0;
    Timer::after(Duration::from_millis(interval_ms)).await;

    if let Some(enc) = unsafe { ENCODER_RIGHT } {
        let guard = enc.lock().await;
        let now = *guard;
        let delta = now - last;
        last = now;

        let rps = delta as f32 / (ppr as f32 * 4.0) / (interval_ms as f32 / 1000.0);
        Some(rps * 60.0)
    } else {
        None
    }
}

// fast position reading
pub fn read_left_count() -> i32 {
    unsafe { ENCODER_LEFT }
        .and_then(|v| v.try_lock().map(|g| *g).ok())
        .unwrap_or(0)
}

// fast position reading
pub fn read_right_count() -> i32 {
    unsafe { ENCODER_RIGHT }
        .and_then(|v| v.try_lock().map(|g| *g).ok())
        .unwrap_or(0)
}
