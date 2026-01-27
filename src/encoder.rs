use embassy_rp::peripherals::{PIN_8, PIN_9, PIN_12, PIN_13, PIO0};
use embassy_rp::pio::{Common, StateMachine};
// use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use crate::encoder_lib::{PioEncoder, PioEncoderProgram};
use crate::robot_parameters_default::robot_constants::*;

/// Encoder struct for both encoders
pub struct EncoderPair<'a> {
    pub encoder_left: PioEncoder<'a, PIO0, 0>,
    pub encoder_right: PioEncoder<'a, PIO0, 1>,
}

/// Pair Encoder Counters
pub struct EncoderCounters {
    pub left: &'static Mutex<NoopRawMutex, i32>,
    pub right: &'static Mutex<NoopRawMutex, i32>,
}

// Global static cells for pulse counters
static LEFT_COUNT: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();
static RIGHT_COUNT: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();

/// avoid using global counters
pub fn init_encoder_counts() -> EncoderCounters {
    let left = LEFT_COUNT.init(Mutex::new(0));
    let right = RIGHT_COUNT.init(Mutex::new(0));
    EncoderCounters { left, right }
}

// Wrap both encoders in one struct for convenience
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

// Lookup table for quadrature decoding
const LUT: [[i8; 4]; 4] = [
    /*prev\curr: 00, 01, 10, 11 */
    [0, 1, -1, 0], // 00 ->
    [-1, 0, 0, 1], // 01 ->
    [1, 0, 0, -1], // 10 ->
    [0, -1, 1, 0], // 11 ->
];

// Left encoder task (read AB and drain the FIFO)
#[embassy_executor::task]
pub async fn encoder_left_task(
    mut encoder: PioEncoder<'static, PIO0, 0>,
    counter: &'static Mutex<NoopRawMutex, i32>,
) {
    let mut prev = encoder.read_ab().await;
    loop {
        let mut delta: i32 = 0;
        let curr = encoder.read_ab().await;
        delta += LUT[prev as usize][curr as usize] as i32;
        prev = curr;

        // fast clean FIFO
        while let Some(c) = encoder.try_read_ab() {
            delta += LUT[prev as usize][c as usize] as i32;
            prev = c;
        }

        if delta != 0 {
            let mut g = counter.lock().await;
            *g = g.wrapping_add(delta * (MOTOR_DIRECTION_LEFT as i32));

            // defmt::info!("Left encoder count = {}", *g);
        }
    }
}

// Right encoder task (read AB and drain the FIFO)
#[embassy_executor::task]
pub async fn encoder_right_task(
    mut encoder: PioEncoder<'static, PIO0, 1>,
    counter: &'static Mutex<NoopRawMutex, i32>,
) {
    let mut prev = encoder.read_ab().await;
    loop {
        let mut delta: i32 = 0;
        let curr = encoder.read_ab().await;
        delta += LUT[prev as usize][curr as usize] as i32;
        prev = curr;

        while let Some(c) = encoder.try_read_ab() {
            delta += LUT[prev as usize][c as usize] as i32;
            prev = c;
        }

        if delta != 0 {
            let mut g = counter.lock().await;
            *g = g.wrapping_add(delta * (MOTOR_DIRECTION_RIGHT as i32));

            // defmt::info!("Right encoder count = {}", *g);
        }
    }
}

/// Asychronously acquiring left RPM
pub async fn get_left_rpm(
    counter: &'static Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    interval_ms: u64,
) -> f32 {
    let before = *counter.lock().await;
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = *counter.lock().await;

    let delta = after.wrapping_sub(before) as f32;
    let rps = delta / cpr_wheel / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

/// Asychronously acquiring right RPM
pub async fn get_right_rpm(
    counter: &'static Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    interval_ms: u64,
) -> f32 {
    let before = *counter.lock().await;
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = *counter.lock().await;

    let delta = after.wrapping_sub(before) as f32;
    let rps = delta / cpr_wheel / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

/// Asychronously acquiring both RPMs
pub async fn get_rpms(
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    interval_ms: u64,
) -> (f32, f32) {
    let left_before = *left_counter.lock().await;
    let right_before = *right_counter.lock().await;

    Timer::after(Duration::from_millis(interval_ms)).await;

    let left_after = *left_counter.lock().await;
    let right_after = *right_counter.lock().await;

    let delta_l = left_after.wrapping_sub(left_before) as f32;
    let delta_r = right_after.wrapping_sub(right_before) as f32;

    let win_s = interval_ms as f32 / 1000.0;
    let rps_l = delta_l / cpr_wheel / win_s;
    let rps_r = delta_r / cpr_wheel / win_s;

    (rps_l * 60.0, rps_r * 60.0)
}

/// Asychronously acquiring both wheel angular speeds in rad/s
/// The velocity in this func is acquired by waiting for interval_ms duration and
/// calculating the difference in counts (The counter is locked during that time)
pub async fn get_wheel_speed_in_rad(
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    interval_ms: u64,
    dt: f32, // equal to interval_ms but in second, just for saving one divide calculation
) -> (f32, f32) {
    let left_before = *left_counter.lock().await;
    let right_before = *right_counter.lock().await;

    Timer::after(Duration::from_millis(interval_ms)).await;

    let left_after = *left_counter.lock().await;
    let right_after = *right_counter.lock().await;

    let delta_l = left_after.wrapping_sub(left_before) as f32;
    let delta_r = right_after.wrapping_sub(right_before) as f32;

    // original rotation speed in rad/s
    let omega_l_raw = (delta_l as f32) * 2.0 * core::f32::consts::PI / (cpr_wheel * dt);
    let omega_r_raw = (delta_r as f32) * 2.0 * core::f32::consts::PI / (cpr_wheel * dt);

    (omega_l_raw, omega_r_raw)
}

/// Synchronous function to get wheel speeds from previous counts
/// The function needs the previous counts and dt to calculate the wheel speed in rad/s
/// (The coounter will be locked only briefly to read the current counts)
pub fn wheel_speed_from_counts_now(
    left_counter: &Mutex<NoopRawMutex, i32>,
    right_counter: &Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    prev_left: i32,
    prev_right: i32,
    dt: f32, // in sec
) -> ((f32, f32), (i32, i32)) {
    // lock as short as possible
    let (left_now, right_now) = {
        let l = *left_counter.try_lock().as_deref().unwrap_or(&0);
        let r = *right_counter.try_lock().as_deref().unwrap_or(&0);
        (l, r)
    };

    let delta_l = left_now.wrapping_sub(prev_left) as f32;
    let delta_r = right_now.wrapping_sub(prev_right) as f32;

    let k = 2.0 * core::f32::consts::PI / cpr_wheel;
    let omega_l = k * delta_l / dt;
    let omega_r = k * delta_r / dt;

    ((omega_l, omega_r), (left_now, right_now))
}
