use embassy_rp::peripherals::{PIN_8, PIN_9, PIN_12, PIN_13, PIO0};
use embassy_rp::pio::{Common, StateMachine};
// use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::{interrupt::typelevel::{Handler, PIO0_IRQ_0}, pac};
use embassy_time::{Duration, Timer};
use portable_atomic::{AtomicI32, Ordering};

use crate::encoder_lib::{PioEncoder, PioEncoderProgram};

/// Encoder struct for both encoders
pub struct EncoderPair<'a> {
    pub encoder_left: PioEncoder<'a, PIO0, 0>,
    pub encoder_right: PioEncoder<'a, PIO0, 1>,
}

/// Pair Encoder Counters
pub struct EncoderCounters {
    pub left: &'static AtomicI32,
    pub right: &'static AtomicI32,
}

static LEFT_COUNT: AtomicI32 = AtomicI32::new(0);
static RIGHT_COUNT: AtomicI32 = AtomicI32::new(0);
static LEFT_PREV_AB: AtomicI32 = AtomicI32::new(-1);
static RIGHT_PREV_AB: AtomicI32 = AtomicI32::new(-1);

/// avoid using global counters
pub fn init_encoder_counts() -> EncoderCounters {
    LEFT_COUNT.store(0, Ordering::Relaxed);
    RIGHT_COUNT.store(0, Ordering::Relaxed);
    let left = &LEFT_COUNT;
    let right = &RIGHT_COUNT;
    EncoderCounters { left, right }
}

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

const LUT: [[i8; 4]; 4] = [
    /*prev\curr: 00, 01, 10, 11 */
    [0, 1, -1, 0], // 00 ->
    [-1, 0, 0, 1], // 01 ->
    [1, 0, 0, -1], // 10 ->
    [0, -1, 1, 0], // 11 ->
];

pub struct EncoderPioIrqHandler;

impl Handler<PIO0_IRQ_0> for EncoderPioIrqHandler {
    unsafe fn on_interrupt() {
        drain_encoder_fifo::<0>(&LEFT_COUNT, &LEFT_PREV_AB);
        drain_encoder_fifo::<1>(&RIGHT_COUNT, &RIGHT_PREV_AB);
        enable_encoder_irq_sources();
    }
}

fn drain_encoder_fifo<const SM: usize>(counter: &AtomicI32, prev_ab: &AtomicI32) {
    let mut prev = prev_ab.load(Ordering::Relaxed);
    let mut delta = 0i32;

    while pac::PIO0.fstat().read().rxempty() & (1u8 << SM) == 0 {
        let curr = (pac::PIO0.rxf(SM).read() & 0b11) as i32;
        if prev >= 0 {
            delta += LUT[prev as usize][curr as usize] as i32;
        }
        prev = curr;
    }

    if prev >= 0 {
        prev_ab.store(prev, Ordering::Relaxed);
    }
    if delta != 0 {
        counter.fetch_add(delta, Ordering::Relaxed);
    }
}

pub fn start_encoder_irq(encoders: EncoderPair<'static>) {
    LEFT_PREV_AB.store(-1, Ordering::Relaxed);
    RIGHT_PREV_AB.store(-1, Ordering::Relaxed);

    drain_encoder_fifo::<0>(&LEFT_COUNT, &LEFT_PREV_AB);
    drain_encoder_fifo::<1>(&RIGHT_COUNT, &RIGHT_PREV_AB);

    enable_encoder_irq_sources();

    core::mem::forget(encoders);
}

fn enable_encoder_irq_sources() {
    pac::PIO0.irqs(0).inte().modify(|m| {
        m.0 |= (1 << 0) | (1 << 1);
    });
}

// Asychronously acquiring left RPM
pub async fn get_left_rpm(
    counter: &'static AtomicI32,
    cpr_wheel: f32,
    interval_ms: u64,
) -> f32 {
    let before = counter.load(Ordering::Relaxed);
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = counter.load(Ordering::Relaxed);

    let delta = after.wrapping_sub(before) as f32;
    let rps = delta / cpr_wheel / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

// Asychronously acquiring right RPM
pub async fn get_right_rpm(
    counter: &'static AtomicI32,
    cpr_wheel: f32,
    interval_ms: u64,
) -> f32 {
    let before = counter.load(Ordering::Relaxed);
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = counter.load(Ordering::Relaxed);

    let delta = after.wrapping_sub(before) as f32;
    let rps = delta / cpr_wheel / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

pub async fn get_rpms(
    left_counter: &'static AtomicI32,
    right_counter: &'static AtomicI32,
    cpr_wheel: f32,
    interval_ms: u64,
) -> (f32, f32) {
    let left_before = left_counter.load(Ordering::Relaxed);
    let right_before = right_counter.load(Ordering::Relaxed);

    Timer::after(Duration::from_millis(interval_ms)).await;

    let left_after = left_counter.load(Ordering::Relaxed);
    let right_after = right_counter.load(Ordering::Relaxed);

    let delta_l = left_after.wrapping_sub(left_before) as f32;
    let delta_r = right_after.wrapping_sub(right_before) as f32;

    let win_s = interval_ms as f32 / 1000.0;
    let rps_l = delta_l / cpr_wheel / win_s;
    let rps_r = delta_r / cpr_wheel / win_s;

    (rps_l * 60.0, rps_r * 60.0)
}

pub async fn get_wheel_speed_in_rad(
    left_counter: &'static AtomicI32,
    right_counter: &'static AtomicI32,
    cpr_wheel: f32,
    interval_ms: u64,
    dt: f32, // equal to interval_ms but in second, just for saving one divide calculation
) -> (f32, f32) {
    let left_before = left_counter.load(Ordering::Relaxed);
    let right_before = right_counter.load(Ordering::Relaxed);

    Timer::after(Duration::from_millis(interval_ms)).await;

    let left_after = left_counter.load(Ordering::Relaxed);
    let right_after = right_counter.load(Ordering::Relaxed);

    let delta_l = left_after.wrapping_sub(left_before) as f32;
    let delta_r = right_after.wrapping_sub(right_before) as f32;

    // original rotation speed in rad/s
    let omega_l_raw = (delta_l as f32) * 2.0 * core::f32::consts::PI / (cpr_wheel * dt);
    let omega_r_raw = (delta_r as f32) * 2.0 * core::f32::consts::PI / (cpr_wheel * dt);

    (omega_l_raw, omega_r_raw)
}

pub async fn wheel_speed_from_counts_now(
    left_counter: &AtomicI32,
    right_counter: &AtomicI32,
    cpr_wheel: f32,
    prev_left: i32,
    prev_right: i32,
    dt: f32, // in sec
) -> ((f32, f32), (i32, i32)) {
    let left_now = left_counter.load(Ordering::Relaxed);
    let right_now = right_counter.load(Ordering::Relaxed);

    let delta_l = left_now.wrapping_sub(prev_left) as f32;
    let delta_r = right_now.wrapping_sub(prev_right) as f32;

    let k = 2.0 * core::f32::consts::PI / cpr_wheel;
    let omega_l = k * delta_l / dt;
    let omega_r = k * delta_r / dt;

    ((omega_l, omega_r), (left_now, right_now))
}
