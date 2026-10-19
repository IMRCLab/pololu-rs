//! Core 0 quadrature encoder driver.
//!
//! PIO state machines and the associated IRQ are exclusively owned by Core 0.

mod pio;

use embassy_rp::{
    Peri,
    interrupt::typelevel::{
        Handler,
        PIO0_IRQ_0,
    },
    pac,
    peripherals::{
        PIN_8,
        PIN_9,
        PIN_12,
        PIN_13,
        PIO0,
    },
    pio::{
        Common,
        StateMachine,
    },
};
use portable_atomic::{
    AtomicI32,
    Ordering,
};

use self::pio::{
    PioEncoder,
    PioEncoderProgram,
};

/* ================= Encoder hardware ================= */

pub struct EncoderPair<'a> {
    pub encoder_left: PioEncoder<'a, PIO0, 0>,
    pub encoder_right: PioEncoder<'a, PIO0, 1>,
}

impl<'a> EncoderPair<'a> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        common: &mut Common<'a, PIO0>,
        sm0: StateMachine<'a, PIO0, 0>,
        sm1: StateMachine<'a, PIO0, 1>,

        pin_left_a: Peri<'a, PIN_12>,
        pin_left_b: Peri<'a, PIN_13>,

        pin_right_a: Peri<'a, PIN_8>,
        pin_right_b: Peri<'a, PIN_9>,
    ) -> Self {
        let program =
            PioEncoderProgram::new(common);

        let encoder_left = PioEncoder::new(
            common,
            sm0,
            pin_left_a,
            pin_left_b,
            &program,
        );

        let encoder_right = PioEncoder::new(
            common,
            sm1,
            pin_right_a,
            pin_right_b,
            &program,
        );

        Self {
            encoder_left,
            encoder_right,
        }
    }
}

/* ================= Encoder counters ================= */

pub struct EncoderCounters {
    pub left: &'static AtomicI32,
    pub right: &'static AtomicI32,
}

static LEFT_COUNT: AtomicI32 =
    AtomicI32::new(0);

static RIGHT_COUNT: AtomicI32 =
    AtomicI32::new(0);

static LEFT_PREVIOUS_AB: AtomicI32 =
    AtomicI32::new(-1);

static RIGHT_PREVIOUS_AB: AtomicI32 =
    AtomicI32::new(-1);

pub fn init_encoder_counts() -> EncoderCounters {
    LEFT_COUNT.store(0, Ordering::Relaxed);
    RIGHT_COUNT.store(0, Ordering::Relaxed);

    LEFT_PREVIOUS_AB.store(-1, Ordering::Relaxed);
    RIGHT_PREVIOUS_AB.store(-1, Ordering::Relaxed);

    EncoderCounters {
        left: &LEFT_COUNT,
        right: &RIGHT_COUNT,
    }
}

/* ================= Quadrature decoding ================= */

const QUADRATURE_LUT: [[i8; 4]; 4] = [
    // Previous/current: 00, 01, 10, 11
    [0, 1, -1, 0],
    [-1, 0, 0, 1],
    [1, 0, 0, -1],
    [0, -1, 1, 0],
];

pub struct EncoderPioIrqHandler;

impl Handler<PIO0_IRQ_0> for EncoderPioIrqHandler {
    unsafe fn on_interrupt() {
        drain_encoder_fifo::<0>(
            &LEFT_COUNT,
            &LEFT_PREVIOUS_AB,
        );

        drain_encoder_fifo::<1>(
            &RIGHT_COUNT,
            &RIGHT_PREVIOUS_AB,
        );

        enable_encoder_irq_sources();
    }
}

fn drain_encoder_fifo<const SM: usize>(
    counter: &AtomicI32,
    previous_ab: &AtomicI32,
) {
    let mut previous =
        previous_ab.load(Ordering::Relaxed);

    let mut delta = 0i32;

    while pac::PIO0
        .fstat()
        .read()
        .rxempty()
        & (1u8 << SM)
        == 0
    {
        let current =
            (pac::PIO0.rxf(SM).read() & 0b11)
                as i32;

        if previous >= 0 {
            delta += QUADRATURE_LUT
                [previous as usize]
                [current as usize]
                as i32;
        }

        previous = current;
    }

    if previous >= 0 {
        previous_ab.store(
            previous,
            Ordering::Relaxed,
        );
    }

    if delta != 0 {
        counter.fetch_add(
            delta,
            Ordering::Relaxed,
        );
    }
}

fn enable_encoder_irq_sources() {
    pac::PIO0.irqs(0).inte().modify(|value| {
        value.0 |= (1 << 0) | (1 << 1);
    });
}

pub fn start_encoder_irq(
    encoders: EncoderPair<'static>,
) {
    LEFT_PREVIOUS_AB.store(-1, Ordering::Relaxed);
    RIGHT_PREVIOUS_AB.store(-1, Ordering::Relaxed);

    drain_encoder_fifo::<0>(
        &LEFT_COUNT,
        &LEFT_PREVIOUS_AB,
    );

    drain_encoder_fifo::<1>(
        &RIGHT_COUNT,
        &RIGHT_PREVIOUS_AB,
    );

    enable_encoder_irq_sources();

    // State machines must remain alive for the rest of the firmware runtime.
    core::mem::forget(encoders);
}

/* ================= Deterministic sampling ================= */

#[derive(Clone, Copy)]
pub struct EncoderSample {
    pub left_count: i32,
    pub right_count: i32,

    pub omega_left: f32,
    pub omega_right: f32,
}

/// Calculate wheel speed from the count difference since the previous control
/// iteration.
///
/// This function is synchronous and contains no timer or `.await`. The Core 0
/// control task owns the sampling period.
pub fn sample_wheel_speed(
    counters: &EncoderCounters,
    previous_left: i32,
    previous_right: i32,
    encoder_cpr: f32,
    dt_s: f32,
) -> EncoderSample {
    let left_count =
        counters.left.load(Ordering::Relaxed);

    let right_count =
        counters.right.load(Ordering::Relaxed);

    let delta_left =
        left_count.wrapping_sub(previous_left)
            as f32;

    let delta_right =
        right_count.wrapping_sub(previous_right)
            as f32;

    let radians_per_count =
        2.0 * core::f32::consts::PI
            / encoder_cpr;

    let omega_left =
        radians_per_count * delta_left / dt_s;

    let omega_right =
        radians_per_count * delta_right / dt_s;

    EncoderSample {
        left_count,
        right_count,
        omega_left,
        omega_right,
    }
}

/// Read the current raw counters without calculating velocity.
pub fn read_counts(
    counters: &EncoderCounters,
) -> (i32, i32) {
    (
        counters.left.load(Ordering::Relaxed),
        counters.right.load(Ordering::Relaxed),
    )
}