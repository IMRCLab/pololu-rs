use embassy_rp::peripherals::{PIN_8, PIN_9, PIN_12, PIN_13, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

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

// Global static cells
static LEFT_COUNT: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();
static RIGHT_COUNT: StaticCell<Mutex<NoopRawMutex, i32>> = StaticCell::new();

/// avoid using global counters
pub fn init_encoder_counts() -> EncoderCounters {
    let left = LEFT_COUNT.init(Mutex::new(0));
    let right = RIGHT_COUNT.init(Mutex::new(0));
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

#[embassy_executor::task]
pub async fn encoder_left_task(
    mut encoder: PioEncoder<'static, PIO0, 0>,
    counter: &'static Mutex<NoopRawMutex, i32>,
) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        let mut guard = counter.lock().await;
        *guard += delta;

        // defmt::info!("Left Encoder Count = {}", *guard);
    }
}

#[embassy_executor::task]
pub async fn encoder_right_task(
    mut encoder: PioEncoder<'static, PIO0, 1>,
    counter: &'static Mutex<NoopRawMutex, i32>,
) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        let mut guard = counter.lock().await;
        *guard += delta;

        // defmt::info!("Right Encoder Count = {}", *guard);
    }
}

// Asychronously acquiring left RPM
pub async fn get_left_rpm(
    counter: &'static Mutex<NoopRawMutex, i32>,
    ppr: u32,
    interval_ms: u64,
) -> f32 {
    let before = *counter.lock().await;
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = *counter.lock().await;

    let delta = after - before;
    let rps = delta as f32 / (ppr as f32 * 4.0) / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

// Asychronously acquiring right RPM
pub async fn get_right_rpm(
    counter: &'static Mutex<NoopRawMutex, i32>,
    ppr: u32,
    interval_ms: u64,
) -> f32 {
    let before = *counter.lock().await;
    Timer::after(Duration::from_millis(interval_ms)).await;
    let after = *counter.lock().await;

    let delta = after - before;
    let rps = delta as f32 / (ppr as f32 * 4.0) / (interval_ms as f32 / 1000.0);
    rps * 60.0
}

// fast left position reading
pub fn read_left_count(counter: &'static Mutex<NoopRawMutex, i32>) -> i32 {
    counter.try_lock().map(|g| *g).unwrap_or(0)
}

// fast right position reading
pub fn read_right_count(counter: &'static Mutex<NoopRawMutex, i32>) -> i32 {
    counter.try_lock().map(|g| *g).unwrap_or(0)
}
