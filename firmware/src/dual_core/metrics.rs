//! Low-overhead dual-core runtime metrics.

use portable_atomic::{AtomicU32, Ordering};

static CONTROL_TICKS: AtomicU32 = AtomicU32::new(0);
static CONTROL_MAX_GAP_US: AtomicU32 = AtomicU32::new(0);
static CONTROL_MISSES: AtomicU32 = AtomicU32::new(0);
static UART_PACKETS: AtomicU32 = AtomicU32::new(0);
static IMU_SAMPLES: AtomicU32 = AtomicU32::new(0);

pub struct IntervalMetrics {
    pub control_ticks: u32,
    pub max_control_gap_us: u32,
    pub control_misses: u32,
    pub uart_packets: u32,
    pub imu_samples: u32,
}

pub fn record_control_tick(gap_us: u32, expected_period_us: u32) {
    CONTROL_TICKS.fetch_add(1, Ordering::Relaxed);
    CONTROL_MAX_GAP_US.fetch_max(gap_us, Ordering::Relaxed);

    if gap_us > expected_period_us.saturating_mul(3) / 2 {
        CONTROL_MISSES.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn record_uart_packet() {
    UART_PACKETS.fetch_add(1, Ordering::Relaxed);
}

pub fn record_imu_sample() {
    IMU_SAMPLES.fetch_add(1, Ordering::Relaxed);
}

pub fn take_interval_metrics() -> IntervalMetrics {
    IntervalMetrics {
        control_ticks: CONTROL_TICKS.swap(0, Ordering::Relaxed),
        max_control_gap_us: CONTROL_MAX_GAP_US.swap(0, Ordering::Relaxed),
        control_misses: CONTROL_MISSES.swap(0, Ordering::Relaxed),
        uart_packets: UART_PACKETS.swap(0, Ordering::Relaxed),
        imu_samples: IMU_SAMPLES.swap(0, Ordering::Relaxed),
    }
}