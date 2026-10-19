//! Core 1 runtime monitor.
//!
//! Printing happens on Core 1 so the Core 0 inner loop never formats logs.

use embassy_time::{Duration, Instant, Ticker};

use crate::{
    dual_core::{
        logging::dropped_log_frames,
        metrics::take_interval_metrics,
        shared_state::{
            active_mode, latest_imu_sample, latest_teleop_command,
        },
    },
    peripherals::uart::{
        uart_rx_dropped_bytes, uart_tx_dropped_packets,
    },
};

#[embassy_executor::task]
pub async fn core1_monitor_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        let now_us = Instant::now().as_micros();
        let metrics = take_interval_metrics();
        let command = latest_teleop_command();
        let imu = latest_imu_sample();

        let command_age_ms = if command.stamp_us == 0 {
            0
        } else {
            now_us.saturating_sub(command.stamp_us) / 1_000
        };

        let imu_age_ms = if imu.stamp_us == 0 {
            0
        } else {
            now_us.saturating_sub(imu.stamp_us) / 1_000
        };

        defmt::info!(
            "MODE={} CTRL={}Hz max_gap={}us misses={} UART={}pkt/s IMU={}Hz cmd_age={}ms imu_age={}ms rx_drop={} tx_drop={} log_drop={}",
            active_mode().as_u8(),
            metrics.control_ticks,
            metrics.max_control_gap_us,
            metrics.control_misses,
            metrics.uart_packets,
            metrics.imu_samples,
            command_age_ms,
            imu_age_ms,
            uart_rx_dropped_bytes(),
            uart_tx_dropped_packets(),
            dropped_log_frames(),
        );
    }
}