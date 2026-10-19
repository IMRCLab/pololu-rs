//! Permanent Core 1 IMU sampling task.

use embassy_rp::{
    i2c::{
        Async,
        I2c,
    },
    peripherals::I2C0,
};
use embassy_time::{
    Duration,
    Instant,
    Ticker,
    Timer,
};

use crate::{
    dual_core::shared_state::{
        ImuSample,
        publish_imu_sample,
    },
    dual_core::metrics::record_imu_sample,
    peripherals::imu::{
        ImuDevice,
        complementary_filter::ComplementaryFilter,
    },
};

/// Core 1 owns I2C0 for the complete runtime.
///
/// I2C transactions and sensor filtering never execute on Core 0.
#[embassy_executor::task]
pub async fn core1_imu_task(
    i2c: I2c<'static, I2C0, Async>,
    period_ms: u64,
) {
    let period_ms = period_ms.max(1);

    let mut imu = ImuDevice::new(i2c);

    // Keep retrying initialization so a temporarily unavailable sensor does
    // not permanently terminate the service task.
    loop {
        match imu.init().await {
            Ok(()) => break,

            Err(_) => {
                defmt::warn!(
                    "Core 1 IMU initialization failed; retrying"
                );

                Timer::after(
                    Duration::from_millis(1000),
                )
                .await;
            }
        }
    }

    defmt::info!(
        "Core 1 IMU task started, period={}ms",
        period_ms,
    );

    let mut filter =
        ComplementaryFilter::new(0.9);

    let mut ticker = Ticker::every(
        Duration::from_millis(period_ms),
    );

    let mut previous_time = Instant::now();
    let mut consecutive_errors = 0u32;

    loop {
        ticker.next().await;

        let now = Instant::now();

        let elapsed_us =
            now.duration_since(previous_time)
                .as_micros();

        previous_time = now;

        // Use measured sampling time. Clamp it so a debugger pause or temporary
        // I2C error cannot cause one enormous filter integration step.
        let dt_s =
            (elapsed_us as f32 / 1_000_000.0)
                .clamp(0.000_1, 0.1);

        let sample = match imu.read_all().await {
            Ok(sample) => {
                consecutive_errors = 0;
                sample
            }

            Err(_) => {
                consecutive_errors =
                    consecutive_errors.saturating_add(1);

                // Rate-limit warnings.
                if consecutive_errors == 1
                    || consecutive_errors % 100 == 0
                {
                    defmt::warn!(
                        "Core 1 IMU read failed, consecutive={}",
                        consecutive_errors,
                    );
                }

                continue;
            }
        };

        filter.update(
            sample.gyroscope,
            sample.acceleration,
            sample.magnetic_field,
            dt_s,
        );

        let (pitch, roll, yaw) =
            filter.get_angles_rad();

        publish_imu_sample(ImuSample {
            acc_x: sample.acceleration[0],
            acc_y: sample.acceleration[1],
            acc_z: sample.acceleration[2],

            gyro_x: sample.gyroscope[0],
            gyro_y: sample.gyroscope[1],
            gyro_z: sample.gyroscope[2],

            roll,
            pitch,
            yaw,

            stamp_us: now.as_micros(),
        });
        record_imu_sample();
    }
}