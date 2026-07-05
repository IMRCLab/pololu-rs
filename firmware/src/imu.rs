use crate::imu::complementary_filter::ComplementaryFilter;
use crate::imu::lis3mdl::Lis3mdl;
use crate::imu::lsm6dso::Lsm6dso;
use crate::orchestrator_signal::STOP_IMU_SIG;
// use crate::imu::madgwick::Madgwick;

use embassy_futures::select::{Either, select};
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c as AsyncI2c;

pub mod complementary_filter;
pub mod lis3mdl;
pub mod lsm6dso;
pub mod madgwick;
pub mod shared_i2c;

// === Global filters ===
// static MADGWICK_CELL: StaticCell<Mutex<NoopRawMutex, Madgwick>> = StaticCell::new();
static COMPLEMENTARY: Mutex<ThreadModeRawMutex, ComplementaryFilter> =
    Mutex::new(ComplementaryFilter::new(0.9));

pub struct ImuPack<'a, T: AsyncI2c> {
    pub i2c: &'a Mutex<ThreadModeRawMutex, T>,
    pub lsm6dso: Lsm6dso<'a, T>,
    pub lis3mdl: Lis3mdl<'a, T>,
    pub complementary: &'static Mutex<ThreadModeRawMutex, ComplementaryFilter>,
    // pub madgwick: &'static Mutex<NoopRawMutex, Madgwick>,
}

impl<'a, T: AsyncI2c + 'a> ImuPack<'a, T> {
    pub fn new(i2c: &'a Mutex<ThreadModeRawMutex, T>) -> Self {
        // let madgwick = MADGWICK_CELL.init(Mutex::new(Madgwick::new(0.1)));
        Self {
            i2c,
            lsm6dso: Lsm6dso::new(i2c),
            lis3mdl: Lis3mdl::new(i2c),
            complementary: &COMPLEMENTARY,
            // madgwick,
        }
    }

    /// Initialize 9-axis
    pub async fn init(&mut self) -> Result<(), T::Error> {
        self.lsm6dso.init().await?;
        self.lis3mdl.init().await?;
        Ok(())
    }

    /// read accel + gyro + mag
    pub async fn read_all(&mut self) -> Result<([f32; 3], [f32; 3], [f32; 3]), T::Error> {
        let accel = self.lsm6dso.read_accel().await?;
        let gyro = self.lsm6dso.read_gyro().await?;
        let mag = self.lis3mdl.read_mag().await?;
        Ok((accel, gyro, mag))
    }
}

#[embassy_executor::task]
pub async fn read_imu_task(i2c: &'static Mutex<ThreadModeRawMutex, I2c<'static, I2C0, Async>>) {
    let mut imu = ImuPack::new(i2c);

    if let Err(_e) = imu.init().await {
        defmt::error!("IMU init failed!");
        return;
    }

    {
        let mut lock = imu.complementary.lock().await;
        *lock = ComplementaryFilter::new(0.9);
    }

    match select(Timer::after_millis(3), STOP_IMU_SIG.wait()).await {
        Either::First(_) => {}
        Either::Second(_) => {
            defmt::info!("IMU task stopped before polling.");
            return;
        }
    }

    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        match select(ticker.next(), STOP_IMU_SIG.wait()).await {
            Either::First(_) => {}
            Either::Second(_) => {
                defmt::info!("IMU task stopped.");
                return;
            }
        }

        match imu.read_all().await {
            Ok((accel, gyro, mag)) => {
                /*
                let mut lock = madgwick.lock().await;
                lock.update(gyro, accel, mag, 0.01);
                let angles = lock.get_euler();
                */
                let mut lock = imu.complementary.lock().await;
                lock.update(gyro, accel, mag, 0.01);
                let (_pitch, _roll, _yaw) = lock.get_angles_deg();

                // Publish raw IMU to shared state + logging channel
                crate::robotstate::write_imu(crate::robotstate::ImuReading {
                    acc_x: accel[0], acc_y: accel[1], acc_z: accel[2],
                    gyro_x: gyro[0], gyro_y: gyro[1], gyro_z: gyro[2],
                    stamp: embassy_time::Instant::now(),
                }).await;

                /*
                defmt::info!(
                    "Accel: x={} y={} z={} | Gyro: x={} y={} z={} | Mag: x={} y={} z={}",
                    accel[0],
                    accel[1],
                    accel[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[0],
                    mag[1],
                    mag[2]
                );*/
                // defmt::info!("Roll: {}, Pitch: {}, Yaw: {}", roll, pitch, yaw);
            }
            Err(_e) => defmt::warn!("IMU read error!"),
        }
    }
}
