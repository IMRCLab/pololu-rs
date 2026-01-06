use crate::imu::complementary_filter::ComplementaryFilter;
use crate::imu::lis3mdl::Lis3mdl;
use crate::imu::lsm6dso::Lsm6dso;
// use crate::imu::madgwick::Madgwick;

use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use static_cell::StaticCell;

pub mod complementary_filter;
pub mod lis3mdl;
pub mod lsm6dso;
pub mod madgwick;
pub mod shared_i2c;

// === Global filters as StaticCell ===
// static MADGWICK_CELL: StaticCell<Mutex<NoopRawMutex, Madgwick>> = StaticCell::new();
static COMPLEMENTARY_CELL: StaticCell<Mutex<NoopRawMutex, ComplementaryFilter>> = StaticCell::new();

pub static IMU_ANGLES_SHARED: Mutex<ThreadModeRawMutex, (f32, f32, f32)> = Mutex::new((0.0, 0.0, 0.0));

pub struct ImuPack<'a, T: AsyncI2c> {
    pub i2c: &'a Mutex<ThreadModeRawMutex, T>,
    pub lsm6dso: Lsm6dso<'a, T>,
    pub lis3mdl: Lis3mdl<'a, T>,
    pub complementary: &'static Mutex<NoopRawMutex, ComplementaryFilter>,
    // pub madgwick: &'static Mutex<NoopRawMutex, Madgwick>,
}

impl<'a, T: AsyncI2c + 'a> ImuPack<'a, T> {
    pub fn new(i2c: &'a Mutex<ThreadModeRawMutex, T>) -> Self {
        //changed the magnetometer confindence from 10 to 1 percent -> indoors magnets are worthless
        let complementary = COMPLEMENTARY_CELL.init(Mutex::new(ComplementaryFilter::new(0.999)));
        // let madgwick = MADGWICK_CELL.init(Mutex::new(Madgwick::new(0.1)));
        Self {
            i2c,
            lsm6dso: Lsm6dso::new(i2c),
            lis3mdl: Lis3mdl::new(i2c),
            complementary,
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
pub async fn read_imu_task(mut imu: ImuPack<'static, I2c<'static, I2C0, Async>>) {
    if let Err(_e) = imu.init().await {
        defmt::error!("IMU init failed!");
        return;
    }

    loop {
        match imu.read_all().await {
            Ok((accel, gyro, mag)) => {
                /*
                let mut lock = madgwick.lock().await;
                lock.update(gyro, accel, mag, 0.01);
                let angles = lock.get_euler();
                */
                let mut lock = imu.complementary.lock().await;
                lock.update(gyro, accel, mag, 0.01);
                let (pitch, roll, yaw) = lock.get_angles_deg();

                {
                    let mut g = IMU_ANGLES_SHARED.lock().await;
                    *g = (pitch, roll, yaw);
                }

                let (_pitch, _roll, _yaw) = lock.get_angles_deg();
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

        Timer::after(Duration::from_millis(10)).await;
    }
}
