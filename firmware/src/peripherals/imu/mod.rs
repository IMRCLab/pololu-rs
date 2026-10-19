//! Core 1 IMU device driver.
//!
//! I2C0 is exclusively owned by one Core 1 sampling task. Sensor drivers
//! borrow it sequentially and therefore require no shared async mutex.

pub mod complementary_filter;
pub mod lis3mdl;
pub mod lsm6dso;
pub mod madgwick;

use embedded_hal_async::i2c::I2c;

use self::{
    lis3mdl::Lis3mdl,
    lsm6dso::Lsm6dso,
};

#[derive(Clone, Copy)]
pub struct ImuRawSample {
    pub acceleration: [f32; 3],
    pub gyroscope: [f32; 3],
    pub magnetic_field: [f32; 3],
}

pub struct ImuDevice<T>
where
    T: I2c,
{
    i2c: T,
    lsm6dso: Lsm6dso,
    lis3mdl: Lis3mdl,
}

impl<T> ImuDevice<T>
where
    T: I2c,
{
    pub fn new(i2c: T) -> Self {
        Self {
            i2c,
            lsm6dso: Lsm6dso::new(),
            lis3mdl: Lis3mdl::new(),
        }
    }

    pub async fn init(
        &mut self,
    ) -> Result<(), T::Error> {
        self.lsm6dso
            .init(&mut self.i2c)
            .await?;

        self.lis3mdl
            .init(&mut self.i2c)
            .await?;

        Ok(())
    }

    pub async fn read_all(
        &mut self,
    ) -> Result<ImuRawSample, T::Error> {
        let acceleration = self
            .lsm6dso
            .read_acceleration(&mut self.i2c)
            .await?;

        let gyroscope = self
            .lsm6dso
            .read_gyroscope(&mut self.i2c)
            .await?;

        let magnetic_field = self
            .lis3mdl
            .read_magnetic_field(&mut self.i2c)
            .await?;

        Ok(ImuRawSample {
            acceleration,
            gyroscope,
            magnetic_field,
        })
    }

    pub fn into_inner(self) -> T {
        self.i2c
    }
}