use crate::imu::shared_i2c::SharedI2c;
use embedded_hal_async::i2c::I2c;

pub struct Lsm6dso<'a, T: I2c> {
    pub i2c: SharedI2c<'a, T>,
    pub address: u8,
    accel_sensitivity: f32,
    gyro_sensitivity: f32,
}

impl<'a, T: I2c> Lsm6dso<'a, T> {
    pub fn new(i2c: SharedI2c<'a, T>) -> Self {
        Self {
            i2c,
            address: 0x6B,
            accel_sensitivity: 0.061,
            gyro_sensitivity: 70.0,
        }
    }

    pub async fn init(&mut self) -> Result<(), T::Error> {
        let mut i2c = self.i2c.lock().await;

        i2c.write(self.address, &[0x12, 0x44]).await?; // CTRL3_C: BDU=1, IF_INC=1
        i2c.write(self.address, &[0x18, 0x38]).await?; // CTRL9_XL: enable XYZ acce
        i2c.write(self.address, &[0x10, 0x30]).await?; // CTRL1_XL: ODR_XL=52Hz, FS=2g
        i2c.write(self.address, &[0x11, 0x5C]).await?; // CTRL2_G: ODR=208Hz, FS=2000dps

        Ok(())
    }

    pub async fn read_accel(&mut self) -> Result<[f32; 3], T::Error> {
        let mut i2c = self.i2c.lock().await;

        let mut buf = [0u8; 2];

        i2c.write_read(self.address, &[0x28], &mut buf).await?;
        let ax = i16::from_le_bytes(buf);

        i2c.write_read(self.address, &[0x2A], &mut buf).await?;
        let ay = i16::from_le_bytes(buf);

        i2c.write_read(self.address, &[0x2C], &mut buf).await?;
        let az = i16::from_le_bytes(buf);

        Ok([
            ax as f32 * self.accel_sensitivity / 1000.0,
            ay as f32 * self.accel_sensitivity / 1000.0,
            az as f32 * self.accel_sensitivity / 1000.0,
        ])
    }

    pub async fn read_gyro(&mut self) -> Result<[f32; 3], T::Error> {
        let mut i2c = self.i2c.lock().await;

        let mut buf = [0u8; 2];

        i2c.write_read(self.address, &[0x22], &mut buf).await?;
        let gx = i16::from_le_bytes(buf);

        i2c.write_read(self.address, &[0x24], &mut buf).await?;
        let gy = i16::from_le_bytes(buf);

        i2c.write_read(self.address, &[0x26], &mut buf).await?;
        let gz = i16::from_le_bytes(buf);

        Ok([
            gx as f32 * self.gyro_sensitivity / 1000.0,
            gy as f32 * self.gyro_sensitivity / 1000.0,
            gz as f32 * self.gyro_sensitivity / 1000.0,
        ])
    }
}
