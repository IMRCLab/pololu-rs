use crate::imu::shared_i2c::SharedI2c;
use embedded_hal_async::i2c::I2c;

// LIS3MDL Magnetometer driver
pub struct Lis3mdl<'a, T: I2c> {
    pub i2c: SharedI2c<'a, T>,
    pub address: u8,
}

impl<'a, T: I2c + 'a> Lis3mdl<'a, T> {
    pub fn new(i2c: SharedI2c<'a, T>) -> Self {
        Self { i2c, address: 0x1E }
    }

    pub async fn init(&mut self) -> Result<(), T::Error> {
        let mut i2c = self.i2c.lock().await;

        i2c.write(self.address, &[0x20, 0b1111_0000]).await?; // CTRL_REG1 (0x20): Temp enable = 1, Ultra-high-perf on X and Y, 80 Hz
        i2c.write(self.address, &[0x21, 0b0000_0000]).await?; // CTRL_REG2 (0x21): +/-4 gauss (00)
        i2c.write(self.address, &[0x22, 0b0000_0000]).await?; // CTRL_REG3 (0x22): Continuous-conversion mode (00)
        i2c.write(self.address, &[0x23, 0b0000_1100]).await?; // CTRL_REG4 (0x23): Ultra-high-perf on Z

        Ok(())
    }

    /// Read magnetometer data in radians
    /// It is not adviced to use magnetometer indoors.
    pub async fn read_mag(&mut self) -> Result<[f32; 3], T::Error> {
        let mut data = [0u8; 6];
        let mut i2c = self.i2c.lock().await;
        i2c.write_read(self.address, &[0x28 | 0x80], &mut data)
            .await?;

        let raw = [
            i16::from_le_bytes([data[0], data[1]]),
            i16::from_le_bytes([data[2], data[3]]),
            i16::from_le_bytes([data[4], data[5]]),
        ];

        // sensitivity: ±4 gauss = 0.14 mgauss/LSB
        Ok([
            raw[0] as f32 / 6842.0,
            raw[1] as f32 / 6842.0,
            raw[2] as f32 / 6842.0,
        ])
    }
}
