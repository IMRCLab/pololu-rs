//! LIS3MDL magnetometer driver.

use embedded_hal_async::i2c::I2c;

pub struct Lis3mdl {
    address: u8,
}

impl Lis3mdl {
    pub const fn new() -> Self {
        Self {
            address: 0x1E,
        }
    }

    pub async fn init<T>(
        &self,
        i2c: &mut T,
    ) -> Result<(), T::Error>
    where
        T: I2c,
    {
        // Temperature enabled, ultra-high-performance X/Y, 80 Hz.
        i2c.write(
            self.address,
            &[0x20, 0b1111_0000],
        )
        .await?;

        // ±4 gauss.
        i2c.write(
            self.address,
            &[0x21, 0b0000_0000],
        )
        .await?;

        // Continuous conversion.
        i2c.write(
            self.address,
            &[0x22, 0b0000_0000],
        )
        .await?;

        // Ultra-high-performance Z.
        i2c.write(
            self.address,
            &[0x23, 0b0000_1100],
        )
        .await?;

        Ok(())
    }

    pub async fn read_magnetic_field<T>(
        &self,
        i2c: &mut T,
    ) -> Result<[f32; 3], T::Error>
    where
        T: I2c,
    {
        let mut data = [0u8; 6];

        i2c.write_read(
            self.address,
            &[0x28 | 0x80],
            &mut data,
        )
        .await?;

        let raw_x = i16::from_le_bytes([
            data[0],
            data[1],
        ]);

        let raw_y = i16::from_le_bytes([
            data[2],
            data[3],
        ]);

        let raw_z = i16::from_le_bytes([
            data[4],
            data[5],
        ]);

        Ok([
            raw_x as f32 / 6842.0,
            raw_y as f32 / 6842.0,
            raw_z as f32 / 6842.0,
        ])
    }
}

impl Default for Lis3mdl {
    fn default() -> Self {
        Self::new()
    }
}