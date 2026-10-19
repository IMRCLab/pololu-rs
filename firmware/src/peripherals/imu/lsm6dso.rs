//! LSM6DSO accelerometer and gyroscope driver.

use embedded_hal_async::i2c::I2c;

pub struct Lsm6dso {
    address: u8,
    acceleration_sensitivity: f32,
    gyroscope_sensitivity: f32,
}

impl Lsm6dso {
    pub const fn new() -> Self {
        Self {
            address: 0x6B,

            // mg/LSB at ±2 g.
            acceleration_sensitivity: 0.061,

            // mdps/LSB at ±2000 dps.
            gyroscope_sensitivity: 70.0,
        }
    }

    pub async fn init<T>(
        &self,
        i2c: &mut T,
    ) -> Result<(), T::Error>
    where
        T: I2c,
    {
        // CTRL3_C: BDU=1, IF_INC=1
        i2c.write(
            self.address,
            &[0x12, 0x44],
        )
        .await?;

        // CTRL9_XL: enable XYZ accelerometer axes
        i2c.write(
            self.address,
            &[0x18, 0x38],
        )
        .await?;

        // CTRL1_XL: 52 Hz, ±2 g
        i2c.write(
            self.address,
            &[0x10, 0x30],
        )
        .await?;

        // CTRL2_G: 208 Hz, ±2000 dps
        i2c.write(
            self.address,
            &[0x11, 0x5C],
        )
        .await?;

        Ok(())
    }

    pub async fn read_acceleration<T>(
        &self,
        i2c: &mut T,
    ) -> Result<[f32; 3], T::Error>
    where
        T: I2c,
    {
        // IF_INC permits reading all three axes in one transaction.
        let mut data = [0u8; 6];

        i2c.write_read(
            self.address,
            &[0x28],
            &mut data,
        )
        .await?;

        let x = i16::from_le_bytes([
            data[0],
            data[1],
        ]);

        let y = i16::from_le_bytes([
            data[2],
            data[3],
        ]);

        let z = i16::from_le_bytes([
            data[4],
            data[5],
        ]);

        Ok([
            x as f32
                * self.acceleration_sensitivity
                / 1000.0,

            y as f32
                * self.acceleration_sensitivity
                / 1000.0,

            z as f32
                * self.acceleration_sensitivity
                / 1000.0,
        ])
    }

    pub async fn read_gyroscope<T>(
        &self,
        i2c: &mut T,
    ) -> Result<[f32; 3], T::Error>
    where
        T: I2c,
    {
        let mut data = [0u8; 6];

        i2c.write_read(
            self.address,
            &[0x22],
            &mut data,
        )
        .await?;

        let x = i16::from_le_bytes([
            data[0],
            data[1],
        ]);

        let y = i16::from_le_bytes([
            data[2],
            data[3],
        ]);

        let z = i16::from_le_bytes([
            data[4],
            data[5],
        ]);

        Ok([
            x as f32
                * self.gyroscope_sensitivity
                / 1000.0,

            y as f32
                * self.gyroscope_sensitivity
                / 1000.0,

            z as f32
                * self.gyroscope_sensitivity
                / 1000.0,
        ])
    }
}

impl Default for Lsm6dso {
    fn default() -> Self {
        Self::new()
    }
}