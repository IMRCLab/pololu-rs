use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

/// Shared I2C type for ACCEL/GYRO sensors and MAG sensors
/// (needed separate because of different addresses on the same bus (with the same pins))
pub type SharedI2c<'a, T> = &'a Mutex<ThreadModeRawMutex, T>;
