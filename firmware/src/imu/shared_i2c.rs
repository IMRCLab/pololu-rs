use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

pub type SharedI2c<'a, T> = &'a Mutex<ThreadModeRawMutex, T>;
