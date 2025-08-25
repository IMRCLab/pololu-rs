#![no_std]

// Global verbosity: v0=silent, v1=basic, v2=full debug
// Usage: cargo embed --bin <binary> --features three-pi,v1
// Examples: cargo embed --bin teleop_control --features three-pi,v2
//           cargo build --bin main --features three-pi,v0
// Import: use crate::{debug_v1, debug_v2, debug_warn, debug_error};
// Macros: debug_warn!/debug_error! always shown, debug_v1!/debug_v2! conditional

#[cfg(feature = "v2")]
#[macro_export]
macro_rules! debug_v2 {
    ($($arg:tt)*) => { defmt::info!($($arg)*) };
}
#[cfg(not(feature = "v2"))]
#[macro_export]
macro_rules! debug_v2 {
    ($($arg:tt)*) => {};
}

#[cfg(any(feature = "v1", feature = "v2"))]
#[macro_export]
macro_rules! debug_v1 {
    ($($arg:tt)*) => { defmt::info!($($arg)*) };
}
#[cfg(not(any(feature = "v1", feature = "v2")))]
#[macro_export]
macro_rules! debug_v1 {
    ($($arg:tt)*) => {};
}

// Errors and warnings always shown
#[macro_export]
macro_rules! debug_error {
    ($($arg:tt)*) => { defmt::error!($($arg)*) };
}

#[macro_export]
macro_rules! debug_warn {
    ($($arg:tt)*) => { defmt::warn!($($arg)*) };
}

pub mod button;
pub mod buzzer;
pub mod diffdrive;
pub mod diffdrive_cascade;
pub mod encoder;
pub mod encoder_lib;
pub mod imu;
pub mod init;
pub mod joystick_control;
pub mod led;
pub mod math;
pub mod motor;
pub mod packet;
pub mod read_robot_config_from_sd;
pub mod robot_parameters_default;
pub mod sdlog;
pub mod trajectory_control;
pub mod trajectory_reading;
pub mod trajectory_signal;
pub mod trajectory_uart;
pub mod uart;
