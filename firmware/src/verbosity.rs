// # Global Verbosity System
// 
// This module provides conditional compilation macros for debug output based on feature flags:
// 
// - **v0** (or no verbosity feature): Silent mode - only errors and warnings
// - **v1**: Basic operation info - key events, control inputs, system status
// - **v2**: Full debug mode - detailed internals, all sensor data, controller states
// 
// ## Build Examples:
// ```bash
// # Silent mode (v0) - Only errors and warnings
// cargo embed --features three-pi
// 
// # Basic info (v1) - Key operations and status
// cargo embed --features three-pi,v1
// 
// # Full debug (v2) - Everything
// cargo embed --features three-pi,v2
// ```

// Level 2 (Full Debug): Everything including detailed internals
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

// Level 1 (Basic Info): Important operations and status
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

// Always show errors and warnings regardless of verbosity level
#[macro_export]
macro_rules! debug_error {
    ($($arg:tt)*) => { defmt::error!($($arg)*) };
}

#[macro_export]
macro_rules! debug_warn {
    ($($arg:tt)*) => { defmt::warn!($($arg)*) };
}
