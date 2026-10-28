//! Dual-core application architecture.
//!
//! Core 0 owns deterministic control.
//! Core 1 owns UART, SD logging, and other asynchronous I/O.

pub mod core0;
pub mod core1;
pub mod logging;
pub mod messages;
pub mod metrics;
pub mod orchestrator;
pub mod shared_state;