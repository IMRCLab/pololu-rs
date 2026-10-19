//! Unified logging state for all dual-core operating modes.
//!
//! Every mode uses the same fixed-size schema. Fields that do not apply to the
//! current mode remain zero.
//!
//! Core 0 builds complete frames and submits them without waiting. Core 1 is
//! the only consumer and owns all SD serialization and file I/O.

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{
        Channel,
        TrySendError,
    },
};
use heapless::Vec;

use portable_atomic::{
    AtomicU32,
    Ordering,
};

use super::messages::RuntimeMode;

/* ================= Validity flags ================= */

/// Command fields contain valid data.
pub const LOG_VALID_COMMAND: u32 = 1 << 0;

/// Pose fields contain valid data.
pub const LOG_VALID_POSE: u32 = 1 << 1;

/// Trajectory setpoint fields contain valid data.
pub const LOG_VALID_SETPOINT: u32 = 1 << 2;

/// Wheel target fields contain valid data.
pub const LOG_VALID_WHEEL_TARGET: u32 = 1 << 3;

/// Encoder/wheel measurement fields contain valid data.
pub const LOG_VALID_WHEEL_MEASUREMENT: u32 = 1 << 4;

/// Motor duty fields contain valid data.
pub const LOG_VALID_MOTOR: u32 = 1 << 5;

/// Tracking error fields contain valid data.
pub const LOG_VALID_TRACKING_ERROR: u32 = 1 << 6;

/// IMU fields contain valid data.
pub const LOG_VALID_IMU: u32 = 1 << 7;

/// Timing fields contain valid data.
pub const LOG_VALID_TIMING: u32 = 1 << 8;

/* ================= Unified schema ================= */

#[derive(Clone, Copy)]
pub struct UnifiedLogFrame {
    /// Monotonically increasing log row number.
    pub sequence: u32,

    /// Snapshot creation time.
    pub t_us: u64,

    /// Current `RuntimeMode` encoded as `u8`.
    pub mode: u8,

    /// Indicates which groups contain meaningful values.
    ///
    /// Fields not marked valid must still remain zero.
    pub valid_mask: u32,

    // Command
    pub command_v: f32,
    pub command_omega: f32,

    // Estimated/current pose
    pub x: f32,
    pub y: f32,
    pub yaw: f32,

    // Trajectory reference
    pub x_des: f32,
    pub y_des: f32,
    pub yaw_des: f32,

    // Feedforward
    pub v_ff: f32,
    pub omega_ff: f32,

    // Wheel velocity targets
    pub omega_l_target: f32,
    pub omega_r_target: f32,

    // Measured wheel velocities
    pub omega_l_measured: f32,
    pub omega_r_measured: f32,

    // Motor output
    pub duty_l: f32,
    pub duty_r: f32,

    // Tracking error
    pub x_error: f32,
    pub y_error: f32,
    pub yaw_error: f32,

    // IMU
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,

    // Runtime timing
    pub control_gap_us: u32,
    pub control_misses: u32,
    pub dropped_log_frames: u32,
}

impl UnifiedLogFrame {
    pub const ZERO: Self = Self {
        sequence: 0,
        t_us: 0,
        mode: RuntimeMode::Menu as u8,
        valid_mask: 0,

        command_v: 0.0,
        command_omega: 0.0,

        x: 0.0,
        y: 0.0,
        yaw: 0.0,

        x_des: 0.0,
        y_des: 0.0,
        yaw_des: 0.0,

        v_ff: 0.0,
        omega_ff: 0.0,

        omega_l_target: 0.0,
        omega_r_target: 0.0,

        omega_l_measured: 0.0,
        omega_r_measured: 0.0,

        duty_l: 0.0,
        duty_r: 0.0,

        x_error: 0.0,
        y_error: 0.0,
        yaw_error: 0.0,

        acc_x: 0.0,
        acc_y: 0.0,
        acc_z: 0.0,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,

        control_gap_us: 0,
        control_misses: 0,
        dropped_log_frames: 0,
    };


    /// Start a frame for a mode with all unavailable fields set to zero.
    pub const fn for_mode(mode: RuntimeMode) -> Self {
        Self {
            mode: mode as u8,
            ..Self::ZERO
        }
    }


    /// Serialize explicitly instead of relying on the Rust memory layout.
    ///
    /// This produces a stable little-endian binary format without struct
    /// padding bytes.
    pub fn to_bytes(&self) -> Vec<u8, 192> {
        let mut bytes = Vec::new();

        let _ = bytes.extend_from_slice(
            &self.sequence.to_le_bytes(),
        );

        let _ = bytes.extend_from_slice(
            &self.t_us.to_le_bytes(),
        );

        let _ = bytes.push(self.mode);

        let _ = bytes.extend_from_slice(
            &self.valid_mask.to_le_bytes(),
        );

        for value in [
            self.command_v,
            self.command_omega,

            self.x,
            self.y,
            self.yaw,

            self.x_des,
            self.y_des,
            self.yaw_des,

            self.v_ff,
            self.omega_ff,

            self.omega_l_target,
            self.omega_r_target,

            self.omega_l_measured,
            self.omega_r_measured,

            self.duty_l,
            self.duty_r,

            self.x_error,
            self.y_error,
            self.yaw_error,

            self.acc_x,
            self.acc_y,
            self.acc_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
        ] {
            let _ = bytes.extend_from_slice(
                &value.to_le_bytes(),
            );
        }

        for value in [
            self.control_gap_us,
            self.control_misses,
            self.dropped_log_frames,
        ] {
            let _ = bytes.extend_from_slice(
                &value.to_le_bytes(),
            );
        }

        bytes
    }
}

impl Default for UnifiedLogFrame {
    fn default() -> Self {
        Self::ZERO
    }
}



/* ================= Core 0 -> Core 1 transport ================= */
/// About 1.28 seconds of buffering at a 50 Hz logging rate.
const LOG_FRAME_CHANNEL_SIZE: usize = 64;

/// Core 0 produces complete frames. Core 1 is the only consumer.
static LOG_FRAME_CHANNEL: Channel<
    CriticalSectionRawMutex,
    UnifiedLogFrame,
    LOG_FRAME_CHANNEL_SIZE,
> = Channel::new();

static DROPPED_LOG_FRAMES: AtomicU32 =
    AtomicU32::new(0);

/// Core 0-side publisher state.
///
/// Keep one instance inside the permanent control task. Publishing never
/// waits for Core 1 or the SD card.
pub struct LogFramePublisher {
    sequence: u32,
}

impl LogFramePublisher {
    pub const fn new() -> Self {
        Self {
            sequence: 0,
        }
    }

    /// Stamp and enqueue one complete frame without blocking.
    pub fn try_publish(
        &mut self,
        mut frame: UnifiedLogFrame,
        t_us: u64,
    ) -> Result<(), UnifiedLogFrame> {
        self.sequence =
            self.sequence.wrapping_add(1);

        frame.sequence = self.sequence;
        frame.t_us = t_us;
        frame.dropped_log_frames =
            dropped_log_frames();

        LOG_FRAME_CHANNEL
            .try_send(frame)
            .map_err(|error| match error {
                TrySendError::Full(frame) => {
                    DROPPED_LOG_FRAMES.fetch_add(
                        1,
                        Ordering::Relaxed,
                    );

                    frame
                }
            })
    }
}

impl Default for LogFramePublisher {
    fn default() -> Self {
        Self::new()
    }
}

/// Called only by the single Core 1 SD logging task.
pub async fn receive_log_frame() -> UnifiedLogFrame {
    LOG_FRAME_CHANNEL.receive().await
}

/// Total number of frames dropped because Core 1 could not drain the queue
/// quickly enough.
pub fn dropped_log_frames() -> u32 {
    DROPPED_LOG_FRAMES.load(Ordering::Relaxed)
}