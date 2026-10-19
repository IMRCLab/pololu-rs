//! Dual-core TeleOp shared state.
//!
//! Core 1:
//! - receives UART commands
//! - publishes the newest TeleOp command
//!
//! Core 0:
//! - reads the newest command
//! - runs the motor control loop
//!
//! Cross-core objects must use `CriticalSectionRawMutex`.

use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::{
        Mutex as BlockingMutex,
        raw::{
            CriticalSectionRawMutex,
            ThreadModeRawMutex,
        },
    },
    signal::Signal,
};
use heapless::Vec as HVec;
use embassy_time::Instant;
use portable_atomic::{
    AtomicU32,
    Ordering,
};


use crate::{
    packet::CmdTeleopPacketMix,
    read_robot_config_from_sd::RobotConfig,
    uart_parser::{
        RecvResult,
        receive_packet,
    },
};

/// The latest velocity command received by Core 1.
#[derive(Clone, Copy)]
pub struct TeleopCommand {
    pub v: f32,
    pub omega: f32,
    pub stamp: Instant,
}
impl TeleopCommand {
    pub const STOPPED: Self = Self {
        v: 0.0,
        omega: 0.0,
        stamp: Instant::MIN,
    };

    pub fn new(v: f32, omega: f32) -> Self {
        Self {
            v,
            omega,
            stamp: Instant::now(),
        }
    }
}

/// Latest-value mailbox.
///
/// UART commands do not need to be queued. Core 0 only needs the newest
/// velocity command, so a single protected value is sufficient.
static TELEOP_COMMAND: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<TeleopCommand>,
> = BlockingMutex::new(
    RefCell::new(TeleopCommand::STOPPED)
);

/// Publish a new command from Core 1.
pub fn publish_command(command: TeleopCommand) {
    TELEOP_COMMAND.lock(|slot| {
        *slot.borrow_mut() = command;
    });
}

/// Read the newest command from Core 0.
pub fn latest_command() -> TeleopCommand {
    TELEOP_COMMAND.lock(|slot| {
        *slot.borrow()
    })
}

/* ================= Runtime metrics ================= */

static UART_PACKETS: AtomicU32 = AtomicU32::new(0);
static CONTROL_TICKS: AtomicU32 = AtomicU32::new(0);
static CONTROL_MAX_GAP_US: AtomicU32 = AtomicU32::new(0);
static CONTROL_MISSES: AtomicU32 = AtomicU32::new(0);
static DROPPED_LOG_FRAMES: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
pub struct RuntimeMetrics {
    pub uart_packets: u32,
    pub control_ticks: u32,
    pub max_control_gap_us: u32,
    pub control_misses: u32,
    pub dropped_log_frames: u32,
}

/// Called by the Core 1 UART task after receiving a valid command packet.
pub fn record_uart_packet() {
    UART_PACKETS.fetch_add(1, Ordering::Relaxed);
}

/// Called by the Core 0 control task once per control iteration.
pub fn record_control_tick(
    control_gap_us: u32,
    expected_period_us: u32,
) {
    CONTROL_TICKS.fetch_add(1, Ordering::Relaxed);

    CONTROL_MAX_GAP_US.fetch_max(
        control_gap_us,
        Ordering::Relaxed,
    );

    // Count a miss if the interval exceeds 1.5 control periods.
    let miss_threshold_us =
        expected_period_us.saturating_mul(3) / 2;

    if control_gap_us > miss_threshold_us {
        CONTROL_MISSES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Called when the control loop cannot enqueue an SD log frame.
pub fn record_dropped_log_frame() {
    DROPPED_LOG_FRAMES.fetch_add(1, Ordering::Relaxed);
}

/// Return statistics accumulated since the previous call.
///
/// `swap(0)` makes these values suitable for printing once per second.
pub fn take_runtime_metrics() -> RuntimeMetrics {
    RuntimeMetrics {
        uart_packets: UART_PACKETS.swap(
            0,
            Ordering::Relaxed,
        ),

        control_ticks: CONTROL_TICKS.swap(
            0,
            Ordering::Relaxed,
        ),

        max_control_gap_us: CONTROL_MAX_GAP_US.swap(
            0,
            Ordering::Relaxed,
        ),

        control_misses: CONTROL_MISSES.swap(
            0,
            Ordering::Relaxed,
        ),

        dropped_log_frames: DROPPED_LOG_FRAMES.swap(
            0,
            Ordering::Relaxed,
        ),
    }
}


/* ================= Core 1 UART task ================= */

const TELEOP_PACKET_LEN: u8 = 9;

/// This signal is never triggered in the current standalone TeleOp test.
///
/// It exists because `receive_packet()` supports stopping a UART consumer
/// during mode switching. Later, when building the complete dual-core
/// orchestrator, this can be replaced by an actual Core 1 stop signal.
static NEVER_STOP_UART: Signal<ThreadModeRawMutex, ()> =
    Signal::new();

/// Receive TeleOp packets on Core 1 and publish the newest command.
///
/// UART bytes themselves are handled by `uart_hw_task`. This task consumes
/// completed bytes from the UART RX channel, decodes them, and updates the
/// cross-core latest-value mailbox.
#[embassy_executor::task]
pub async fn core1_teleop_uart_task(
    robot_cfg: RobotConfig,
) {
    let mut frame: HVec<u8, 32> = HVec::new();

    loop {
        let (len, packet_stamp) =
            match receive_packet(
                &mut frame,
                &NEVER_STOP_UART,
            )
            .await
            {
                RecvResult::Packet { len, stamp } => {
                    (len, stamp)
                }

                // NEVER_STOP_UART is not signalled in this test.
                RecvResult::Stop => return,
            };

        if len != TELEOP_PACKET_LEN {
            continue;
        }

        let Some(packet) =
            CmdTeleopPacketMix::from_bytes(&frame)
        else {
            continue;
        };

        /* ---------- Linear velocity ---------- */

        let raw_v = packet.linear_velocity;

        let v_sign = if raw_v >= 0.0 {
            1.0
        } else {
            -1.0
        };

        let normalized_v =
            ((raw_v.abs() - 1000.0) / 19000.0)
                .clamp(0.0, 1.0);

        let v = v_sign
            * robot_cfg.max_speed
            * 0.5
            * (libm::expf(2.0 * normalized_v) - 1.0)
            / (libm::expf(2.0) - 1.0);

        /* ---------- Angular velocity ---------- */

        let raw_omega =
            packet.steering_angle
                * core::f32::consts::PI
                / (180.0 * 0.15);

        let omega_sign = if raw_omega >= 0.0 {
            1.0
        } else {
            -1.0
        };

        // Intentionally not clamped.
        let omega = omega_sign
            * robot_cfg.max_omega
            * (
                libm::expf(
                    raw_omega.abs()
                        / robot_cfg.max_omega
                ) - 1.0
            )
            / (libm::expf(1.0) - 1.0);

        publish_command(TeleopCommand {
            v,
            omega,
            stamp: packet_stamp,
        });

        record_uart_packet();
    }
}
