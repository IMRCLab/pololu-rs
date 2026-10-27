//! Cross-core communication primitives.
//!
//! Latest-value mailboxes are used for high-rate replaceable data.
//! Bounded channels are used for commands that must be processed in order.

use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::{
        Mutex as BlockingMutex,
        raw::CriticalSectionRawMutex,
    },
    channel::{
        Channel,
        TryReceiveError,
        TrySendError,
    },
    signal::Signal,
};
use portable_atomic::{
    AtomicBool,
    AtomicU8,
    Ordering,
};

use crate::dual_core::core0::teleop::ControlReference;

use super::messages::{
    ControlCommand,
    LoggerCommand,
    RuntimeMode,
    UiCommand,
};

/* ================= Active mode ================= */

static ACTIVE_MODE: AtomicU8 =
    AtomicU8::new(RuntimeMode::Menu as u8);

static TRAJECTORY_STATE_CHANGED: Signal<
    CriticalSectionRawMutex,
    (),
> = Signal::new();

/// Called by the Core 0 orchestrator after completing a safe mode transition.
pub fn set_active_mode(mode: RuntimeMode) {
    ACTIVE_MODE.store(mode.as_u8(), Ordering::Release);
    TRAJECTORY_STATE_CHANGED.signal(());
}

/// Can be read by tasks on either core.
pub fn active_mode() -> RuntimeMode {
    RuntimeMode::from_selection(
        ACTIVE_MODE.load(Ordering::Acquire),
    )
    .unwrap_or(RuntimeMode::Menu)
}

/* ================= Latest-value data ================= */

#[derive(Clone, Copy)]
pub struct TeleopCommand {
    pub v: f32,
    pub omega: f32,

    /// Timestamp captured when the UART packet was received.
    pub stamp_us: u64,
}

impl TeleopCommand {
    pub const ZERO: Self = Self {
        v: 0.0,
        omega: 0.0,
        stamp_us: 0,
    };
}

#[derive(Clone, Copy)]
pub struct MocapSample {
    pub x: f32,
    pub y: f32,
    pub z: f32,

    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    pub stamp_us: u64,
}

impl MocapSample {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,

        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,

        stamp_us: 0,
    };
}

struct MocapMailbox {
    sample: MocapSample,
    fresh: bool,
}

impl MocapMailbox {
    const EMPTY: Self = Self {
        sample: MocapSample::ZERO,
        fresh: false,
    };
}

#[derive(Clone, Copy)]
pub struct ImuSample {
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,

    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,

    /// Complementary-filter orientation in radians.
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    pub stamp_us: u64,
}

impl ImuSample {
    pub const ZERO: Self = Self {
        acc_x: 0.0,
        acc_y: 0.0,
        acc_z: 0.0,

        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,

        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,

        stamp_us: 0,
    };
}

#[derive(Clone, Copy)]
pub struct OdometrySample {
    pub v: f32,
    pub omega: f32,
    pub stamp_us: u64,
}

impl OdometrySample {
    pub const ZERO: Self = Self {
        v: 0.0,
        omega: 0.0,
        stamp_us: 0,
    };
}

#[derive(Clone, Copy)]
pub struct TrajectoryOutput {
    pub stamp_us: u64,
    pub valid: bool,

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

    // Tracking error
    pub x_error: f32,
    pub y_error: f32,
    pub yaw_error: f32,
}

impl TrajectoryOutput {
    pub const ZERO: Self = Self {
        stamp_us: 0,
        valid: false,

        x: 0.0,
        y: 0.0,
        yaw: 0.0,

        // Trajectory reference
        x_des: 0.0,
        y_des: 0.0,
        yaw_des: 0.0,

        // Feedforward
        v_ff: 0.0,
        omega_ff: 0.0,

        // Tracking error
        x_error: 0.0,
        y_error: 0.0,
        yaw_error: 0.0,
    };
}

static TELEOP_COMMAND: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<TeleopCommand>,
> = BlockingMutex::new(
    RefCell::new(TeleopCommand::ZERO)
);

static TRAJ_COMMAND: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<ControlReference>,
> = BlockingMutex::new(
    RefCell::new(ControlReference::ZERO)
);

static MOCAP_SAMPLE: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<MocapMailbox>,
> = BlockingMutex::new(
    RefCell::new(MocapMailbox::EMPTY)
);

static IMU_SAMPLE: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<ImuSample>,
> = BlockingMutex::new(
    RefCell::new(ImuSample::ZERO)
);

static ODOMETRY_SAMPLE: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<OdometrySample>,
> = BlockingMutex::new(
    RefCell::new(OdometrySample::ZERO)
);

static TRAJECTORY_OUTPUT: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<TrajectoryOutput>,
> = BlockingMutex::new(
    RefCell::new(TrajectoryOutput::ZERO)
);

/* ================= Mailbox access ================= */

pub fn publish_teleop_command(command: TeleopCommand) {
    TELEOP_COMMAND.lock(|slot| {
        *slot.borrow_mut() = command;
    });
}

pub fn latest_teleop_command() -> TeleopCommand {
    TELEOP_COMMAND.lock(|slot| {
        *slot.borrow()
    })
}

pub fn clear_teleop_command() {
    publish_teleop_command(TeleopCommand::ZERO);
}

pub fn publish_traj_command(command: ControlReference) {
    TRAJ_COMMAND.lock(|slot| {
        *slot.borrow_mut() = command;
    });
}

pub fn latest_traj_command() -> ControlReference {
    TRAJ_COMMAND.lock(|slot| {
        *slot.borrow()
    })
}

pub fn clear_traj_command() {
    publish_traj_command(ControlReference::ZERO);
}

pub fn publish_mocap_sample(sample: MocapSample) {
    MOCAP_SAMPLE.lock(|slot| {
        let mut mailbox = slot.borrow_mut();
        mailbox.sample = sample;
        mailbox.fresh = true;
    });
}

pub fn latest_mocap_sample() -> MocapSample {
    MOCAP_SAMPLE.lock(|slot| {
        slot.borrow().sample
    })
}

pub fn take_fresh_mocap_sample() -> Option<MocapSample> {
    MOCAP_SAMPLE.lock(|slot| {
        let mut mailbox = slot.borrow_mut();
        if mailbox.fresh {
            mailbox.fresh = false;
            Some(mailbox.sample)
        } else {
            None
        }
    })
}

pub fn clear_mocap_sample() {
    MOCAP_SAMPLE.lock(|slot| {
        *slot.borrow_mut() = MocapMailbox::EMPTY;
    });
}

pub fn publish_imu_sample(sample: ImuSample) {
    IMU_SAMPLE.lock(|slot| {
        *slot.borrow_mut() = sample;
    });
}

pub fn latest_imu_sample() -> ImuSample {
    IMU_SAMPLE.lock(|slot| {
        *slot.borrow()
    })
}

pub fn clear_imu_sample() {
    publish_imu_sample(ImuSample::ZERO);
}

/// Published by the Core 0 wheel controller for trajectory-state estimation.
pub fn publish_odometry_sample(sample: OdometrySample) {
    ODOMETRY_SAMPLE.lock(|slot| {
        *slot.borrow_mut() = sample;
    });
}

pub fn latest_odometry_sample() -> OdometrySample {
    ODOMETRY_SAMPLE.lock(|slot| {
        *slot.borrow()
    })
}

/// Published by the Core 0 trajectory task and consumed by the motor/logging loop.
pub fn publish_trajectory_output(output: TrajectoryOutput) {
    TRAJECTORY_OUTPUT.lock(|slot| {
        *slot.borrow_mut() = output;
    });
}

pub fn latest_trajectory_output() -> TrajectoryOutput {
    TRAJECTORY_OUTPUT.lock(|slot| {
        *slot.borrow()
    })
}

pub fn clear_trajectory_output() {
    publish_trajectory_output(TrajectoryOutput::ZERO);
}

/* ================= Reliable command channels ================= */

/// Core 1 UART router -> Core 0 control/orchestrator.
///
/// Mode changes and parameter writes must not be silently overwritten.
static CONTROL_COMMAND_CHANNEL:
    Channel<
        CriticalSectionRawMutex,
        ControlCommand,
        16,
    > = Channel::new();

/// Core 0 orchestrator -> Core 1 SD logging service.
static LOGGER_COMMAND_CHANNEL:
    Channel<
        CriticalSectionRawMutex,
        LoggerCommand,
        8,
    > = Channel::new();


/// Core 0 orchestrator -> Core 1 LED/buzzer service.
static UI_COMMAND_CHANNEL:
    Channel<
        CriticalSectionRawMutex,
        UiCommand,
        16,
    > = Channel::new();


static PARAMETER_UPDATE_CHANNEL:
    Channel<CriticalSectionRawMutex, (u8, f32), 16> = Channel::new();

/// Reliably send an ordered command from Core 1 to Core 0.
///
/// The caller may wait when the channel is full. This is acceptable for mode
/// and parameter commands, but this function must never be called by the
/// real-time control loop.
pub async fn send_control_command(
    command: ControlCommand,
) {
    CONTROL_COMMAND_CHANNEL.send(command).await;
}

/// Non-blocking variant for contexts that must not wait.
pub fn try_send_control_command(
    command: ControlCommand,
) -> Result<(), ControlCommand> {
    CONTROL_COMMAND_CHANNEL
        .try_send(command)
        .map_err(|error| match error {
            TrySendError::Full(command) => command,
        })
}

/// Called by the Core 0 orchestrator.
pub async fn receive_control_command() -> ControlCommand {
    CONTROL_COMMAND_CHANNEL.receive().await
}

/// Send a command to the Core 1 logging service.
pub async fn send_logger_command(
    command: LoggerCommand,
) {
    LOGGER_COMMAND_CHANNEL.send(command).await;
}

/// Non-blocking variant for safety-sensitive Core 0 code.
pub fn try_send_logger_command(
    command: LoggerCommand,
) -> Result<(), LoggerCommand> {
    LOGGER_COMMAND_CHANNEL
        .try_send(command)
        .map_err(|error| match error {
            TrySendError::Full(command) => command,
        })
}

/// Called by the single Core 1 logging task.
pub async fn receive_logger_command() -> LoggerCommand {
    LOGGER_COMMAND_CHANNEL.receive().await
}


/// Send a UI command from non-real-time code.
pub async fn send_ui_command(command: UiCommand) {
    UI_COMMAND_CHANNEL.send(command).await;
}

/// Non-blocking UI notification for safety-sensitive Core 0 code.
pub fn try_send_ui_command(
    command: UiCommand,
) -> Result<(), UiCommand> {
    UI_COMMAND_CHANNEL
        .try_send(command)
        .map_err(|error| match error {
            TrySendError::Full(command) => command,
        })
}

/// Called only by the Core 1 UI task.
pub async fn receive_ui_command() -> UiCommand {
    UI_COMMAND_CHANNEL.receive().await
}



pub async fn send_parameter_update(parameter_id: u8, value: f32) {
    PARAMETER_UPDATE_CHANNEL.send((parameter_id, value)).await;
}

/// Called by the Core 0 control loop without waiting.
pub fn try_receive_parameter_update() -> Option<(u8, f32)> {
    match PARAMETER_UPDATE_CHANNEL.try_receive() {
        Ok(update) => Some(update),
        Err(TryReceiveError::Empty) => None,
    }
}



static TRAJECTORY_RUNNING: AtomicBool = AtomicBool::new(false);

pub fn set_trajectory_running(running: bool) {
    TRAJECTORY_RUNNING.store(running, Ordering::Release);
    TRAJECTORY_STATE_CHANGED.signal(());
}

pub fn trajectory_running() -> bool {
    TRAJECTORY_RUNNING.load(Ordering::Acquire)
}

pub async fn wait_trajectory_state_changed() {
    TRAJECTORY_STATE_CHANGED.wait().await;
}



/* ================= Session reset ================= */

/// Clear mode-specific latest-value inputs.
///
/// Call this during a safe mode transition after the motor output has been
/// forced to zero.
pub fn clear_mode_inputs() {
    clear_teleop_command();
    clear_traj_command();
    clear_mocap_sample();
    clear_trajectory_output();
    set_trajectory_running(false);
}