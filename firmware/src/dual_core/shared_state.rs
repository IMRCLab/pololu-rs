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
    }
};
use portable_atomic::{
    AtomicBool,
    AtomicU8,
    Ordering,
};

use super::messages::{
    ControlCommand,
    LoggerCommand,
    RuntimeMode,
    UiCommand,
};

/* ================= Active mode ================= */

static ACTIVE_MODE: AtomicU8 =
    AtomicU8::new(RuntimeMode::Menu as u8);

/// Called by the Core 0 orchestrator after completing a safe mode transition.
pub fn set_active_mode(mode: RuntimeMode) {
    ACTIVE_MODE.store(mode.as_u8(), Ordering::Release);
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

static TELEOP_COMMAND: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<TeleopCommand>,
> = BlockingMutex::new(
    RefCell::new(TeleopCommand::ZERO)
);

static MOCAP_SAMPLE: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<MocapSample>,
> = BlockingMutex::new(
    RefCell::new(MocapSample::ZERO)
);

static IMU_SAMPLE: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<ImuSample>,
> = BlockingMutex::new(
    RefCell::new(ImuSample::ZERO)
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

pub fn publish_mocap_sample(sample: MocapSample) {
    MOCAP_SAMPLE.lock(|slot| {
        *slot.borrow_mut() = sample;
    });
}

pub fn latest_mocap_sample() -> MocapSample {
    MOCAP_SAMPLE.lock(|slot| {
        *slot.borrow()
    })
}

pub fn clear_mocap_sample() {
    publish_mocap_sample(MocapSample::ZERO);
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
}

pub fn trajectory_running() -> bool {
    TRAJECTORY_RUNNING.load(Ordering::Acquire)
}



/* ================= Session reset ================= */

/// Clear mode-specific latest-value inputs.
///
/// Call this during a safe mode transition after the motor output has been
/// forced to zero.
pub fn clear_mode_inputs() {
    clear_teleop_command();
    clear_mocap_sample();
    set_trajectory_running(false);
}