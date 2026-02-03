//! Robot State Management
//!
//! Centralized state management for the robot:
//! - Mutex<T> for shared state with multiple readers (latest value)
//! - Channel<T> for producer-consumer communication (commands/events)
//!
//! Migration note:
//! - trajectory_signal::WheelCmd -> robotstate::WheelCmd
//! - trajectory_signal::WHEEL_CMD_CH -> robotstate::WHEEL_CMD_CH  
//! - trajectory_signal::PoseAbs -> robotstate::Pose (alias kept for compatibility)
//! - trajectory_signal::LAST_STATE -> robotstate::POSE (legacy kept for now)

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use heapless::Vec;

// =============================================================================
//                            CHANNEL CAPACITIES
// =============================================================================
const CMD_CHANNEL_SIZE: usize = 4;
const SENSOR_CHANNEL_SIZE: usize = 4;

// =============================================================================
//                          SHARED STATE (Mutex)
// =============================================================================

/// Current robot pose
/// Writer: UART task (mocap data)
/// Readers: Position Controller, Logger
pub static POSE: Mutex<ThreadModeRawMutex, Pose> = Mutex::new(Pose::DEFAULT);

/// Current trajectory setpoint
/// Writer: Trajectory reader (from SD card)
/// Readers: Position Controller, Logger
pub static SETPOINT: Mutex<ThreadModeRawMutex, Setpoint> = Mutex::new(Setpoint::DEFAULT);


///Current traj waypoint (setpoint minus vel)
/// Writer: UART
/// Reader: Traj controller single waypoint
pub static WAYPOINT: Mutex<ThreadModeRawMutex, Waypoint> = Mutex::new(Waypoint::DEFAULT);


/// Current encoder readings (measured wheel speeds)
/// Writer: Inner Loop (after low-pass filtering)
/// Readers: Inner Loop (PI controller), Logger
pub static ENCODER: Mutex<ThreadModeRawMutex, EncoderReading> = Mutex::new(EncoderReading::DEFAULT);

/// Current motor duty cycles
/// Writer: Inner Loop (PI controller output)
/// Readers: Logger
pub static MOTOR: Mutex<ThreadModeRawMutex, MotorDuty> = Mutex::new(MotorDuty::DEFAULT);

/// Current tracking errors (in body frame, from position controller)
/// Writer: Position Controller (trajectory_control.rs)
/// Readers: Logger
pub static TRACKING_ERROR: Mutex<ThreadModeRawMutex, TrackingError> =
    Mutex::new(TrackingError::DEFAULT);

/// Current wheel command (latest value for logging)
/// Writer: Position Controller / TeleOp
/// Readers: Logger (SD, Radio)
pub static WHEEL_CMD: Mutex<ThreadModeRawMutex, WheelCmd> = Mutex::new(WheelCmd::DEFAULT);

/// Current IMU reading (latest value for logging)
/// Writer: IMU Task
/// Readers: Logger
/// note: currently not in use
pub static IMU: Mutex<ThreadModeRawMutex, ImuReading> = Mutex::new(ImuReading::DEFAULT);

// =============================================================================
//                              CHANNELS
// =============================================================================

/// Wheel speed commands: Position Controller/TeleOp → Inner Loop
/// Contains desired wheel angular velocities (rad/s)
/// Inner Loop uses PI control to achieve these speeds
pub static WHEEL_CMD_CH: Channel<ThreadModeRawMutex, WheelCmd, CMD_CHANNEL_SIZE> = Channel::new();

/// Encoder readings: Inner Loop → External streaming consumer (optional)
/// Note: Inner Loop uses ENCODER mutex directly for PI control feedback.
/// This channel is for streaming raw/filtered readings to external tasks if needed.
pub static ENCODER_CH: Channel<ThreadModeRawMutex, EncoderReading, SENSOR_CHANNEL_SIZE> =
    Channel::new();

// =============================================================================
//                          DATA STRUCTURES
// =============================================================================

/// Robot pose in world frame
#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub x: f32,     // meters
    pub y: f32,     // meters
    pub z: f32,     // meters
    pub roll: f32,  // rad
    pub pitch: f32, // rad
    pub yaw: f32,   // rad
    pub stamp: Instant,
}

impl Pose {
    pub const DEFAULT: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        stamp: Instant::MIN,
    };
}

impl Default for Pose {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Trajectory setpoint (desired state from trajectory planner)
#[derive(Debug, Clone, Copy)]
pub struct Setpoint {
    pub x_des: f32,   // meters
    pub y_des: f32,   // meters
    pub yaw_des: f32, // rad
    pub v_ff: f32,    // feedforward linear velocity [m/s]
    pub w_ff: f32,    // feedforward angular velocity [rad/s]
}


impl Setpoint {
    pub const DEFAULT: Self = Self {
        x_des: 0.0,
        y_des: 0.0,
        yaw_des: 0.0,
        v_ff: 0.0,
        w_ff: 0.0,
    };
}

impl Default for Setpoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

///Trajectroy waypoint, is a setpoint minus the veloceties
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub x_des: f32,   // meters
    pub y_des: f32,   // meters
    pub yaw_des: f32, // rad
}


impl Waypoint {
    pub const DEFAULT: Self = Self {
        x_des: 0.0,
        y_des: 0.0,
        yaw_des: 0.0,
    };
}

impl Default for Waypoint {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wheel speed command (output of position controller / teleop)
#[derive(Debug, Clone, Copy)]
pub struct WheelCmd {
    pub omega_l: f32, // left wheel angular velocity [rad/s]
    pub omega_r: f32, // right wheel angular velocity [rad/s]
    pub stamp: Instant,
}

impl WheelCmd {
    pub const DEFAULT: Self = Self {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::MIN,
    };

    pub fn new(omega_l: f32, omega_r: f32) -> Self {
        Self {
            omega_l,
            omega_r,
            stamp: Instant::now(),
        }
    }
}

impl Default for WheelCmd {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Motor duty cycle output
#[derive(Debug, Clone, Copy)]
pub struct MotorDuty {
    pub left: f32,  // duty cycle [-1, 1]
    pub right: f32, // duty cycle [-1, 1]
}

impl MotorDuty {
    pub const DEFAULT: Self = Self {
        left: 0.0,
        right: 0.0,
    };
}

impl Default for MotorDuty {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Tracking errors in body frame (computed by position controller)
#[derive(Debug, Clone, Copy)]
pub struct TrackingError {
    pub x_err: f32,   // body-frame x error [m]
    pub y_err: f32,   // body-frame y error [m]
    pub yaw_err: f32, // yaw error [rad]
}

impl TrackingError {
    pub const DEFAULT: Self = Self {
        x_err: 0.0,
        y_err: 0.0,
        yaw_err: 0.0,
    };
}

impl Default for TrackingError {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Encoder readings (measured wheel speeds)
#[derive(Debug, Clone, Copy)]
pub struct EncoderReading {
    pub omega_l: f32, // left wheel [rad/s]
    pub omega_r: f32, // right wheel [rad/s]
    pub stamp: Instant,
}

impl EncoderReading {
    pub const DEFAULT: Self = Self {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::MIN,
    };
}

impl Default for EncoderReading {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// IMU readings
#[derive(Debug, Clone, Copy)]
pub struct ImuReading {
    pub acc_x: f32, // m/s²
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32, // rad/s
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub stamp: Instant,
}

impl ImuReading {
    pub const DEFAULT: Self = Self {
        acc_x: 0.0,
        acc_y: 0.0,
        acc_z: 0.0,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,
        stamp: Instant::MIN,
    };
}

impl Default for ImuReading {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Complete state snapshot for logging
#[derive(Debug, Clone, Copy)]
pub struct LogSnapshot {
    pub t_ms: u32,

    // Pose
    pub x: f32,
    pub y: f32,
    pub yaw: f32,

    // Setpoint
    pub x_des: f32,
    pub y_des: f32,
    pub yaw_des: f32,

    // Feedforward
    pub v_ff: f32,
    pub w_ff: f32,

    // Wheel command
    pub omega_l_cmd: f32,
    pub omega_r_cmd: f32,

    // Encoder (actual speeds)
    pub omega_l_meas: f32,
    pub omega_r_meas: f32,

    // Motor output
    pub duty_l: f32,
    pub duty_r: f32,

    // Errors
    pub x_err: f32,
    pub y_err: f32,
    pub yaw_err: f32,
}

impl LogSnapshot {
    pub const DEFAULT: Self = Self {
        t_ms: 0, //t
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
        x_des: 0.0,
        y_des: 0.0,
        yaw_des: 0.0,
        v_ff: 0.0,
        w_ff: 0.0,
        omega_l_cmd: 0.0,
        omega_r_cmd: 0.0,
        omega_l_meas: 0.0,
        omega_r_meas: 0.0,
        duty_l: 0.0,
        duty_r: 0.0,
        x_err: 0.0,
        y_err: 0.0,
        yaw_err: 0.0,
    };

    /// Pack as f32 (full precision, 72 bytes payload)
    pub fn to_bytes_f32(&self) -> Vec<u8, 80> {
        let mut v: Vec<u8, 80> = Vec::new();

        v.extend_from_slice(&self.t_ms.to_le_bytes()).ok();
        for f in [
            self.x,
            self.y,
            self.yaw,
            self.x_des,
            self.y_des,
            self.yaw_des,
            self.v_ff,
            self.w_ff,
            self.omega_l_cmd,
            self.omega_r_cmd,
            self.omega_l_meas,
            self.omega_r_meas,
            self.duty_l,
            self.duty_r,
            self.x_err,
            self.y_err,
            self.yaw_err,
        ] {
            v.extend_from_slice(&f.to_le_bytes()).ok();
        }
        v
    }

    /// Pack as i16 fixed-point (compact, 38 bytes payload)
    pub fn to_bytes_compact(&self) -> Vec<u8, 48> {
        let mut v: Vec<u8, 48> = Vec::new();

        v.extend_from_slice(&self.t_ms.to_le_bytes()).ok();

        //position: mm precision
        v.extend_from_slice(&to_i16_mm(self.x).to_le_bytes()).ok();
        v.extend_from_slice(&to_i16_mm(self.y).to_le_bytes()).ok();
        v.extend_from_slice(&to_i16_mm(self.x_des).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_mm(self.y_des).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_mm(self.x_err).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_mm(self.y_err).to_le_bytes())
            .ok();

        //angles: re1/100 rad precision
        v.extend_from_slice(&to_i16_centirad(self.yaw).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.yaw_des).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.yaw_err).to_le_bytes())
            .ok();

        //velocities
        v.extend_from_slice(&to_i16_mm(self.v_ff).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.w_ff).to_le_bytes())
            .ok();

        //wheel speeds
        v.extend_from_slice(&to_i16_centirad(self.omega_l_cmd).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.omega_r_cmd).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.omega_l_meas).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_centirad(self.omega_r_meas).to_le_bytes())
            .ok();

        //duty cycles
        v.extend_from_slice(&to_i16_duty(self.duty_l).to_le_bytes())
            .ok();
        v.extend_from_slice(&to_i16_duty(self.duty_r).to_le_bytes())
            .ok();

        v
    }
}

impl Default for LogSnapshot {
    fn default() -> Self {
        Self::DEFAULT
    }
}

// =============================================================================
//                       FIXED-POINT CONVERSION HELPERS
// =============================================================================

/// Convert f32 meters to i16 millimeters (±32.7m range, 1mm precision)
#[inline]
fn to_i16_mm(val: f32) -> i16 {
    (val * 1000.0).clamp(-32768.0, 32767.0) as i16
}

/// Convert f32 radians to i16 centiradians (±327 rad range, 0.01 rad precision)
#[inline]
fn to_i16_centirad(val: f32) -> i16 {
    (val * 100.0).clamp(-32768.0, 32767.0) as i16
}

/// Convert f32 duty cycle to i16 (±3.27 range, 0.0001 precision)
#[inline]
fn to_i16_duty(val: f32) -> i16 {
    (val * 10000.0).clamp(-32768.0, 32767.0) as i16
}

// =============================================================================
//                         POSE READ/WRITE
// =============================================================================

//called in: trajectory uart (mocap data)
pub async fn write_pose(pose: Pose) {
    let mut guard = POSE.lock().await;
    *guard = pose;
}

//called in: position controller, logger
pub async fn read_pose() -> Pose {
    *POSE.lock().await
}

// =============================================================================
//                        SETPOINT READ/WRITE
// =============================================================================

//called in: trajectory reader (from SD card)
pub async fn write_setpoint(setpoint: Setpoint) {
    let mut guard = SETPOINT.lock().await;
    *guard = setpoint;
}

//called in: position controller, logger
pub async fn read_setpoint() -> Setpoint {
    *SETPOINT.lock().await
}
// =============================================================================
//                        Waypoint READ/WRITE
// =============================================================================

//called in: trajectory reader (from SD card)
pub async fn write_waypoint(waypoint: Waypoint) {
    let mut guard = WAYPOINT.lock().await;
    *guard = waypoint;
}

//called in: position controller, logger
pub async fn read_waypoint() -> Waypoint {
    *WAYPOINT.lock().await
}



// =============================================================================
//                        ENCODER READ/WRITE
// =============================================================================

/// Write encoder reading to mutex
/// Called in: Inner Loop (after low-pass filtering raw encoder counts)
pub async fn write_encoder(enc: EncoderReading) {
    let mut guard = ENCODER.lock().await;
    *guard = enc;
}

/// Read latest encoder reading
/// Called in: Inner Loop (PI controller feedback), Logger
pub async fn read_encoder() -> EncoderReading {
    *ENCODER.lock().await
}

// =============================================================================
//                         MOTOR READ/WRITE
// =============================================================================

//called in: inner loop (PI controller output)
pub async fn write_motor(duty: MotorDuty) {
    let mut guard = MOTOR.lock().await;
    *guard = duty;
}

//called in: logger
pub async fn read_motor() -> MotorDuty {
    *MOTOR.lock().await
}

// =============================================================================
//                      TRACKING ERROR READ/WRITE
// =============================================================================

//called in: position controller after computing errors
pub async fn write_tracking_error(err: TrackingError) {
    let mut guard = TRACKING_ERROR.lock().await;
    *guard = err;
}

//called in: logger
pub async fn read_tracking_error() -> TrackingError {
    *TRACKING_ERROR.lock().await
}

// =============================================================================
//                      WHEEL COMMAND READ/WRITE
// =============================================================================

/// Write wheel command to mutex (for logging) AND send to channel (for inner loop)
/// Called in: position controller, teleop
pub async fn write_wheel_cmd(cmd: WheelCmd) {
    // Update mutex for loggers
    {
        let mut guard = WHEEL_CMD.lock().await;
        *guard = cmd;
    }
    // Send to channel for inner loop (drain old commands first)
    while WHEEL_CMD_CH.try_receive().is_ok() {}
    let _ = WHEEL_CMD_CH.try_send(cmd);
}

/// Read latest wheel command (for logging)
/// Called in: logger (SD, Radio)
pub async fn read_wheel_cmd() -> WheelCmd {
    *WHEEL_CMD.lock().await
}

/// Stop motors immediately (sync, non-blocking)
/// Only updates channel, not mutex - use for emergency stops
/// Called in: trajectory control stop handlers
pub fn stop_motors() {
    while WHEEL_CMD_CH.try_receive().is_ok() {}
    let _ = WHEEL_CMD_CH.try_send(WheelCmd::new(0.0, 0.0));
}

// =============================================================================
//                         IMU READ/WRITE
// =============================================================================

/// Write IMU reading to mutex
/// Called in: IMU task
pub async fn write_imu(imu: ImuReading) {
    let mut guard = IMU.lock().await;
    *guard = imu;
}

/// Read latest IMU reading (for logging)
/// Called in: logger
pub async fn read_imu() -> ImuReading {
    *IMU.lock().await
}

// =============================================================================
//                      WHEEL COMMAND CHANNEL (legacy)
// =============================================================================

//called in: position controller, teleop
//non-blocking, drains old commands to keep only latest
//DEPRECATED: prefer write_wheel_cmd() which updates both mutex and channel
pub fn send_wheel_cmd(cmd: WheelCmd) {
    while WHEEL_CMD_CH.try_receive().is_ok() {}
    let _ = WHEEL_CMD_CH.try_send(cmd);
}

//called in: inner loop
//non-blocking, returns None if no command available
pub fn try_recv_wheel_cmd() -> Option<WheelCmd> {
    WHEEL_CMD_CH.try_receive().ok()
}

//called in: inner loop
//blocking, waits for command
pub async fn recv_wheel_cmd() -> WheelCmd {
    WHEEL_CMD_CH.receive().await
}

// =============================================================================
//                         LOG SNAPSHOT HELPERS
// =============================================================================

//builds a complete log snapshot from current mutex values + local errors
//called in: position controller after computing control
pub async fn build_log_snapshot(
    wheel_cmd: &WheelCmd,
    x_err: f32,
    y_err: f32,
    yaw_err: f32,
) -> LogSnapshot {
    let pose = read_pose().await;
    let setpoint = read_setpoint().await;
    let encoder = read_encoder().await;
    let motor = read_motor().await;

    LogSnapshot {
        t_ms: Instant::now().as_millis() as u32,
        x: pose.x,
        y: pose.y,
        yaw: pose.yaw,
        x_des: setpoint.x_des,
        y_des: setpoint.y_des,
        yaw_des: setpoint.yaw_des,
        v_ff: setpoint.v_ff,
        w_ff: setpoint.w_ff,
        omega_l_cmd: wheel_cmd.omega_l,
        omega_r_cmd: wheel_cmd.omega_r,
        omega_l_meas: encoder.omega_l,
        omega_r_meas: encoder.omega_r,
        duty_l: motor.left,
        duty_r: motor.right,
        x_err,
        y_err,
        yaw_err,
    }
}

/// Builds a complete log snapshot reading ALL values from mutexes
/// including tracking errors. Use this in periodic UART logging tasks.
pub async fn build_log_snapshot_from_state() -> LogSnapshot {
    let pose = read_pose().await;
    let setpoint = read_setpoint().await;
    let encoder = read_encoder().await;
    let motor = read_motor().await;
    let err = read_tracking_error().await;
    let wheel_cmd = read_wheel_cmd().await;
    //TODO: read IMU if needed
    //add it to the log snapshot

    LogSnapshot {
        t_ms: Instant::now().as_millis() as u32,
        x: pose.x,
        y: pose.y,
        yaw: pose.yaw,
        x_des: setpoint.x_des,
        y_des: setpoint.y_des,
        yaw_des: setpoint.yaw_des,
        v_ff: setpoint.v_ff,
        w_ff: setpoint.w_ff,
        omega_l_cmd: wheel_cmd.omega_l,
        omega_r_cmd: wheel_cmd.omega_r,
        omega_l_meas: encoder.omega_l,
        omega_r_meas: encoder.omega_r,
        duty_l: motor.left,
        duty_r: motor.right,
        x_err: err.x_err,
        y_err: err.y_err,
        yaw_err: err.yaw_err,
    }
}

// //builds snapshot and sends to log channel
// //called in: position controller after computing wheel_cmd and errors
// pub async fn send_log(wheel_cmd: &WheelCmd, x_err: f32, y_err: f32, yaw_err: f32) {
//     let snapshot = build_log_snapshot(wheel_cmd, x_err, y_err, yaw_err).await;
//     let _ = LOG_CH.try_send(snapshot);
// }

use crate::orchestrator_signal::STOP_LOG_SENDING_SIG;
use crate::packet::StateLoopBackPacketF32;
use crate::uart::UART_TX_CHANNEL;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker};

/// Periodically builds LogSnapshot from current state and sends via UART
/// This task respects the STOP_LOG_SENDING_SIG signal
#[embassy_executor::task]
pub async fn uart_log_sending_task(robot_id: u8, period_ms: u64) {
    defmt::info!("uart_log_sending_task started (period={}ms)", period_ms);

    // Wait a bit for UART to stabilize
    embassy_time::Timer::after_millis(500).await;
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));

    loop {
        let tick_fut = ticker.next();
        let stop_fut = STOP_LOG_SENDING_SIG.wait();

        match select(tick_fut, stop_fut).await {
            Either::First(_) => {
                // Build snapshot from current mutex state (includes tracking errors)
                let snapshot = build_log_snapshot_from_state().await;

                // Pack and send via UART
                let pkt = StateLoopBackPacketF32::new(robot_id, snapshot);
                let data = pkt.to_bytes_compact();
                let _ = UART_TX_CHANNEL.try_send(data);
            }
            Either::Second(_) => {
                defmt::info!("uart_log_sending_task stopped.");
                break;
            }
        }
    }
}
