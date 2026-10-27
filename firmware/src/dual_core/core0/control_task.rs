//! Permanent shared wheel-speed inner loop on Core 0.

use embassy_time::{Duration, Instant, Ticker};

use crate::{
    dual_core::{
        logging::{
            LOG_VALID_COMMAND, LOG_VALID_IMU, LOG_VALID_MOCAP, LOG_VALID_MOTOR, LOG_VALID_POSE, LOG_VALID_SETPOINT, LOG_VALID_TIMING, LOG_VALID_TRACKING_ERROR, LOG_VALID_WHEEL_MEASUREMENT, LOG_VALID_WHEEL_TARGET, LogFramePublisher, UnifiedLogFrame,
        }, messages::RuntimeMode, metrics::record_control_tick, shared_state::{active_mode, clear_trajectory_output, latest_imu_sample, latest_teleop_command, latest_traj_command, latest_trajectory_output, try_receive_parameter_update,},
    }, peripherals::{
        encoder::{EncoderCounters, read_counts, sample_wheel_speed},
        motor::MotorController,
        sdcard::RobotConfig,
    },
};

use super::teleop::{ControlReference, WheelTarget, resolve_reference};

const STOP_TARGET_RAD_S: f32 = 0.1;
const FILTER_FREQUENCY_HZ: f32 = 3.0;
const LOG_PERIOD_US: u64 = 20_000;

struct WheelControllerOutput {
    omega_left_measured: f32,
    omega_right_measured: f32,
    duty_left: f32,
    duty_right: f32,
}

struct WheelSpeedController {
    integral_left: f32,
    integral_right: f32,
    previous_error_left: f32,
    previous_error_right: f32,
    previous_count_left: i32,
    previous_count_right: i32,
    filtered_omega_left: f32,
    filtered_omega_right: f32,
    filter_alpha: f32,
    dt_s: f32,
}

impl WheelSpeedController {
    fn new(counters: &EncoderCounters, dt_s: f32) -> Self {
        let (left_count, right_count) = read_counts(counters);
        let tau = 1.0 / (2.0 * core::f32::consts::PI * FILTER_FREQUENCY_HZ);
        let filter_alpha = dt_s / (tau + dt_s);

        Self {
            integral_left: 0.0,
            integral_right: 0.0,
            previous_error_left: 0.0,
            previous_error_right: 0.0,
            previous_count_left: left_count,
            previous_count_right: right_count,
            filtered_omega_left: 0.0,
            filtered_omega_right: 0.0,
            filter_alpha,
            dt_s,
        }
    }

    fn reset(&mut self, counters: &EncoderCounters) {
        let (left_count, right_count) = read_counts(counters);

        self.integral_left = 0.0;
        self.integral_right = 0.0;
        self.previous_error_left = 0.0;
        self.previous_error_right = 0.0;
        self.previous_count_left = left_count;
        self.previous_count_right = right_count;
        self.filtered_omega_left = 0.0;
        self.filtered_omega_right = 0.0;
    }

    fn update(
        &mut self,
        target: WheelTarget,
        counters: &EncoderCounters,
        config: &RobotConfig,
    ) -> WheelControllerOutput {
        let sample = sample_wheel_speed(
            counters,
            self.previous_count_left,
            self.previous_count_right,
            config.encoder_cpr,
            self.dt_s,
        );

        self.previous_count_left = sample.left_count;
        self.previous_count_right = sample.right_count;

        self.filtered_omega_left +=
            self.filter_alpha * (sample.omega_left - self.filtered_omega_left);

        self.filtered_omega_right +=
            self.filter_alpha * (sample.omega_right - self.filtered_omega_right);

        let stop_requested = target.omega_left.abs() < STOP_TARGET_RAD_S
            && target.omega_right.abs() < STOP_TARGET_RAD_S;

        let (duty_left, duty_right) = if stop_requested {
            self.clear_pid();
            (0.0, 0.0)
        } else {
            self.calculate_pid(target, config)
        };

        WheelControllerOutput {
            omega_left_measured: self.filtered_omega_left,
            omega_right_measured: self.filtered_omega_right,
            duty_left,
            duty_right,
        }
    }

    fn clear_pid(&mut self) {
        self.integral_left = 0.0;
        self.integral_right = 0.0;
        self.previous_error_left = 0.0;
        self.previous_error_right = 0.0;
    }

    fn calculate_pid(
        &mut self,
        target: WheelTarget,
        config: &RobotConfig,
    ) -> (f32, f32) {
        let error_left = target.omega_left - self.filtered_omega_left;
        let error_right = target.omega_right - self.filtered_omega_right;

        self.integral_left = (self.integral_left
            + config.ki_inner * self.dt_s * error_left)
            .clamp(-2.0, 2.0);

        self.integral_right = (self.integral_right
            + config.ki_inner * self.dt_s * error_right)
            .clamp(-2.0, 2.0);

        let derivative_left =
            (error_left - self.previous_error_left) / self.dt_s;

        let derivative_right =
            (error_right - self.previous_error_right) / self.dt_s;

        self.previous_error_left = error_left;
        self.previous_error_right = error_right;

        let max_left = config.motor_max_duty_left.abs();
        let max_right = config.motor_max_duty_right.abs();

        let output_left = (config.kp_inner * error_left
            + self.integral_left
            + config.kd_inner * derivative_left)
            .clamp(-max_left, max_left);

        let output_right = (config.kp_inner * error_right
            + self.integral_right
            + config.kd_inner * derivative_right)
            .clamp(-max_right, max_right);

        (
            output_left * config.motor_direction_left,
            output_right * config.motor_direction_right,
        )
    }
}

fn resolve_mode_reference(
    mode: RuntimeMode,
    now_us: u64,
    config: &RobotConfig,
) -> ControlReference {
    match mode {
        RuntimeMode::TeleOp | RuntimeMode::CtrlAction => {
            resolve_reference(mode, latest_teleop_command(), now_us, config)
        }
        RuntimeMode::TrajMocap | RuntimeMode::TrajOnboard | RuntimeMode::TrajOnboard2 => {
            latest_traj_command()
        }
        RuntimeMode::Menu => {
            ControlReference::ZERO
        }
    }
}

fn fill_imu_log(frame: &mut UnifiedLogFrame, now_us: u64) {
    let imu = latest_imu_sample();

    if imu.stamp_us == 0 || now_us.saturating_sub(imu.stamp_us) > 100_000 {
        return;
    }

    frame.valid_mask |= LOG_VALID_IMU;
    frame.acc_x = imu.acc_x;
    frame.acc_y = imu.acc_y;
    frame.acc_z = imu.acc_z;
    frame.gyro_x = imu.gyro_x;
    frame.gyro_y = imu.gyro_y;
    frame.gyro_z = imu.gyro_z;
}

fn fill_mocap_log(frame: &mut UnifiedLogFrame, now_us: u64) {
    let mocap = crate::dual_core::shared_state::latest_mocap_sample();

    if mocap.stamp_us == 0 || now_us.saturating_sub(mocap.stamp_us) > 100_000 {
        return;
    }

    frame.valid_mask |= LOG_VALID_MOCAP;
    frame.x_raw = mocap.x;
    frame.y_raw = mocap.y;
    frame.yaw_raw = mocap.yaw;
}

fn fill_traj_log(frame: &mut UnifiedLogFrame, now_us: u64) {
    let traj = latest_trajectory_output();

    if !traj.valid || now_us.saturating_sub(traj.stamp_us) > 100_000 {
        return;
    }

    frame.valid_mask |= LOG_VALID_POSE | LOG_VALID_SETPOINT | LOG_VALID_TRACKING_ERROR;
    frame.x = traj.x;
    frame.y = traj.y;
    frame.yaw = traj.yaw;

    frame.x_des = traj.x_des;
    frame.y_des = traj.y_des;
    frame.yaw_des = traj.yaw_des;

    frame.v_ff = traj.v_ff;
    frame.omega_ff = traj.omega_ff;
    
    frame.x_error = traj.x_error;
    frame.y_error = traj.y_error;
    frame.yaw_error = traj.yaw_error;

    clear_trajectory_output();
}

/// Shared high-frequency wheel-speed inner loop.
///
/// This is the only task that owns and writes motor PWM.
#[embassy_executor::task]
pub async fn core0_control_task(
    mut motor: MotorController,
    encoder_counts: EncoderCounters,
    mut config: RobotConfig,
) {
    let period_ms = config.joystick_control_dt_ms.max(1);
    let period_us = period_ms * 1_000;
    let dt_s = period_ms as f32 / 1_000.0;

    let mut ticker = Ticker::every(Duration::from_millis(period_ms));
    let mut controller = WheelSpeedController::new(&encoder_counts, dt_s);
    let mut log_publisher = LogFramePublisher::new();

    let mut previous_tick = Instant::now();
    let mut previous_mode = RuntimeMode::Menu;
    let mut control_misses = 0u32;
    let mut log_elapsed_us = 0u64;

    motor.stop();

    defmt::info!("Core 0 inner loop started, period={}ms", period_ms);

    loop {
        ticker.next().await;

        while let Some((parameter_id, value)) = try_receive_parameter_update() {
            apply_parameter_update(&mut config, parameter_id, value);
        }

        let now = Instant::now();
        let now_us = now.as_micros();
        let control_gap_us = now.duration_since(previous_tick).as_micros();
        previous_tick = now;

        record_control_tick(
            control_gap_us.min(u32::MAX as u64) as u32,
            period_us.min(u32::MAX as u64) as u32,
        );

        if control_gap_us > period_us * 3 / 2 {
            control_misses = control_misses.saturating_add(1);
        }

        let mode = active_mode();

        if mode != previous_mode {
            motor.stop();
            controller.reset(&encoder_counts);
            previous_mode = mode;
        }

        let reference = resolve_mode_reference(mode, now_us, &config);
        let output = controller.update(reference.wheel_target, &encoder_counts, &config);

        motor.set_speed(output.duty_left, output.duty_right);

        let mut frame = UnifiedLogFrame::for_mode(mode);

        frame.valid_mask = LOG_VALID_WHEEL_TARGET
            | LOG_VALID_WHEEL_MEASUREMENT
            | LOG_VALID_MOTOR
            | LOG_VALID_TIMING;

        if reference.command_valid {
            frame.valid_mask |= LOG_VALID_COMMAND;
        }

        frame.command_v = reference.command_v;
        frame.command_omega = reference.command_omega;
        frame.omega_l_target = reference.wheel_target.omega_left;
        frame.omega_r_target = reference.wheel_target.omega_right;
        frame.omega_l_measured = output.omega_left_measured;
        frame.omega_r_measured = output.omega_right_measured;
        frame.duty_l = output.duty_left;
        frame.duty_r = output.duty_right;
        frame.control_gap_us = control_gap_us.min(u32::MAX as u64) as u32;
        frame.control_misses = control_misses;

        fill_imu_log(&mut frame, now_us);
        fill_mocap_log(&mut frame, now_us);
        log_elapsed_us = log_elapsed_us.saturating_add(period_us);
        
        if log_elapsed_us >= LOG_PERIOD_US {
            log_elapsed_us -= LOG_PERIOD_US;
            fill_traj_log(&mut frame, now_us);
            let _ = log_publisher.try_publish(frame, now_us);
        }
    }
}


fn apply_parameter_update(config: &mut RobotConfig, parameter_id: u8, value: f32) {
    match parameter_id {
        0 => config.robot_id = value as u8,

        // Control period cannot be changed while the ticker is running.
        // It will be supported later through an explicit loop restart.
        1 => {}

        2 => config.traj_following_dt_s = value,
        3 => config.wheel_radius = value,
        4 => config.wheel_base = value,
        5 => config.motor_direction_left = value,
        6 => config.motor_direction_right = value,
        7 => config.motor_max_duty_left = value,
        8 => config.motor_max_duty_right = value,
        9 => config.k_clip = value,
        10 => config.kp_inner = value,
        11 => config.ki_inner = value,
        12 => config.kd_inner = value,
        13 => config.kx_traj = value,
        14 => config.ky_traj = value,
        15 => config.ktheta_traj = value,
        16 => config.gear_ratio = value,
        17 => config.encoder_cpr = value,
        18 => config.max_speed = value,
        19 => config.max_omega = value,
        20 => config.wheel_max = value,
        _ => {}
    }
}