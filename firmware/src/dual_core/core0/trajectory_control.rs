use crate::dual_core::core0::teleop::{ControlReference, WheelTarget};
use crate::dual_core::messages::RuntimeMode;
use crate::dual_core::logging::{
    LOG_VALID_POSE, LOG_VALID_SETPOINT, LOG_VALID_TRACKING_ERROR, LOG_VALID_WHEEL_TARGET,
    LogFramePublisher, UnifiedLogFrame,
};
use crate::dual_core::shared_state::{
    TrajectoryOutput, active_mode, latest_odometry_sample, publish_traj_command, publish_trajectory_output, set_trajectory_running, take_fresh_mocap_sample, trajectory_running, wait_trajectory_state_changed,
};
use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, STOP_ALL};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::Ordering;

use crate::led::led_set;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate::{
    TRAJECTORY_CONTROL_EVENT, WHEEL_CMD_CH, WheelCmd,
};
use crate::robotstate;

use super::setpoint::{trajectory, SetpointFinder};
pub use super::control_types::*;

pub enum TrajectoryResult {
    Completed,
    Stopped,
}

/* =============================== Main Unified Loop ========================================== */

async fn run_unified_loop(
    finder: &SetpointFinder,
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    let initial_mocap = take_fresh_mocap_sample().filter(|sample| {
        sample.stamp_us != 0
            && Instant::now()
                .as_micros()
                .saturating_sub(sample.stamp_us)
                <= 100_000
            && sample.x.is_finite()
            && sample.y.is_finite()
            && sample.yaw.is_finite()
    });

    let fallback_pose = match finder {
        SetpointFinder::SdCard { states, .. } => states
            .first()
            .map(|pose| (pose.x, pose.y, pose.yaw))
            .unwrap_or((robot.s.x, robot.s.y, robot.s.theta.rad())),
        _ => {
            let first_setpoint = finder.get_setpoint(robot, 0.0).des;
            (
                first_setpoint.x,
                first_setpoint.y,
                first_setpoint.theta.rad(),
            )
        }
    };

    let (initial_x, initial_y, initial_yaw) = initial_mocap.map_or(
        fallback_pose,
        |sample| (sample.x, sample.y, sample.yaw),
    );

    let mut ekf = crate::ekf::Ekf::default_at(
        initial_x,
        initial_y,
        initial_yaw,
    );

    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));
    let dt = robot_cfg.traj_following_dt_s;
    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);
    let run_mode = active_mode();

    loop {
        if active_mode() != run_mode {
            publish_traj_command(ControlReference::ZERO);
            defmt::info!("Trajectory stopped by command");
            return TrajectoryResult::Stopped;
        }

        if !trajectory_running() {
            let pause_start = Instant::now();
            publish_traj_command(ControlReference::ZERO);
            defmt::info!("Execute loop paused");
            loop {
                if active_mode() != run_mode {
                    publish_traj_command(ControlReference::ZERO);
                    defmt::info!("Trajectory stopped by command");
                    return TrajectoryResult::Stopped;
                }
                if trajectory_running() {
                    break;
                }
                wait_trajectory_state_changed().await;
            }
            pause_offset += Instant::now() - pause_start;
        } else {
            match select(
                ticker.next(),
                wait_trajectory_state_changed(),
            )
            .await
            {
                Either::First(_) => {}
                Either::Second(_) => continue,
            }
        }

        if active_mode() != run_mode {
            publish_traj_command(ControlReference::ZERO);
            defmt::info!("Trajectory stopped by command");
            return TrajectoryResult::Stopped;
        }
        if !trajectory_running() {
            continue;
        }

        // TODO stop all
        
        // ---- 1. State Estimation (inline EKF) ----
        let odom = latest_odometry_sample();
        ekf.predict(odom.v, odom.omega, dt);

        let mocap = take_fresh_mocap_sample();
        if let Some(mocap) = mocap {
            ekf.update(&crate::math::Vec3::new(mocap.x, mocap.y, mocap.yaw));
        }

        let (fx, fy, fth) = ekf.state();

        // ---- 2. Setpoint ----
        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let setpoint = finder.get_setpoint(robot, t_sec);

        // ---- 3. Control ----
        robot.s.x = fx;
        robot.s.y = fy;
        robot.s.theta = SO2::new(fth);
        let (action, v, w, x_err, y_err, th_err) = controller.control(robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;


        // ---- 4. Output ----
        publish_traj_command(ControlReference {
            command_v: v, 
            command_omega: w,
            wheel_target: WheelTarget {
                omega_left: ul,
                omega_right: ur,
            },
            command_valid: true,
        });

        publish_trajectory_output(TrajectoryOutput {
            stamp_us: Instant::now().as_micros() as u64,
            valid: true,

            x: fx,
            y: fy,
            yaw: fth,

            // Trajectory reference
            x_des: setpoint.des.x,
            y_des: setpoint.des.y,
            yaw_des: setpoint.des.theta.rad(),

            // Feedforward
            v_ff: setpoint.vdes,
            omega_ff: setpoint.wdes,

            // Tracking error
            x_error: x_err,
            y_error: y_err,
            yaw_error: th_err,
        });
        // ---- 5. Completion check ----
        if let Some(dur) = finder.duration() {
            if t_sec >= dur { 
                publish_traj_command(ControlReference::ZERO); 
                defmt::info!("Trajectory complete after {}s", t_sec);
                return TrajectoryResult::Completed; 
            }
        }
    }
}

/* =============================== Trajectory Tasks ========================================== */

#[embassy_executor::task]
pub async fn trajectory_following_task(cfg: Option<RobotConfig>) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();
    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj,
        robot_cfg.ky_traj,
        robot_cfg.ktheta_traj,
    );
    let task_mode = active_mode();

    loop {
        if active_mode() != task_mode {
            publish_traj_command(ControlReference::ZERO);
            return;
        }

        if !trajectory_running() {
            wait_trajectory_state_changed().await;
            continue;
        }

        let result = match task_mode {
            RuntimeMode::TrajMocap => {
                let Some(tr) = trajectory() else {
                    defmt::warn!("No trajectory registered!");
                    set_trajectory_running(false);
                    continue;
                };
                let finder = SetpointFinder::SdCard {
                    states: tr.states.as_slice(),
                    actions: tr.actions.as_slice(),
                    dt_s: robot_cfg.traj_following_dt_s,
                };
                defmt::info!("Starting trajectory-following task in TrajMocap mode");
                run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg).await
            }
            RuntimeMode::TrajOnboard => {
                let init = robotstate::read_ekf_state().await;
                let finder = SetpointFinder::Figure8 {
                    duration: 8.0,
                    ax: 0.3,
                    ay: 0.3,
                    x0: init.x,
                    y0: init.y,
                    phi0: init.yaw,
                };
                defmt::info!("Starting trajectory-following task in TrajOnboard mode");
                run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg).await
            }
            RuntimeMode::TrajOnboard2 => {
                let init = robotstate::read_ekf_state().await;
                let max_spin_fraction: f32 = 0.8;
                let wd_spin = 2.0 * robot_cfg.wheel_radius * robot_cfg.wheel_max
                    / robot_cfg.wheel_base
                    * max_spin_fraction;
                let finder = SetpointFinder::Spin {
                    duration: 3.0,
                    wd_spin,
                    x0: init.x,
                    y0: init.y,
                    phi0: init.yaw,
                };
                defmt::info!("Starting trajectory-following task in TrajOnboard2 mode");
                run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg).await
            }
            _ => return,
        };

        match result {
            TrajectoryResult::Completed => set_trajectory_running(false),
            TrajectoryResult::Stopped => {
                publish_traj_command(ControlReference::ZERO);
                return;
            }
        }
    }
}
