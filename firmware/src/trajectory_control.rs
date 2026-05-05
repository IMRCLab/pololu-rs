use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, STOP_ALL};
use embassy_futures::select::{Either, select, select3, Either3};
use embassy_time::{Duration, Instant, Ticker, Timer};
use portable_atomic::Ordering;

use crate::led::led_set;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate::{
    TRAJECTORY_CONTROL_EVENT, WHEEL_CMD_CH, WheelCmd,
};
use crate::robotstate;
use crate::setpoint::{SetpointFinder, TRAJ_REF};

pub use crate::control_types::*;

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

    
    let init = robotstate::read_ekf_state().await;
    let mut ekf = crate::ekf::Ekf::default_at(init.x, init.y, init.yaw);

    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));
    let dt = robot_cfg.traj_following_dt_s;
    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        // ---- Tick / Pause / Stop ----
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;
        
        // defmt::info!("OuterLoop Tick");
        
        match tick_result {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command {
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                    });
                    defmt::info!("Trajectory stopped by command");
                    return TrajectoryResult::Stopped;
                }
            }
            Either3::Third(_) => {
                // Pause: zero wheels and wait for resume or stop
                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                });
                let pause_start = Instant::now();
                defmt::info!("Execute loop paused");
                match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                    Either::First(_) => {
                        pause_offset += Instant::now() - pause_start;
                        defmt::info!("Execute loop resumed");
                    }
                    Either::Second(_) => {
                        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                            omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                        });
                        return TrajectoryResult::Stopped;
                    }
                }
                continue;
            }
        }

        if STOP_ALL.load(Ordering::Relaxed) {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
            });
            defmt::info!("Trajectory stopped via STOP_ALL");
            break;
        }

        // ---- 1. State Estimation (inline EKF) ----
        let odom = robotstate::read_odom().await;
        ekf.predict(odom.v, odom.w, dt);
        if robotstate::get_and_clear_pose_fresh() {
            let mocap = robotstate::read_pose().await;
            ekf.update(&crate::math::Vec3::new(mocap.x, mocap.y, mocap.yaw));
        }
        let (fx, fy, fth) = ekf.state();
        robotstate::write_ekf_state(robotstate::RobotPose {
            x: fx, y: fy, yaw: fth, stamp: Instant::now(),
            ..robotstate::RobotPose::DEFAULT
        }).await;

        // ---- 2. Setpoint ----
        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let setpoint = finder.get_setpoint(robot, t_sec);
        
        if (t.as_millis() as u32) % 500 < 50 {
            defmt::info!("t_sec={}: Setpoint pos=({},{},{}), vdes={}, wdes={}", 
                t_sec, setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(), setpoint.vdes, setpoint.wdes);
        }

        robotstate::write_setpoint(robotstate::Setpoint {
            x_des: setpoint.des.x,
            y_des: setpoint.des.y,
            yaw_des: setpoint.des.theta.rad(),
            v_ff: setpoint.vdes,
            w_ff: setpoint.wdes,
            stamp: Instant::now(),
        }).await;

        // ---- 3. Control ----
        robot.s.x = fx;
        robot.s.y = fy;
        robot.s.theta = SO2::new(fth);
        let (action, x_err, y_err, th_err) = controller.control(robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        if (t.as_millis() as u32) % 500 < 50 {
            defmt::info!("Control: ul={}, ur={}, errors=({},{},{})", ul, ur, x_err, y_err, th_err);
        }

        // ---- 4. Output ----
        while WHEEL_CMD_CH.try_receive().is_ok() {}
        let _ = WHEEL_CMD_CH.try_send(WheelCmd { omega_l: ul, omega_r: ur, stamp: Instant::now() });
        robotstate::write_wheel_cmd(robotstate::WheelCmd::new(ul, ur)).await;
        robotstate::write_tracking_error(robotstate::TrackingError {
            x_err, y_err, yaw_err: th_err,
        }).await;

        // ---- 5. Completion check ----
        if let Some(dur) = finder.duration() {
            if t_sec >= dur { 
                let _ = WHEEL_CMD_CH.try_send(WheelCmd { omega_l: 0.0, omega_r: 0.0, stamp: Instant::now() });
                defmt::info!("Trajectory complete after {}s", t_sec);
                return TrajectoryResult::Completed; 
            }
        }
    }
    TrajectoryResult::Stopped
}

/* =============================== Trajectory Tasks ========================================== */

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();
    defmt::info!("Initializing sd-card trajectory mode");

    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius, robot_cfg.wheel_base,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
    );
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj, robot_cfg.ky_traj, robot_cfg.ktheta_traj,
    );

    loop {
        // Wait for start command — but also watch for a stop signal so the task
        // exits cleanly even when idle (prevents zombie task accumulation).
        let command = match select(TRAJECTORY_CONTROL_EVENT.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
            Either::First(cmd) => cmd,
            Either::Second(_) => {
                defmt::warn!("STOP outer (idle) -> exit");
                return;
            }
        };
        if !command { continue; } // was a stop command, keep waiting

        let (states, actions, dt_s) = {
            let g = TRAJ_REF.lock().await;
            let t = g.borrow();
            if let Some(tr) = t.as_ref() {
                (tr.states.as_slice(), tr.actions.as_slice(), robot_cfg.traj_following_dt_s)
            } else {
                defmt::warn!("No trajectory registered!");
                continue;
            }
        };

        let finder = SetpointFinder::SdCard { states, actions, dt_s };
        
        let exec_fut = run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg);
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(res) => {
                if let TrajectoryResult::Completed = res { defmt::info!("Trajectory completed."); }
                led_set(false).await;
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj(
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();
    defmt::info!("Initializing onboard trajectory 1 mode (Figure 8)");

    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius, robot_cfg.wheel_base,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
    );
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj, robot_cfg.ky_traj, robot_cfg.ktheta_traj,
    );

    loop {
        // Wait for start command — also watch stop signal to prevent zombie tasks.
        let command = match select(TRAJECTORY_CONTROL_EVENT.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
            Either::First(cmd) => cmd,
            Either::Second(_) => {
                defmt::warn!("STOP outer (idle) -> exit");
                return;
            }
        };
        if !command { continue; }

        let init = robotstate::read_ekf_state().await;
        let finder = SetpointFinder::Figure8 {
            duration: 8.0,
            ax: 0.3,
            ay: 0.3,
            x0: init.x,
            y0: init.y,
            phi0: init.yaw,
        };
        
        let exec_fut = run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg);
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(res) => {
                if let TrajectoryResult::Completed = res { defmt::info!("Trajectory completed."); }
                led_set(false).await;
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj2(
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();
    defmt::info!("Initializing onboard trajectory 2 mode (Spin)");

    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius, robot_cfg.wheel_base,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
        -robot_cfg.wheel_max, robot_cfg.wheel_max,
    );
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj, robot_cfg.ky_traj, robot_cfg.ktheta_traj,
    );

    loop {
        // Wait for start command — also watch stop signal to prevent zombie tasks.
        let command = match select(TRAJECTORY_CONTROL_EVENT.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
            Either::First(cmd) => cmd,
            Either::Second(_) => {
                defmt::warn!("STOP outer (idle) -> exit");
                return;
            }
        };
        if !command { continue; }

        let init = robotstate::read_ekf_state().await;
        let max_spin_fraction: f32 = 0.8;
        let wd_spin = 2.0 * robot_cfg.wheel_radius * robot_cfg.wheel_max / robot_cfg.wheel_base * max_spin_fraction;

        let finder = SetpointFinder::Spin {
            duration: 3.0,
            wd_spin,
            x0: init.x,
            y0: init.y,
            phi0: init.yaw,
        };
        
        let exec_fut = run_unified_loop(&finder, &mut robot, &mut controller, &robot_cfg);
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(res) => {
                if let TrajectoryResult::Completed = res { defmt::info!("Trajectory completed."); }
                led_set(false).await;
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
    }
}