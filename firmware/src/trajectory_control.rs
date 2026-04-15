use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, STOP_MOCAP_UPDATE_SIG};
use core::cell::RefCell;
use core::f32::consts::PI;
use defmt::{info};
use embassy_futures::block_on;
use embassy_futures::select::{Either, select, select3, Either3};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embassy_time::{Duration, Instant, Ticker};
use libm::{cosf, sinf};
use static_cell::StaticCell;

use crate::encoder::wheel_speed_from_counts_now;
use crate::led::{self};
use crate::motor::MotorController;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robot_parameters_default::robot_constants::*;
use crate::led::LED_SHARED;
use crate::sdlog::{SdLogger, TrajControlLog, SDLOGGER_SHARED};
use crate::trajectory_reading::{Action, Pose, Trajectory};
use crate::robotstate::{
    MOCAP_SIG as STATE_SIG, TRAJECTORY_CONTROL_EVENT, WHEEL_CMD_CH, WheelCmd,
};

use crate::robotstate;

use portable_atomic::{AtomicBool, Ordering};

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

// =============================== Save Trajectory ================================
static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
    Mutex::new(RefCell::new(None));
pub static TRAJ_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();
static TRAJ_CELL: StaticCell<Trajectory> = StaticCell::new();

pub fn store_trajectory(traj: Trajectory) -> &'static Trajectory {
    TRAJ_CELL.init(traj)
}

pub fn register_trajectory(traj: &'static Trajectory) {
    block_on(async {
        let g = TRAJ_REF.lock().await;
        *g.borrow_mut() = Some(traj);
        TRAJ_READY.signal(());
    })
}

pub async fn led_set(on: bool) {
    let mut g = LED_SHARED.lock().await;
    if let Some(led) = g.as_mut() {
        if on { led.on(); } else { led.off(); }
    } else {
        defmt::warn!("LED not available; skip.");
    }
}

pub async fn with_sdlogger<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut SdLogger) -> R
{
    let mut g = SDLOGGER_SHARED.lock().await;
    if let Some(l) = g.as_mut() {
        Some(f(l))
    } else {
        defmt::warn!("SdLogger not available; skip.");
        None
    }
}


// ====================== TODO: ===========================
// (1) add abstraction layer for trajectory selection
// (2) time to test the position controller with mocap system
// ====================== TODO: ===========================
pub use crate::control_types::*;

/* =============================== Main Tasks ========================================== */

// Command-controlled trajectory following for gain tunning
async fn execute_trajectory_loop_with_control_from_sdcard(
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    // sdlogger: &mut Option<SdLogger>,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    /* ========================= Load trajectory points ============================ */
    let (states, actions) = {
        let g = TRAJ_REF.lock().await;
        let t = g.borrow();
        let tr = t.as_ref().expect("Trajectory is not set");
        (&tr.states, &tr.actions)
    };

    let len = core::cmp::min(states.len(), actions.len());
    defmt::info!("Starting control task with {} states and actions", len);
    /* ============================================================================= */

    /* ============================== Setup Ticker ================================= */
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));
    /* ============================================================================= */

    /* ============================================================================= */
    


    /* =============== Fusion state: EKF ================= */
    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    // Seed fused pose from mocap if available, otherwise from trajectory start.
    let init_fresh = robotstate::get_and_clear_pose_fresh();
    let mut ekf = if init_fresh {
        let mocap = robotstate::read_pose().await;
        fused_x = mocap.x;
        fused_y = mocap.y;
        fused_yaw = mocap.yaw;
        info!("SD traj: initial pose from mocap ({},{},{})", fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        // No mocap — use the first state from the trajectory file as initial pose.
        fused_x = states[0].x;
        fused_y = states[0].y;
        fused_yaw = states[0].yaw;
        info!("SD traj: initial pose from trajectory start ({},{},{}) (no mocap)", fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    };
    /* ============================================================================= */

    /* ======================== Get precise start time ============================= */
    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);
    /* ============================================================================= */

    loop {
        /* ======== wait for either the timer tick, a trajectory command, or pause ========= */
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {
                // timer tick: normal loop execution
            }
            Either3::Second(command) => {
                // command received during trajectory execution
                if !command {
                    // Stop command - immediately stop motors
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0,
                        omega_r: 0.0,
                        stamp: Instant::now(),
                    });
                    defmt::info!("Stopping trajectory by command");
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

        // in case a stop was requested
        if STOP_ALL.load(Ordering::Relaxed) {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Trajectory stopped via STOP_ALL");
            break;
        }
        /* ========================================================================= */

        /* ========================= Pose Fusion ===================================== */
        let odom_now = robotstate::read_odom().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
        if pose_is_fresh {
            let mocap_pose = robotstate::read_pose().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* ========================================================================= */

        /* ====================== Current elapsed time ============================= */
        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;
        /* ========================================================================= */
        let mut i = (t_sec / robot_cfg.traj_following_dt_s) as usize;
        if i >= len {
            i = len - 1;
        }

        let Pose {
            x: x_d,
            y: y_d,
            yaw: theta_d,
        } = states[i + 1];
        let Action { v: vd, omega: wd } = actions[i]; // only n-1 actions(n states, i starts from 1)
        /* ========================================================================= */

        /* =================== Create Setpoint from reference ====================== */
        let setpoint = DiffdriveSetpointCascade {
            des: DiffdriveStateCascade {
                x: x_d,
                y: y_d,
                theta: SO2::new(theta_d),
            },
            vdes: vd,
            wdes: wd,
        };
        /* ========================================================================= */

        /* ==================== Control ============================================ */
        let (ul, ur, x_error, y_error, theta_error);

        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        let (action, x_e, y_e, yaw_e) = controller.control(&robot, setpoint);
        ul = action.ul;
        ur = action.ur;
        x_error = x_e;
        y_error = y_e;
        theta_error = yaw_e;
        /* ========================================================================= */

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        // Write EKF state, setpoint, tracking error, wheel cmd to robotstate
        robotstate::write_ekf_state(robotstate::RobotPose {
            x: fused_x,
            y: fused_y,
            yaw: fused_yaw,
            ..robotstate::RobotPose::DEFAULT
        }).await;
        robotstate::write_setpoint(robotstate::Setpoint {
            x_des: x_d,
            y_des: y_d,
            yaw_des: theta_d,
            v_ff: vd,
            w_ff: wd,
            stamp: Instant::now(),
        }).await;
        robotstate::write_tracking_error(robotstate::TrackingError {
            x_err: x_error,
            y_err: y_error,
            yaw_err: theta_error,
        }).await;
        robotstate::write_wheel_cmd(robotstate::WheelCmd::new(ul, ur)).await;

        // Log trajectory control — read from robotstate (single source of truth)
        let rs_pose = robotstate::read_ekf_state().await;
        let rs_sp = robotstate::read_setpoint().await;
        let rs_err = robotstate::read_tracking_error().await;
        let rs_odom = robotstate::read_odom().await;
        let rs_wc = robotstate::read_wheel_cmd().await;

        let log: TrajControlLog = TrajControlLog {
            timestamp_ms: t_ms,
            target_x: rs_sp.x_des,
            target_y: rs_sp.y_des,
            target_theta: rs_sp.yaw_des,
            actual_x: rs_pose.x,
            actual_y: rs_pose.y,
            actual_theta: rs_pose.yaw,
            target_vx: rs_sp.v_ff,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: rs_odom.v,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: cosf(rs_sp.yaw_des),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: sinf(rs_sp.yaw_des),
            actual_qw: cosf(rs_pose.yaw),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: sinf(rs_pose.yaw),
            xerror: rs_err.x_err,
            yerror: rs_err.y_err,
            thetaerror: rs_err.yaw_err,
            ul: rs_wc.omega_l,
            ur: rs_wc.omega_r,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
        );

        if t_sec >= (robot_cfg.traj_following_dt_s * (len as f32)) {
            //stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Trajectory complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    //ensure motors are stopped (check logic again ...)
    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    });

    TrajectoryResult::Stopped
}

// Command-controlled trajectory following from SDCard
#[embassy_executor::task]
pub async fn diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
    // mut sdlogger: Option<SdLogger>,
    // led_device: Option<led::Led>,
    cfg: Option<RobotConfig>,
) {
    // ============ Robot Configuration ==========
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        defmt::info!("Load Robot Params from SD CARD!");
        robot_cfg = cfg.unwrap();
    } else {
        defmt::info!("Load Robot Params from DEFAULT!");
        robot_cfg = RobotConfig::default();
    }

    // ============ Initialize robot model =========
    defmt::info!("Initializing command-controlled diffdrive robot model");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        robot_cfg.wheel_max,
    );
    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );

    // Initialize controller
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj,
        robot_cfg.ky_traj,
        robot_cfg.ktheta_traj,
    );

    // Initialize logger
    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;
    // if let Some(ref mut logger) = sdlogger {
    //     logger.write_traj_control_header();
    // }

    defmt::info!("Waiting for trajectory control commands...");
    defmt::info!("Commands: 't' = start, 's' = stop");

    // Drain any stale start/pause signals that may have been set
    // by a previous UART task racing with the orchestrator shutdown.
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

    loop {
        // If user request stop task, stop immediately
        let start_fut = TRAJ_RESUME_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select3(start_fut, stop_fut, pause_fut).await {
            // STRAT/RESTART
            Either3::First(start) => {
                if !start {
                    continue;
                }
                defmt::info!("Starting trajectory!");
                led_set(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle) -> wait RESUME/STOP");
                led_set(false).await;
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                        Either::First(_) => { 
                            defmt::info!("RESUME(idle)"); 
                            break;
                        }
                        Either::Second(_) => {
                            defmt::warn!("STOP outer loop (idle) -> exit");
                        }
                    }
                }
                continue;
            }
        }

        

        let exec_fut = execute_trajectory_loop_with_control_from_sdcard(
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Trajectory completed.");
                    },
                    TrajectoryResult::Stopped   => {
                        defmt::info!("Trajectory stopped internally.");
                    }
                }
                led_set(false).await;
                // Clear stale resume/pause signals so the outer loop
                // doesn't immediately re-enter execution on the next iteration.
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
        defmt::info!("Waiting for next command...");
    }
}

/// Mocap Update Signal
#[embassy_executor::task]
pub async fn mocap_update_task() {
    loop {
        
        match select(STATE_SIG.wait(), STOP_MOCAP_UPDATE_SIG.wait()).await {
            Either::First(new_pose) => {
                let stamped = robotstate::MocapPose {
                    stamp: Instant::now(),
                    ..new_pose
                };
                robotstate::write_pose(stamped).await;
                robotstate::set_pose_fresh(true);
            }

            Either::Second(_) => {
                defmt::info!("mocap_update_task stopped by STOP_MOCAP_UPDATE_SIG");
                return;
            }
        }
    }
}

/* ===================================================================================== */

/* ========================== Onboard Trajectory Mode ================================= */

async fn execute_trajectory_loop_onboard(
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    let duration: f32 = 3.0;
    let ax: f32 = 0.3;
    let ay: f32 = 0.3;

    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    // --- Unified pose fusion state ---
    // "Anchor" = the odometry snapshot at the moment we last received a fresh mocap pose.
    // Between mocap updates we compute: pose = last_mocap + (odom_now - odom_anchor).
    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    // Wait a moment for first mocap/odom data to arrive
    Timer::after_millis(100).await;

    // Try to get initial mocap pose; fall back to pure odometry if unavailable.
    let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
    let mut ekf = if pose_is_fresh {
        let mocap_pose = robotstate::read_pose().await;
        fused_x = mocap_pose.x;
        fused_y = mocap_pose.y;
        fused_yaw = mocap_pose.yaw;
        info!("Onboard traj: initial pose from mocap ({},{},{})",
            fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        fused_x = 0.0;
        fused_y = 0.0;
        fused_yaw = 0.0;
        info!("Onboard traj: initial pose set to origin (no mocap)");
        crate::ekf::Ekf::default_at_origin()
    };

    let first_pose_x = fused_x;
    let first_pose_y = fused_y;
    let first_pose_yaw = fused_yaw;

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {} // normal tick
            Either3::Second(command) => {
                if !command {
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0,
                        omega_r: 0.0,
                        stamp: Instant::now(),
                    });
                    defmt::info!("Onboard traj stopped by command");
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
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Onboard traj stopped via STOP_ALL");
            break;
        }

        /* =================== Pose Fusion =================== */
        let odom_now = robotstate::read_odom().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
        if pose_is_fresh {
            let mocap_pose = robotstate::read_pose().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* =================================================== */

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;
        let setpoint = robot.figure8_reference(
            t_sec,
            duration,
            ax,
            ay,
            first_pose_x,
            first_pose_y,
            first_pose_yaw,
        );

        // Update robot state from fused pose
        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        // Compute wheel commands: always use controller (closed-loop), regardless of mocap
        let (action, x_error, y_error, theta_error) = controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        // Write EKF state, setpoint, tracking error, wheel cmd to robotstate
        robotstate::write_ekf_state(robotstate::RobotPose {
            x: fused_x,
            y: fused_y,
            yaw: fused_yaw,
            ..robotstate::RobotPose::DEFAULT
        }).await;
        robotstate::write_setpoint(robotstate::Setpoint {
            x_des: setpoint.des.x,
            y_des: setpoint.des.y,
            yaw_des: setpoint.des.theta.rad(),
            v_ff: setpoint.vdes,
            w_ff: setpoint.wdes,
            stamp: Instant::now(),
        }).await;
        robotstate::write_tracking_error(robotstate::TrackingError {
            x_err: x_error,
            y_err: y_error,
            yaw_err: theta_error,
        }).await;
        robotstate::write_wheel_cmd(robotstate::WheelCmd::new(ul, ur)).await;

        // Log trajectory control — read from robotstate (single source of truth)
        let rs_pose = robotstate::read_ekf_state().await;
        let rs_sp = robotstate::read_setpoint().await;
        let rs_err = robotstate::read_tracking_error().await;
        let rs_odom = robotstate::read_odom().await;
        let rs_wc = robotstate::read_wheel_cmd().await;

        let log = TrajControlLog {
            timestamp_ms: t_ms,
            target_x: rs_sp.x_des,
            target_y: rs_sp.y_des,
            target_theta: rs_sp.yaw_des,
            actual_x: rs_pose.x,
            actual_y: rs_pose.y,
            actual_theta: rs_pose.yaw,
            target_vx: rs_sp.v_ff,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: rs_odom.v,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: cosf(rs_sp.yaw_des),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: sinf(rs_sp.yaw_des),
            actual_qw: cosf(rs_pose.yaw),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: sinf(rs_pose.yaw),
            xerror: rs_err.x_err,
            yerror: rs_err.y_err,
            thetaerror: rs_err.yaw_err,
            ul: rs_wc.omega_l,
            ur: rs_wc.omega_r,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
        );

        if t_sec >= duration {
            //stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Onboard trajectory complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    });

    TrajectoryResult::Stopped
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj(
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();

    defmt::info!("Initializing onboard trajectory mode (figure-8)");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius, robot_cfg.wheel_base, robot_cfg.wheel_max,
    );

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

    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;

    defmt::info!("Waiting for trajectory control commands...");

    // Drain any stale start/pause signals that may have been set
    // by a previous UART task racing with the orchestrator shutdown.
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

    loop {
        let start_fut = TRAJ_RESUME_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select3(start_fut, stop_fut, pause_fut).await {
            Either3::First(start) => {
                if !start {
                    continue;
                }
                defmt::info!("Starting onboard trajectory!");
                led_set(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle) -> wait RESUME/STOP");
                led_set(false).await;
                match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                    Either::First(_) => {
                        defmt::info!("RESUME(idle)");
                    }
                    Either::Second(_) => {
                        defmt::warn!("STOP outer loop (idle) -> exit");
                        return;
                    }
                }
                continue;
            }
        }

        let exec_fut = execute_trajectory_loop_onboard(
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Onboard trajectory completed.");
                    }
                    TrajectoryResult::Stopped => {
                        defmt::info!("Onboard trajectory stopped internally.");
                    }
                }
                led_set(false).await;
                // Clear stale resume/pause signals so the outer loop
                // doesn't immediately re-enter execution on the next iteration.
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
        defmt::info!("Waiting for next command...");
    }
}

/* ===================================================================================== */

/* ======================== Onboard Trajectory 2 (Demo) ================================ */

async fn execute_trajectory_loop_onboard2(
    robot: &mut DiffdriveCascade,
    _controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    // ---- Spin-in-place for 8 seconds ----
    let duration: f32 = 3.0;
    // Body angular velocity for spin-in-place.
    // Max body w = 2 * r * w_wheel / L
    let max_spin_fraction: f32 = 0.8;
    let wd_spin = 2.0 * robot_cfg.wheel_radius * robot_cfg.wheel_max / robot_cfg.wheel_base * max_spin_fraction;


    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    Timer::after_millis(100).await;

    let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
    let mut ekf = if pose_is_fresh {
        let mocap_pose = robotstate::read_pose().await;
        fused_x = mocap_pose.x;
        fused_y = mocap_pose.y;
        fused_yaw = mocap_pose.yaw;
        info!("Onboard traj2: initial pose from mocap ({},{},{})",
            fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        fused_x = 0.0;
        fused_y = 0.0;
        fused_yaw = 0.0;
        info!("Onboard traj2: initial pose set to origin (no mocap)");
        crate::ekf::Ekf::default_at_origin()
    };

    let first_pose_x = fused_x;
    let first_pose_y = fused_y;
    let first_pose_yaw = fused_yaw;

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command {
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                    });
                    defmt::info!("Onboard traj2 stopped by command");
                    return TrajectoryResult::Stopped;
                }
            }
            Either3::Third(_) => {
                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                });
                let pause_start = Instant::now();
                defmt::info!("Execute loop paused (traj2)");
                match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                    Either::First(_) => {
                        pause_offset += Instant::now() - pause_start;
                        defmt::info!("Execute loop resumed (traj2)");
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
            defmt::info!("Onboard traj2 stopped via STOP_ALL");
            break;
        }

        /* =================== Pose Fusion =================== */
        let odom_now = robotstate::read_odom().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
        if pose_is_fresh {
            let mocap_pose = robotstate::read_pose().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* =================================================== */

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;

        // ---- Spin in place via spinning reference + controller ----
        let setpoint = robot.spinning_at_wd(
            wd_spin,
            t_sec,
            first_pose_x,
            first_pose_y,
            first_pose_yaw,
        );

        // Update robot state from fused pose
        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        // Closed-loop control (same as figure-8 onboard traj)
        let (action, x_error, y_error, theta_error) = _controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {}
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul, omega_r: ur, stamp: Instant::now(),
        });

        // Write EKF state, setpoint, tracking error, wheel cmd to robotstate
        robotstate::write_ekf_state(robotstate::RobotPose {
            x: fused_x,
            y: fused_y,
            yaw: fused_yaw,
            ..robotstate::RobotPose::DEFAULT
        }).await;
        robotstate::write_setpoint(robotstate::Setpoint {
            x_des: setpoint.des.x,
            y_des: setpoint.des.y,
            yaw_des: setpoint.des.theta.rad(),
            v_ff: setpoint.vdes,
            w_ff: setpoint.wdes,
            stamp: Instant::now(),
        }).await;
        robotstate::write_tracking_error(robotstate::TrackingError {
            x_err: x_error,
            y_err: y_error,
            yaw_err: theta_error,
        }).await;
        robotstate::write_wheel_cmd(robotstate::WheelCmd::new(ul, ur)).await;

        // Log trajectory control — read from robotstate (single source of truth)
        let rs_pose = robotstate::read_ekf_state().await;
        let rs_sp = robotstate::read_setpoint().await;
        let rs_err = robotstate::read_tracking_error().await;
        let rs_odom = robotstate::read_odom().await;
        let rs_wc = robotstate::read_wheel_cmd().await;

        let log = TrajControlLog {
            timestamp_ms: t_ms,
            target_x: rs_sp.x_des,
            target_y: rs_sp.y_des,
            target_theta: rs_sp.yaw_des,
            actual_x: rs_pose.x,
            actual_y: rs_pose.y,
            actual_theta: rs_pose.yaw,
            target_vx: rs_sp.v_ff,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: rs_odom.v,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: cosf(rs_sp.yaw_des),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: sinf(rs_sp.yaw_des),
            actual_qw: cosf(rs_pose.yaw),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: sinf(rs_pose.yaw),
            xerror: rs_err.x_err,
            yerror: rs_err.y_err,
            thetaerror: rs_err.yaw_err,
            ul: rs_wc.omega_l,
            ur: rs_wc.omega_r,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t2={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
        );

        if t_sec >= duration {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
            });
            defmt::info!("Onboard trajectory 2 complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
    });
    TrajectoryResult::Stopped
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj2(
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();

    defmt::info!("Initializing onboard trajectory 2 mode (demo)");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius, robot_cfg.wheel_base, robot_cfg.wheel_max,
    );

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

    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;

    defmt::info!("Waiting for trajectory 2 control commands...");

    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

    loop {
        let start_fut = TRAJ_RESUME_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select3(start_fut, stop_fut, pause_fut).await {
            Either3::First(start) => {
                if !start { continue; }
                defmt::info!("Starting onboard trajectory 2!");
                led_set(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop (traj2) -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle traj2) -> wait RESUME/STOP");
                led_set(false).await;
                match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                    Either::First(_) => {
                        defmt::info!("RESUME(idle traj2)");
                    }
                    Either::Second(_) => {
                        defmt::warn!("STOP outer loop (idle traj2) -> exit");
                        return;
                    }
                }
                continue;
            }
        }

        let exec_fut = execute_trajectory_loop_onboard2(
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Onboard trajectory 2 completed.");
                    }
                    TrajectoryResult::Stopped => {
                        defmt::info!("Onboard trajectory 2 stopped internally.");
                    }
                }
                led_set(false).await;
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec (traj2) -> exit");
                led_set(false).await;
                return;
            }
        }
        defmt::info!("Waiting for next command (traj2)...");
    }
}

/* ===================================================================================== */

/* ========================== Not use or abandoned ==================================== */
// Outer Loop
#[embassy_executor::task]
pub async fn diffdrive_outer_loop(
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
    cfg: Option<RobotConfig>,
) {
    STOP_ALL.store(false, Ordering::Relaxed);

    // ============ Robot Configuration ==========
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        robot_cfg = cfg.unwrap();
    } else {
        robot_cfg = RobotConfig::default();
    }

    // ================ Setup Ticker ===============
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    // ============ Initialize robot model =========
    defmt::info!("Initializing diffdrive robot model");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        robot_cfg.wheel_max,
    );
    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );

    // Initialize controller
    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj,
        robot_cfg.ky_traj,
        robot_cfg.ktheta_traj,
    );

    // Initialize logger
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    /* ================ Record First Pose w.r.t the selected mode ================== */
    Timer::after_millis(100).await; // has to wait until the first poses comes
    // Direct MoCap feedback for this legacy task
    let first_pose = robotstate::read_pose().await;
    /* ============================================================================= */

    // Get percise start time
    let start = Instant::now();

    loop {
        ticker.next().await;

        // get robot pose
        let pose = robotstate::read_pose().await;

        // current elapsed time
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;

        /* Bezier Curve Trajectory Generation & Following */
        let bezier_duration: f32 = 8.0; // seconds

        /*
        hint: check the yaw angle zero axis alignment in mocap. currently: robot looking in y-axis direction
        is theta_ref = 0
        */
        let bezier_point = PointCascade {
            p0x: (0.0),
            p0y: (0.0),
            p1x: (0.0),
            p1y: (0.1),
            p2x: (0.9),
            p2y: (1.0),
            p3x: (1.0),
            p3y: (1.0),
            x_ref: (first_pose.x),
            y_ref: (first_pose.y),
            theta_ref: (first_pose.yaw + 0.5 * PI),
        };

        let setpoint = robot.beziercurve(bezier_point, t_sec, bezier_duration);

        /* ===================================================== */

        // let circle_duration: f32 = 10.0;

        // let phi0 = SO2::new(first_pose.yaw + 0.5 * PI);
        // let setpoint = robot.circle_reference_t(
        //     0.5,
        //     circle_duration,
        //     t_sec,
        //     first_pose.x,
        //     first_pose.y,
        //     phi0,
        // );
        /* ===================================================== */

        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw + 0.5 * PI); // mocap yaw is 90deg off

        // Compute wheel commands: always use controller (closed-loop), regardless of mocap
        let (action, x_error, y_error, theta_error) = controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        let log = TrajControlLog {
            timestamp_ms: t_ms,
            target_x: setpoint.des.x,
            target_y: setpoint.des.y,
            target_theta: setpoint.des.theta.rad(),
            actual_x: robot.s.x,
            actual_y: robot.s.y,
            actual_theta: robot.s.theta.rad(),
            target_vx: setpoint.vdes,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: 0.0,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: setpoint.des.theta.cos(),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: setpoint.des.theta.sin(),
            actual_qw: robot.s.theta.cos(),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: robot.s.theta.sin(),
            xerror: x_error,
            yerror: y_error,
            thetaerror: theta_error,
            ul: ul,
            ur: ur,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, pose=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{})",
            t_sec,
            pose.x, pose.y, pose.yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
        );

        if t_sec >= bezier_duration {
            //stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Bezier trajectory complete after {}s", t_sec);
            break;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    });
}