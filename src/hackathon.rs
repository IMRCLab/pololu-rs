use crate::diffdrive::DiffdriveAction;
use crate::math::SO2;
use core::f32::consts::PI;
use defmt::info;
use embassy_futures::select::select;
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::Ordering;
use heapless::Vec;

use crate::led;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::sdlog::{SdLogger, TrajControlLog};
use crate::trajectory_reading::{Action, Pose};
use crate::trajectory_signal::{
    LAST_STATE, RobotInfo, TRAJECTORY_CONTROL_EVENT, WHEEL_CMD_CH, WheelCmd
};

use crate::trajectory_control::{
    ControlMode, DiffdriveActionCascade, DiffdriveCascade, DiffdriveControllerCascade, DiffdriveSetpointCascade, DiffdriveStateCascade, STOP_ALL, TRAJ_REF, TrajectoryResult
};

use libm::sqrtf;

pub struct HackathonAction{
    velocity: (f32, f32), //velocity in x and y
    w: f32, //angular velocity
}

// Command-controlled trajectory following from SDCard (hackathon version)
#[embassy_executor::task]
pub async fn hackathon_command_loop(
    mode: ControlMode,
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
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
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }

    defmt::info!("Waiting for trajectory control commands...");
    defmt::info!("Commands: 't' = start, 's' = stop");

    loop {
        // Wait for start command (true)
        loop {
            let command = TRAJECTORY_CONTROL_EVENT.wait().await;
            if command {
                // Start command received
                defmt::info!("Starting trajectory following from command!");
                led.on();
                STOP_ALL.store(false, Ordering::Relaxed);
                break;
            } else {
                // Stop command received while not running - ignore
                defmt::info!("Stop command received while not running - ignoring");
            }
        }

        // Execute trajectory until stop command, completion, or restart
        let result = hackathon_trajectory_following(
            mode,
            &mut robot,
            &mut controller,
            &mut sdlogger,
            &robot_cfg,
        )
        .await;

        match result {
            TrajectoryResult::Completed => {
                defmt::info!("Trajectory completed - waiting for next command");
                led.off();
                // Loop back to wait for next start command
            }
            TrajectoryResult::Stopped => {
                defmt::info!("Trajectory stopped by command");
                led.off();
                // Loop back to wait for next start command
            }
        }
    }
}

// Command-controlled trajectory following for gain tunning
async fn hackathon_trajectory_following(
    mode: ControlMode,
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    sdlogger: &mut Option<SdLogger>,
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

    /* ================ Record First Pose w.r.t the selected mode ================== */
    // let mut first_pose: PoseAbs = PoseAbs {
    //     x: 0.0,
    //     y: 0.0,
    //     z: 0.0,
    //     roll: 0.0,
    //     pitch: 0.0,
    //     yaw: 0.0,
    // };

    // Timer::after_millis(100).await; // Wait until the first pose comes
    // match mode {
    //     ControlMode::WithMocapController => {
    //         // Initialize first pose from mocap
    //         first_pose = {
    //             let s = LAST_STATE.lock().await;
    //             *s
    //         };
    //     }
    //     ControlMode::DirectDuty => {
    //         info!("Using direct duty control, initial pose (0, 0, 0)");
    //     }
    // }
    /* ============================================================================= */

    /* ======================== Get precise start time ============================= */
    let start = Instant::now();
    /* ============================================================================= */

    loop {
        /* ======== wait for either the timer tick or a trajectory command ========= */
        let either_result =
            select(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait()).await;

        match either_result {
            embassy_futures::select::Either::First(_) => {
                // timer tick: normal loop execution
            }
            embassy_futures::select::Either::Second(command) => {
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

        /* ========================= Get robot pose ================================ */
        let robot_state = {
            let s = LAST_STATE.lock().await;
            s.clone()  // Only one clone needed
        };

        //access all data from the cloned state, LAST_STATE is free to get overwritten in uart
        let other_robots = &robot_state.other_robots;  // Just a reference
        let pose = robot_state.pose;

        //sanity check ...
        info!(
            "Current Pose: x = {}, y = {}, yaw = {}",
            pose.x, pose.y, pose.yaw
        );
        info!(
            "Other Robots count: {}", 
            other_robots.len()
        );

        //Iterate over other robots
        for robot_info in other_robots.iter() {
            info!(
                "Robot {}: pos=({}, {}), vel=({}, {}), angle={}",
                robot_info.robot_id,
                robot_info.distance_x,
                robot_info.distance_y,
                robot_info.vel_x,
                robot_info.vel_y,
                robot_info.angle
            );
        }
        /* ========================================================================= */

        /* ====================== Current elapsed time ============================= */
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;
        /* ========================================================================= */

        /* ============= get corresponding reference state and action ============== */
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

        info!(
            "setpoint: x = {}, y = {}, theta = {}, v = {}, w = {}",
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            setpoint.vdes,
            setpoint.wdes
        );
        /* ========================================================================= */

        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw);

        let (ul, ur, x_error, y_error, theta_error);

        match mode {
            ControlMode::WithMocapController => {
                info!("Mocap Control Mode");
                let (action, x_e, y_e, yaw_e) = controller.control(&robot, setpoint);
                ul = action.ul;
                ur = action.ur;
                x_error = x_e;
                y_error = y_e;
                theta_error = yaw_e;

                //call emergency stop controller here ...
                let modified_action = emergency_breaking_controller(robot, action, other_robots, robot_cfg);
                let ul = modified_action.ul;
                let ur = modified_action.ur;

                while WHEEL_CMD_CH.try_receive().is_ok() {} //drain old commands
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });
            }
            ControlMode::DirectDuty => {
                let w_rad = setpoint.wdes;
                let vd = setpoint.vdes;

                ur = (2.0 * vd + robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                ul = (2.0 * vd - robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                x_error = 0.0;
                y_error = 0.0;
                theta_error = 0.0;

                while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });
            }
        }

        // Log trajectory control
        let log: TrajControlLog = TrajControlLog {
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

        if let Some(logger) = sdlogger {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, pos=({},{},{}), posd=({},{},{}), v={}, w={}rad/s ({}deg/s), u=({},{}), (xerr,yerr,theterr)=({},{},{})",
            t_sec,
            pose.x,
            pose.y,
            pose.yaw,
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            setpoint.vdes,
            setpoint.wdes,
            w_deg,
            ul,
            ur,
            x_error,
            y_error,
            theta_error
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

// ========================= HACKATHON SAFETY CONTROLLER =========================

/// Replace controller.control(&robot, setpoint) from the trajectory control loop with this now. 
pub fn hackathon_control(
    robot: &DiffdriveCascade,
    setpoint: DiffdriveSetpointCascade,
    other_robots: &[RobotInfo],  // Use slice - works with any size Vec
    robot_cfg: &RobotConfig,
) -> HackathonAction {
    // Now you can access other robots directly without any allocation:
    for robot_info in other_robots.iter() {
        //robots are 10 cm diameter circles.

        //code collision avoidance here ...
        let distance = sqrtf(robot_info.distance_x * robot_info.distance_x + robot_info.distance_y * robot_info.distance_y);
        if distance < 0.5 { } // 50cm safety distance
           
    }
    
    /*
    >>>>>>>>>>>>>>>>>>>>> HACKATHON DISTANCE AVOIDANCE CONTROLLER >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    
    
    
     */
    
    HackathonAction {
        velocity: (0.0, 0.0),
        w: 0.0,
    }
}

//remodel this for the firmware
pub fn emergency_breaking_controller(robot: &DiffdriveCascade, nominal_action: DiffdriveActionCascade, other_robots: &[RobotInfo], robot_cfg: &RobotConfig) -> DiffdriveActionCascade {
    let mut modified_action = nominal_action;
    //the action that we were supposed to do....

    //accessing the other robots with their respective:
    //distance_x, distance_y, vel_x, vel_y
    for robot_info in other_robots.iter() {
        //robots are 10 cm diameter circles.
    let min_dist = 0.2; // m
        //code collision avoidance here ...
        let distance = sqrtf(robot_info.distance_x * robot_info.distance_x + robot_info.distance_y * robot_info.distance_y);
        //closing speed d_dot
        info!("Distance to robot {}: {}", robot_info.robot_id, distance);
        let rel_vel_normal = (robot_info.distance_x * robot_info.vel_x + robot_info.distance_y * robot_info.vel_y) / distance;
        info!("Relative normal velocity to robot {}: {}", robot_info.robot_id, rel_vel_normal);
        if rel_vel_normal < 0.0 {
            // polulus moving towards each other
            let max_breaking = 1.0; // m/s^2
            let breaking_distance = (rel_vel_normal * rel_vel_normal) / (4.0 * max_breaking);
            if distance < breaking_distance + min_dist {
                info!("Emergency breaking activated due to robot {}", robot_info.robot_id);
                // need to brake, set motorspeeds to zero. 
                modified_action.ul = 0.0;
                modified_action.ur = 0.0;
            }
        }
        if distance < min_dist {
            info!("Emergency breaking activated due to proximity to robot {}", robot_info.robot_id);
            modified_action.ul = 0.0;
            modified_action.ur = 0.0;
        }
           
    }
    modified_action
}
