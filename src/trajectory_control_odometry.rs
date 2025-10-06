use crate::math::SO2;
use core::f32::consts::PI;
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::led::{self};
use crate::odometry::{ODOMETRY_CHANNEL, get_latest_odometry};
use crate::read_robot_config_from_sd::RobotConfig;
use crate::sdlog::{SdLogger, TrajControlLog};
use crate::trajectory_control::PointCascade;
use crate::trajectory_control::{DiffdriveCascade, DiffdriveControllerCascade, STOP_ALL};
use crate::trajectory_signal::{LAST_STATE, PoseAbs, WHEEL_CMD_CH, WheelCmd};
use portable_atomic::Ordering;

// Odometry-based outer loop function
#[embassy_executor::task]
pub async fn diffdrive_outer_loop_with_odometry(
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
    cfg: Option<RobotConfig>,
) {
    STOP_ALL.store(false, Ordering::Relaxed);

    // ============ Robot Configuration ==========
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();

    // ================ Setup Ticker ===============
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    // ============ Initialize robot model =========
    defmt::info!("Initializing diffdrive robot model with odometry");
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

    // Wait for first odometry reading to establish initial pose
    defmt::info!("Waiting for initial odometry reading...");
    let odometry_receiver = ODOMETRY_CHANNEL.receiver();
    let initial_odom = odometry_receiver.receive().await;
    let first_pose = initial_odom;

    defmt::info!(
        "Initial pose from odometry: x={}, y={}, theta={}",
        first_pose.x,
        first_pose.y,
        first_pose.theta.rad()
    );

    // Get precise start time
    let start = Instant::now();

    loop {
        ticker.next().await;

        // Get latest odometry data
        let current_odom = match get_latest_odometry().await {
            Some(odom) => odom,
            None => {
                defmt::warn!("No odometry data available, skipping control cycle");
                continue;
            }
        };

        // Current elapsed time
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;

        // Generate circle trajectory
        let circle_duration: f32 = 15.0; // Longer duration for testing
        let circle_radius: f32 = 0.3; // Smaller radius for indoor testing
        let phi0 = SO2::new(first_pose.theta.rad() + 0.5 * PI);

        let setpoint = robot.circle_reference_t(
            circle_radius,
            circle_duration,
            t_sec,
            first_pose.x,
            first_pose.y,
            phi0,
        );

        // Update robot state from odometry
        robot.s.x = current_odom.x;
        robot.s.y = current_odom.y;
        robot.s.theta = current_odom.theta;

        // Compute control action
        let (action, x_error, y_error, theta_error) = controller.control(&robot, setpoint);

        // Send wheel commands
        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: action.ul,
            omega_r: action.ur,
            stamp: Instant::now(),
        });

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
            actual_vx: current_odom.v,
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
            ul: action.ul,
            ur: action.ur,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        if let Some(ref mut logger) = sdlogger {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }

        defmt::info!(
            "t={}s, odom=({},{},{}), target=({},{},{}), v={}, w={}, err=({},{},{})",
            t_sec,
            current_odom.x,
            current_odom.y,
            current_odom.theta.rad(),
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            setpoint.vdes,
            setpoint.wdes,
            x_error,
            y_error,
            theta_error
        );

        // Check for completion
        if t_sec >= circle_duration {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            STOP_ALL.store(true, Ordering::Relaxed);
            defmt::info!("Trajectory complete after {}s", t_sec);
            break;
        }
    }
}

//see if basic functionality is running.
#[embassy_executor::task]
pub async fn test_odometry_only() {
    defmt::info!("=== ODOMETRY TEST ===");
    let receiver = ODOMETRY_CHANNEL.receiver();

    for i in 0..50 {
        // Test for 5 seconds at 100ms intervals
        let odom = receiver.receive().await;
        defmt::info!(
            "Test {}: x={}, y={}, theta={}, v={}, w={}",
            i,
            odom.x,
            odom.y,
            odom.theta.rad(),
            odom.v,
            odom.w
        );
        Timer::after_millis(100).await;
    }
}

//testing functionality with forward propagation, tracking odometry and receiving mocap data
#[embassy_executor::task]
pub async fn testing_odometry(
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

    // Initialize logger
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    /* ================ Record First Pose w.r.t the selected mode ================== */
    let mut first_pose: PoseAbs = PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    // Wait for first odometry reading to establish initial pose
    defmt::info!("Waiting for initial odometry reading...");
    let odometry_receiver = ODOMETRY_CHANNEL.receiver();
    let initial_odom = odometry_receiver.receive().await;
    let mut pose_odom = initial_odom;

    /* ============================================================================= */

    // Get percise start time
    let start = Instant::now();

    loop {
        ticker.next().await;

        // get robot pose
        let pose = {
            let s = LAST_STATE.lock().await;
            *s
        };

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
            p1y: (0.05),
            p2x: (0.45),
            p2y: (0.5),
            p3x: (0.5),
            p3y: (0.5),
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

        // Get latest odometry data
        let current_odom = match get_latest_odometry().await {
            Some(odom) => odom,
            None => {
                defmt::warn!("No odometry data available, skipping control cycle");
                continue;
            }
        };

        //forward propagation of a trajectory for testing odometry
        let w_rad = setpoint.wdes;
        let vd = setpoint.vdes;

        // let state_des = DiffdriveStateCascade {
        //     x: setpoint.des.x,
        //     y: setpoint.des.y,
        //     theta: setpoint.des.theta,
        // };

        let ur = (2.0 * vd + robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
            / (2.0 * robot_cfg.wheel_radius);
        let ul = (2.0 * vd - robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
            / (2.0 * robot_cfg.wheel_radius);

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain the old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        // log trajectory control
        let log: TrajControlLog = TrajControlLog {
            timestamp_ms: t_ms,
            //target is the mocap data output now
            target_x: robot.s.x,
            target_y: robot.s.y,
            target_theta: robot.s.theta.rad(),
            //estimated data from the odometry
            actual_x: current_odom.x,
            actual_y: current_odom.y,
            actual_theta: current_odom.theta.rad(),
            //using for logging the desired trajectory here.
            target_vx: setpoint.des.x,
            target_vy: setpoint.des.y,
            target_vz: setpoint.des.theta.rad() as f32,
            actual_vx: 0.0,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: 0.0,
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: 0.0,
            actual_qw: 0.0,
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: 0.0,
            xerror: 0.0,
            yerror: 0.0,
            thetaerror: 0.0,
            ul: ul,
            ur: ur,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        if let Some(ref mut logger) = sdlogger {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, pos_mocap = ({},{},{}), pos_odom = ({},{},{}), target=({},{},{}), v={}, w={}, ul={}, ur={}",
            t_sec,
            robot.s.x,
            robot.s.y,
            robot.s.theta.rad(),
            current_odom.x,
            current_odom.y,
            current_odom.theta.rad(),
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            setpoint.vdes,
            setpoint.wdes,
            ul,
            ur
        );

        if t_sec >= bezier_duration {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            STOP_ALL.store(true, Ordering::Relaxed);
            defmt::info!("Trajectory complete after {}s", t_sec);
            break;
        }
    }
}
