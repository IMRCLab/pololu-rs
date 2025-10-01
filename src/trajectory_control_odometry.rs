use crate::math::SO2;
use crate::motor::MotorController;
use core::f32::consts::PI;
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::led::{self};
use crate::odometry::{ODOMETRY_CHANNEL, get_latest_odometry};
use crate::read_robot_config_from_sd::RobotConfig;
use crate::sdlog::{SdLogger, TrajControlLog};
use crate::trajectory_control::{DiffdriveCascade, DiffdriveControllerCascade, STOP_ALL};
use crate::trajectory_signal::{PoseAbs, WHEEL_CMD_CH, WheelCmd};
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
    motor: MotorController,
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
    defmt::info!("=== ODOMETRY + MOCAP TEST ===");
    let receiver = ODOMETRY_CHANNEL.receiver();

    // ================ Setup Ticker ===============
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));
    // ================ Setup Circle Trajectory ===============
    let radius = 0.30; // 30 cm
    let omega = 2.0 * PI / 30.0; // one circle in 30 seconds

    let mut pose: PoseAbs = PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    loop {
        ticker.next().await;

        //TODO: Datatypes for logging, prepare logging functionality
        //propagate the trjectory by calculating duties without controller
        //compare odometry data with mocap and log - actual vs desired.
    }
}
