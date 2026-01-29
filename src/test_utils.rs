use core::f32::consts::PI;
use defmt::info;
use embassy_time::Timer;
use embassy_time::{Duration, Instant, Ticker};

use crate::led::{self};
use crate::math::SO2;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate::{
    Pose, TrackingError, WheelCmd, read_pose, stop_motors, write_tracking_error, write_wheel_cmd,
};
use crate::sdlog::{SdLogger, log_traj_control};
use crate::trajectory_control::{
    ControlMode, PointCascade, STOP_ALL, create_robot_and_controller, log_robot_init,
};

use portable_atomic::Ordering;

/// Basic shapes trajectory following test task
#[embassy_executor::task]
pub async fn traj_follow_test_basic_shapes(
    mode: ControlMode,
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
    cfg: Option<RobotConfig>,
) {
    STOP_ALL.store(false, Ordering::Relaxed);

    // ============ Robot Configuration ==========
    let robot_cfg = cfg.unwrap_or_default();

    // ================ Setup Ticker ===============
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    // ============ Initialize robot model =========
    log_robot_init(&robot_cfg);
    let (mut robot, mut controller) = create_robot_and_controller(&robot_cfg);

    // Initialize logger
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    /* ================ Record First Pose w.r.t the selected mode ================== */
    let mut first_pose: Pose = Pose {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        stamp: Instant::now(),
    };

    Timer::after_millis(100).await; // has to wait until the first poses comes
    match mode {
        ControlMode::WithMocapController => {
            // initialise first pose from mocap:
            first_pose = read_pose().await;
        }
        ControlMode::DirectDuty => {
            info!("Using direct duty control, initial pose (0, 0, 0)");
        }
    }
    /* ============================================================================= */

    // Get percise start time
    let start = Instant::now();

    loop {
        ticker.next().await;

        // get robot pose
        let pose = read_pose().await;

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
        robot.s.theta = SO2::new(pose.yaw);

        let mut ul = 0.0;
        let mut ur = 0.0;
        let (x_error, y_error, theta_error);

        match mode {
            ControlMode::WithMocapController => {
                info!("mocap");
                let (action, x_e, y_e, yaw_e) = controller.control(&robot, setpoint);
                ul = action.ul;
                ur = action.ur;
                x_error = x_e;
                y_error = y_e;
                theta_error = yaw_e;

                // Write tracking errors to mutex for logging
                write_tracking_error(TrackingError {
                    x_err: x_error,
                    y_err: y_error,
                    yaw_err: theta_error,
                })
                .await;

                write_wheel_cmd(WheelCmd::new(ul, ur)).await;
                led.on();
            }
            ControlMode::DirectDuty => {
                info!("no mocap");
                let w_rad = setpoint.wdes;
                let vd = setpoint.vdes;

                let ur = (2.0 * vd + robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                let ul = (2.0 * vd - robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                x_error = 0.0;
                y_error = 0.0;
                theta_error = 0.0;

                write_wheel_cmd(WheelCmd::new(ul, ur)).await;
                led.off();
            }
        }

        // log trajectory control
        log_traj_control(
            t_ms,
            &setpoint,
            &robot,
            x_error,
            y_error,
            theta_error,
            ul,
            ur,
        )
        .await;

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

        if t_sec >= bezier_duration {
            stop_motors();
            STOP_ALL.store(true, Ordering::Relaxed);
            defmt::info!("Trajectory complete after {}s", t_sec);
            break;
        }
    }
}
