use crate::math::SO2;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::sdlog::TrajControlLog;
use crate::trajectory_control::{ControlMode, STOP_ALL, with_sdlogger};
use crate::trajectory_control::{
    DiffdriveCascade, DiffdriveControllerCascade, PointCascade, TrajectoryResult,
};
use crate::robotstate;
use crate::robotstate::{WHEEL_CMD_CH, WheelCmd};

use embassy_time::{Duration, Instant, Ticker};
use libm::{cosf, sinf, sqrtf};
use portable_atomic::Ordering;

// ======================== Constants ========================
const GOTO_DURATION_S: f32 = 5.0; //only meant for short ways
const GOTO_DT_S: f32 = 0.1; // 10 Hz control loop

// goal pose for go-to
#[derive(Clone, Copy, Debug)]
pub struct GoToPose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32, // rad
}

//need: start pose (x,y,theta) and goal pose ToGoPose (x_g, y_g, theta_g)
//these are p0 and p3.
//to get p2 and p3 we use the condition that they must be aligned to the yaw angle, meaning that p2 must be tangential to theta_start and p3 must be tangential to theta_goal
// to find the distance,
fn build_bezier_points(start: &robotstate::MocapPose, goal: &GoToPose) -> PointCascade {
    let dx = goal.x - start.x;
    let dy = goal.y - start.y;
    let dist = sqrtf(dx * dx + dy * dy);

    //tangent handle: 1/3 of straight line distance, to limit convex hull expansion (heuristic only)
    let handle = dist / 3.0;
    //min length to not let curve degenerate
    let handle = if handle < 0.02 { 0.02 } else { handle };

    //get control points in local frame
    PointCascade {
        p0x: 0.0,
        p0y: 0.0,

        //turn about yaw
        p1x: handle * cosf(start.yaw),
        p1y: handle * sinf(start.yaw),

        p2x: dx - handle * cosf(goal.yaw),
        p2y: dy - handle * sinf(goal.yaw),

        p3x: dx,
        p3y: dy,

        //ref frame <- start pose
        x_ref: start.x,
        y_ref: start.y,
        theta_ref: start.yaw,
    }
}

//Execute a go-to-pose trajectory using bezier curve with mocap feedback
//
// Reads the current pose from `robotstate::read_pose()`
// Generates a cubic Bezier from current -> goal
// 10Hz default
/// - Uses `robotstate::get_and_clear_pose_fresh()` for mocap/feedforward fallback
pub async fn goto_pose(
    goal: GoToPose,
    mode: ControlMode,
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    // ==================== Get current pose ====================
    let start_pose = robotstate::read_pose().await;

    defmt::info!(
        "goto: start=({},{},{}) -> goal=({},{},{})",
        start_pose.x,
        start_pose.y,
        start_pose.yaw,
        goal.x,
        goal.y,
        goal.yaw
    );

    // ==================== Build Bézier ====================
    let bezier = build_bezier_points(&start_pose, &goal);

    // ==================== Setup ticker ====================
    let mut ticker = Ticker::every(Duration::from_millis((GOTO_DT_S * 1000.0) as u64));

    let start = Instant::now();

    // ==================== Control loop ====================
    loop {
        ticker.next().await;

        // Check stop
        if STOP_ALL.load(Ordering::Relaxed) {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("goto: stopped via STOP_ALL");
            return TrajectoryResult::Stopped;
        }

        // ============ Elapsed time ============
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;

        if t_sec >= GOTO_DURATION_S {
            // Stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("goto: reached goal after {}s", t_sec);
            return TrajectoryResult::Completed;
        }

        // ============ Get pose + freshness ============
        let pose = robotstate::read_pose().await;
        let pose_is_fresh = robotstate::get_and_clear_pose_fresh();

        // ============ Bézier setpoint at time t ============
        let setpoint = robot.beziercurve(bezier.clone(), t_sec, GOTO_DURATION_S);

        // ============ Control ============
        let (ul, ur, x_error, y_error, theta_error);

        match mode {
            ControlMode::WithMocapController => {
                robot.s.x = pose.x;
                robot.s.y = pose.y;
                robot.s.theta = SO2::new(pose.yaw);

                if pose_is_fresh {
                    let (action, x_e, y_e, yaw_e) = controller.control(robot, setpoint);
                    ul = action.ul;
                    ur = action.ur;
                    x_error = x_e;
                    y_error = y_e;
                    theta_error = yaw_e;
                } else {
                    defmt::warn!("goto: mocap stale -> feedforward");
                    ul = (2.0 * setpoint.vdes - robot_cfg.wheel_base * setpoint.wdes)
                        / (2.0 * robot_cfg.wheel_radius);
                    ur = (2.0 * setpoint.vdes + robot_cfg.wheel_base * setpoint.wdes)
                        / (2.0 * robot_cfg.wheel_radius);
                    x_error = 0.0;
                    y_error = 0.0;
                    theta_error = 0.0;
                }
            }
            ControlMode::DirectDuty => {
                ul = (2.0 * setpoint.vdes - robot_cfg.wheel_base * setpoint.wdes)
                    / (2.0 * robot_cfg.wheel_radius);
                ur = (2.0 * setpoint.vdes + robot_cfg.wheel_base * setpoint.wdes)
                    / (2.0 * robot_cfg.wheel_radius);
                x_error = 0.0;
                y_error = 0.0;
                theta_error = 0.0;
            }
        }

        // ============ Send wheel command ============
        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        // ============ Log ============
        let t_ms = t.as_millis() as u32;
        let log = TrajControlLog {
            timestamp_ms: t_ms,
            target_x: setpoint.des.x,
            target_y: setpoint.des.y,
            target_theta: setpoint.des.theta.rad(),
            actual_x: pose.x,
            actual_y: pose.y,
            actual_theta: pose.yaw,
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
            actual_qw: cosf(pose.yaw),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: sinf(pose.yaw),
            xerror: x_error,
            yerror: y_error,
            thetaerror: theta_error,
            ul,
            ur,
            dutyl: 0.0,
            dutyr: 0.0,
        };

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        })
        .await;

        defmt::info!(
            "goto: t={}s pos=({},{},{}) des=({},{},{}) v={} w={} u=({},{}) err=({},{},{})",
            t_sec,
            pose.x,
            pose.y,
            pose.yaw,
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            setpoint.vdes,
            setpoint.wdes,
            ul,
            ur,
            x_error,
            y_error,
            theta_error
        );
    }
}
