use crate::math::SO2;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::orchestrator_signal::STOP_ALL;
use crate::control_types::{
    DiffdriveCascade, DiffdriveControllerCascade, PointCascade, TrajectoryResult,
};
use crate::robotstate;
use crate::robotstate::{WHEEL_CMD_CH, WheelCmd};

use embassy_time::{Duration, Instant, Ticker};
use libm::{cosf, sinf, sqrtf};
use portable_atomic::Ordering;

// ======================== Constants ========================
const GOTO_DURATION_S: f32 = 5.0; //only meant for short ways
//const GOTO_DT_S: f32 = 0.1; // 10 Hz control loop -> now handled dynamically

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
    let dt_s = robot_cfg.traj_following_dt_s;
    let mut ticker = Ticker::every(Duration::from_millis((dt_s * 1000.0) as u64));

    // ==================== Initialize EKF ==================
    let mut fused_x: f32 = start_pose.x;
    let mut fused_y: f32 = start_pose.y;
    let mut fused_yaw: f32 = start_pose.yaw;
    let mut ekf = crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw);

    let start = Instant::now();

    // ==================== Control loop ====================
    loop {
        ticker.next().await;

        // Check stop
        if STOP_ALL.load(Ordering::Relaxed) {
            robotstate::stop_motors();
            defmt::info!("goto: stopped via STOP_ALL");
            return TrajectoryResult::Stopped;
        }

        // ============ Elapsed time ============
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;

        if t_sec >= GOTO_DURATION_S {
            // Stop motors
            robotstate::stop_motors();
            defmt::info!("goto: reached goal after {}s", t_sec);
            return TrajectoryResult::Completed;
        }

        /* =================== Pose Fusion =================== */
        let odom_now = robotstate::read_odom().await;
        ekf.predict(odom_now.v, odom_now.w, dt_s);

        let pose_is_fresh = robotstate::get_and_clear_pose_fresh();
        if pose_is_fresh {
            let mocap_pose = robotstate::read_pose().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;

        // Publish to robotstate for other tasks (logger, telemetry)
        robotstate::write_ekf_state(robotstate::RobotPose {
            x: fx, y: fy, yaw: fth, stamp: Instant::now(),
            ..robotstate::RobotPose::DEFAULT
        }).await;

        // ============ Bézier setpoint at time t ============
        let setpoint = robot.beziercurve(bezier.clone(), t_sec, GOTO_DURATION_S);
        
        robotstate::write_setpoint(robotstate::Setpoint {
            x_des: setpoint.des.x,
            y_des: setpoint.des.y,
            yaw_des: setpoint.des.theta.rad(),
            v_ff: setpoint.vdes,
            w_ff: setpoint.wdes,
            stamp: Instant::now(),
        }).await;

        // ============ Control ============
        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        let (action, x_error, y_error, theta_error) = controller.control(robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;


        // ============ Send wheel command ============
        robotstate::write_wheel_cmd(robotstate::WheelCmd::new(ul, ur)).await;
        robotstate::write_tracking_error(robotstate::TrackingError {
            x_err: x_error, y_err: y_error, yaw_err: theta_error,
        }).await;



        defmt::info!(
            "goto: t={}s pos=({},{},{}) des=({},{},{}) v={} w={} u=({},{}) err=({},{},{})",
            t_sec,
            fused_x,
            fused_y,
            fused_yaw,
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
