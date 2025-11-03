use crate::math::SO2;
use crate::sdlog::{SdLogger, TrajControlLog};
// use crate::trajectory_reading::Pose;
use core::f32::consts::PI;
use embassy_futures::select::{Either, select};
// use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use libm::{atan2f, cosf, sinf, sqrtf};

use crate::led::{self};
use crate::motor::MotorController;
use crate::robot_parameters_default::robot_constants::*;
use crate::trajectory_signal::{LAST_STATE, PoseAbs, STATE_SIG};

// pub static DIFFDRIVE_TRAJECTORY_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

#[derive(Debug, Copy, Clone)]
pub struct DiffdriveState {
    pub x: f32,     // m
    pub y: f32,     // m
    pub theta: SO2, // rad
}

#[derive(Debug)]
pub struct DiffdriveAction {
    pub ul: f32, // rad/s
    pub ur: f32, // rad/s
}

#[derive(Debug)]
pub struct Diffdrive {
    pub r: f32,      // wheel radius [m]
    pub l: f32,      // distance between wheels [m]
    pub ul_min: f32, // action limit [rad/s]
    pub ul_max: f32, // action limit [rad/s]
    pub ur_min: f32, // action limit [rad/s]
    pub ur_max: f32, // action limit [rad/s]
    pub s: DiffdriveState,
}

#[derive(Debug, Copy, Clone)]
pub struct DiffdriveSetpoint {
    pub des: DiffdriveState,
    pub vdes: f32,
    pub wdes: f32,
}

#[derive(Debug)]
pub struct DiffdriveController {
    pub kx: f32,
    pub ky: f32,
    pub kth: f32,
}

#[derive(Debug, Clone)]
pub struct Point {
    pub p0x: f32,
    pub p0y: f32,
    pub p1x: f32,
    pub p1y: f32,
    pub p2x: f32,
    pub p2y: f32,
    pub p3x: f32,
    pub p3y: f32,
    pub x_ref: f32,
    pub y_ref: f32,
    pub theta_ref: f32,
}

impl Diffdrive {
    pub fn new(r: f32, l: f32, ul_min: f32, ul_max: f32, ur_min: f32, ur_max: f32) -> Self {
        Self {
            r,
            l,
            ul_min,
            ul_max,
            ur_min,
            ur_max,
            s: DiffdriveState {
                x: 0.0,
                y: 0.0,
                theta: SO2::new(0.0),
            },
        }
    }

    pub fn circle_params_origin_normal_left(
        &self,
        x0: f32,
        y0: f32,
        theta0: f32,
        r: f32,
        omega_abs: f32,
    ) -> (f32, f32, f32, f32) {
        // compute the unit vector from the origin to the initial position
        let d = sqrtf(x0 * x0 + y0 * y0);
        // if the initial position is close to zero, then set a random direction
        let (ux, uy) = if d > 1e-6 {
            (x0 / d, y0 / d)
        } else {
            (0.0, 1.0)
        };

        // left circle, so rotate CCW for 90 degrees.
        let (nlx, nly) = (-uy, ux);

        // Circle center is on the orthogonal lines, distance is the radius
        let cx = x0 + r * nlx;
        let cy = y0 + r * nly;

        // initial phase, the angle between the center to p0 and the x axis
        let phi0 = atan2f(y0 - cy, x0 - cx);

        // t_hat = (-sin phi0, cos phi0)
        let (tx, ty) = (-sinf(phi0), cosf(phi0));

        // initial direction vector
        let (hx, hy) = (cosf(theta0), sinf(theta0));

        // choose the sign for the angular velocity and make sure it aligns with the
        let s = if hx * tx + hy * ty >= 0.0 { 1.0 } else { -1.0 };
        let omega_signed = s * omega_abs;

        (cx, cy, phi0, omega_signed)
    }

    pub fn circle_reference_t(
        &self,
        t_s: f32,   // time [s]
        r: f32,     // radius [m]
        omega: f32, // angular velocity
        x0: f32,    // center x
        y0: f32,    // center y
        phi0: f32,  // intial phase
    ) -> DiffdriveSetpoint {
        let phi = phi0 + omega * t_s;
        let (s, c) = (sinf(phi), cosf(phi));

        let x = x0 + r * c;
        let y = y0 + r * s;

        let xd = -r * omega * s;
        let yd = r * omega * c;

        let xdd = -r * omega * omega * c;
        let ydd = -r * omega * omega * s;

        let vdes = sqrtf(xd * xd + yd * yd); // ≈ r*|omega|
        let denom = (xd * xd + yd * yd).max(1e-9);
        let wdes = (ydd * xd - xdd * yd) / denom; // ≈ omega

        let theta = SO2::new(atan2f(yd, xd));

        let des = DiffdriveState { x, y, theta };
        DiffdriveSetpoint { des, vdes, wdes }
    }

    pub fn circlereference(
        &self,
        t: f32,
        r: f32,
        wd: f32,
        x0: f32,
        y0: f32,
        theta0: f32,
    ) -> DiffdriveSetpoint {
        //apply scaling of t to reduce the angular velocity desired
        let mut x = r * cosf(t * wd);
        let mut y = r * sinf(t * wd);
        //note: multiply with inner derivative
        let xd = -r * sinf(t * wd) * wd;
        let yd = r * cosf(t * wd) * wd;
        let xdd = -r * cosf(t * wd) * wd * wd;
        let ydd = -r * sinf(t * wd) * wd * wd;

        let vdes: f32 = sqrtf(xd * xd + yd * yd); //r * wd;
        let wdes: f32 = (ydd * xd - xdd * yd) / (xd * xd + yd * yd); //wd
        let theta_loc = atan2f(yd, xd);

        //transform into body coordinates and to circle starting point.
        x = x + x0 - x * cosf(theta0);
        y = y + y0 - y * sinf(theta0);
        let theta = SO2::new(theta0 + theta_loc);

        let des: DiffdriveState = DiffdriveState { x, y, theta };
        DiffdriveSetpoint { des, vdes, wdes }
    }

    pub fn beziercurve(&self, p: Point, t: f32, tau: f32) -> DiffdriveSetpoint {
        //issue: Zumo has trouble keeping the angular velocity for the whole trajectory
        let t_norm = t / tau;
        let one_minus_t = 1.0 - t_norm;

        // position in local (curve) frame
        let mut x = one_minus_t * one_minus_t * one_minus_t * p.p0x
            + 3.0 * t_norm * one_minus_t * one_minus_t * p.p1x
            + 3.0 * t_norm * t_norm * one_minus_t * p.p2x
            + t_norm * t_norm * t_norm * p.p3x;
        let mut y = one_minus_t * one_minus_t * one_minus_t * p.p0y
            + 3.0 * t_norm * one_minus_t * one_minus_t * p.p1y
            + 3.0 * t_norm * t_norm * one_minus_t * p.p2y
            + t_norm * t_norm * t_norm * p.p3y;

        // time-derivatives
        let xd = (3.0 / tau) * one_minus_t * one_minus_t * (p.p1x - p.p0x)
            + 6.0 * (t / (tau * tau)) * one_minus_t * (p.p2x - p.p1x)
            + (3.0 / tau) * t_norm * t_norm * (p.p3x - p.p2x);
        let yd = (3.0 / tau) * one_minus_t * one_minus_t * (p.p1y - p.p0y)
            + 6.0 * (t / (tau * tau)) * one_minus_t * (p.p2y - p.p1y)
            + (3.0 / tau) * t_norm * t_norm * (p.p3y - p.p2y);
        let xdd: f32 = (6.0 / (tau * tau)) * one_minus_t * (p.p2x - 2.0 * p.p1x + p.p0x)
            + 6.0 * (t / (tau * tau * tau)) * (p.p3x - 2.0 * p.p2x + p.p1x);
        let ydd: f32 = (6.0 / (tau * tau)) * one_minus_t * (p.p2y - 2.0 * p.p1y + p.p0y)
            + 6.0 * (t / (tau * tau * tau)) * (p.p3y - 2.0 * p.p2y + p.p1y);

        let vdes: f32 = sqrtf(xd * xd + yd * yd);
        let wdes: f32 = (ydd * xd - xdd * yd) / (xd * xd + yd * yd);

        // tangent in local frame
        let theta_loc = atan2f(yd, xd);
        // initial tangent at u=0

        //note: affine transformation to align the beziercurve trajectory with the robots initial position and
        //the tangent of the curve in controlpoint p0 aligning with the robots heading.
        let theta0 = atan2f(p.p1y - p.p0y, p.p1x - p.p0x);
        let dtheta = p.theta_ref + 0.5 * PI - theta0; //calc offset robot heading vs bezier start tangent
        let theta = theta_loc - (theta0 - p.theta_ref); //add that diff to all thetas in the trajectory

        // rotation + translation
        let x_rot = x * cosf(dtheta) - y * sinf(dtheta); //turn whole trajectory by theta offset 
        let y_rot = x * sinf(dtheta) + y * cosf(dtheta);
        x = p.x_ref + x_rot;
        y = p.y_ref + y_rot;

        let theta = SO2::new(theta);

        let des: DiffdriveState = DiffdriveState { x, y, theta };
        DiffdriveSetpoint { des, vdes, wdes }
    }

    //only for simulation?
    pub fn step(&mut self, action: DiffdriveAction, dt: f32) {
        let x_new = self.s.x + (0.5 * self.r) * (action.ul + action.ur) * (self.s.theta).cos() * dt;
        let y_new = self.s.y + (0.5 * self.r) * (action.ul + action.ur) * (self.s.theta).sin() * dt;
        let theta_new = self.s.theta.rad() + (self.r / self.l) * (action.ur - action.ul) * dt;
        self.s.x = x_new;
        self.s.y = y_new;
        self.s.theta = SO2::new(theta_new);
    }
}

impl DiffdriveController {
    pub fn new(kx: f32, ky: f32, kth: f32) -> Self {
        Self { kx, ky, kth }
    }

    pub fn control(
        &self,
        robot: &Diffdrive,
        setpoint: DiffdriveSetpoint,
    ) -> (DiffdriveAction, f32, f32, f32) {
        // Compute the error of the states in body frame
        let xerror: f32 = robot.s.theta.cos() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.sin() * (setpoint.des.y - robot.s.y);
        let yerror: f32 = -robot.s.theta.sin() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.cos() * (setpoint.des.y - robot.s.y);
        let therror: f32 = SO2::error(setpoint.des.theta, robot.s.theta);

        // Compute the control inputs using feedback linearization
        let v: f32 = setpoint.vdes * cosf(therror) + self.kx * xerror;
        let w: f32 = setpoint.wdes + setpoint.vdes * (self.ky * yerror + sinf(therror) * self.kth);

        // Convert to wheel speeds
        let ur = (2.0 * v + robot.l * w) / (2.0 * robot.r);
        let ul = (2.0 * v - robot.l * w) / (2.0 * robot.r);

        // Return the action and the errors as a tuple
        (DiffdriveAction { ul, ur }, xerror, yerror, therror)
    }
}

/// Embassy task for diffdrive trajectory following control
/// This task demonstrates circle and bezier curve trajectory following using the diffdrive model
#[embassy_executor::task]
pub async fn diffdrive_control_task(
    motor: MotorController,
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
) {
    // Wait for trajectory system to be ready
    //DIFFDRIVE_TRAJECTORY_READY.wait().await;

    // Initialize robot model
    defmt::info!("Initializing diffdrive robot model");
    defmt::info!(
        "Wheel radius: {}, Wheel base: {}, Wheel speed max: {}",
        WHEEL_RADIUS,
        WHEEL_BASE,
        WHEEL_MAX,
    );
    let mut robot = Diffdrive::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX, // maximum rotation velocity of the wheels in rad/s
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );

    // Initialize controller
    let controller = DiffdriveController::new(KX_TRAJ, KY_TRAJ, KTHETA_TRAJ);
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    //Log on SD Card
    // Wait for first mocap pose. not needed for now.
    //FIRST_MESSAGE.wait().await; //--> no mocap needed for diff flatness generated action sequence
    //wait here for s short while:
    let mut mocap = false;
    Timer::after(Duration::from_millis(1000)).await;
    defmt::info!("waiting briefly for poses, start trajectory following with or without mocap");
    let mut pose: PoseAbs = {
        let s = LAST_STATE.lock().await;
        //set mocap to true, if a pose is available here:
        if s.pose.x != 0.0 || s.pose.y != 0.0 || s.pose.yaw != 0.0 {
            mocap = true;
        };
        //*s;
        s.pose.clone()
    };

    // initialise robot state from mocap if available. if not it is (0,0,0)
    robot.s.x = pose.x;
    robot.s.y = pose.y;
    robot.s.theta = SO2::new(pose.yaw);

    let initial_position_x = pose.x;
    let initial_position_y = pose.y;
    let initial_position_yaw = pose.yaw;

    let mut ticker = Ticker::every(Duration::from_millis((TRAJ_FOLLOWING_DT_S * 1000.0) as u64)); //define waiting time in loop

    defmt::info!("Starting diffdrive trajectory following");

    // Demo trajectories
    let circle_radius = 0.2; // 50cm radius
    let circle_duration = 8.0; // 20 seconds

    //set a desired angular velocity for the circle trajectory
    let wd = (2.0 * PI) / circle_duration; // rad/s, desired angular velocity

    // let bezier_duration = 8.0; // 30 seconds
    // let bezier_point = Point {
    //     p0x: 0.0,
    //     p0y: 0.0,
    //     p1x: 0.25,
    //     p1y: 0.25,
    //     p2x: 0.5,
    //     p2y: 0.5,
    //     p3x: 0.75,
    //     p3y: 0.75,
    //     x_ref: initial_position_x,
    //     y_ref: initial_position_y,
    //     theta_ref: initial_position_yaw,
    // };
    // let (cx, cy, phi0, omega) = robot.circle_params_origin_normal_left(
    //     initial_position_x,
    //     initial_position_y,
    //     initial_position_yaw,
    //     circle_radius,
    //     wd,
    // );

    //bezier example
    //let setpoint = robot.beziercurve(bezier_point.clone(), t_counter * DT_S, bezier_duration);

    //counter for timestep in trajectory generation
    let mut t_counter = 0.0;

    //make q0 of type diffdrive state
    //led.off();

    // percisely get the start time of the trajectory following task
    let start = Instant::now();
    loop {
        //TODO: test circle with mocap position controller
        //test diffdrive circle with mocap position controller
        //test bezier curve with mocap position controller

        //ideas: write minimal interface with keyboard controls (wasd) to drive the robot around
        //write menu to switch modes with keyboard commands
        //modes: wasd control, trajectory control with 1: circle, 2:bezier
        //3: trajectory following from sdcard

        // Only compute and publish setpoint for speed controller

        // Place pose handling at the beginning of the loop

        // All control logic executes here (only when ticker fires)
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as f32;

        // let setpoint = robot.beziercurve(bezier_point.clone(), t_counter * DT_S, bezier_duration);
        // let state_des = setpoint.des;
        // let vd = setpoint.vdes;
        // let wd = setpoint.wdes;

        let w_rad = wd; // rad/s
        let w_deg = w_rad * 180.0 / PI; // deg/s

        let vd = circle_radius * wd; // 0.5 × 0.785 = 0.393 m/s = 39.3 cm/s

        let phi0 = SO2::new(initial_position_yaw);

        let state_des = DiffdriveState {
            x: initial_position_x
                + circle_radius * (cosf(phi0.rad() + w_rad * t_sec) - cosf(phi0.rad())),
            y: initial_position_y
                + circle_radius * (sinf(phi0.rad() + w_rad * t_sec) - sinf(phi0.rad())),
            theta: SO2::new(phi0.rad() + w_rad * t_sec),
        };

        let setpoint = DiffdriveSetpoint {
            des: state_des,
            vdes: vd,
            wdes: wd,
        };

        let state_des = DiffdriveState {
            x: setpoint.des.x,
            y: setpoint.des.y,
            theta: setpoint.des.theta,
        };

        let ur;
        let ul;
        let mut xerror = 0.0;
        let mut yerror = 0.0;
        let mut therror = 0.0;

        if mocap {
            //if a pose was received, hand it to the controller
            defmt::info!(
                "handing to the controller: setpoint wd: {}, vd: {}",
                setpoint.wdes,
                setpoint.vdes
            );
            let controloutput = controller.control(&robot, setpoint);
            ur = controloutput.0.ur;
            ul = controloutput.0.ul;
            xerror = controloutput.1;
            yerror = controloutput.2;
            therror = controloutput.3;
            led.on();
        } else {
            //drive blindly with the last known pose and desired speeds and angles from diff flatness computation.
            defmt::info!("no mocap data, drive blind with actions.");
            ur = (2.0 * vd + robot.l * w_rad) / (2.0 * robot.r);
            ul = (2.0 * vd - robot.l * w_rad) / (2.0 * robot.r);
            led.off();
        }

        let dutyl = (ul / WHEEL_MAX).clamp(-1.0, 1.0);
        let dutyr = (ur / WHEEL_MAX).clamp(-1.0, 1.0);

        let log: TrajControlLog = TrajControlLog {
            timestamp_ms: t_ms as u32,
            target_x: state_des.x,
            target_y: state_des.y,
            target_theta: state_des.theta.rad(),
            actual_x: robot.s.x,
            actual_y: robot.s.y,
            actual_theta: robot.s.theta.rad(),
            target_vx: vd,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: 0.0,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: state_des.theta.cos(),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: state_des.theta.sin(),
            actual_qw: robot.s.theta.cos(),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: robot.s.theta.sin(),
            xerror: xerror,
            yerror: yerror,
            thetaerror: therror,
            ul: ul,
            ur: ur,
            dutyl: dutyl,
            dutyr: dutyr,
        };

        if let Some(ref mut logger) = sdlogger {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }

        motor
            .set_speed(dutyl * MOTOR_DIRECTION_LEFT, dutyr * MOTOR_DIRECTION_RIGHT)
            .await;

        defmt::info!(
            "t={}s, pos = ({},{},{}) posd = ({},{},{}), v={}, w={} rad/s ({} deg/s), u=({},{}), duty = ({},{}), (xerr, yerr, thetaerr) = ({},{},{})",
            t_sec,
            pose.x,
            pose.y,
            pose.yaw,
            state_des.x,
            state_des.y,
            state_des.theta.rad(),
            vd,
            w_rad,
            w_deg,
            ul,
            ur,
            dutyl,
            dutyr,
            xerror,
            yerror,
            therror
        );

        if t_sec > circle_duration {
            motor.set_speed(0.0, 0.0).await;
            defmt::info!(
                "Diffdrive trajectory following complete after {} steps",
                t_counter
            );
            break;
        }

        //capture next pose
        match select(STATE_SIG.wait(), ticker.next()).await {
            Either::First(new_pose) => {
                pose = new_pose;
                robot.s.x = pose.x;
                robot.s.y = pose.y;
                robot.s.theta = SO2::new(pose.yaw);
                defmt::info!("New pose received: ({}, {}, {})", pose.x, pose.y, pose.yaw);
                mocap = true;
                ticker.next().await; //wait for the ticket to pass after the new pose was captured (wrong, curz this will cause the task to wait another 10ms)
            }
            Either::Second(_) => {
                // Ticker fired before new pose came in: need to use last pose that we had.
                mocap = false;
                defmt::warn!(
                    "Control tick - using pose: ({}, {}, {})",
                    pose.x,
                    pose.y,
                    pose.yaw
                );
            }
        }

        t_counter += 1.0;
    }
}

/*
// TODO: test circle with mocap position controller
// test diffdrive circle with mocap position controller
// test bezier curve with mocap position controller

// ideas: write minimal interface with keyboard controls (wasd) to drive the robot around
// write menu to switch modes with keyboard commands
// modes: wasd control, trajectory control with 1: circle, 2:bezier
// 3: trajectory following from sdcard

// Only compute and publish setpoint for speed controller

// Place pose handling at the beginning of the loop

// All control logic executes here (only when ticker fires)
*/

/// Embassy task for diffdrive trajectory following control
/// This task demonstrates circle and bezier curve trajectory following using the diffdrive model
#[embassy_executor::task]
pub async fn diffdrive_control_task_new(
    motor: MotorController,
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
) {
    // Wait for trajectory system to be ready, only used when the SD card trajectory loading is enabled
    // DIFFDRIVE_TRAJECTORY_READY.wait().await;

    // Initialize robot model
    defmt::info!("Initializing diffdrive robot model");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        WHEEL_RADIUS,
        WHEEL_BASE,
        WHEEL_MAX,
    );
    let mut robot = Diffdrive::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX, // maximum rotation velocity of the wheels in rad/s
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );

    // Initialize controller
    let controller = DiffdriveController::new(KX_TRAJ, KY_TRAJ, KTHETA_TRAJ);
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    // FIRST_MESSAGE.wait().await; //--> no mocap needed for diff flatness generated action sequence
    Timer::after(Duration::from_millis(1000)).await;
    defmt::info!("waiting briefly for poses, start trajectory following with or without mocap");

    let mut pose: PoseAbs = {
        let s = LAST_STATE.lock().await;
        //*s;
        s.pose.clone()
    };
    let mut mocap = !(pose.x == 0.0 && pose.y == 0.0 && pose.yaw == 0.0);

    if mocap {
        defmt::info!("mocap is on, true initial position is recorded.")
    }
    else {
        defmt::info!("mocap is on, initial position is set to the origin (0, 0, 0).")
    }

    // initialise robot state from mocap if available. if not it is (0,0,0)
    robot.s.x = pose.x;
    robot.s.y = pose.y;
    robot.s.theta = SO2::new(pose.yaw);

    let initial_position_x = pose.x;
    let initial_position_y = pose.y;
    let initial_position_yaw = pose.yaw;
    let phi0 = SO2::new(initial_position_yaw);

    let mut ticker = Ticker::every(Duration::from_millis((TRAJ_FOLLOWING_DT_S * 1000.0) as u64)); //define waiting time in loop

    defmt::info!("Starting diffdrive trajectory following");

    /* =================== Demo Circle Trajectories ===================== */
    let circle_radius = 0.2; // 50cm radius
    let circle_duration = 8.0; // 20 seconds
    let wd = (2.0 * PI) / circle_duration; // rad/s, desired angular velocity
    /* ================================================================== */

    /* ================ Demo Bezier Curve Trajectories ================== */
    // let bezier_duration = 8.0; // 30 seconds
    // let bezier_point = Point {
    //     p0x: 0.0,
    //     p0y: 0.0,
    //     p1x: 0.25,
    //     p1y: 0.25,
    //     p2x: 0.5,
    //     p2y: 0.5,
    //     p3x: 0.75,
    //     p3y: 0.75,
    //     x_ref: initial_position_x,
    //     y_ref: initial_position_y,
    //     theta_ref: initial_position_yaw,
    // };

    // // bezier example
    // let setpoint = robot.beziercurve(bezier_point.clone(), t_sec, bezier_duration);
    /* ================================================================== */

    // percisely get the start time of the trajectory following task
    let start = Instant::now();

    loop {
        
        ticker.next().await;

        pose = {
            let s = LAST_STATE.lock().await;
            //*s;
            s.pose.clone()
        };
        mocap = !(pose.x == 0.0 && pose.y == 0.0 && pose.yaw == 0.0);

        if mocap {
            robot.s.x = pose.x;
            robot.s.y = pose.y;
            robot.s.theta = SO2::new(pose.yaw);
            defmt::info!("Using mocap pose: ({}, {}, {})", pose.x, pose.y, pose.yaw);
        }
        else {
            defmt::warn!("No mocap data, using last pose: ({}, {}, {})", pose.x, pose.y, pose.yaw);
        }

        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as f32;

        /* ============== Bezier Curve Generation ============= */
        // let setpoint = robot.beziercurve(bezier_point.clone(), t_counter * DT_S, bezier_duration);
        // let state_des = setpoint.des;
        // let vd = setpoint.vdes;
        // let wd = setpoint.wdes;
        // let w_rad = wd;
        // let w_deg = w_rad * 180 / PI;
        /* ==================================================== */

        /* =============== Demo Circle Generation ============= */
        let w_rad = wd;
        let w_deg = w_rad * 180.0 / PI;
        let vd = circle_radius * wd;

        let state_des = DiffdriveState {
            x: initial_position_x
                + circle_radius * (cosf(phi0.rad() + w_rad * t_sec) - cosf(phi0.rad())),
            y: initial_position_y
                + circle_radius * (sinf(phi0.rad() + w_rad * t_sec) - sinf(phi0.rad())),
            theta: SO2::new(phi0.rad() + w_rad * t_sec),
        };
        /* ===================================================== */

        // Load the reference to a setpoint
        let setpoint = DiffdriveSetpoint {
            des: state_des,
            vdes: vd,
            wdes: wd,
        };

        let (ur, ul, x_error, y_error, theta_error) = if mocap {
            defmt::info!("mocap data received, drive with controller.");
            let control_output = controller.control(&robot, setpoint);
            (
                control_output.0.ur,
                control_output.0.ul,
                control_output.1,
                control_output.2,
                control_output.3,
            )
        }
        else {
            // drive blindly with the last known pose and desired speeds and angles from diff flatness computation.
            defmt::info!("no mocap data, drive blind with actions.");
            let ur = (2.0 * vd + robot.l * w_rad) / (2.0 * robot.r);
            let ul = (2.0 * vd - robot.l * w_rad) / (2.0 * robot.r);
            (ur, ul, 0.0, 0.0, 0.0)
        };

        if mocap {
            led.on();
        }
        else {
            led.off();
        }

        // Duty Circle Conversion
        let dutyl = (ul / WHEEL_MAX).clamp(-1.0, 1.0);
        let dutyr = (ur / WHEEL_MAX).clamp(-1.0, 1.0);

        let log: TrajControlLog = TrajControlLog {
            timestamp_ms: t_ms as u32,
            target_x: state_des.x,
            target_y: state_des.y,
            target_theta: state_des.theta.rad(),
            actual_x: robot.s.x,
            actual_y: robot.s.y,
            actual_theta: robot.s.theta.rad(),
            target_vx: vd,
            target_vy: 0.0,
            target_vz: 0.0,
            actual_vx: 0.0,
            actual_vy: 0.0,
            actual_vz: 0.0,
            target_qw: state_des.theta.cos(),
            target_qx: 0.0,
            target_qy: 0.0,
            target_qz: state_des.theta.sin(),
            actual_qw: robot.s.theta.cos(),
            actual_qx: 0.0,
            actual_qy: 0.0,
            actual_qz: robot.s.theta.sin(),
            xerror: x_error,
            yerror: y_error,
            thetaerror: theta_error,
            ul: ul,
            ur: ur,
            dutyl: dutyl,
            dutyr: dutyr,
        };

        if let Some(ref mut logger) = sdlogger {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }

        motor
            .set_speed(dutyl * MOTOR_DIRECTION_LEFT, dutyr * MOTOR_DIRECTION_RIGHT)
            .await;

        defmt::info!(
            "t={}s, pos = ({},{},{}) posd = ({},{},{}), v={}, w={}rad/s ({}deg/s), u=({},{}), duty = ({},{}), (x_err, y_err, theta_err) = ({},{},{})",
            t_sec,
            pose.x,
            pose.y,
            pose.yaw,
            state_des.x,
            state_des.y,
            state_des.theta.rad(),
            vd,
            w_rad,
            w_deg,
            ul,
            ur,
            dutyl,
            dutyr,
            x_error,
            y_error,
            theta_error
        );

        if t_sec >= circle_duration {
            motor.set_speed(0.0, 0.0).await;
            defmt::info!(
                "Diffdrive trajectory following complete after {} s",
                t_sec
            );
            break;
        }
    }
}
