use crate::math::SO2;
use core::f32::consts::PI;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use libm::{atan2f, cosf, sinf, sqrtf};

use crate::motor::{Motor, MotorController};
use crate::trajectory_signal::{FIRST_MESSAGE, LAST_STATE, PoseAbs, STATE_SIG};

// Robot constants for diffdrive control
#[cfg(feature = "zumo")]
mod robot_constants_diffdrive {
    pub const DT_S: f32 = 0.1;
    pub const WHEEL_RADIUS: f32 = 0.02; // 20mm wheel radius
    pub const WHEEL_BASE: f32 = 0.099; // 99mm wheelbase
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const KX: f32 = 1.0;
    pub const KY: f32 = 1.0;
    pub const KTHETA: f32 = 2.0;
}

#[cfg(feature = "three-pi")]
mod robot_constants_diffdrive {
    pub const DT_S: f32 = 0.1;
    pub const WHEEL_RADIUS: f32 = 0.016; // 16mm wheel radius 
    pub const WHEEL_BASE: f32 = 0.0842; // 84.2mm wheelbase
    pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
    pub const KX: f32 = 1.0;
    pub const KY: f32 = 1.0;
    pub const KTHETA: f32 = 2.0;
}

#[cfg(not(any(feature = "zumo", feature = "three-pi")))]
mod robot_constants_diffdrive {
    pub const DT_S: f32 = 0.1; // Default to 100ms time step
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const KX: f32 = 1.0;
    pub const KY: f32 = 1.0;
    pub const KTHETA: f32 = 2.0;
    pub const GEAR_RATIO: f32 = 75.81;
    pub const ENCODER_CPR: f32 = -GEAR_RATIO * 12.0 / 4.0;
    pub const MAX_SPEED: f32 = 0.65; // 65 cm/s, according to datasheet 75:1 Gear Ratio
    //find out the actual maximum speed of Zumo
    pub const WHEEL_MAX: f32 = 0.233 * MAX_SPEED / WHEEL_RADIUS; // Maximum wheel speed in m/s scaling is experimentally derived
}
//TODO:
//clean up robot struct and timer
//add abstraction layer for trajectory selection
//time to test the position controller with mocap system
use robot_constants_diffdrive::*;

//pub static DIFFDRIVE_TRAJECTORY_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

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

#[derive(Debug)]
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

    pub fn circlereference(&self, t: f32, r: f32, wd: f32) -> DiffdriveSetpoint {
        //apply scaling of t to reduce the angular velocity desired
        let x = r * cosf(t * wd);
        let y = r * sinf(t * wd);
        //note: multiply with inner derivative
        let xd = -r * sinf(t * wd) * wd;
        let yd = r * cosf(t * wd) * wd;
        let xdd = -r * cosf(t * wd) * wd * wd;
        let ydd = -r * sinf(t * wd) * wd * wd;

        let vdes: f32 = sqrtf(xd * xd + yd * yd); //r * wd;
        let wdes: f32 = (ydd * xd - xdd * yd) / (xd * xd + yd * yd); //wd
        let theta = SO2::new(atan2f(yd, xd));

        let des: DiffdriveState = DiffdriveState { x, y, theta };
        DiffdriveSetpoint { des, vdes, wdes }
    }

    pub fn beziercurve(&self, p: Point, t: f32, tau: f32) -> DiffdriveSetpoint {
        //issue: Zumo has trouble keeping the angular velocity for the whole trajectory
        let t_norm = t / tau;
        let one_minus_t = 1.0 - t_norm;

        let mut x = one_minus_t * one_minus_t * one_minus_t * p.p0x
            + 3.0 * t_norm * one_minus_t * one_minus_t * p.p1x
            + 3.0 * t_norm * t_norm * one_minus_t * p.p2x
            + t_norm * t_norm * t_norm * p.p3x;
        let mut y = one_minus_t * one_minus_t * one_minus_t * p.p0y
            + 3.0 * t_norm * one_minus_t * one_minus_t * p.p1y
            + 3.0 * t_norm * t_norm * one_minus_t * p.p2y
            + t_norm * t_norm * t_norm * p.p3y;
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
        let theta = SO2::new(atan2f(yd, xd));

        x += p.x_ref;
        y += p.y_ref;

        let des: DiffdriveState = DiffdriveState { x, y, theta };
        DiffdriveSetpoint { des, vdes, wdes }
    }

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

    pub fn control(&self, robot: &Diffdrive, setpoint: DiffdriveSetpoint) -> DiffdriveAction {
        // Compute the error of the states
        let xerror: f32 = robot.s.theta.cos() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.sin() * (setpoint.des.y - robot.s.y);
        let yerror: f32 = -robot.s.theta.sin() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.cos() * (setpoint.des.y - robot.s.y);
        let therror: f32 = SO2::error(setpoint.des.theta, robot.s.theta);

        // Compute the control inputs using feedback linearization
        let v: f32 = setpoint.vdes * cosf(therror) + self.kx * xerror;
        let w: f32 = setpoint.wdes + setpoint.vdes * (self.ky * yerror + sinf(therror) * self.kth);

        // Convert to wheel speeds
        let mut ur = (2.0 * v + robot.l * w) / (2.0 * robot.r);
        let mut ul = (2.0 * v - robot.l * w) / (2.0 * robot.r);
        //trying to give out an rpm value

        // Clip actions to permissible range
        ur = ur.clamp(robot.ur_min, robot.ur_max);
        ul = ul.clamp(robot.ul_min, robot.ul_max);

        DiffdriveAction { ul, ur }
    }
}

/// Embassy task for diffdrive trajectory following control
/// This task demonstrates circle and bezier curve trajectory following using the diffdrive model
#[embassy_executor::task]
pub async fn diffdrive_control_task(motor: MotorController) {
    // Wait for trajectory system to be ready
    //DIFFDRIVE_TRAJECTORY_READY.wait().await;

    // Initialize robot model
    let mut robot = Diffdrive::new(
        WHEEL_RADIUS, // 0.02 [m]
        WHEEL_BASE,   // 0.099 [m]
        -1.0,
        1.0,
        -1.0,
        1.0,
        //-MAX_SPEED,   // left wheel min
        //MAX_SPEED,    // left wheel max
        //-MAX_SPEED,   // right wheel min
        //MAX_SPEED,    // right wheel max
    ); //some max values ....

    // Initialize controller
    let controller = DiffdriveController::new(KX, KY, KTHETA);

    // Wait for first mocap pose. not needed for now.
    //FIRST_MESSAGE.wait().await; --> no mocap needed for diff flatness generated action sequence

    let mut pose: PoseAbs = {
        let s = LAST_STATE.lock().await;
        *s
    };

    // Update robot state from mocap
    robot.s.x = pose.x;
    robot.s.y = pose.y;
    robot.s.theta = SO2::new(pose.yaw);

    let mut ticker = Ticker::every(Duration::from_millis((DT_S * 1000.0) as u64)); //define waiting time in loop
    let start = Instant::now();

    defmt::info!("Starting diffdrive trajectory following");

    // Demo trajectories
    let circle_radius = 0.2; // 20cm radius
    let circle_duration = 4.0; // 4 seconds

    // Bezier curve example doesn't look great.
    let bezier_duration = 30.0; // 30 seconds
    let bezier_point = Point {
        p0x: 0.0,
        p0y: 0.0,
        p1x: 0.0,
        p1y: 0.5,
        p2x: 0.5,
        p2y: 0.0,
        p3x: 0.5,
        p3y: 0.5,
        x_ref: 0.0,
        y_ref: 0.0,
        theta_ref: 0.0,
    };

    //counter for timestep in trajectory generation
    let mut t_counter = 0.0;

    loop {
        // Only compute and publish setpoint for speed controller
        match select(STATE_SIG.wait(), ticker.next()).await {
            Either::First(new_pose) => {
                pose = new_pose;
                robot.s.x = pose.x;
                robot.s.y = pose.y;
                robot.s.theta = SO2::new(pose.yaw);
                continue;
            }
            Either::Second(_) => {}
        }

        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;

        let wd = 0.39; // rad/s, desired angular velocity

        //let setpoint = robot.circlereference(t_counter * DT_S, circle_radius, wd);
        let setpoint = robot.beziercurve(bezier_point.clone(), t_counter * DT_S, bezier_duration);
        let w = setpoint.wdes;
        let v = setpoint.vdes;

        //calculate the angular velocity by defining how fast the robot should move about the circle.
        //let w = 2.0 * PI / circle_duration; // rad/s
        //from that get the linear velocity with the circle radius
        //let v = w * circle_radius; // m/s

        //for output logging
        let w_rad = w; // rad/s
        let w_deg = w_rad * 180.0 / PI; // deg/s

        // Convert to wheel speeds
        let mut ur = (2.0 * v + robot.l * w) / (2.0 * robot.r); //rad/s
        let mut ul = (2.0 * v - robot.l * w) / (2.0 * robot.r);
        //trying to give out an rpm value

        // Clip actions to permissible range in case they go over limits
        ur = (ur / WHEEL_MAX).clamp(robot.ur_min, robot.ur_max);
        ul = (ul / WHEEL_MAX).clamp(robot.ul_min, robot.ul_max);

        //give the caclulated duty to the motors scaled by 1.66
        //let mut duty_left = ul_ff * 1.66;
        //let mut duty_right = ur_ff * 1.66; //example scaling factor, adjust as needed
        // Clamp the duties to the motor limits

        motor
            .set_speed(ul * MOTOR_DIRECTION_LEFT, ur * MOTOR_DIRECTION_RIGHT)
            .await;

        //and give it to the motor:

        defmt::info!(
            "t={}s, posd = ({},{},{}), v={}, w={} rad/s ({} deg/s), u_ff=({},{})",
            t_sec,
            setpoint.des.x,
            setpoint.des.y,
            setpoint.des.theta.rad(),
            v,
            w_rad,
            w_deg,
            ul,
            ur
        );

        // Stop after complete trajectory
        // if t_sec > circle_duration {
        //     motor.set_speed(0.0, 0.0).await;
        //     defmt::info!("Diffdrive trajectory following complete");
        //     break;
        // }

        //stop after the counter has arrived at bezier_duration / DT_S
        if t_counter > bezier_duration / DT_S {
            //motor.set_speed(0.0, 0.0).await;
            motor.set_speed(0.0, 0.0).await;
            defmt::info!(
                "Diffdrive trajectory following complete after {} steps",
                t_counter
            );
            break;
        }
        //Timer::after(Duration::from_millis((DT_S * 1000.0) as u64)).await;
        //increase t_counter;
        t_counter += 1.0;
    }
}

#[inline]
fn wrap_angle(a: f32) -> f32 {
    let mut x = (a + PI) % (2.0 * PI);
    if x < 0.0 {
        x += 2.0 * PI;
    }
    x - PI
}
