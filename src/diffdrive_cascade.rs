use crate::math::SO2;
use core::f32::consts::PI;
use defmt::info;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use libm::{atan2f, cosf, sinf, sqrtf};

use crate::encoder::wheel_speed_from_counts_now;
use crate::led::{self};
use crate::motor::MotorController;
use crate::robot_parameters_default::robot_constants::*;
use crate::sdlog::{SdLogger, TrajControlLog};
use crate::trajectory_signal::{
    FIRST_MESSAGE, LAST_STATE, PoseAbs, STATE_SIG, WHEEL_CMD_CH, WheelCmd,
};

use portable_atomic::{AtomicBool, Ordering};

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

// ====================== TODO: ===========================
// (1) add abstraction layer for trajectory selection
// (2) time to test the position controller with mocap system
// ====================== TODO: ===========================
#[derive(Debug, Copy, Clone)]
pub struct DiffdriveStateCascade {
    pub x: f32,     // m
    pub y: f32,     // m
    pub theta: SO2, // rad
}

#[derive(Debug)]
pub struct DiffdriveActionCascade {
    pub ul: f32, // rad/s
    pub ur: f32, // rad/s
}

#[derive(Debug)]
pub struct DiffdriveCascade {
    pub r: f32,      // wheel radius [m]
    pub l: f32,      // distance between wheels [m]
    pub ul_min: f32, // action limit [rad/s]
    pub ul_max: f32, // action limit [rad/s]
    pub ur_min: f32, // action limit [rad/s]
    pub ur_max: f32, // action limit [rad/s]
    pub s: DiffdriveStateCascade,
}

#[derive(Debug, Copy, Clone)]
pub struct DiffdriveSetpointCascade {
    pub des: DiffdriveStateCascade,
    pub vdes: f32,
    pub wdes: f32,
}

#[derive(Debug)]
pub struct DiffdriveControllerCascade {
    pub kx: f32,
    pub ky: f32,
    pub kth: f32,
}

impl DiffdriveControllerCascade {
    pub fn new(kx: f32, ky: f32, kth: f32) -> Self {
        Self { kx, ky, kth }
    }

    pub fn control(
        &self,
        robot: &DiffdriveCascade,
        setpoint: DiffdriveSetpointCascade,
    ) -> (DiffdriveActionCascade, f32, f32, f32) {
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
        (DiffdriveActionCascade { ul, ur }, xerror, yerror, therror)
    }
}

#[derive(Debug, Clone)]
pub struct PointCascade {
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

impl DiffdriveCascade {
    pub fn new(r: f32, l: f32, ul_min: f32, ul_max: f32, ur_min: f32, ur_max: f32) -> Self {
        Self {
            r,
            l,
            ul_min,
            ul_max,
            ur_min,
            ur_max,
            s: DiffdriveStateCascade {
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
        t_s: f32,   // 时间 [s]
        r: f32,     // radius [m]
        omega: f32, // angular velocity
        x0: f32,    // center x
        y0: f32,    // center y
        phi0: f32,  // intial phase
    ) -> DiffdriveSetpointCascade {
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

        let des = DiffdriveStateCascade { x, y, theta };
        DiffdriveSetpointCascade { des, vdes, wdes }
    }

    pub fn circlereference(
        &self,
        t: f32,
        r: f32,
        wd: f32,
        x0: f32,
        y0: f32,
        theta0: f32,
    ) -> DiffdriveSetpointCascade {
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

        let des: DiffdriveStateCascade = DiffdriveStateCascade { x, y, theta };
        DiffdriveSetpointCascade { des, vdes, wdes }
    }

    pub fn beziercurve(&self, p: PointCascade, t: f32, tau: f32) -> DiffdriveSetpointCascade {
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

        // time-derivatives (your original formulas)
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
        let dtheta = theta0 - p.theta_ref; //calc offset robot heading vs bezier start tangent
        let theta = theta_loc - (theta0 - p.theta_ref); //add that diff to all thetas in the trajectory

        // rotation + translation (use temps!)
        let x_rot = x * cosf(dtheta) - y * sinf(dtheta); //turn whole trajectory by theta offset 
        let y_rot = x * sinf(dtheta) + y * cosf(dtheta);
        x = p.x_ref + x_rot;
        y = p.y_ref + y_rot;

        let theta = SO2::new(theta);

        let des: DiffdriveStateCascade = DiffdriveStateCascade { x, y, theta };
        DiffdriveSetpointCascade { des, vdes, wdes }
    }

    // only for simulation?
    pub fn step(&mut self, action: DiffdriveActionCascade, dt: f32) {
        let x_new = self.s.x + (0.5 * self.r) * (action.ul + action.ur) * (self.s.theta).cos() * dt;
        let y_new = self.s.y + (0.5 * self.r) * (action.ul + action.ur) * (self.s.theta).sin() * dt;
        let theta_new = self.s.theta.rad() + (self.r / self.l) * (action.ur - action.ul) * dt;
        self.s.x = x_new;
        self.s.y = y_new;
        self.s.theta = SO2::new(theta_new);
    }
}

#[derive(Copy, Clone)]
pub enum ControlMode {
    WithMocapController,
    DirectDuty,
}

#[embassy_executor::task]
pub async fn wheel_speed_inner_loop(
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32); // previous error
    let (kp, ki, kd) = (0.01, 0.02, 0.000); // Pololu gains
    // let (kp, ki, kd) = (0.01, 0.02, 0.000); // Zumo gains (needs to be tuned)

    // =========== Filter Parameters ==============
    let dt: f32 = 0.02; // 20 ms
    let fc_hz: f32 = 3.0;
    let tau: f32 = 1.0 / (2.0 * core::f32::consts::PI * fc_hz);
    let alpha: f32 = dt / (tau + dt);

    let mut prev_l = 0i32;
    let mut prev_r = 0i32;

    let mut omega_l_lp: f32 = 0.0;
    let mut omega_r_lp: f32 = 0.0;

    let mut last_cmd = WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    };

    loop {
        ticker.next().await;

        if STOP_ALL.load(Ordering::Relaxed) {
            motor.set_speed(0.0, 0.0).await;
            break;
        }

        while let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
            last_cmd = cmd; // drain the channel
        }

        // raw angular velocity of the wheel
        let ((omega_l_raw, omega_r_raw), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            ENCODER_CPR,
            prev_l,
            prev_r,
            dt,
        );
        prev_l = ln;
        prev_r = rn;

        // =========== Low Pass Filter ==============
        omega_l_lp = omega_l_lp + alpha * (omega_l_raw - omega_l_lp);
        omega_r_lp = omega_r_lp + alpha * (omega_r_raw - omega_r_lp);

        // error
        let el = last_cmd.omega_l - omega_l_lp;
        let er = last_cmd.omega_r - omega_r_lp;

        // error integration
        il = (il + ki * dt * el).clamp(-0.3, 0.3);
        ir = (ir + ki * dt * er).clamp(-0.3, 0.3);

        // error differentiation
        let dl = (el - prev_el) / dt;
        let dr = (er - prev_er) / dt;

        prev_el = el;
        prev_er = er;

        // PID (normally the D term is disabled, but just in case)
        let u_l = (kp * el + il + kd * dl).clamp(-1.0, 1.0);
        let u_r = (kp * er + ir + kd * dr).clamp(-1.0, 1.0);

        let duty_l = u_l * MOTOR_DIRECTION_LEFT;
        let duty_r = u_r * MOTOR_DIRECTION_RIGHT;

        defmt::info!(
            "meas ω L: {}, R: {}, duty L: {}, duty R: {}",
            omega_l_lp,
            omega_r_lp,
            duty_l,
            duty_r
        );

        motor.set_speed(duty_l, duty_r).await;
    }
}

// Outer Loop
#[embassy_executor::task]
pub async fn diffdrive_outer_loop(
    mode: ControlMode,
    mut sdlogger: Option<SdLogger>,
    mut led: led::Led,
) {
    STOP_ALL.store(false, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis((TRAJ_FOLLOWING_DT_S * 1000.0) as u64));

    // Initialize robot model
    defmt::info!("Initializing diffdrive robot model");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        WHEEL_RADIUS,
        WHEEL_BASE,
        WHEEL_MAX,
    );
    let mut robot = DiffdriveCascade::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX,
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );

    // Initialize controller
    let controller = DiffdriveControllerCascade::new(KX_TRAJ, KY_TRAJ, KTHETA_TRAJ);

    // Initialize logger
    if let Some(ref mut logger) = sdlogger {
        logger.write_traj_control_header();
    }
    defmt::info!("csv header written");

    /* =================== Demo Circle Trajectories Parameters ===================== */
    let circle_radius = 0.6;
    let circle_duration = 8.0;
    let wd = 2.0 * PI / circle_duration;
    /* ============================================================================= */

    /* ================ Record First Pose w.r.t the selected mode ================== */
    let mut first_pose: PoseAbs = PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    match mode {
        ControlMode::WithMocapController => {
            // initialise first pose from mocap:
            first_pose = {
                let s = LAST_STATE.lock().await;
                *s
            };
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
        let pose = {
            let s = LAST_STATE.lock().await;
            *s
        };

        // current elapsed time
        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;

        // generate next setpoint
        let setpoint = robot.circlereference(
            t_sec,
            circle_radius,
            wd,
            first_pose.x,
            first_pose.y,
            first_pose.yaw,
        );

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

                while WHEEL_CMD_CH.try_receive().is_ok() {} // drain the old commands
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });
                led.on();
            }
            ControlMode::DirectDuty => {
                info!("no mocap");
                let w_rad = setpoint.wdes;
                let vd = setpoint.vdes;

                // let state_des = DiffdriveStateCascade {
                //     x: setpoint.des.x,
                //     y: setpoint.des.y,
                //     theta: setpoint.des.theta,
                // };

                let ur = (2.0 * vd + WHEEL_BASE * w_rad) / (2.0 * WHEEL_RADIUS);
                let ul = (2.0 * vd - WHEEL_BASE * w_rad) / (2.0 * WHEEL_RADIUS);
                x_error = 0.0;
                y_error = 0.0;
                theta_error = 0.0;

                while WHEEL_CMD_CH.try_receive().is_ok() {} // drain the old commands
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });
                led.off();
            }
        }

        // log trajectory control
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

        if let Some(ref mut logger) = sdlogger {
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

#[embassy_executor::task]
pub async fn test_outer_loop(mode: ControlMode) {
    STOP_ALL.store(false, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis((TRAJ_FOLLOWING_DT_S * 1000.0) as u64));
    let mut robot = DiffdriveCascade::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX,
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );
    let controller = DiffdriveControllerCascade::new(KX_TRAJ, KY_TRAJ, KTHETA_TRAJ);

    let mut t_counter = 0.0;
    let start = Instant::now();
    let circle_radius = 0.4;
    let circle_duration = 10.0;

    let mut first_pose: PoseAbs = PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    //initialize depending on control mode:
    match mode {
        ControlMode::WithMocapController => {
            // wait for a pose to be received from mocap system
            info!("waiting for first pose from mocap system...");
            FIRST_MESSAGE.wait().await;
            // initialise first pose from mocap:
            first_pose = {
                let s = LAST_STATE.lock().await;
                *s
            };
        }
        ControlMode::DirectDuty => {
            info!("Using direct duty control, initial pose (0,0,0)");
        }
    }

    loop {
        ticker.next().await;

        // generate next setpoint
        let t = (Instant::now() - start).as_millis() as f32 / 1000.0;
        let wd = 2.0 * PI / circle_duration;
        let mut setpoint = robot.circlereference(
            t,
            circle_radius,
            wd,
            first_pose.x,
            first_pose.y,
            first_pose.yaw,
        );

        // get robot pose
        let pose = {
            let s = LAST_STATE.lock().await;
            *s
        };
        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw);

        setpoint.vdes = circle_radius * wd;
        setpoint.wdes = -wd;
        match mode {
            ControlMode::WithMocapController => {
                info!("mocap");
                let (action, _xerror, _yerror, _yawerror) = controller.control(&robot, setpoint);
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: action.ul,
                    omega_r: action.ur,
                    stamp: Instant::now(),
                });
            }
            ControlMode::DirectDuty => {
                info!("no mocap");
                let w_rad = setpoint.wdes;
                let w_deg = w_rad * 180.0 / PI; // deg/s

                let vd = setpoint.vdes;

                let state_des = DiffdriveStateCascade {
                    x: setpoint.des.x,
                    y: setpoint.des.y,
                    theta: setpoint.des.theta,
                };
                let ur = (2.0 * vd + robot.l * w_rad) / (2.0 * robot.r);
                let ul = (2.0 * vd - robot.l * w_rad) / (2.0 * robot.r);
                // info!("robot l: {}, robot_radius: {}", robot.l, robot.r);

                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });

                defmt::info!(
                    "t={}s, posd = ({},{},{}), v={}, w={} rad/s ({} deg/s), u_ff=({},{})",
                    t,
                    state_des.x,
                    state_des.y,
                    state_des.theta.rad(),
                    vd,
                    w_rad,
                    w_deg,
                    ul,
                    ur
                );
            }
        }

        if t_counter > circle_duration / TRAJ_FOLLOWING_DT_S {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            STOP_ALL.store(true, Ordering::Relaxed);
            break;
        }
        t_counter += 1.0;
    }
}

/// Mocap Update Signal
#[embassy_executor::task]
pub async fn mocap_update_task() {
    loop {
        let new_pose = STATE_SIG.wait().await;
        let mut s = LAST_STATE.lock().await;
        *s = new_pose;
    }
}
