#![allow(clippy::never_loop)]

use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, STOP_MOCAP_UPDATE_SIG};
use core::cell::RefCell;
use core::f32::consts::PI;
use defmt::{info};
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_futures::select::{Either, select, select3, Either3};
use embassy_sync::mutex::Mutex;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
// use embassy_time::Timer;
use embassy_time::{Duration, Instant, Ticker};
use libm::{atan2f, cosf, sinf, sqrtf};
use micro_qp::{
    admm::{AdmmSettings, AdmmSolver},
    types::{MatMN, VecN},
};
use static_cell::StaticCell;

use crate::encoder::wheel_speed_from_counts_now;
use crate::motor::MotorController;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robot_parameters_default::robot_constants::*;
use crate::led::led_set;
use crate::sdlog::{log_traj_control, with_sdlogger};
use crate::trajectory_reading::{Action, Pose as TrajPose, Trajectory};
use crate::trajectory_signal::{
    STATE_SIG, TRAJECTORY_CONTROL_EVENT,
};
use crate::robotstate::{
    Setpoint, TrackingError, WHEEL_CMD_CH, WheelCmd, read_pose, read_setpoint, read_waypoint, stop_motors, write_pose, write_tracking_error, write_wheel_cmd
};
use crate::parameter_sync::send_running;

use portable_atomic::{AtomicBool, Ordering};

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

// =============================== Save Trajectory ================================
static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
    Mutex::new(RefCell::new(None));
pub static TRAJ_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();

/// Static storage for trajectory loaded from SD card
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


/// Create robot model and controller from config
pub fn create_robot_and_controller(cfg: &RobotConfig) -> (DiffdriveCascade, DiffdriveControllerCascade) {
    let robot = DiffdriveCascade::new(
        cfg.wheel_radius,
        cfg.wheel_base,
        -cfg.wheel_max,
        cfg.wheel_max,
        -cfg.wheel_max,
        cfg.wheel_max,
    );
    let controller = DiffdriveControllerCascade::new(
        cfg.kx_traj,
        cfg.ky_traj,
        cfg.ktheta_traj,
    );
    (robot, controller)
}

/// Log robot initialization info
pub fn log_robot_init(cfg: &RobotConfig) {
    defmt::info!("Initializing diffdrive robot model");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        cfg.wheel_radius,
        cfg.wheel_base,
        cfg.wheel_max,
    );
}

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
    pub qp_solver: Option<AdmmSolver<2, 2>>,
}

/* ====================================================================================================== */
/// The trajectory following is done by a cascade controller, where the outer loop minimizes 
/// the position error and generates the desired linear and angular velocities and the inner loop 
/// converts the linear and angular velocites into wheel speed and minimizes the wheel speed errors.
/* ====================================================================================================== */

/// Outer loop position controller
impl DiffdriveControllerCascade {
    pub fn new(kx: f32, ky: f32, kth: f32) -> Self {
        // prepare solver
        let (h, a) = Self::build_h_a(WHEEL_RADIUS, WHEEL_BASE, 1.0);
        let mut solver = AdmmSolver::<2, 2>::new();
        solver.settings = AdmmSettings {
            rho: 0.01,
            eps_pri: 1e-7,
            eps_dual: 1e-7,
            max_iter: 300,
            sigma: 1e-9,
            mu: 10.0,
            tau_inc: 2.0,
            tau_dec: 2.0,
            rho_min: 1e-6,
            rho_max: 1e6,
            adapt_interval: 25,
        };
        assert!(solver.prepare(&h, &a));

        Self {
            kx,
            ky,
            kth,
            qp_solver: Some(solver),
        }
    }

    #[inline]
    fn build_h_a(r: f32, l: f32, lambda: f32) -> (MatMN<2, 2>, MatMN<2, 2>) {
        // a = [ r/L, -r/L ], b = [ r/2, r/2 ]
        let a0 = r / l;
        let a1 = -r / l;
        let b0 = r * 0.5;
        let b1 = r * 0.5;

        // H = 2(aa^T + lambd*bb^T)
        let mut h = MatMN::<2, 2>::zero();
        let aa = [[a0 * a0, a0 * a1], [a1 * a0, a1 * a1]];
        let bb = [[b0 * b0, b0 * b1], [b1 * b0, b1 * b1]];
        for i in 0..2 {
            for j in 0..2 {
                h.set(i, j, 2.0 * (aa[i][j] + lambda * bb[i][j]));
            }
        }

        // A = I (box constraints)
        let mut a = MatMN::<2, 2>::zero();
        a.set(0, 0, 1.0);
        a.set(1, 1, 1.0);

        (h, a)
    }

    pub fn control(
        &mut self,
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
        let ur = (2.0 * v + 1.0 * robot.l * w) / (2.0 * robot.r);
        let ul = (2.0 * v - 1.0 * robot.l * w) / (2.0 * robot.r);

        // Return the action and the errors as a tuple
        (DiffdriveActionCascade { ul, ur }, xerror, yerror, therror)
    }

    pub fn control_with_qp(
        &mut self,
        robot: &DiffdriveCascade,
        setpoint: DiffdriveSetpointCascade,
        lambda_eff: f32,
    ) -> (DiffdriveActionCascade, f32, f32, f32) {
        let xerror: f32 = robot.s.theta.cos() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.sin() * (setpoint.des.y - robot.s.y);
        let yerror: f32 = -robot.s.theta.sin() * (setpoint.des.x - robot.s.x)
            + robot.s.theta.cos() * (setpoint.des.y - robot.s.y);
        let therror: f32 = SO2::error(setpoint.des.theta, robot.s.theta);

        let v: f32 = setpoint.vdes * cosf(therror) + self.kx * xerror;
        let w: f32 = setpoint.wdes + setpoint.vdes * (self.ky * yerror + sinf(therror) * self.kth);

        let solver = self.qp_solver.as_mut().unwrap();
        let a: [_; 2] = [robot.r / robot.l, -robot.r / robot.l];
        let b = [robot.r * 0.5, robot.r * 0.5];

        let mut f = VecN::<2>::zero();
        for i in 0..2 {
            f.data[i] = -2.0 * (w * a[i] + lambda_eff * v * b[i]);
        }

        let mut l = VecN::<2>::zero();
        let mut u = VecN::<2>::zero();
        l.data = [robot.ur_min, robot.ul_min];
        u.data = [robot.ur_max, robot.ul_max];

        let (x, _iters) = solver.solve(&f, &l, &u);
        let ur = x.data[0];
        let ul = x.data[1];

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

    pub fn spinning(
        &self,
        duration: f32,
        t: f32,
        x0: f32,
        y0: f32,
        theta0: f32,
    ) -> DiffdriveSetpointCascade {
        //rotate 4 times about itself within the given duration
        let wd = 4.0 * 2.0 * PI / duration;

        let x = x0;
        let y = y0;
        let theta = SO2::new(theta0 + wd * t);

        let des: DiffdriveStateCascade = DiffdriveStateCascade { x, y, theta };
        DiffdriveSetpointCascade {
            des,
            vdes: 0.0,
            wdes: wd,
        }
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
        let x_local = r * cosf(t * wd);
        let y_local = r * sinf(t * wd);
        //note: multiply with inner derivative
        let xd = -r * sinf(t * wd) * wd;
        let yd = r * cosf(t * wd) * wd;
        let xdd = -r * cosf(t * wd) * wd * wd;
        let ydd = -r * sinf(t * wd) * wd * wd;

        let vdes: f32 = sqrtf(xd * xd + yd * yd); //r * wd;
        let wdes: f32 = (ydd * xd - xdd * yd) / (xd * xd + yd * yd); //wd
        let theta_loc = atan2f(yd, xd);

        //transform from local circle coordinates to world coordinates
        //rotate by tangent of initial heading and translate to starting position
        let c = cosf(theta0 - PI / 2.0);
        let s = sinf(theta0 - PI / 2.0);

        let x = x0 + (x_local - r) * c - y_local * s;
        let y = y0 + (x_local - r) * s + y_local * c;

        //orientation follows the circle tangent
        let theta = SO2::new(theta0 + theta_loc);

        let des: DiffdriveStateCascade = DiffdriveStateCascade { x, y, theta };
        DiffdriveSetpointCascade { des, vdes, wdes }
    }

    pub fn circle_reference_t(
        &self,
        r: f32,               // radius [m]
        circle_duration: f32, // circle duration
        t_s: f32,             // current time [s]
        x0: f32,              // center x
        y0: f32,              // center y
        phi0: SO2,            // intial phase
    ) -> DiffdriveSetpointCascade {
        let wd = 2.0 * PI / circle_duration;
        let vd = wd * r;
        let state_des = DiffdriveStateCascade {
            x: x0 + r * (sinf(phi0.rad() + wd * t_s) - sinf(phi0.rad())),
            y: y0 + r * (-cosf(phi0.rad() + wd * t_s) + cosf(phi0.rad())),
            theta: SO2::new(phi0.rad() + wd * t_s),
        };

        DiffdriveSetpointCascade {
            des: state_des,
            vdes: vd,
            wdes: wd,
        }
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

#[derive(Debug, Copy, Clone)]
pub enum ControlMode {
    WithMocapController,
    DirectDuty,
}

#[derive(Debug, Copy, Clone)]
pub enum TrajectoryResult {
    Completed,
    Stopped,
}

/* =============================== Main Tasks ========================================== */
// Inner Loop
#[embassy_executor::task]
pub async fn wheel_speed_inner_loop(
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cfg: Option<RobotConfig>,
    
) {
    let robot_cfg = cfg.unwrap_or_default();

    let mut ticker = Ticker::every(Duration::from_millis(20));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32);
    let (kp, ki, kd) = (robot_cfg.kp_inner, robot_cfg.ki_inner, robot_cfg.kd_inner);

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
        match select3(ticker.next(), STOP_WHEEL_INNER_SIG.wait(), TRAJ_PAUSE_SIG.wait()).await {
            Either3::First(_) => { 
                // Normal loop 
            }
            Either3::Second(_) => {
                motor.set_speed(0.0, 0.0).await;
                defmt::warn!("STOP inner loop -> exit");
                return;    // Stop inner loop, stop trajectory following task, back to menu
            }
            Either3::Third(_) => {
                motor.set_speed(0.0, 0.0).await;
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), STOP_WHEEL_INNER_SIG.wait()).await {
                        Either::First(_) => break,        // back to normal loop
                        Either::Second(_) => {
                            motor.set_speed(0.0, 0.0).await;
                            return;
                        }
                    }
                }
                continue;
            }
        }

        while let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
            last_cmd = cmd; // drain the channel
        }

        // raw angular velocity of the wheel
        let ((omega_l_raw, omega_r_raw), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            robot_cfg.encoder_cpr,
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
        il = (il + ki * dt * el).clamp(-2.0, 2.0);
        ir = (ir + ki * dt * er).clamp(-2.0, 2.0);

        // error differentiation
        let dl = (el - prev_el) / dt;
        let dr = (er - prev_er) / dt;

        prev_el = el;
        prev_er = er;

        // PID (normally the D term is disabled, but just in case)
        let u_l = (kp * el + il + kd * dl).clamp(-1.0, 1.0);
        let u_r = (kp * er + ir + kd * dr).clamp(-1.0, 1.0);

        let duty_l = u_l * robot_cfg.motor_direction_left;
        let duty_r = u_r * robot_cfg.motor_direction_right;

        // defmt::info!(
        //     "{}, {}, meas ω L: {}, R: {}, duty L: {}, duty R: {}",
        //     last_cmd.omega_l,
        //     last_cmd.omega_r,
        //     omega_l_lp,
        //     omega_r_lp,
        //     duty_l,
        //     duty_r
        // );

        motor.set_speed(duty_l, duty_r).await;
    }
}

// Command-controlled trajectory following for gain tunning
async fn execute_trajectory_loop_with_control_from_sdcard(
    mode: ControlMode,
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
            embassy_futures::select::select(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait()).await;

        match either_result {
            embassy_futures::select::Either::First(_) => {
                // timer tick: normal loop execution
            }
            embassy_futures::select::Either::Second(command) => {
                // command received during trajectory execution
                if !command {
                    // Stop command - immediately stop motors
                    stop_motors();
                    defmt::info!("Stopping trajectory by command");
                    return TrajectoryResult::Stopped;
                }
            }
        }

        // in case a stop was requested
        if STOP_ALL.load(Ordering::Relaxed) {
            stop_motors();
            defmt::info!("Trajectory stopped via STOP_ALL");
            break;
        }
        /* ========================================================================= */

        /* ========================= Get robot pose ================================ */
        let pose = read_pose().await;
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
        
        
        
    
        let TrajPose {
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

                // Write tracking errors to mutex for logging
                write_tracking_error(TrackingError {
                    x_err: x_error,
                    y_err: y_error,
                    yaw_err: theta_error,
                }).await;

                write_wheel_cmd(WheelCmd::new(ul, ur)).await;
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

                // Write zero errors for DirectDuty mode
                write_tracking_error(TrackingError::DEFAULT).await;

                write_wheel_cmd(WheelCmd::new(ul, ur)).await;
            }
        }

        // Log trajectory control
        log_traj_control(t_ms, &setpoint, robot, x_error, y_error, theta_error, ul, ur).await;

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
            stop_motors();
            defmt::info!("Trajectory complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    //ensure motors are stopped
    stop_motors();

    TrajectoryResult::Stopped
}

// Command-controlled trajectory following from SDCard
#[embassy_executor::task]
pub async fn diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
    mode: ControlMode,
    // mut sdlogger: Option<SdLogger>,
    // led_device: Option<led::Led>,
    cfg: Option<RobotConfig>,
) {
    // ============ Robot Configuration ==========
    let robot_cfg = cfg.unwrap_or_default();
    defmt::info!("Robot config loaded (from SD card if available)");

    // ============ Initialize robot model =========
    log_robot_init(&robot_cfg);
    let (mut robot, mut controller) = create_robot_and_controller(&robot_cfg);

    // Initialize logger and write header
    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;

    defmt::info!("Waiting for trajectory control commands...");
    defmt::info!("Commands: 't' = start, 's' = stop");

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
                send_running(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop -> exit");
                led_set(false).await;
                send_running(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle) -> wait RESUME/STOP");
                led_set(false).await;
                send_running(false).await;
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
            mode,
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();

        match select3(exec_fut, stop_fut, pause_fut).await {
            Either3::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Trajectory completed.");
                    },
                    TrajectoryResult::Stopped   => {
                        defmt::info!("Trajectory stopped internally.");
                    }
                }
                led_set(false).await;
                send_running(false).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                send_running(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::info!("PAUSE outer during exec -> wait RESUME/STOP");
                led_set(false).await;
                send_running(false).await;
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                        Either::First(_) => {
                            defmt::info!("RESUME outer -> restart exec loop");
                            led_set(true).await;
                            send_running(true).await;
                            break;
                        }
                        Either::Second(_) => {
                            defmt::warn!("STOP outer during pause -> exit");
                            return;
                        }
                    }
                }
                continue;
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
                // Update robotstate::POSE mutex
                write_pose(new_pose).await;
            }

            Either::Second(_) => {
                defmt::info!("mocap_update_task stopped by STOP_MOCAP_UPDATE_SIG");
                return;
            }
        }
    }
}
/* =========================TODO / COMMMENTS==========================================
    returntype??

    where do we get new poses -> mocap -> but is it actually read anywhere?

    do we need to run the controller in its own little loop -> I think yes otherwise it will not correct /CONTROLL

 ===================================================================================== 

*/

/// utility that receives one setpoint at a time, drives there, and waits for new setpoint
pub async fn execute_tracjectory_loop_single_waypoint(
    cfg: Option<RobotConfig>,
)->TrajectoryResult{
    //set up consts
    let v_set:f32 = 0.01;
    let w_set:f32 = 0.001;

    // ============ Robot Configuration ==========
    let robot_cfg = cfg.unwrap_or_default();
    defmt::info!("Robot config loaded (from SD card if available)");

    // ============ Initialize robot model =========
    log_robot_init(&robot_cfg);
    let (mut robot, mut controller) = create_robot_and_controller(&robot_cfg);

    /* ============================== Setup Ticker ================================= */
    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));
    /* ============================================================================= */
    
    //outer loop -> looking for new setpoints
    loop{
        /* ======== wait for either the timer tick or a trajectory command ========= */
        let either_result =
            embassy_futures::select::select(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait()).await;

        match either_result {
            embassy_futures::select::Either::First(_) => {
                // timer tick: normal loop execution

                //evtl hier die neue pose abfangen
            }
            embassy_futures::select::Either::Second(command) => {
                // command received during trajectory execution
                if !command {
                    // Stop command - immediately stop motors
                    stop_motors();
                    defmt::info!("Stopping trajectory by command");
                    break;
                }
            }
        }

        // in case a stop was requested
        if STOP_ALL.load(Ordering::Relaxed) {
            stop_motors();
            defmt::info!("Trajectory stopped via STOP_ALL");
            break;
        }
        /* ========================================================================= */
        //some logic to fetch new setpoints
        //some logic to compare new setpoints vs current state
        //if new setpoint != current state -> start driving
        
        //-> wait for new pose channel 
        //so wie beim timer -> in den timer step
        //compare logic


        //from christoph
        let waypoint = read_waypoint().await; //if new setpoint, update here, else it stays the same 
        //===========================actural controll loop================================
        // update 
        let pose = read_pose().await;
        let setpoint:Setpoint = Setpoint {
             x_des: (waypoint.x_des), y_des: (waypoint.y_des), yaw_des: (waypoint.yaw_des), v_ff: (v_set), w_ff: (w_set) 
            };
            //write setpoint from waypoint, add default v
        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw);

        // check if pose is reached +- small difference x,y, z, theta
        // let dx = robot.s.x - setpoint.des.x;
        // let dy = robot.s.y - setpoint.des.y;
        // let dist = sqrtf(dy * dy + dx * dx);
        // if dist < goal_dist {
        //     defmt::info!("Desired point reached, stopped");
        //     stop_motors();
        //     break;
        // }

        let (ul, ur, x_error, y_error, theta_error);

        //control Mode matching skipped, should only work in MOCAP
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
        }).await;

        write_wheel_cmd(WheelCmd::new(ul, ur)).await;
    }
        
     return TrajectoryResult::Stopped;
}


/* ===================================================================================== */