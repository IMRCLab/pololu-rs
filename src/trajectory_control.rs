use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, STOP_MOCAP_UPDATE_SIG};
use crate::packet::StateLoopBackPacketF32;
use crate::uart::update_robot_state;
use core::cell::RefCell;
use core::f32::consts::PI;
use defmt::{info};
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_futures::select::{Either, select, select3, Either3};
use embassy_sync::mutex::Mutex;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Timer;
use embassy_time::{Duration, Instant, Ticker};
use libm::{atan2f, cosf, sinf, sqrtf};
use micro_qp::{
    admm::{AdmmSettings, AdmmSolver},
    types::{MatMN, VecN},
};
use static_cell::StaticCell;

use crate::encoder::wheel_speed_from_counts_now;
use crate::led::{self};
use crate::motor::MotorController;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robot_parameters_default::robot_constants::*;
use crate::led::LED_SHARED;
use crate::sdlog::{SdLogger, TrajControlLog, SDLOGGER_SHARED};
use crate::trajectory_reading::{Action, Pose, Trajectory};
use crate::trajectory_signal::{
    LAST_STATE, POSE_FRESH, PoseAbs, STATE_SIG, TRAJECTORY_CONTROL_EVENT, WHEEL_CMD_CH, WheelCmd,
};

use crate::odometry::{ODOM_STATE, OdomPose};

use portable_atomic::{AtomicBool, Ordering};

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

// =============================== Save Trajectory ================================
static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
    Mutex::new(RefCell::new(None));
pub static TRAJ_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();
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

pub async fn led_set(on: bool) {
    let mut g = LED_SHARED.lock().await;
    if let Some(led) = g.as_mut() {
        if on { led.on(); } else { led.off(); }
    } else {
        defmt::warn!("LED not available; skip.");
    }
}

pub async fn with_sdlogger<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut SdLogger) -> R
{
    let mut g = SDLOGGER_SHARED.lock().await;
    if let Some(l) = g.as_mut() {
        Some(f(l))
    } else {
        defmt::warn!("SdLogger not available; skip.");
        None
    }
}


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
    pub qp_solver: Option<AdmmSolver<2, 2>>,
}

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

    pub fn spinning_at_wd(&self, wd: f32, t: f32, x0: f32, y0: f32, theta0: f32) -> DiffdriveSetpointCascade {
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

    //https://en.wikipedia.org/wiki/Lemniscate_of_Gerono
    //parametric curve:
    // x = cos(phi), y = sin(phi)*cos(phi) = 0.5*sin(2*phi) 
    pub fn figure8_reference(
        &self,
        t: f32,
        duration: f32,
        ax: f32,
        ay: f32,
        x0: f32,
        y0: f32,
        phi0: f32,
    ) -> DiffdriveSetpointCascade {
        let w = 2.0 * PI / duration;

        //flats
        let x_loc = ax * sinf(w * t);
        let y_loc = ay * sinf(2.0 * w * t) / 2.0;

        //derivatives
        let xd = ax * w * cosf(w * t);
        let yd = ay * w * cosf(2.0 * w * t);

        //second derivatives
        let xdd = -ax * w * w * sinf(w * t);
        let ydd = -2.0 * ay * w * w * sinf(2.0 * w * t);

        //get actions from derivatives
        let vdes = sqrtf(xd * xd + yd * yd);
        let wdes = (ydd * xd - xdd * yd) / (xd * xd + yd * yd);
        let theta_loc = atan2f(yd, xd);

        //transform into world frame (in case of tracking)
        let c = cosf(phi0);
        let s = sinf(phi0);
        let x = x0 + x_loc * c - y_loc * s;
        let y = y0 + x_loc * s + y_loc * c;
        let theta = SO2::new(theta_loc + phi0);

        let des = DiffdriveStateCascade { x, y, theta };
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
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        robot_cfg = cfg.unwrap();
    } else {
        robot_cfg = RobotConfig::default();
    }

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32);
    let (kp, ki, kd) = (robot_cfg.kp_inner, robot_cfg.ki_inner, robot_cfg.kd_inner);

    // =========== Filter Parameters ==============
    let dt: f32 = 0.01; // 10 ms
    let fc_hz: f32 = 3.0;
    let tau: f32 = 1.0 / (2.0 * core::f32::consts::PI * fc_hz);
    let alpha: f32 = dt / (tau + dt);

    //need to get current state of the encoders, because it is potentially leading to v spikes when switching programs.
    let mut prev_l = *left_counter.lock().await;
    let mut prev_r = *right_counter.lock().await;

    let mut omega_l_lp: f32 = 0.0;
    let mut omega_r_lp: f32 = 0.0;

    let mut last_cmd = WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    };

    loop {
        match select3(STOP_WHEEL_INNER_SIG.wait(), TRAJ_PAUSE_SIG.wait(), ticker.next()).await {
            Either3::First(_) => {
                motor.set_speed(0.0, 0.0).await;
                defmt::warn!("STOP inner loop -> exit");
                return;
            }
            Either3::Second(_) => {
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
                //reset the controller errors before resuming to avoid v spikes.
                il = 0.0; ir = 0.0;
                prev_el = 0.0; prev_er = 0.0;
                omega_l_lp = 0.0; omega_r_lp = 0.0;
                prev_l = *left_counter.lock().await;
                prev_r = *right_counter.lock().await;
                last_cmd = WheelCmd { omega_l: 0.0, omega_r: 0.0, stamp: Instant::now() };
                continue;
            }
            Either3::Third(_) => {
                // Normal loop - ticker fired
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

    /* =============== Fusion state: EKF ================= */
    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    // Seed fused pose from mocap if available, otherwise from trajectory start.
    let init_fresh = POSE_FRESH.load(Ordering::Acquire);
    let mut ekf = if init_fresh {
        let mocap = *LAST_STATE.lock().await;
        fused_x = mocap.x;
        fused_y = mocap.y;
        fused_yaw = mocap.yaw;
        info!("SD traj: initial pose from mocap ({},{},{})", fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        // No mocap — use the first state from the trajectory file as initial pose.
        fused_x = states[0].x;
        fused_y = states[0].y;
        fused_yaw = states[0].yaw;
        info!("SD traj: initial pose from trajectory start ({},{},{}) (no mocap)", fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    };
    /* ============================================================================= */

    /* ======================== Get precise start time ============================= */
    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);
    /* ============================================================================= */

    loop {
        /* ======== wait for either the timer tick, a trajectory command, or pause ========= */
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {
                // timer tick: normal loop execution
            }
            Either3::Second(command) => {
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
            Either3::Third(_) => {
                // Pause: zero wheels and wait for resume or stop
                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                });
                let pause_start = Instant::now();
                defmt::info!("Execute loop paused");
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                        Either::First(_) => {
                            pause_offset += Instant::now() - pause_start;
                            defmt::info!("Execute loop resumed");
                            break;
                        }
                        Either::Second(_) => {
                            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                            });
                            return TrajectoryResult::Stopped;
                        }
                    }
                }
                continue;
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

        /* ========================= Pose Fusion ===================================== */
        let odom_now = *ODOM_STATE.lock().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = POSE_FRESH.swap(false, Ordering::Acquire);
        if pose_is_fresh {
            let mocap_pose = *LAST_STATE.lock().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* ========================================================================= */

        /* ====================== Current elapsed time ============================= */
        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;
        /* ========================================================================= */
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
        /* ========================================================================= */

        /* ==================== Control ============================================ */
        let (ul, ur, x_error, y_error, theta_error);

        match mode {
            ControlMode::WithMocapController => {
                robot.s.x = fused_x;
                robot.s.y = fused_y;
                robot.s.theta = SO2::new(fused_yaw);

                let (action, x_e, y_e, yaw_e) = controller.control(&robot, setpoint);
                ul = action.ul;
                ur = action.ur;
                x_error = x_e;
                y_error = y_e;
                theta_error = yaw_e;
            }
            ControlMode::DirectDuty => {
                let w_rad = setpoint.wdes;
                let vd = setpoint.vdes;

                ur = (2.0 * vd + robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                ul = (2.0 * vd - robot_cfg.k_clip * robot_cfg.wheel_base * w_rad)
                    / (2.0 * robot_cfg.wheel_radius);
                x_error = setpoint.des.x - fused_x;
                y_error = setpoint.des.y - fused_y;
                let mut dtheta = setpoint.des.theta.rad() - fused_yaw;
                dtheta = libm::atan2f(libm::sinf(dtheta), libm::cosf(dtheta));
                theta_error = dtheta;
            }
        }
        /* ========================================================================= */

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        update_robot_state(StateLoopBackPacketF32 {
            header: 0xA1,
            robot_id: 1,
            pos_x: 1.0,
            pos_y: 2.0,
            pos_z: 3.0,
            vel_x: 4.0,
            vel_y: 5.0,
            vel_z: 6.0,
            qw: 1.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
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
            actual_vx: odom_now.v,
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

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
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

// Command-controlled trajectory following from SDCard
#[embassy_executor::task]
pub async fn diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
    _mode: ControlMode,
    // mut sdlogger: Option<SdLogger>,
    // led_device: Option<led::Led>,
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
    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;
    // if let Some(ref mut logger) = sdlogger {
    //     logger.write_traj_control_header();
    // }

    defmt::info!("Waiting for trajectory control commands...");
    defmt::info!("Commands: 't' = start, 's' = stop");

    // Drain any stale start/pause signals that may have been set
    // by a previous UART task racing with the orchestrator shutdown.
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

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
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle) -> wait RESUME/STOP");
                led_set(false).await;
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
            _mode,
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Trajectory completed.");
                    },
                    TrajectoryResult::Stopped   => {
                        defmt::info!("Trajectory stopped internally.");
                    }
                }
                led_set(false).await;
                // Clear stale resume/pause signals so the outer loop
                // doesn't immediately re-enter execution on the next iteration.
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
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
                let mut s = LAST_STATE.lock().await;
                *s = PoseAbs { stamp: Instant::now(), ..new_pose };
                crate::trajectory_signal::POSE_FRESH.store(true, Ordering::Release);
            }

            Either::Second(_) => {
                defmt::info!("mocap_update_task stopped by STOP_MOCAP_UPDATE_SIG");
                return;
            }
        }
    }
}

/* ===================================================================================== */

/* ========================== Onboard Trajectory Mode ================================= */

async fn execute_trajectory_loop_onboard(
    _mode: ControlMode,
    robot: &mut DiffdriveCascade,
    controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    let duration: f32 = 3.0;
    let ax: f32 = 0.3;
    let ay: f32 = 0.3;

    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    // --- Unified pose fusion state ---
    // "Anchor" = the odometry snapshot at the moment we last received a fresh mocap pose.
    // Between mocap updates we compute: pose = last_mocap + (odom_now - odom_anchor).
    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    // Wait a moment for first mocap/odom data to arrive
    Timer::after_millis(100).await;

    // Try to get initial mocap pose; fall back to pure odometry if unavailable.
    let pose_is_fresh = POSE_FRESH.load(Ordering::Acquire);
    let mut ekf = if pose_is_fresh {
        let mocap_pose = *LAST_STATE.lock().await;
        fused_x = mocap_pose.x;
        fused_y = mocap_pose.y;
        fused_yaw = mocap_pose.yaw;
        info!("Onboard traj: initial pose from mocap ({},{},{})",
            fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        fused_x = 0.0;
        fused_y = 0.0;
        fused_yaw = 0.0;
        info!("Onboard traj: initial pose set to origin (no mocap)");
        crate::ekf::Ekf::default_at_origin()
    };

    let first_pose_x = fused_x;
    let first_pose_y = fused_y;
    let first_pose_yaw = fused_yaw;

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {} // normal tick
            Either3::Second(command) => {
                if !command {
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0,
                        omega_r: 0.0,
                        stamp: Instant::now(),
                    });
                    defmt::info!("Onboard traj stopped by command");
                    return TrajectoryResult::Stopped;
                }
            }
            Either3::Third(_) => {
                // Pause: zero wheels and wait for resume or stop
                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                });
                let pause_start = Instant::now();
                defmt::info!("Execute loop paused");
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                        Either::First(_) => {
                            pause_offset += Instant::now() - pause_start;
                            defmt::info!("Execute loop resumed");
                            break;
                        }
                        Either::Second(_) => {
                            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                            });
                            return TrajectoryResult::Stopped;
                        }
                    }
                }
                continue;
            }
        }

        if STOP_ALL.load(Ordering::Relaxed) {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Onboard traj stopped via STOP_ALL");
            break;
        }

        /* =================== Pose Fusion =================== */
        let odom_now = *ODOM_STATE.lock().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = POSE_FRESH.swap(false, Ordering::Acquire);
        if pose_is_fresh {
            let mocap_pose = *LAST_STATE.lock().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* =================================================== */

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;
        let setpoint = robot.figure8_reference(
            t_sec,
            duration,
            ax,
            ay,
            first_pose_x,
            first_pose_y,
            first_pose_yaw,
        );

        // Update robot state from fused pose
        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        // Compute wheel commands: always use controller (closed-loop), regardless of mocap
        let (action, x_error, y_error, theta_error) = controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        let log = TrajControlLog {
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
            actual_vx: odom_now.v,
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

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
        );

        if t_sec >= duration {
            //stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Onboard trajectory complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    });

    TrajectoryResult::Stopped
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj(
    _mode: ControlMode,
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();

    defmt::info!("Initializing onboard trajectory mode (figure-8)");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius, robot_cfg.wheel_base, robot_cfg.wheel_max,
    );

    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );

    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj,
        robot_cfg.ky_traj,
        robot_cfg.ktheta_traj,
    );

    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;

    defmt::info!("Waiting for trajectory control commands...");

    // Drain any stale start/pause signals that may have been set
    // by a previous UART task racing with the orchestrator shutdown.
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

    loop {
        let start_fut = TRAJ_RESUME_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select3(start_fut, stop_fut, pause_fut).await {
            Either3::First(start) => {
                if !start {
                    continue;
                }
                defmt::info!("Starting onboard trajectory!");
                led_set(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle) -> wait RESUME/STOP");
                led_set(false).await;
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                        Either::First(_) => {
                            defmt::info!("RESUME(idle)");
                            break;
                        }
                        Either::Second(_) => {
                            defmt::warn!("STOP outer loop (idle) -> exit");
                            return;
                        }
                    }
                }
                continue;
            }
        }

        let exec_fut = execute_trajectory_loop_onboard(
            _mode,
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Onboard trajectory completed.");
                    }
                    TrajectoryResult::Stopped => {
                        defmt::info!("Onboard trajectory stopped internally.");
                    }
                }
                led_set(false).await;
                // Clear stale resume/pause signals so the outer loop
                // doesn't immediately re-enter execution on the next iteration.
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec -> exit");
                led_set(false).await;
                return;
            }
        }
        defmt::info!("Waiting for next command...");
    }
}

/* ===================================================================================== */

/* ======================== Onboard Trajectory 2 (Demo) ================================ */

async fn execute_trajectory_loop_onboard2(
    _mode: ControlMode,
    robot: &mut DiffdriveCascade,
    _controller: &mut DiffdriveControllerCascade,
    robot_cfg: &RobotConfig,
) -> TrajectoryResult {
    // ---- Spin-in-place for 8 seconds ----
    let duration: f32 = 3.0;
    // Body angular velocity for spin-in-place.
    // Max body w = 2 * r * w_wheel / L
    let max_spin_fraction: f32 = 0.8;
    let wd_spin = 2.0 * robot_cfg.wheel_radius * robot_cfg.wheel_max / robot_cfg.wheel_base * max_spin_fraction;


    let mut ticker = Ticker::every(Duration::from_millis(
        (robot_cfg.traj_following_dt_s * 1000.0) as u64,
    ));

    let mut fused_x: f32;
    let mut fused_y: f32;
    let mut fused_yaw: f32;

    Timer::after_millis(100).await;

    let pose_is_fresh = POSE_FRESH.load(Ordering::Acquire);
    let mut ekf = if pose_is_fresh {
        let mocap_pose = *LAST_STATE.lock().await;
        fused_x = mocap_pose.x;
        fused_y = mocap_pose.y;
        fused_yaw = mocap_pose.yaw;
        info!("Onboard traj2: initial pose from mocap ({},{},{})",
            fused_x, fused_y, fused_yaw);
        crate::ekf::Ekf::default_at(fused_x, fused_y, fused_yaw)
    } else {
        fused_x = 0.0;
        fused_y = 0.0;
        fused_yaw = 0.0;
        info!("Onboard traj2: initial pose set to origin (no mocap)");
        crate::ekf::Ekf::default_at_origin()
    };

    let first_pose_x = fused_x;
    let first_pose_y = fused_y;
    let first_pose_yaw = fused_yaw;

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        let tick_result =
            select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await;

        match tick_result {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command {
                    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                        omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                    });
                    defmt::info!("Onboard traj2 stopped by command");
                    return TrajectoryResult::Stopped;
                }
            }
            Either3::Third(_) => {
                while WHEEL_CMD_CH.try_receive().is_ok() {}
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                });
                let pause_start = Instant::now();
                defmt::info!("Execute loop paused (traj2)");
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                        Either::First(_) => {
                            pause_offset += Instant::now() - pause_start;
                            defmt::info!("Execute loop resumed (traj2)");
                            break;
                        }
                        Either::Second(_) => {
                            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
                            });
                            return TrajectoryResult::Stopped;
                        }
                    }
                }
                continue;
            }
        }

        if STOP_ALL.load(Ordering::Relaxed) {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
            });
            defmt::info!("Onboard traj2 stopped via STOP_ALL");
            break;
        }

        /* =================== Pose Fusion =================== */
        let odom_now = *ODOM_STATE.lock().await;

        ekf.predict(odom_now.v, odom_now.w, robot_cfg.traj_following_dt_s);

        let pose_is_fresh = POSE_FRESH.swap(false, Ordering::Acquire);
        if pose_is_fresh {
            let mocap_pose = *LAST_STATE.lock().await;
            ekf.update(&crate::math::Vec3::new(mocap_pose.x, mocap_pose.y, mocap_pose.yaw));
        }

        let (fx, fy, fth) = ekf.state();
        fused_x = fx;
        fused_y = fy;
        fused_yaw = fth;
        /* =================================================== */

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let t_ms = t.as_millis() as u32;

        // ---- Spin in place via spinning reference + controller ----
        let setpoint = robot.spinning_at_wd(
            wd_spin,
            t_sec,
            first_pose_x,
            first_pose_y,
            first_pose_yaw,
        );

        // Update robot state from fused pose
        robot.s.x = fused_x;
        robot.s.y = fused_y;
        robot.s.theta = SO2::new(fused_yaw);

        // Closed-loop control (same as figure-8 onboard traj)
        let (action, x_error, y_error, theta_error) = _controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {}
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul, omega_r: ur, stamp: Instant::now(),
        });

        let log = TrajControlLog {
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
            actual_vx: odom_now.v,
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

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t2={}s, fused=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{}), fresh={}",
            t_sec,
            fused_x, fused_y, fused_yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
            pose_is_fresh
        );

        if t_sec >= duration {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
            });
            defmt::info!("Onboard trajectory 2 complete after {}s", t_sec);
            return TrajectoryResult::Completed;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0, omega_r: 0.0, stamp: Instant::now(),
    });
    TrajectoryResult::Stopped
}

#[embassy_executor::task]
pub async fn diffdrive_outer_loop_onboard_traj2(
    _mode: ControlMode,
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig = cfg.unwrap_or_default();

    defmt::info!("Initializing onboard trajectory 2 mode (demo)");
    defmt::info!(
        "Wheel radius[m]: {}, Wheel base[m]: {}, Wheel rotate speed max[rad/s]: {}",
        robot_cfg.wheel_radius, robot_cfg.wheel_base, robot_cfg.wheel_max,
    );

    let mut robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );

    let mut controller = DiffdriveControllerCascade::new(
        robot_cfg.kx_traj,
        robot_cfg.ky_traj,
        robot_cfg.ktheta_traj,
    );

    let _ = with_sdlogger(|logger| logger.write_traj_control_header()).await;

    defmt::info!("Waiting for trajectory 2 control commands...");

    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();

    loop {
        let start_fut = TRAJ_RESUME_SIG.wait();
        let pause_fut = TRAJ_PAUSE_SIG.wait();
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select3(start_fut, stop_fut, pause_fut).await {
            Either3::First(start) => {
                if !start { continue; }
                defmt::info!("Starting onboard trajectory 2!");
                led_set(true).await;
            }
            Either3::Second(_) => {
                defmt::warn!("STOP outer loop (traj2) -> exit");
                led_set(false).await;
                return;
            }
            Either3::Third(_) => {
                defmt::warn!("PAUSE (idle traj2) -> wait RESUME/STOP");
                led_set(false).await;
                loop {
                    match select(TRAJ_RESUME_SIG.wait(), STOP_TRAJ_OUTER_SIG.wait()).await {
                        Either::First(_) => {
                            defmt::info!("RESUME(idle traj2)");
                            break;
                        }
                        Either::Second(_) => {
                            defmt::warn!("STOP outer loop (idle traj2) -> exit");
                            return;
                        }
                    }
                }
                continue;
            }
        }

        let exec_fut = execute_trajectory_loop_onboard2(
            _mode,
            &mut robot,
            &mut controller,
            &robot_cfg,
        );
        let stop_fut = STOP_TRAJ_OUTER_SIG.wait();

        match select(exec_fut, stop_fut).await {
            Either::First(result) => {
                match result {
                    TrajectoryResult::Completed => {
                        defmt::info!("Onboard trajectory 2 completed.");
                    }
                    TrajectoryResult::Stopped => {
                        defmt::info!("Onboard trajectory 2 stopped internally.");
                    }
                }
                led_set(false).await;
                TRAJ_RESUME_SIG.reset();
                TRAJ_PAUSE_SIG.reset();
            }
            Either::Second(_) => {
                defmt::warn!("STOP outer during exec (traj2) -> exit");
                led_set(false).await;
                return;
            }
        }
        defmt::info!("Waiting for next command (traj2)...");
    }
}

/* ===================================================================================== */

/* ========================== Not use or abandoned ==================================== */
// Outer Loop
#[embassy_executor::task]
pub async fn diffdrive_outer_loop(
    mode: ControlMode,
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
    defmt::info!("csv header written");

    /* ================ Record First Pose w.r.t the selected mode ================== */
    let mut first_pose: PoseAbs = PoseAbs::default();

    Timer::after_millis(100).await; // has to wait until the first poses comes
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
        robot.s.theta = SO2::new(pose.yaw + 0.5 * PI); // mocap yaw is 90deg off

        // Compute wheel commands: always use controller (closed-loop), regardless of mocap
        let (action, x_error, y_error, theta_error) = controller.control(&robot, setpoint);
        let ul = action.ul;
        let ur = action.ur;

        while WHEEL_CMD_CH.try_receive().is_ok() {} // drain old commands
        let _ = WHEEL_CMD_CH.try_send(WheelCmd {
            omega_l: ul,
            omega_r: ur,
            stamp: Instant::now(),
        });

        let log = TrajControlLog {
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

        let _ = with_sdlogger(|logger| {
            logger.log_traj_control_as_csv(&log);
            logger.flush();
        }).await;

        let w_deg = setpoint.wdes * 180.0 / PI;
        defmt::info!(
            "t={}s, pose=({},{},{}), des=({},{},{}), v={}, w={}°/s, u=({},{}), err=({},{},{})",
            t_sec,
            pose.x, pose.y, pose.yaw,
            setpoint.des.x, setpoint.des.y, setpoint.des.theta.rad(),
            setpoint.vdes, w_deg,
            ul, ur,
            x_error, y_error, theta_error,
        );

        if t_sec >= bezier_duration {
            //stop motors
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            defmt::info!("Bezier trajectory complete after {}s", t_sec);
            break;
        }
    }

    let _ = WHEEL_CMD_CH.try_send(WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    });
}