use crate::math::SO2;
use core::f32::consts::PI;
use libm::{atan2f, cosf, sinf, sqrtf};
use micro_qp::{
    admm::{AdmmSettings, AdmmSolver},
    types::{MatMN, VecN},
};

use crate::robot_parameters_default::robot_constants::*;

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

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ControlMode {
    WithMocapController,
    DirectDuty,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TrajectoryResult {
    Completed,
    Stopped,
}
