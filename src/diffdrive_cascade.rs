use crate::math::SO2;
use core::f32::consts::PI;
use defmt::info;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};

use crate::diffdrive::{Diffdrive, DiffdriveController, DiffdriveState};
use crate::encoder::wheel_speed_from_counts_now;
use crate::motor::MotorController;
use crate::robot_parameters_default::robot_constants::*;
use crate::trajectory_signal::{
    FIRST_MESSAGE, LAST_STATE, PoseAbs, STATE_SIG, WHEEL_CMD_CH, WheelCmd,
};

use portable_atomic::{AtomicBool, Ordering};

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

// ====================== TODO: ===========================
// (1) add abstraction layer for trajectory selection
// (2) time to test the position controller with mocap system
// ====================== TODO: ===========================

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
pub async fn diffdrive_outer_loop(mode: ControlMode) {
    STOP_ALL.store(false, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis((DT_S * 1000.0) as u64));
    let mut robot = Diffdrive::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX,
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );
    let controller = DiffdriveController::new(KX, KY, KTHETA);

    let mut t_counter = 0.0;
    let start = Instant::now();
    let circle_radius = 0.3;
    let circle_duration = 7.0;

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
            //wait for a pose to be received from mocap system
            info!("waiting for first pose from mocap system...");
            FIRST_MESSAGE.wait().await;
            //initialise first pose from mocap:
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

        //generate next setpoint
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

        //get robot pose
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

                let state_des = DiffdriveState {
                    x: setpoint.des.x,
                    y: setpoint.des.y,
                    theta: setpoint.des.theta,
                };

                let ur = (2.0 * vd + WHEEL_BASE * w_rad) / (2.0 * WHEEL_RADIUS);
                let ul = (2.0 * vd - WHEEL_BASE * w_rad) / (2.0 * WHEEL_RADIUS);

                while WHEEL_CMD_CH.try_receive().is_ok() {} // drain the old command
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

        if t_counter > circle_duration / DT_S {
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

#[embassy_executor::task]
pub async fn test_outer_loop(mode: ControlMode) {
    STOP_ALL.store(false, Ordering::Relaxed);
    let mut ticker = Ticker::every(Duration::from_millis((DT_S * 1000.0) as u64));
    let mut robot = Diffdrive::new(
        WHEEL_RADIUS,
        WHEEL_BASE,
        -WHEEL_MAX,
        WHEEL_MAX,
        -WHEEL_MAX,
        WHEEL_MAX,
    );
    let controller = DiffdriveController::new(KX, KY, KTHETA);

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
            //wait for a pose to be received from mocap system
            info!("waiting for first pose from mocap system...");
            FIRST_MESSAGE.wait().await;
            //initialise first pose from mocap:
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

        //generate next setpoint
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

        //get robot pose
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

                let state_des = DiffdriveState {
                    x: setpoint.des.x,
                    y: setpoint.des.y,
                    theta: setpoint.des.theta,
                };
                let ur = (2.0 * vd + robot.l * w_rad) / (2.0 * robot.r);
                let ul = (2.0 * vd - robot.l * w_rad) / (2.0 * robot.r);
                // info!("robot l: {}, robot_radius: {}", robot.l, robot.r);

                while WHEEL_CMD_CH.try_receive().is_ok() {} // 先清空旧的
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });

                // let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                //     omega_l: ul,
                //     omega_r: ur,
                //     stamp: Instant::now(),
                // });

                // let mut duty_l = (ul / (2.0 * WHEEL_MAX)).clamp(robot.ul_min, robot.ul_max)
                //     * MOTOR_DIRECTION_LEFT;
                // let mut duty_r = (ur / (2.0 * WHEEL_MAX)).clamp(robot.ur_min, robot.ur_max)
                //     * MOTOR_DIRECTION_RIGHT;

                // if ul.abs() < 1.0 && ur.abs() < 1.0 {
                //     duty_l = 0.0;
                //     duty_r = 0.0;
                // }

                // if let Some(m) = motor.as_mut() {
                //     m.set_speed(duty_l, duty_r).await;
                // }

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

        if t_counter > circle_duration / DT_S {
            let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                omega_l: 0.0,
                omega_r: 0.0,
                stamp: Instant::now(),
            });
            STOP_ALL.store(true, Ordering::Relaxed);
            // if let Some(m) = motor.as_mut() {
            //     info!("here");
            //     // m.set_speed(0.0, 0.0).await;
            // }
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
