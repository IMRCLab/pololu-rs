use crate::math::SO2;
use core::f32::consts::PI;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder};
use embassy_time::{Duration, Instant, Ticker};

use crate::motor::MotorController;
use crate::trajectory_signal::{LAST_STATE, STATE_SIG, WHEEL_CMD_CH, WheelCmd};

use portable_atomic::{AtomicBool, AtomicI32, Ordering};

use crate::diffdrive::{
    Diffdrive, DiffdriveController, DiffdriveState, robot_constants_diffdrive::*,
};

pub static ENC_LEFT_DELTA: AtomicI32 = AtomicI32::new(0);
pub static ENC_RIGHT_DELTA: AtomicI32 = AtomicI32::new(0);

pub static STOP_ALL: AtomicBool = AtomicBool::new(false);

//TODO:
// clean up robot struct and timer
// add abstraction layer for trajectory selection
// time to test the position controller with mocap system

#[derive(Copy, Clone)]
pub enum ControlMode {
    WithInnerSpeedLoop,
    DirectDuty,
}

#[embassy_executor::task]
pub async fn encoder_left_task_cascade(
    mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 0>,
) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        ENC_LEFT_DELTA.fetch_add(delta, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn encoder_right_task_cascade(
    mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 1>,
) {
    loop {
        let dir = encoder.read().await;
        let delta = match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        ENC_RIGHT_DELTA.fetch_add(delta, Ordering::Relaxed);
    }
}

/// inner loop
#[embassy_executor::task]
pub async fn wheel_speed_inner_loop(motor: MotorController) {
    let mut ticker = Ticker::every(Duration::from_millis(5));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (kp, ki) = (0.8, 0.0);

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

        if let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
            last_cmd = cmd;
        }

        let dl = ENC_LEFT_DELTA.swap(0, Ordering::AcqRel);
        let dr = ENC_RIGHT_DELTA.swap(0, Ordering::AcqRel);
        let dt = 0.005;

        let omega_l_meas = (dl as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);
        let omega_r_meas = (dr as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);

        let el = last_cmd.omega_l - omega_l_meas;
        let er = last_cmd.omega_r - omega_r_meas;

        il = (il + ki * dt * el).clamp(-0.3, 0.3);
        ir = (ir + ki * dt * er).clamp(-0.3, 0.3);

        let duty_l = (kp * el + il).clamp(-1.0, 1.0) * MOTOR_DIRECTION_LEFT;
        let duty_r = (kp * er + ir).clamp(-1.0, 1.0) * MOTOR_DIRECTION_RIGHT;

        motor.set_speed(duty_l, duty_r).await;
    }
}

// outer loop
#[embassy_executor::task]
pub async fn diffdrive_outer_loop(mut motor: Option<MotorController>, mode: ControlMode) {
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
    let circle_duration = 10.0;

    loop {
        ticker.next().await;

        let pose = {
            let s = LAST_STATE.lock().await;
            *s
        };
        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw);

        let t = (Instant::now() - start).as_secs() as f32;
        let setpoint = robot.circlereference(t, circle_radius, 0.5);

        match mode {
            ControlMode::WithInnerSpeedLoop => {
                let (action, _xerror, _yerror, _yawerror) = controller.control(&robot, setpoint);
                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: action.ul,
                    omega_r: action.ur,
                    stamp: Instant::now(),
                });
            }
            ControlMode::DirectDuty => {
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

                let mut duty_l = (ul / (2.0 * WHEEL_MAX)).clamp(robot.ul_min, robot.ul_max)
                    * MOTOR_DIRECTION_LEFT;
                let mut duty_r = (ur / (2.0 * WHEEL_MAX)).clamp(robot.ur_min, robot.ur_max)
                    * MOTOR_DIRECTION_RIGHT;

                if ul.abs() < 1.0 && ur.abs() < 1.0 {
                    duty_l = 0.0;
                    duty_r = 0.0;
                }

                if let Some(m) = motor.as_mut() {
                    m.set_speed(duty_l, duty_r).await;
                }

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
            if let Some(m) = motor.as_mut() {
                m.set_speed(0.0, 0.0).await;
            }
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
