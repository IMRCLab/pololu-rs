use crate::math::SO2;
use core::f32::consts::PI;
use defmt::info;
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

// #[embassy_executor::task]
// pub async fn encoder_left_task_cascade(
//     mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 0>,
// ) {
//     loop {
//         let dir = encoder.read().await;
//         let delta = match dir {
//             Direction::Clockwise => 1,
//             Direction::CounterClockwise => -1,
//         };
//         ENC_LEFT_DELTA.fetch_add(delta, Ordering::Relaxed);
//     }
// }

// #[embassy_executor::task]
// pub async fn encoder_right_task_cascade(
//     mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 1>,
// ) {
//     loop {
//         let dir = encoder.read().await;
//         let delta = match dir {
//             Direction::Clockwise => 1,
//             Direction::CounterClockwise => -1,
//         };
//         ENC_RIGHT_DELTA.fetch_add(delta, Ordering::Relaxed);
//     }
// }

const LUT: [[i8; 4]; 4] = [
    /*prev\curr: 00, 01, 10, 11 */
    [0, 1, -1, 0], // 00 ->
    [-1, 0, 0, 1], // 01 ->
    [1, 0, 0, -1], // 10 ->
    [0, -1, 1, 0], // 11 ->
];

#[embassy_executor::task]
pub async fn encoder_left_task_cascade(
    mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 0>,
) {
    let mut prev = encoder.read_ab().await;
    loop {
        let mut delta: i32 = 0;
        let curr = encoder.read_ab().await;
        delta += LUT[prev as usize][curr as usize] as i32;
        prev = curr;
        while let Some(c) = encoder.try_read_ab() {
            delta += LUT[prev as usize][c as usize] as i32;
            prev = c;
        }
        if delta != 0 {
            ENC_LEFT_DELTA.fetch_add(delta * MOTOR_DIRECTION_LEFT as i32, Ordering::AcqRel);
        }
    }
}

#[embassy_executor::task]
pub async fn encoder_right_task_cascade(
    mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO0, 1>,
) {
    let mut prev = encoder.read_ab().await;
    loop {
        let mut delta: i32 = 0;
        let curr = encoder.read_ab().await;
        delta += LUT[prev as usize][curr as usize] as i32;
        prev = curr;
        while let Some(c) = encoder.try_read_ab() {
            delta += LUT[prev as usize][c as usize] as i32;
            prev = c;
        }
        if delta != 0 {
            ENC_RIGHT_DELTA.fetch_add(delta * MOTOR_DIRECTION_RIGHT as i32, Ordering::AcqRel);
        }
    }
}

/// inner loop
// #[embassy_executor::task]
// pub async fn wheel_speed_inner_loop(motor: MotorController) {
//     let mut ticker = Ticker::every(Duration::from_millis(20));
//     let (mut il, mut ir) = (0.0f32, 0.0f32);
//     let (kp, ki) = (0.1, 0.01);

//     let mut last_cmd = WheelCmd {
//         omega_l: 0.0,
//         omega_r: 0.0,
//         stamp: Instant::now(),
//     };

//     loop {
//         ticker.next().await;

//         if STOP_ALL.load(Ordering::Relaxed) {
//             motor.set_speed(0.0, 0.0).await;
//             break;
//         }

//         if let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
//             last_cmd = cmd;
//         }

//         let dl = ENC_LEFT_DELTA.swap(0, Ordering::AcqRel);
//         let dr = ENC_RIGHT_DELTA.swap(0, Ordering::AcqRel);
//         let dt = 0.02;

//         let omega_l_meas = (dl as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);
//         let omega_r_meas = (dr as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);

//         let el = last_cmd.omega_l - omega_l_meas;
//         let er = last_cmd.omega_r - omega_r_meas;

//         il = (il + ki * dt * el).clamp(-0.3, 0.3);
//         ir = (ir + ki * dt * er).clamp(-0.3, 0.3);

//         let u_l = (kp * el + il).clamp(-1.0, 1.0);
//         let u_r = (kp * er + ir).clamp(-1.0, 1.0);

//         let duty_l = u_l * MOTOR_DIRECTION_LEFT;
//         let duty_r = u_r * MOTOR_DIRECTION_RIGHT;

//         defmt::info!("rpm meas: {}, rpm meas: {}", omega_l_meas, omega_r_meas);
//         defmt::info!("duty left: {}, duty right: {}", duty_l, duty_r);

//         motor.set_speed(duty_l, duty_r).await;
//     }
// }

#[embassy_executor::task]
pub async fn wheel_speed_inner_loop(motor: MotorController) {
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (kp, ki) = (0.1, 0.08);

    // ---- 滤波参数 ----
    let dt: f32 = 0.02; // 20 ms
    let fc_hz: f32 = 8.0; // 一阶低通截止频率，按需 3~8 Hz 调
    let tau: f32 = 1.0 / (2.0 * core::f32::consts::PI * fc_hz);
    let alpha: f32 = dt / (tau + dt); // EMA 系数，0..1，越大响应越快

    // 可选：中值滤波缓存（3 点）
    let mut buf_l: [f32; 3] = [0.0; 3];
    let mut buf_r: [f32; 3] = [0.0; 3];
    let mut idx: usize = 0;

    // 低通状态
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

        if let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
            last_cmd = cmd;
        }

        let dl = ENC_LEFT_DELTA.swap(0, Ordering::AcqRel);
        let dr = ENC_RIGHT_DELTA.swap(0, Ordering::AcqRel);

        // 原始角速度（rad/s）
        let omega_l_raw = (dl as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);
        let omega_r_raw = (dr as f32) * 2.0 * core::f32::consts::PI / (ENCODER_CPR * dt);

        // ---------- 可选：3 点中值滤波（先去尖峰，再低通） ----------
        // 打开下方 6 行注释以启用
        // buf_l[idx] = omega_l_raw;
        // buf_r[idx] = omega_r_raw;
        // idx = (idx + 1) % 3;
        // let mut tl = buf_l;
        // tl.sort_by(|a, b| a.total_cmp(b));
        // let mut tr = buf_r;
        // tr.sort_by(|a, b| a.total_cmp(b));
        // let (omega_l_denoised, omega_r_denoised) = (tl[1], tr[1]);

        // 若不启用中值滤波，就直接用原始值进入低通：
        let (omega_l_denoised, omega_r_denoised) = (omega_l_raw, omega_r_raw);

        // ---------- 一阶低通（EMA） ----------
        omega_l_lp = omega_l_lp + alpha * (omega_l_denoised - omega_l_lp);
        omega_r_lp = omega_r_lp + alpha * (omega_r_denoised - omega_r_lp);

        // 用滤波后的速度参与控制
        let el = last_cmd.omega_l - omega_l_lp;
        let er = last_cmd.omega_r - omega_r_lp;

        il = (il + ki * dt * el).clamp(-0.8, 0.8);
        ir = (ir + ki * dt * er).clamp(-0.8, 0.8);

        let u_l = (kp * el + il).clamp(-1.0, 1.0);
        let u_r = (kp * er + ir).clamp(-1.0, 1.0);

        let duty_l = u_l * MOTOR_DIRECTION_LEFT;
        let duty_r = u_r * MOTOR_DIRECTION_RIGHT;

        // 如果想打印 RPM，按下面转换（当前变量仍是 rad/s）
        // let to_rpm = 60.0 / (2.0 * core::f32::consts::PI);
        defmt::info!("meas rpm L: {}, R: {}", omega_l_lp, omega_r_lp,);
        defmt::info!("duty left: {}, duty right: {}", duty_l, duty_r);

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
    let circle_radius = 0.4;
    let circle_duration = 20.0;

    loop {
        ticker.next().await;

        let pose = {
            let s = LAST_STATE.lock().await;
            *s
        };
        robot.s.x = pose.x;
        robot.s.y = pose.y;
        robot.s.theta = SO2::new(pose.yaw);

        let t = (Instant::now() - start).as_millis() as f32 / 1000.0;
        let setpoint = robot.circlereference(t, circle_radius, (2.0 * PI) / circle_duration);

        match mode {
            ControlMode::WithInnerSpeedLoop => {
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

                let _ = WHEEL_CMD_CH.try_send(WheelCmd {
                    omega_l: ul,
                    omega_r: ur,
                    stamp: Instant::now(),
                });

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
            if let Some(m) = motor.as_mut() {
                info!("here");
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
