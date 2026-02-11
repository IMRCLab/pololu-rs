use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::orchestrator_signal::{
    LEN_FUNC_SELECT_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_MOTOR_CTRL_SIG, STOP_TELEOP_UART_SIG,
    decode_functionality_select_command,
};
use crate::packet::{CmdLegacyPacketF32, CmdTeleopPacketMix, StateLoopBackPacketF32};
use crate::read_robot_config_from_sd::RobotConfig;
use crate::trajectory_uart::UartCfg;
use crate::uart::UART_RX_CHANNEL;
use crate::uart::{SharedUart, update_robot_state};
use defmt::info;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::msos::DescriptorSetInformation;
use heapless::Vec as HVec;
// Import the global verbosity macros
use crate::debug_warn;

use heapless::Vec;

const PI: f32 = core::f32::consts::PI;
const TELEOP_PACK_LEN: u8 = 9;

// Import the selected constants into the module scope
use crate::robot_parameters_default::robot_constants::*;

// Create Signal for quitting teleop mode

/// Get the gear ratio for runtime identification
pub fn get_gear_ratio() -> f32 {
    GEAR_RATIO
}

/// Get motor direction multipliers for runtime identification
pub fn get_motor_directions() -> (f32, f32) {
    (MOTOR_DIRECTION_LEFT, MOTOR_DIRECTION_RIGHT)
}

#[derive(Copy, Clone, Debug)]
pub struct ControlCommand {
    pub left_speed: f32,
    pub right_speed: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct ControlCommandUnicycle {
    pub v: f32,
    pub omega: f32,
}

pub static CONTROL_CMD: Mutex<ThreadModeRawMutex, ControlCommand> = Mutex::new(ControlCommand {
    left_speed: 0.0,
    right_speed: 0.0,
});

pub static CONTROL_CMD_UNICYCLE: Mutex<ThreadModeRawMutex, ControlCommandUnicycle> =
    Mutex::new(ControlCommandUnicycle { v: 0.0, omega: 0.0 });

/* ===================================== Unused ========================================== */
/* ================ Joy Stick Speed Control Task (without pd controller) ================= */
#[embassy_executor::task]
pub async fn motor_task(motor: MotorController) {
    loop {
        let cmd = CONTROL_CMD.lock().await.clone();
        motor
            .set_speed(
                (cmd.left_speed as f32) / 10000.0,
                (cmd.right_speed as f32) / 10000.0,
            )
            .await;

        // 50ms
        Timer::after(Duration::from_millis(50)).await;
    }
}

/* ===================== simple uart teleop command receiving task ======================= */
#[embassy_executor::task]
pub async fn robot_command_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        let timeout = Timer::after(Duration::from_millis(500));
        let byte_future = async {
            let mut uart = uart.lock().await;
            let mut b = [0u8; 1];
            match uart.read(&mut b).await {
                Ok(_) => Some(b[0]),
                Err(_) => None,
            }
        };

        match select(timeout, byte_future).await {
            Either::First(_) => {
                {
                    let mut lock = CONTROL_CMD.lock().await;
                    *lock = ControlCommand {
                        left_speed: 0.0,
                        right_speed: 0.0,
                    };
                    debug_warn!("UART timeout, stop motors");
                }
                buffer.clear();
                continue;
            }

            Either::Second(Some(byte)) => {
                if buffer.is_empty() && !(byte == 9 || byte == 13 || byte == 17) {
                    continue;
                }

                buffer.push(byte).ok();

                if buffer.len() == 2 && buffer[1] != 0x3C {
                    buffer.clear();
                    continue;
                }

                let expected_len = buffer[0] as usize + 1;
                if buffer.len() == expected_len {
                    if let Some(pkt) = CmdLegacyPacketF32::from_bytes(&buffer) {
                        let cmd = ControlCommand {
                            left_speed: pkt.left_pwm_duty * pkt.left_direction,
                            right_speed: pkt.right_pwm_duty * pkt.right_direction,
                        };

                        {
                            let mut lock = CONTROL_CMD.lock().await;
                            *lock = cmd;
                        }

                        info!("Updated Control: {}, {}", cmd.left_speed, cmd.right_speed);
                    }
                    buffer.clear();
                }
            }

            Either::Second(None) => {
                continue;
            }
        }
    }
}
/* ===================== simple uart teleop command receiving task ======================= */
/* ===================================== Unused ========================================== */

/* ================================== currently used ===================================== */
/* ========================== Joy Stick Speed Control Task =============================== */
#[embassy_executor::task]
pub async fn teleop_motor_control_task(
    //adapted from inner loop
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        defmt::info!("Load Robot Params from SD CARD!");
        robot_cfg = cfg.unwrap();
    } else {
        defmt::info!("Load Robot Params from DEFAULT!");
        robot_cfg = RobotConfig::default();
    }

    let mut ticker = Ticker::every(Duration::from_millis(robot_cfg.joystick_control_dt_ms));

    // PID state variables
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32);
    let (kp, ki, kd) = (robot_cfg.kp_inner, robot_cfg.ki_inner, robot_cfg.kd_inner);

    // =========== Filter Parameters ==============
    let dt: f32 = robot_cfg.joystick_control_dt_ms as f32 / 1000.0;
    let fc_hz: f32 = 3.0; // Match trajectory control - integral term helps smooth quantization noise
    let tau: f32 = 1.0 / (2.0 * PI * fc_hz);
    let alpha: f32 = dt / (tau + dt);

    let mut prev_l = 0i32;
    let mut prev_r = 0i32;

    let mut omega_l_lp: f32 = 0.0;
    let mut omega_r_lp: f32 = 0.0;

    loop {
        match select(ticker.next(), STOP_MOTOR_CTRL_SIG.wait()).await {
            Either::First(_) => {}
            Either::Second(_) => {
                motor.set_speed(0.0, 0.0).await;
                {
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = ControlCommandUnicycle { v: 0.0, omega: 0.0 };
                }
                return;
            }
        }

        let cmd = CONTROL_CMD_UNICYCLE.lock().await.clone();
        // defmt::info!("unicyclecmds: ({},{})", cmd.v, cmd.omega);
        let v = cmd.v; // m/s
        let omega = cmd.omega; //rad/s

        // Convert unicycle commands to differential drive wheel velocities
        let v_left = v - omega * robot_cfg.wheel_base / 2.0;
        let v_right = v + omega * robot_cfg.wheel_base / 2.0;

        defmt::info!("wheel speeds in meter/s: ({},{})", v_left, v_right);

        // Convert linear wheel velocity to angular velocity (rad/s)
        let omega_l_target = v_left / robot_cfg.wheel_radius;
        let omega_r_target = v_right / robot_cfg.wheel_radius;

        defmt::info!(
            "wheel angular velocities (rad/s): target L: {}, R: {}",
            omega_l_target,
            omega_r_target
        );

        // Get raw angular velocity of the wheels using encoder counts
        use crate::encoder::wheel_speed_from_counts_now;
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

        defmt::info!(
            "wheel speed counts: target: ({},{}), reading raw: ({},{}), lp: ({},{})",
            omega_l_target,
            omega_r_target,
            omega_l_raw,
            omega_r_raw,
            omega_l_lp,
            omega_r_lp
        );

        // =========== Low Pass Filter ==============
        omega_l_lp = omega_l_lp + alpha * (omega_l_raw - omega_l_lp);
        omega_r_lp = omega_r_lp + alpha * (omega_r_raw - omega_r_lp);

        // Error calculation
        let el = omega_l_target - omega_l_lp;
        let er = omega_r_target - omega_r_lp;

        // Error integration with anti-windup (helps smooth encoder quantization)
        il = (il + ki * dt * el).clamp(-2.0, 2.0);
        ir = (ir + ki * dt * er).clamp(-2.0, 2.0);

        // Error differentiation
        let dl = (el - prev_el) / dt;
        let dr = (er - prev_er) / dt;

        prev_el = el;
        prev_er = er;

        // PID control (normally the D term is disabled, kd = 0.0)
        let u_l = (kp * el + il + kd * dl).clamp(-1.0, 1.0);
        let u_r = (kp * er + ir + kd * dr).clamp(-1.0, 1.0);

        // Apply robot-specific motor direction corrections
        let duty_l = -u_l * robot_cfg.motor_direction_left;
        let duty_r = -u_r * robot_cfg.motor_direction_right;

        // defmt::info!("duty: ({},{})", duty_l, duty_r);
        defmt::info!(
            "target ω L: {}, R: {}, meas ω L: {}, R: {}, error L: {}, R: {}, duty L: {}, duty R: {}",
            omega_l_target,
            omega_r_target,
            omega_l_lp,
            omega_r_lp,
            el,
            er,
            duty_l,
            duty_r
        );

        motor.set_speed(duty_l, duty_r).await;

        Timer::after(Duration::from_millis(robot_cfg.joystick_control_dt_ms)).await;
    }
}
/* ========================== Joy Stick Speed Control Task =============================== */

/* ============== uart teleop command receiving task with nonlinear mapping ============== */
#[embassy_executor::task]
pub async fn teleop_uart_task(cfg: UartCfg) {
    let mut frame: HVec<u8, 32> = HVec::new();

    loop {
        let read_len_fut = UART_RX_CHANNEL.receive();
        let timeout_len_fut = Timer::after(Duration::from_millis(1));
        let stop_fut = STOP_TELEOP_UART_SIG.wait();

        let len: Option<u8> = match select3(read_len_fut, timeout_len_fut, stop_fut).await {
            Either3::First(v) => Some(v),
            Either3::Second(_) => None,
            Either3::Third(_) => {
                info!("mocap uart: stop signal on len -> exit");
                return;
            }
        };

        let Some(len) = len else {
            // TimeOut Or Error
            // info!("teleop illegal timeout");
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        // let len = len_buf[0];
        if !(len == TELEOP_PACK_LEN || len == LEN_FUNC_SELECT_CMD) {
            // info!("teleop illegal");
            continue; // illegal Length
        }

        // ================= Read PAYLOAD =================
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            let read_byte_fut = UART_RX_CHANNEL.receive();
            let timeout_fut = Timer::after(Duration::from_millis(2));
            let stop_fut = STOP_TELEOP_UART_SIG.wait();

            let b_opt: Option<u8> = match select3(read_byte_fut, timeout_fut, stop_fut).await {
                Either3::First(b) => Some(b),
                Either3::Second(_) => None,
                Either3::Third(_) => {
                    info!("uart: stop signal on payload -> exit");
                    return;
                }
            };

            let Some(b) = b_opt else {
                frame.clear();
                got = 0;
                break;
            };

            frame.push(b).ok();
            got += 1;
        }

        if got != need {
            continue;
        }

        if len == TELEOP_PACK_LEN {
            // info!("buffer len {}", buffer.len());
            if let Some(pkt) = CmdTeleopPacketMix::from_bytes(&frame) {
                let v = {
                    let raw = pkt.linear_velocity as f32;
                    let sign = if raw >= 0.0 { 1.0 } else { -1.0 };
                    let abs_raw = raw.abs();

                    let normalized = ((abs_raw - 1000.0) / 19000.0).clamp(0.0, 1.0);
                    let speed_upper_limits = MAX_SPEED;
                    let speed = speed_upper_limits * (libm::expf(2.0 * normalized) - 1.0)
                        / (libm::expf(2.0) - 1.0);
                    sign * speed
                };

                let cmd = ControlCommandUnicycle {
                    v: v,
                    omega: {
                        let raw_omega = pkt.steering_angle * PI / (180.0 * 0.1);
                        let sign = if raw_omega >= 0.0 { 1.0 } else { -1.0 };
                        let abs_omega = raw_omega.abs();

                        let scaled_omega = MAX_OMEGA * (libm::expf(abs_omega / MAX_OMEGA) - 1.0)
                            / (libm::expf(1.0) - 1.0);
                        sign * scaled_omega
                    },
                };

                {
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = cmd;
                }

                debug_warn!("Updated Control: {}, {}", cmd.v, cmd.omega);
            }
        }

        if len == LEN_FUNC_SELECT_CMD {
            if let Some(sel) = decode_functionality_select_command(&frame, cfg.robot_id) {
                let target = match sel {
                    0 => Mode::Menu,
                    1 => Mode::TeleOp,
                    2 => Mode::TrajMocap,
                    3 => Mode::TrajDuty,
                    4 => Mode::CtrlAction,
                    _ => Mode::Menu,
                };
                let _ = ORCH_CH.try_send(OrchestratorMsg::SwitchTo(target));
                return;
            }
            continue;
        }
    }
}
/* ============== uart teleop command receiving task with nonlinear mapping ============== */

/* ============== Control Action UART task - direct (v, omega) from ROS ================= */
/// Control Action UART task - direct (v, omega) from ROS without joystick mapping
#[embassy_executor::task]
pub async fn control_action_uart_task(cfg: UartCfg) {
    let mut frame: HVec<u8, 32> = HVec::new();

    loop {
        let read_len_fut = UART_RX_CHANNEL.receive();
        let timeout_len_fut = Timer::after(Duration::from_millis(1));
        let stop_fut = STOP_TELEOP_UART_SIG.wait(); // reuse existing signal

        let len: Option<u8> = match select3(read_len_fut, timeout_len_fut, stop_fut).await {
            Either3::First(v) => Some(v),
            Either3::Second(_) => None,
            Either3::Third(_) => {
                info!("control_action uart: stop signal -> exit");
                return;
            }
        };

        let Some(len) = len else {
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        if !(len == TELEOP_PACK_LEN || len == LEN_FUNC_SELECT_CMD) {
            continue;
        }

        // Read payload (same as teleop_uart_task)
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            let read_byte_fut = UART_RX_CHANNEL.receive();
            let timeout_fut = Timer::after(Duration::from_millis(2));
            let stop_fut = STOP_TELEOP_UART_SIG.wait();

            let b_opt: Option<u8> = match select3(read_byte_fut, timeout_fut, stop_fut).await {
                Either3::First(b) => Some(b),
                Either3::Second(_) => None,
                Either3::Third(_) => {
                    info!("control_action uart: stop signal on payload -> exit");
                    return;
                }
            };

            let Some(b) = b_opt else {
                frame.clear();
                got = 0;
                break;
            };

            frame.push(b).ok();
            got += 1;
        }

        if got != need {
            continue;
        }

        if len == TELEOP_PACK_LEN {
            //Receive Unicycle Commands here directly
            if let Some(pkt) = CmdTeleopPacketMix::from_bytes(&frame) {
                let cmd = ControlCommandUnicycle {
                    v: pkt.linear_velocity,    // m/s
                    omega: pkt.steering_angle, // rad/s
                };

                {
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = cmd;
                }

                debug_warn!("CtrlAction: v={}, omega={}", cmd.v, cmd.omega);
            }
        }

        if len == LEN_FUNC_SELECT_CMD {
            if let Some(sel) = decode_functionality_select_command(&frame, cfg.robot_id) {
                let target = match sel {
                    0 => Mode::Menu,
                    1 => Mode::TeleOp,
                    2 => Mode::TrajMocap,
                    3 => Mode::TrajDuty,
                    4 => Mode::CtrlAction,
                    _ => Mode::Menu,
                };
                let _ = ORCH_CH.try_send(OrchestratorMsg::SwitchTo(target));
                return;
            }
        }
    }
}
/* ============== Control Action UART task - direct (v, omega) from ROS ================= */

/* ===================================== Unused ========================================== */
/* ================ Joy Stick Speed Control Task (without pd controller) ================= */
// #[embassy_executor::task]
// pub async fn motor_task(motor: MotorController) {
//     loop {
//         let cmd = CONTROL_CMD.lock().await.clone();
//         motor
//             .set_speed(
//                 (cmd.left_speed as f32) / 10000.0,
//                 (cmd.right_speed as f32) / 10000.0,
//             )
//             .await;

//         // 50ms
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }

// /* ===================== simple uart teleop command receiving task ======================= */
// #[embassy_executor::task]
// pub async fn robot_command_task(uart: SharedUart<'static>) {
//     let mut buffer: Vec<u8, 32> = Vec::new();

//     loop {
//         let timeout = Timer::after(Duration::from_millis(500));
//         let byte_future = async {
//             let mut uart = uart.lock().await;
//             let mut b = [0u8; 1];
//             match uart.read(&mut b).await {
//                 Ok(_) => Some(b[0]),
//                 Err(_) => None,
//             }
//         };

//         match select(timeout, byte_future).await {
//             Either::First(_) => {
//                 {
//                     let mut lock = CONTROL_CMD.lock().await;
//                     *lock = ControlCommand {
//                         left_speed: 0.0,
//                         right_speed: 0.0,
//                     };
//                     debug_warn!("UART timeout, stop motors");
//                 }
//                 buffer.clear();
//                 continue;
//             }

//             Either::Second(Some(byte)) => {
//                 if buffer.is_empty() && !(byte == 9 || byte == 13 || byte == 17) {
//                     continue;
//                 }

//                 buffer.push(byte).ok();

//                 if buffer.len() == 2 && buffer[1] != 0x3C {
//                     buffer.clear();
//                     continue;
//                 }

//                 let expected_len = buffer[0] as usize + 1;
//                 if buffer.len() == expected_len {
//                     if let Some(pkt) = CmdLegacyPacketF32::from_bytes(&buffer) {
//                         let cmd = ControlCommand {
//                             left_speed: pkt.left_pwm_duty * pkt.left_direction,
//                             right_speed: pkt.right_pwm_duty * pkt.right_direction,
//                         };

//                         {
//                             let mut lock = CONTROL_CMD.lock().await;
//                             *lock = cmd;
//                         }

//                         info!("Updated Control: {}, {}", cmd.left_speed, cmd.right_speed);
//                     }
//                     buffer.clear();
//                 }
//             }

//             Either::Second(None) => {
//                 continue;
//             }
//         }
//     }
// }
// /* ===================== simple uart teleop command receiving task ======================= */
// /* ===================================== Unused ========================================== */
