use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::orchestrator_signal::{
    LEN_FUNC_SELECT_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_MOTOR_CTRL_SIG, STOP_TELEOP_UART_SIG,
    decode_functionality_select_command,
};
use crate::packet::{CmdLegacyPacketF32, CmdTeleopPacketMix};
use crate::trajectory_uart::UartCfg;
use crate::uart::SharedUart;
use core::cmp::min;
use defmt::info;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
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
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
) {
    // initial filter coefficient 0.0
    let mut filtered_rpm_left = 0.0;
    let mut filtered_rpm_right = 0.0;

    // Filterr coefficient alpha ∈ (0,1), smaller value means smoother/better
    let alpha = 0.1;

    let mut error_sum_left = 0.0;
    let mut error_sum_right = 0.0;

    let kp = 25.0;
    let ki = 1.0;

    let mut ticker = Ticker::every(Duration::from_millis(JOYSTICK_CONTROL_DT));

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
        let v = cmd.v; // m/s
        let omega = cmd.omega; //rad/s

        let v_left = v - omega * WHEEL_BASE / 2.0;
        let v_right = v + omega * WHEEL_BASE / 2.0;

        let rpm_left_target = v_left / (2.0 * PI * WHEEL_RADIUS) * 60.0;
        let rpm_right_target = v_right / (2.0 * PI * WHEEL_RADIUS) * 60.0;

        let (rpm_left_now, rpm_right_now) = get_rpms(
            left_counter,
            right_counter,
            ENCODER_CPR,
            JOYSTICK_CONTROL_DT,
        )
        .await;
        filtered_rpm_left = alpha * rpm_left_now + (1.0 - alpha) * filtered_rpm_left;
        filtered_rpm_right = alpha * rpm_right_now + (1.0 - alpha) * filtered_rpm_right;

        let error_left = rpm_left_target - filtered_rpm_left;
        let error_right = rpm_right_target - filtered_rpm_right;

        error_sum_left += error_left;
        error_sum_right += error_right;

        let mut duty_left = ((kp * error_left + ki * error_sum_left) / 10000.0).clamp(-1.0, 1.0);
        let mut duty_right = ((kp * error_right + ki * error_sum_right) / 10000.0).clamp(-1.0, 1.0);

        // info!("Set Speed: {}, {}", duty_left, duty_right);
        // info!(
        //     "rpm_left_target: {}, rpm_left_now: {}",
        //     rpm_left_target, rpm_left_now
        // );
        // info!(
        //     "rpm_right_target: {}, rpm_right_now: {}",
        //     rpm_right_target, rpm_right_now
        // );

        if v_left.abs() < 0.1 {
            //slack for zero speed recognition
            duty_left = 0.0;
            error_sum_left = 0.0; // Reset error when speed is close to zero
        }
        if v_right.abs() < 0.1 {
            //slack for zero speed recognition
            duty_right = 0.0;
            error_sum_right = 0.0; // Reset error when speed is close to zero
        }

        // Apply robot-specific motor direction corrections
        motor
            .set_speed(
                duty_left * MOTOR_DIRECTION_LEFT,
                duty_right * MOTOR_DIRECTION_RIGHT,
            )
            .await;

        // Timer::after(Duration::from_millis(JOYSTICK_CONTROL_DT)).await;
    }
}
/* ========================== Joy Stick Speed Control Task =============================== */

/* ============== uart teleop command receiving task with nonlinear mapping ============== */
#[embassy_executor::task]
pub async fn teleop_uart_task(uart: SharedUart<'static>, cfg: UartCfg) {
    let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, 32> = HVec::new();

    loop {
        let read_len_fut = async {
            let mut u = uart.lock().await;
            match u.read(&mut len_buf).await {
                Ok(()) => Some(len_buf[0]), // read 1 byte
                Err(_) => None,
            }
        };

        let timeout_len_fut = Timer::after(Duration::from_millis(1));
        // let stop_fut = STOP_MOCAP_UART_SIG.wait();

        // let timeout_fut = Timer::after(Duration::from_millis(1000));
        // let read_byte_fut = async {
        //     let mut uart = uart.lock().await;
        //     let mut b = [0u8; 1];
        //     // buffer.push(b[0]).ok();
        //     match uart.read(&mut b).await {
        //         Ok(_) => Some(b[0]),
        //         Err(_) => None,
        //     }
        // };
        let stop_fut = STOP_TELEOP_UART_SIG.wait();

        let len: Option<u8> = match select3(read_len_fut, timeout_len_fut, stop_fut).await {
            Either3::First(v) => v,
            Either3::Second(_) => None,
            Either3::Third(_) => {
                info!("mocap uart: stop signal on len -> exit");
                return;
            }
        };

        let Some(_len) = len else {
            // TimeOut Or Error
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        let len = len_buf[0];
        if !(len == TELEOP_PACK_LEN || len == LEN_FUNC_SELECT_CMD) {
            continue; // illegal Length
        }

        // ================= Read PAYLOAD =================
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            // maximally read 32 bytes at a time

            let mut chunk = [0u8; 18];
            let take = min(need - got, chunk.len());

            let read_chunk_fut = async {
                let mut u = uart.lock().await;
                u.read(&mut chunk[..take]).await.is_ok()
            };
            let timeout_chunk_fut = Timer::after(Duration::from_millis(2));
            let stop_fut = STOP_TELEOP_UART_SIG.wait();

            let ok: bool = match select3(read_chunk_fut, timeout_chunk_fut, stop_fut).await {
                Either3::First(ok) => ok,
                Either3::Second(_) => false,
                Either3::Third(_) => {
                    info!("uart: stop signal on payload -> exit");
                    return;
                }
            };

            if !ok {
                // Current frame is lost, abandon and quit reading this frame
                Timer::after(Duration::from_micros(300)).await;
                got = 0;
                frame.clear();
                break;
            }

            // attach to frame
            let _ = frame.extend_from_slice(&chunk[..take]);
            got += take;
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
