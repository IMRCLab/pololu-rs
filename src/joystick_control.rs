use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::packet::{CmdLegacyPacketF32, CmdTeleopPacketMix};
use crate::uart::SharedUart;
// Import the global verbosity macros
use crate::debug_warn;

use heapless::Vec;

const PI: f32 = core::f32::consts::PI;

// Import the selected constants into the module scope
use crate::robot_parameters_default::robot_constants::*;

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

/*




/* ================================== currently used ===================================== */
/* ========================== Joy Stick Speed Control Task =============================== */
*/
#[embassy_executor::task]
pub async fn motor_control_task(
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

    loop {
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

        Timer::after(Duration::from_millis(JOYSTICK_CONTROL_DT)).await;
    }
}
/* ========================== Joy Stick Speed Control Task =============================== */

/* ============== uart teleop command receiving task with nonlinear mapping ============== */
#[embassy_executor::task]
pub async fn robot_command_control_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        let timeout = Timer::after(Duration::from_millis(1000));
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
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = ControlCommandUnicycle { v: 0.0, omega: 0.0 };
                    debug_warn!("UART timeout, stop motors");
                }
                buffer.clear();
                continue;
            }

            Either::Second(Some(byte)) => {
                if buffer.is_empty() && !(byte == 2 || byte == 8 || byte == 9) {
                    // correct teleop command buffer length should be 9.
                    // header(1 byte) + float(4 bytes) + float(4 bytes)
                    continue;
                }

                buffer.push(byte).ok();

                if buffer.len() == 2 && buffer[1] != 0x3C {
                    buffer.clear();
                    continue;
                }

                let expected_len = buffer[0] as usize + 1;
                if buffer.len() == expected_len {
                    if let Some(pkt) = CmdTeleopPacketMix::from_bytes(&buffer) {
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

                                let scaled_omega = MAX_OMEGA
                                    * (libm::expf(abs_omega / MAX_OMEGA) - 1.0)
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
                    buffer.clear();
                }
            }

            Either::Second(None) => {
                continue;
            }
        }
    }
}
/* ============== uart teleop command receiving task with nonlinear mapping ============== */
