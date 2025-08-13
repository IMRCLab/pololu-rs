use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::packet::{CmdLegacyPacketMix, CmdLegacyPacketF32, CmdTeleopPacketMix};
use crate::uart::SharedUart;

use heapless::Vec;

/// Robot-specific constants - conditionally compiled
const PI: f32 = core::f32::consts::PI;
const SAMPLE_MS: u64 = 20; // Sample Period = 20 ms

// Zumo robot constants
#[cfg(feature = "zumo")]
mod robot_constants {
    pub const GEAR_RATIO: f32 = 75.81;
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0; // = 909.72
    pub const WHEEL_BASE: f32 = 0.099;
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;  // Zumo has reversed motors
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
}

// 3Pi robot constants
#[cfg(feature = "three-pi")]
mod robot_constants {
    pub const GEAR_RATIO: f32 = 29.86;
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0; // = 358.32
    pub const WHEEL_BASE: f32 = 0.0842;
    pub const WHEEL_RADIUS: f32 = 0.016;
    pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;   // 3Pi has normal motor directions
    pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
}

// Default values for testing when no features are active
#[cfg(not(any(feature = "zumo", feature = "three-pi")))]
mod robot_constants {
    pub const GEAR_RATIO: f32 = 75.81; // Default to Zumo values for testing
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;  // Default to Zumo behavior
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
}

// Import the selected constants into the module scope
use robot_constants::*;

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

/// === Joy Stick Control Task (without pd controller) ===
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

/// === Joy Stick Control Task ===
#[embassy_executor::task]
pub async fn motor_control_task(
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
) {
    // 初始滤波值设为 0.0 initial filter coefficient
    let mut filtered_rpm_left = 0.0;
    let mut filtered_rpm_right = 0.0;

    // 滤波系数 α ∈ (0,1)，越小越平滑 Filterr coefficient alpha ∈ (0,1), smaller value means smoother/better
    let alpha = 0.1;

    let mut error_sum_left = 0.0;
    let mut error_sum_right = 0.0;

    let kp = 60.0;
    let ki = 1.0;

    loop {
        let cmd = CONTROL_CMD_UNICYCLE.lock().await.clone();
        let v = cmd.v;          // m/s
        let omega = cmd.omega; //rad/s

        let v_left = v - omega * WHEEL_BASE / 2.0;
        let v_right = v + omega * WHEEL_BASE / 2.0;

        let rpm_left_target = v_left / (2.0 * core::f32::consts::PI * WHEEL_RADIUS) * 60.0;
        let rpm_right_target = v_right / (2.0 * core::f32::consts::PI * WHEEL_RADIUS) * 60.0;

        let (rpm_left_now, rpm_right_now) =
            get_rpms(left_counter, right_counter, ENCODER_CPR, SAMPLE_MS).await;
        filtered_rpm_left = alpha * rpm_left_now + (1.0 - alpha) * filtered_rpm_left;
        filtered_rpm_right = alpha * rpm_right_now + (1.0 - alpha) * filtered_rpm_right;

        let error_left = rpm_left_target - filtered_rpm_left;
        let error_right = rpm_right_target - filtered_rpm_right;

        error_sum_left += error_left;
        error_sum_right += error_right;

        let duty_left = ((kp * error_left + ki * error_sum_left) / 10000.0).clamp(-1.0, 1.0);
        let duty_right = ((kp * error_right + ki * error_sum_right) / 10000.0).clamp(-1.0, 1.0);

        info!("Set Speed: {}, {}", duty_left, duty_right);
        info!(
            "rpm_left_target: {}, rpm_left_now: {}",
            rpm_left_target, rpm_left_now
        );
        info!(
            "rpm_right_target: {}, rpm_right_now: {}",
            rpm_right_target, rpm_right_now
        );

        if v == 0.0 {
            //motor.set_speed(0.0, 0.0).await; in case that the robot is stopped, reset the error sum
            error_sum_left = 0.0;
            error_sum_right = 0.0;
        }
        // } else {
        //     motor.set_speed(duty_left, duty_right).await;
        // }

        // Apply robot-specific motor direction corrections
        motor.set_speed(
            duty_left * MOTOR_DIRECTION_LEFT, 
            duty_right * MOTOR_DIRECTION_RIGHT
        ).await;

        Timer::after(Duration::from_millis(SAMPLE_MS)).await;
    }
}

#[embassy_executor::task]
pub async fn robot_command_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        // Set Timer
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
                // Timeout, Stop Pololu
                {
                    let mut lock = CONTROL_CMD.lock().await;
                    *lock = ControlCommand {
                        left_speed: 0.0,
                        right_speed: 0.0,
                    };
                    defmt::warn!("UART timeout, stop motors");
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

                        defmt::info!("Updated Control: {}, {}", cmd.left_speed, cmd.right_speed);
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
/*
#[embassy_executor::task]
pub async fn robot_command_control_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        // Set Timer
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
                // Timeout, Stop Pololu
                {
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = ControlCommandUnicycle { v: 0.0, omega: 0.0 };
                    defmt::warn!("UART timeout, stop motors");
                }
                buffer.clear();
                continue;
            }

            Either::Second(Some(byte)) => {
                if buffer.is_empty() && !(byte == 9 || byte == 15 || byte == 17) {
                    continue;
                }

                buffer.push(byte).ok();

                if buffer.len() == 2 && buffer[1] != 0x3C {
                    buffer.clear();
                    continue;
                }

                let expected_len = buffer[0] as usize + 1;
                if buffer.len() == expected_len {
                    if let Some(pkt) = CmdLegacyPacketMix::from_bytes(&buffer) {
                        let v = pkt.right_direction as f32;
                        let omega = pkt.left_direction;
                        
                        //scale v in CmdLegacyPacketMix with 0.00001 and omega with PI / (180.0 * 0.5)
                        let cmd = ControlCommandUnicycle {
                            v: v * 0.00001, // scale 0 - 20000 "thrust" to 0 - 0.2 m/s
                            omega: omega * PI / (180.0 * 0.5), // turn about the steering angle within half a second
                        };

                        {
                            let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                            *lock = cmd;
                        }
                        // defmt::info!(
                        //     "Whole command: field1={}, field2={}, v={}, omega={}",
                        //     pkt.left_pwm_duty, pkt.right_pwm_duty, cmd.v, cmd.omega
                        // );
                        defmt::info!("Updated Control: {}, {}", cmd.v, cmd.omega);
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
*/


#[embassy_executor::task]
pub async fn robot_command_control_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        // Set Timer
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
                // Timeout, Stop Pololu
                {
                    let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                    *lock = ControlCommandUnicycle { v: 0.0, omega: 0.0 };
                    defmt::warn!("UART timeout, stop motors");
                }
                buffer.clear();
                continue;
            }

            Either::Second(Some(byte)) => {
                if buffer.is_empty() && !(byte == 2 || byte == 8 || byte == 9) { 
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
                            
                            //deadzone is handled in ROS node
                            let normalized = ((abs_raw - 1000.0) / 19000.0).clamp(0.0, 1.0);
                            let speed = 0.1 * (libm::expf(2.0 * normalized) - 1.0) / (libm::expf(2.0) - 1.0); //set max speed to 0.1
                            sign * speed  // Apply original sign for forward/reverse
                    
                        };
                        let cmd = ControlCommandUnicycle {
                            v: v, // scale 0 - 20000 "thrust" to 0 - 0.2 m/s //f32
                            //i prefer a nonlinear scaling for better haptic feedback
                            omega: pkt.steering_angle * PI / (180.0 * 0.5 ), // scale 0 - 20000 "steering" to 0 - 0.2 rad/s should turn by the desired angle within half a second
                        };

                        {
                            let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
                            *lock = cmd;
                        }

                        defmt::info!("Updated Control: {}, {}", cmd.v, cmd.omega);
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
