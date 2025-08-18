use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::packet::{CmdLegacyPacketMix, CmdLegacyPacketF32, CmdTeleopPacketMix};
use crate::uart::SharedUart;
use crate::{debug_v1, debug_v2, debug_warn, debug_error};

use heapless::Vec;

const PI: f32 = core::f32::consts::PI;
const SAMPLE_MS: u64 = 20;

#[cfg(feature = "three-pi")]
const MIN_RPM: f32 = -2388.0;
const MAX_RPM: f32 = 2388.0;

#[cfg(feature = "zumo")]
mod robot_constants {
    use super::SAMPLE_MS;
    pub const GEAR_RATIO: f32 = 75.81; // Zumo gear ratio
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0/4.0;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0; // Zumo motor direction is inverse
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const MIN_RPM: f32 = -95.5; // Experimentally determined
    pub const MAX_RPM: f32 = 95.5;
    pub const Ts : f32 = SAMPLE_MS as f32 / 1000.0;
    pub const kp: f32 = 0.6;
    pub const ki: f32 = 8.0;
    pub const kd: f32 = 0.005;
    pub const Kaw: f32 = 0.1;
    pub const Kpwm: f32 = 0.1;
    pub const u_0: f32 = 0.0;
}

#[cfg(feature = "three-pi")]
mod robot_constants {
    use super::SAMPLE_MS;
    pub const GEAR_RATIO: f32 = 29.86; // Adjusted for 3Pi gear ratio
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0/4.0; 
    pub const WHEEL_BASE: f32 = 0.0842; 
    pub const WHEEL_RADIUS: f32 = 0.016;
    pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
    pub const MIN_RPM: f32 = -119.4;
    pub const MAX_RPM: f32 = 119.4;
    pub const Ts : f32 = SAMPLE_MS as f32 / 1000.0;
    pub const kp: f32 = 0.5; // Proportional gain
    pub const ki: f32 = 0.2; // Integral gain
    pub const kd: f32 = 0.003; // Derivative gain
    pub const Kaw: f32 = 0.1; // Anti-windup gain
    pub const Kpwm: f32 = 0.1; // PWM gain
    pub const u_0: f32 = 0.0;
}

#[cfg(not(any(feature = "zumo", feature = "three-pi")))]
mod robot_constants {
    use super::SAMPLE_MS;
    pub const GEAR_RATIO: f32 = 75.81;
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0/4.0;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const MIN_RPM: f32 = -95.5; //experimentally determined
    pub const MAX_RPM: f32 = 95.5;
    pub const Ts : f32 = SAMPLE_MS as f32 / 1000.0;
    pub const kp: f32 = 0.6; // Proportional gain
    pub const ki: f32 = 1.0; // Integral gain
    pub const kd: f32 = 0.005; // Derivative gain
    pub const Kaw: f32 = 0.1; // Anti-windup gain
    pub const Kpwm: f32 = 0.1; // PWM gain
    pub const u_0: f32 = 0.0;
}

use robot_constants::*;

pub fn get_gear_ratio() -> f32 {
    GEAR_RATIO
}

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

        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
pub async fn motor_control_task(
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
) {
    debug_warn!("Motor control task starting - this message always appears");
    debug_v1!("Verbosity level 1 enabled - basic control info will be shown");
    debug_v2!("Verbosity level 2 enabled - detailed debug info will be shown");

    let mut e_k_left = 0.0;
    let mut e_k1_left = 0.0;
    let mut e_k2_left = 0.0;
    let mut e_k_right = 0.0;
    let mut e_k1_right = 0.0;
    let mut e_k2_right = 0.0;

    let mut u_k_left = 0.0;
    let mut u_k1_left = 0.0;
    let mut u_k_right = 0.0;
    let mut u_k1_right = 0.0;
    let mut u_sat_left = 0.0;
    let mut u_sat_right = 0.0;
    
    let mut I_left = 0.0;
    let mut I_right = 0.0;

    let alpha = 0.3;
    let mut filtered_rpm_left = 0.0;
    let mut filtered_rpm_right = 0.0;

    loop {
        let cmd = CONTROL_CMD_UNICYCLE.lock().await.clone();
        
        debug_v1!("Raw commands: v={}, omega={}", cmd.v, cmd.omega);
        
        let v = cmd.v;
        let omega = cmd.omega;

        let v_left = v - omega * WHEEL_BASE / 2.0;
        let v_right = v + omega * WHEEL_BASE / 2.0;

        let rpm_left_target = v_left / (2.0 * PI * WHEEL_RADIUS) * 60.0;
        let rpm_right_target = v_right /(2.0 * PI * WHEEL_RADIUS) * 60.0;
        

        let (rpm_left_now, rpm_right_now) = 
            get_rpms(left_counter, right_counter, ENCODER_CPR, SAMPLE_MS).await;

        filtered_rpm_left = alpha * rpm_left_now + (1.0 - alpha) * filtered_rpm_left;
        filtered_rpm_right = alpha * rpm_right_now + (1.0 - alpha) * filtered_rpm_right;

        let deadband = 2.0;
        e_k_left = rpm_left_target - filtered_rpm_left;
        if e_k_left.abs() < deadband {
            e_k_left = 0.0;
        }
        
        e_k_right = rpm_right_target - filtered_rpm_right;
        if e_k_right.abs() < deadband {
            e_k_right = 0.0;
        }

        debug_v2!("Left RPM - Raw: {}, Filtered: {}, Target: {}, Error: {}", rpm_left_now, filtered_rpm_left, rpm_left_target, e_k_left);
        debug_v2!("Right RPM - Raw: {}, Filtered: {}, Target: {}, Error: {}", rpm_right_now, filtered_rpm_right, rpm_right_target, e_k_right);

        let P_inc_left = kp * (e_k_left - e_k1_left);
        let I_inc_left = ki * Ts * e_k_left; 
        let D_inc_left = kd * (e_k_left - e_k1_left) / Ts; 
        I_left += I_inc_left;
        u_k_left = u_k1_left + P_inc_left + I_inc_left + D_inc_left;

        let P_inc_right = kp * (e_k_right - e_k1_right);
        let I_inc_right = ki * Ts * e_k_right; 
        let D_inc_right = kd * (e_k_right - e_k1_right) / Ts;
        I_right += I_inc_right;
        u_k_right = u_k1_right + P_inc_right + I_inc_right + D_inc_right;

        e_k1_left = e_k_left;
        e_k1_right = e_k_right;

        u_k1_left = u_k_left;
        u_k1_right = u_k_right;

        debug_v2!("Left - P_inc: {}, I_total: {}, I_inc: {}, D_inc: {}, u_k: {}", P_inc_left, I_left, I_inc_left, D_inc_left, u_k_left);
        debug_v2!("Right - P_inc: {}, I_total: {}, I_inc: {}, D_inc: {}, u_k: {}", P_inc_right, I_right, I_inc_right, D_inc_right, u_k_right);
    
        u_sat_left = (u_k_left).clamp(-1.0f32, 1.0f32);
        u_sat_right = (u_k_right).clamp(-1.0f32, 1.0f32);

        if v_left.abs() < 0.01 && v_right.abs() < 0.01 {
            I_left = 0.0;
            I_right = 0.0;
            u_sat_left = 0.0;
            u_sat_right = 0.0;
            e_k1_left = 0.0;
            e_k1_right = 0.0;
            e_k_left = 0.0;
            e_k_right = 0.0;
            u_k_left = 0.0;
            u_k_right = 0.0;
            debug_v2!("v left and right close to zero - resetting integrator, control values, and derivative history");
        }

        let max_integrator = 0.5;
        if I_left.abs() > max_integrator {
            I_left = I_left.signum() * max_integrator;
            debug_v2!("Left integrator clamped to prevent windup");
        }
        if I_right.abs() > max_integrator {
            I_right = I_right.signum() * max_integrator;
            debug_v2!("Right integrator clamped to prevent windup");
        }

        #[cfg(feature = "three-pi")]
        let duty_left = (u_sat_left).clamp(-1.0, 1.0);
        #[cfg(feature = "three-pi")]
        {
            if u_sat_left.abs() > 0.95 {
            I_left = 0.0;
            u_k_left = u_sat_left;
            e_k1_left = e_k_left;

            debug_warn!("Left motor saturated - resetting integrator, control value, and derivative history");
            }
        }

        #[cfg(feature = "three-pi")]
        let duty_right = (u_sat_right).clamp(-1.0, 1.0);
        
        #[cfg(feature = "three-pi")]
        {
            if u_sat_right.abs() > 0.95 {
            I_right = 0.0;
            u_k_right = u_sat_right;
            e_k1_right = e_k_right;
            debug_warn!("Right motor saturated - resetting integrator, control value, and derivative history");
            }
        }
        #[cfg(not(feature = "three-pi"))]
        let duty_left = u_k_left.clamp(-1.0, 1.0);
        #[cfg(not(feature = "three-pi"))]
        let duty_right = u_k_right.clamp(-1.0, 1.0);

        
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
                            let speed = 0.4 * (libm::expf(2.0 * normalized) - 1.0) / (libm::expf(2.0) - 1.0);
                            sign * speed
                    
                        };
                        let cmd = ControlCommandUnicycle {
                            v: v,
                            omega: {
                                let raw_omega = pkt.steering_angle * PI / (180.0 * 0.1);
                                let sign = if raw_omega >= 0.0 { 1.0 } else { -1.0 };
                                let abs_omega = raw_omega.abs();
                                
                                let max_omega = 200.0;
                                let scaled_omega = max_omega * (libm::expf(abs_omega / max_omega) - 1.0) / (libm::expf(1.0) - 1.0);
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
