use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

use crate::encoder::get_rpms;
use crate::motor::MotorController;
use crate::packet::{CmdLegacyPacketMix, CmdLegacyPacketF32, CmdTeleopPacketMix};
use crate::uart::SharedUart;
// Import the global verbosity macros
use crate::{debug_v1, debug_v2, debug_warn, debug_error};

use heapless::Vec;

/// Robot-specific constants - conditionally compiled
const PI: f32 = core::f32::consts::PI;
const SAMPLE_MS: u64 = 20; // Sample Period = 20 ms

#[cfg(feature = "three-pi")]
// minum and max rpm values for the motors
const MIN_RPM: f32 = -2388.0; // Minimum RPM value from assumption 4m/s max speed, wheel radius 0.016m -> rmp_max = 2388
const MAX_RPM: f32 = 2388.0;  // Maximum RPM value
// Controler constants


// Zumo robot constants
#[cfg(feature = "zumo")]
mod robot_constants {
    use super::SAMPLE_MS; // Import from parent scope
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
    use super::SAMPLE_MS; // Import from parent scope
    pub const GEAR_RATIO: f32 = 29.86;
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0; // = 358.32
    pub const WHEEL_BASE: f32 = 0.0842;
    pub const WHEEL_RADIUS: f32 = 0.016;
    pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;   // 3Pi has normal motor directions
    pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
    pub const MIN_RPM: f32 = -2388.0; // Minimum RPM value from assumption 4m/s max speed, wheel radius 0.016m -> rmp_max = 2388
    pub const MAX_RPM: f32 = 2388.0;  // Maximum RPM value
        //---- Constants for RPM control ----
    pub const Ts : f32 = SAMPLE_MS as f32 / 1000.0; // Sample time in seconds
    pub const kp: f32 = 60.0; // PWM per rpm
    pub const ki: f32 = 1.0; // PWM per (rpm*s)
    pub const Kaw: f32 = 0.1; // anti windup gain
    pub const Kpwm: f32 = 0.1; // feed forward slow PWM/rpm
    pub const u_0: f32 = 0.0; // static friction Vorsteuerung not used yet
}

// Default values for testing when no features are active
#[cfg(not(any(feature = "zumo", feature = "three-pi")))]
mod robot_constants {
    use super::SAMPLE_MS; // Import from parent scope
    // Default values for testing
    pub const GEAR_RATIO: f32 = 75.81; // Default to Zumo values for testing
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;  // Default to Zumo behavior
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const MIN_RPM: f32 = -2388.0; // Minimum RPM value from assumption 4m/s max speed, wheel radius 0.016m -> rmp_max = 2388
    pub const MAX_RPM: f32 = 2388.0;  // Maximum RPM value
    //---- Constants for RPM control ----
    pub const Ts : f32 = SAMPLE_MS as f32 / 1000.0; // Sample time in seconds
    pub const kp: f32 = 10.0; // PWM per rpm
    pub const ki: f32 = 1.0; // PWM per (rpm*s)
    pub const Kaw: f32 = 0.1; // anti windup gain
    pub const Kpwm: f32 = 0.1; // feed forward slow PWM/rpm
    pub const u_0: f32 = 0.0; // static friction Vorsteuerung
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

//get conversion counts to rpm = delta_N/ (CPR * Ts) * 60.0
// fn counts_to_rpm(dN: i32) -> f32 { //counted ticks to rpm
//     (dN as f32) / (ENCODER_CPR * Ts) * 60.0
// }
//use get_rpms from crate encoder instead

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
    // Verbosity level demonstration at startup
    debug_warn!("Motor control task starting - this message always appears");
    debug_v1!("Verbosity level 1 enabled - basic control info will be shown");
    debug_v2!("Verbosity level 2 enabled - detailed debug info will be shown");

    //errors
    let mut e_k_left = 0.0; // Error at k
    let mut e_k1_left = 0.0; // Error at k-1
    let mut e_k_right = 0.0; // Error at k
    let mut e_k1_right = 0.0; // Error at k-1

    //u
    let mut u_k_left = 0.0;
    let mut u_k1_left = 0.0;
    let mut u_k_right = 0.0;
    let mut u_k1_right = 0.0;
    let mut u_sat_left = 0.0;
    let mut u_sat_right = 0.0;
    
    // Integrator states (separate from u_k for proper anti-windup)
    let mut I_left = 0.0;
    let mut I_right = 0.0;

    //complementary filter
    let alpha = 0.2; // Increased from 0.1 for better responsiveness
    let mut filtered_rpm_left = 0.0;
    let mut filtered_rpm_right = 0.0;

    loop {
        //calc target rpm
        let cmd = CONTROL_CMD_UNICYCLE.lock().await.clone(); //v [m/s] , omega [rad/s]
        
        // Robust zero value recognition with dead-zone handling
        const VELOCITY_DEADZONE: f32 = 1e-6; // 0.000001 m/s threshold
        const OMEGA_DEADZONE: f32 = 1e-4;    // 0.0001 rad/s threshold
        
        let v_filtered = if cmd.v.abs() < VELOCITY_DEADZONE { 0.0 } else { cmd.v };
        let omega_filtered = if cmd.omega.abs() < OMEGA_DEADZONE { 0.0 } else { cmd.omega };
        
        let v_left = v_filtered - omega_filtered * WHEEL_BASE / 2.0; // [m/s]
        let v_right = v_filtered + omega_filtered * WHEEL_BASE / 2.0; //[m/s]

        let mut rpm_left_target = v_left / (2.0 * PI * WHEEL_RADIUS) * 60.0; //conversion to rpm
        let mut rpm_right_target = v_right /(2.0 * PI * WHEEL_RADIUS) * 60.0;
        
        // Additional RPM-level zero detection to prevent tiny drift values
        const RPM_DEADZONE: f32 = 0.5; // should be unnotable movement.
        if rpm_left_target.abs() < RPM_DEADZONE { rpm_left_target = 0.0; }
        if rpm_right_target.abs() < RPM_DEADZONE { rpm_right_target = 0.0; }

        //control loop
        let (rpm_left_now, rpm_right_now) = 
            get_rpms(left_counter, right_counter, ENCODER_CPR, SAMPLE_MS).await;

        // Filter the RPM values with complementary filter for noise reduction
        filtered_rpm_left = alpha * rpm_left_now + (1.0 - alpha) * filtered_rpm_left;
        filtered_rpm_right = alpha * rpm_right_now + (1.0 - alpha) * filtered_rpm_right;

        // Calculate error using filtered RPM values
        e_k_left  = rpm_left_target - filtered_rpm_left; //k error
        e_k_right = rpm_right_target - filtered_rpm_right; //k error

        // Debug logging for zero detection - Level 2
        if (cmd.v.abs() < VELOCITY_DEADZONE && cmd.v != 0.0) || (cmd.omega.abs() < OMEGA_DEADZONE && cmd.omega != 0.0) {
            debug_v2!("Zero detection: cmd.v={}, cmd.omega={} -> filtered v={}, omega={}", 
                      cmd.v, cmd.omega, v_filtered, omega_filtered);
        }

        // RPM values and targets - Level 1
        debug_v1!("Left RPM - Raw: {}, Filtered: {}, Target: {}, Error: {}", rpm_left_now, filtered_rpm_left, rpm_left_target, e_k_left);
        debug_v1!("Right RPM - Raw: {}, Filtered: {}, Target: {}, Error: {}", rpm_right_now, filtered_rpm_right, rpm_right_target, e_k_right);

        // PI Controller + awr
        // Left wheel
        let P_inc_left = kp * (e_k_left - e_k1_left);
        let I_inc_left = ki * Ts * e_k_left + Kaw * (u_sat_left - u_k1_left); // Anti-windup uses previous u_sat
        I_left += I_inc_left; // Accumulate integrator
        u_k_left = u_k1_left + P_inc_left + I_inc_left;

        // Right wheel  
        let P_inc_right = kp * (e_k_right - e_k1_right);
        let I_inc_right = ki * Ts * e_k_right + Kaw * (u_sat_right - u_k1_right); // Anti-windup uses previous u_sat
        I_right += I_inc_right; // Accumulate integrator
        u_k_right = u_k1_right + P_inc_right + I_inc_right;

        //enforce robustness for zero targets
        if rpm_left_target == 0.0 && rpm_right_target == 0.0 {
            I_left = 0.0;  
            I_right = 0.0;
            u_k_left = 0.0;
            u_k_right = 0.0;
            debug_v2!("Zero target detected - resetting integrators");
        }

        // Update previous errors
        e_k1_left = e_k_left; //k-1 error left
        e_k1_right = e_k_right; //k-1 error right

        // Controller internals - Level 2
        debug_v2!("Left - P_inc: {}, I_total: {}, I_inc: {}, u_k: {}", P_inc_left, I_left, I_inc_left, u_k_left);
        debug_v2!("Right - P_inc: {}, I_total: {}, I_inc: {}, u_k: {}", P_inc_right, I_right, I_inc_right, u_k_right);
        //Anti-Windup
        #[cfg(feature = "three-pi")]
        {
            u_sat_left = (u_k_left/MAX_RPM).clamp(MIN_RPM / 2.0, MAX_RPM / 2.0); //actual max values are experimental
            u_sat_right = (u_k_right/MAX_RPM).clamp(MIN_RPM / 2.0, MAX_RPM / 2.0);
        }
        #[cfg(not(feature = "three-pi"))]
        {
            //Why 10000? was default value in the original code
            u_sat_left = (u_k_left / 10000.0).clamp(-1.0, 1.0);
            u_sat_right = (u_k_right / 10000.0).clamp(-1.0, 1.0);
        }

        // Saturated control values - Level 2
        debug_v2!("Saturated Control Values: u_sat_left: {}, u_sat_right: {}", u_sat_left, u_sat_right);
        // Update the previous control values
        u_k1_left = u_k_left;
        u_k1_right = u_k_right;

        // Calculate the duty cycle for the motors:
        // For "three-pi", scale by dividing the saturated control by MAX_RPM to get a value in [-1.0, 1.0].
        // For other robots, use the already normalized value in u_sat_left/u_sat_right.
        #[cfg(feature = "three-pi")]
        //let duty_left = (u_sat_left / MAX_RPM).clamp(-1.0, 1.0);
        let duty_left = (u_sat_left).clamp(-1.0, 1.0);  //scale to pwm top

        #[cfg(feature = "three-pi")]
        //let duty_right = (u_sat_right / MAX_RPM).clamp(-1.0, 1.0);
        let duty_right = (u_sat_right).clamp(-1.0, 1.0); // scale to pwm top


        #[cfg(not(feature = "three-pi"))]
        let duty_left = u_sat_left;
        #[cfg(not(feature = "three-pi"))]
        let duty_right = u_sat_right; 

        // Override duty cycle to zero if zero targets detected
        let (final_duty_left, final_duty_right) = if rpm_left_target == 0.0 && rpm_right_target == 0.0 {
            debug_v2!("Zero targets detected - forcing duty cycle to zero");
            (0.0, 0.0)
        } else {
            (duty_left, duty_right)
        };

        // Final duty cycle values - Level 2
        debug_v2!("Duty Cycle: Left {}, Right {}", final_duty_left, final_duty_right);
        
        motor.set_speed(
            final_duty_left * MOTOR_DIRECTION_LEFT, 
            final_duty_right * MOTOR_DIRECTION_RIGHT
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
                            
                            //deadzone is handled in ROS node
                            let normalized = ((abs_raw - 1000.0) / 19000.0).clamp(0.0, 1.0);
                            let speed = 0.2 * (libm::expf(2.0 * normalized) - 1.0) / (libm::expf(2.0) - 1.0); //set max speed to 0.1
                            sign * speed  // Apply original sign for forward/reverse
                    
                        };
                        let cmd = ControlCommandUnicycle {
                            v: v, // scale 0 - 20000 "thrust" to 0 - 0.05 m/s with exponential scaling
                            // Nonlinear scaling for omega similar to v
                            omega: {
                                let raw_omega = pkt.steering_angle * PI / (180.0 * 0.1); // Base scaling for 0.2s turn time
                                let sign = if raw_omega >= 0.0 { 1.0 } else { -1.0 };
                                let abs_omega = raw_omega.abs();
                                
                                // Apply exponential scaling for better control feel
                                let max_omega = 200.0; // Max angular velocity in rad/s
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
