use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::Vec;

use crate::read_robot_config_from_sd::RobotConfig;
use crate::sdlog::{SD_COMMAND_CHANNEL, SD_RESULT_CHANNEL, SdCommand};
use crate::uart::UART_TX_CHANNEL;

/// Global robot configuration that can be updated at runtime
pub static ROBOT_CONFIG: Mutex<ThreadModeRawMutex, Option<RobotConfig>> = Mutex::new(None);

/// Command IDs for special operations
pub const CMD_SAVE_CONFIG_TO_SD: u8 = 0xFF;

/// Request saving config to SD card and wait for result
pub async fn request_save_config_to_sd() -> bool {
    // Get the current config
    let config = match get_robot_config() {
        Some(c) => c,
        None => return false,
    };

    // Send command to SD logger task
    SD_COMMAND_CHANNEL.send(SdCommand::SaveConfig(config)).await;

    // Wait for result
    SD_RESULT_CHANNEL.receive().await
}

/// Initialize the global robot config
pub fn init_robot_config(config: RobotConfig) {
    if let Ok(mut guard) = ROBOT_CONFIG.try_lock() {
        *guard = Some(config);
    }
}

/// Get a copy of the current robot config
pub fn get_robot_config() -> Option<RobotConfig> {
    if let Ok(guard) = ROBOT_CONFIG.try_lock() {
        *guard
    } else {
        None
    }
}

/// Packet for sending a single parameter via UART
pub struct ParameterPacket {
    pub length: u8,
    pub header: u8,
    pub param_id: u8,
    pub value: f32,
    pub checksum: u16,
}

//TODO: unify the package types on the rust side and get a better method of verifying that a certain type of package arrived on the robot
impl ParameterPacket {
    pub fn new(param_id: u8, value: f32) -> Self {
        let mut packet = Self {
            length: 22,
            header: 0x3C,
            param_id,
            value,
            checksum: 0,
        };
        packet.checksum = packet.calculate_checksum();
        packet
    }

    //crc implementation
    fn calculate_checksum(&self) -> u16 {
        let value_bytes = self.value.to_le_bytes();
        let sum = self.length as u16
            + self.header as u16
            + self.param_id as u16
            + value_bytes[0] as u16
            + value_bytes[1] as u16
            + value_bytes[2] as u16
            + value_bytes[3] as u16;
        sum
    }

    pub fn to_bytes(&self) -> Vec<u8, 64> {
        let mut bytes = Vec::new();
        bytes.push(self.length).ok();
        bytes.push(self.header).ok();
        bytes.push(self.param_id).ok();

        let value_bytes = self.value.to_le_bytes();
        for byte in value_bytes {
            bytes.push(byte).ok();
        }

        let checksum_bytes = self.checksum.to_le_bytes();
        bytes.push(checksum_bytes[0]).ok();
        bytes.push(checksum_bytes[1]).ok();

        bytes
    }
}

//send a single parameter to the dongle out via UART
pub async fn send_parameter(param_id: u8, value: f32) -> Result<(), ()> {
    let packet = ParameterPacket::new(param_id, value);
    let bytes = packet.to_bytes();

    let sender = UART_TX_CHANNEL.sender();
    sender.send(bytes).await;
    Ok(())
}

/// Send all robot parameters to the dongle (one-time function, no periodic updates)
pub async fn send_robot_parameters_to_dongle(config: &RobotConfig) -> Result<(), ()> {
    let parameters = [
        (0, config.robot_id as f32),
        (1, config.joystick_control_dt_ms as f32),
        (2, config.traj_following_dt_s),
        (3, config.wheel_radius),
        (4, config.wheel_base),
        (5, config.motor_direction_left),
        (6, config.motor_direction_right),
        (7, config.motor_max_duty_left),
        (8, config.motor_max_duty_right),
        (9, config.k_clip),
        (10, config.kp_inner),
        (11, config.ki_inner),
        (12, config.kd_inner),
        (13, config.kx_traj),
        (14, config.ky_traj),
        (15, config.ktheta_traj),
        (16, config.gear_ratio),
        (17, config.encoder_cpr),
        (18, config.max_speed),
        (19, config.max_omega),
        (20, config.wheel_max),
    ];

    let mut success_count = 0;

    for (param_id, value) in parameters {
        match send_parameter(param_id, value).await {
            Ok(()) => success_count += 1,
            Err(()) => {}
        }

        // Small delay between parameter sends to avoid overwhelming the channel
        embassy_time::Timer::after_millis(10).await;
    }

    if success_count == parameters.len() {
        Ok(())
    } else {
        Err(())
    }
}

/// Parameter write request packet from dongle (CRTP-style)
/// Format: [length, header, param_id, value (4 bytes f32), checksum (2 bytes)]
pub struct ParameterWriteRequest {
    pub param_id: u8,
    pub value: f32,
}

impl ParameterWriteRequest {
    /// Parse a parameter write request from raw bytes
    /// Expected format: [length, header(0x3C), param_id, value[4], checksum[2]]
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        // Check minimum length: length(1) + header(1) + param_id(1) + value(4) + checksum(2) = 9
        if data.len() < 9 {
            return None;
        }

        // Check header matches our parameter packet header
        if data[1] != 0x3C {
            return None;
        }

        let param_id = data[2];
        let value = f32::from_le_bytes([data[3], data[4], data[5], data[6]]);

        Some(Self { param_id, value })
    }
}

/// Update a single parameter in the global config and echo it back
pub async fn handle_parameter_write(param_id: u8, value: f32) {
    // Update the parameter in the global config
    if let Ok(mut guard) = ROBOT_CONFIG.try_lock() {
        if let Some(ref mut config) = *guard {
            update_config_parameter(config, param_id, value);
        }
    }

    // Echo the parameter back to the dongle
    let _ = send_parameter(param_id, value).await;
}

/// Update a specific parameter in the RobotConfig based on param_id
fn update_config_parameter(config: &mut RobotConfig, param_id: u8, value: f32) {
    match param_id {
        0 => config.robot_id = value as u8,
        1 => config.joystick_control_dt_ms = value as u64,
        2 => config.traj_following_dt_s = value,
        3 => config.wheel_radius = value,
        4 => config.wheel_base = value,
        5 => config.motor_direction_left = value,
        6 => config.motor_direction_right = value,
        7 => config.motor_max_duty_left = value,
        8 => config.motor_max_duty_right = value,
        9 => config.k_clip = value,
        10 => config.kp_inner = value,
        11 => config.ki_inner = value,
        12 => config.kd_inner = value,
        13 => config.kx_traj = value,
        14 => config.ky_traj = value,
        15 => config.ktheta_traj = value,
        16 => config.gear_ratio = value,
        17 => config.encoder_cpr = value,
        18 => config.max_speed = value,
        19 => config.max_omega = value,
        20 => config.wheel_max = value,
        _ => {} // Unknown parameter, ignore
    }
}

/// Command request packet from dongle
/// Format: [length=2, header=0x3C, command_id]
pub struct CommandRequest {
    pub command_id: u8,
}

impl CommandRequest {
    /// Parse a command request from raw bytes
    /// Expected format: [length=2, header(0x3C), command_id]
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 3 {
            return None;
        }

        // Check length byte = 2 and header = 0x3C
        if data[0] != 2 || data[1] != 0x3C {
            return None;
        }

        Some(Self {
            command_id: data[2],
        })
    }
}

/// Send an acknowledgement packet for a command
pub async fn send_command_ack(command_id: u8, success: bool) {
    let mut bytes: Vec<u8, 64> = Vec::new();
    bytes.push(3).ok(); // length
    bytes.push(0x3C).ok(); // header
    bytes.push(command_id).ok(); // echo command
    bytes.push(if success { 1 } else { 0 }).ok(); // status

    let sender = UART_TX_CHANNEL.sender();
    sender.send(bytes).await;
}
