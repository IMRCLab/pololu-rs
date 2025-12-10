use crate::robotstate::LogSnapshot;
use defmt::{info, warn};
use heapless::Vec;

#[derive(Debug)]
pub struct CmdLegacyPacketU16 {
    pub header: u8,
    pub left_pwm_duty: u16,
    pub right_pwm_duty: u16,
    pub left_direction: u16,
    pub right_direction: u16,
}

impl CmdLegacyPacketU16 {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 10 {
            warn!("Invalid U16 packet length");
            return None;
        }
        Some(Self {
            header: data[1],
            left_pwm_duty: u16::from_le_bytes([data[2], data[3]]),
            right_pwm_duty: u16::from_le_bytes([data[4], data[5]]),
            left_direction: u16::from_le_bytes([data[6], data[7]]),
            right_direction: u16::from_le_bytes([data[8], data[9]]),
        })
    }
}

#[derive(Debug)]
pub struct CmdLegacyPacketF32 {
    pub header: u8,
    pub left_pwm_duty: f32,
    pub right_pwm_duty: f32,
    pub left_direction: f32,
    pub right_direction: f32,
}

impl CmdLegacyPacketF32 {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 18 {
            warn!("Invalid F32 packet length");
            return None;
        }
        Some(Self {
            header: data[1],
            left_pwm_duty: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            right_pwm_duty: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            left_direction: f32::from_le_bytes([data[10], data[11], data[12], data[13]]),
            right_direction: f32::from_le_bytes([data[14], data[15], data[16], data[17]]),
        })
    }
}

#[derive(Debug)]
pub struct CmdLegacyPacketMix {
    pub header: u8,
    pub left_pwm_duty: f32,
    pub right_pwm_duty: f32,
    pub left_direction: u16,
    pub right_direction: u16,
}

impl CmdLegacyPacketMix {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 14 {
            warn!("Invalid F32 packet length");
            return None;
        }
        Some(Self {
            header: data[1],
            left_pwm_duty: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            right_pwm_duty: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            left_direction: u16::from_le_bytes([data[10], data[11]]),
            right_direction: u16::from_le_bytes([data[12], data[13]]),
        })
    }
}

#[derive(Debug)]
pub struct CmdLegacyPacketTeleop {
    pub header: u8,
    pub not_assigned1: f32,
    pub not_assigned2: f32,
    pub angular_vel: f32,
    pub linear_vel: u16,
}

impl CmdLegacyPacketTeleop {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 16 {
            warn!("Invalid Teleop packet length");
            return None;
        }
        Some(Self {
            header: data[1],
            not_assigned1: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            not_assigned2: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            angular_vel: f32::from_le_bytes([data[10], data[11], data[12], data[13]]),
            linear_vel: u16::from_le_bytes([data[14], data[15]]),
        })
    }
}

#[derive(Debug)]
pub struct CmdTeleopPacketMix {
    pub header: u8,
    pub linear_velocity: f32,
    pub steering_angle: f32,
}

impl CmdTeleopPacketMix {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 9 {
            warn!("Invalid Teleop packet length");
            return None;
        }
        Some(Self {
            header: data[0],
            linear_velocity: f32::from_le_bytes([data[1], data[2], data[3], data[4]]),
            steering_angle: f32::from_le_bytes([data[5], data[6], data[7], data[8]]),
        })
    }
}

#[derive(Debug)]
pub struct MocapPosesPacketF32Test {
    pub header: u8,
    pub robot_id: u8,
    pub pos_x: f32,
    pub pos_y: f32,
    pub pos_z: f32,
    pub quat: u32,
}

impl MocapPosesPacketF32Test {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 19 {
            warn!("Invalid Poses packet length");
            return None;
        }
        info!("buffer: {}", data);
        Some(Self {
            header: data[1],
            robot_id: data[2],
            pos_x: f32::from_le_bytes([data[3], data[4], data[5], data[6]]),
            pos_y: f32::from_le_bytes([data[7], data[8], data[9], data[10]]),
            pos_z: f32::from_le_bytes([data[11], data[12], data[13], data[14]]),
            quat: u32::from_le_bytes([data[15], data[16], data[17], data[18]]),
        })
    }
}

#[derive(Debug)]
pub struct MocapPosesPacketF32 {
    pub header: u8,
    pub robot_id: u8,
    pub pos_x: f32,
    pub pos_y: f32,
    pub pos_z: f32,
    pub quat: u32,
}

impl MocapPosesPacketF32 {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() != 18 {
            warn!("Invalid Poses packet length");
            return None;
        }
        info!("buffer: {}", data);
        Some(Self {
            header: data[0],
            robot_id: data[1],
            pos_x: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            pos_y: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            pos_z: f32::from_le_bytes([data[10], data[11], data[12], data[13]]),
            quat: u32::from_le_bytes([data[14], data[15], data[16], data[17]]),
        })
    }
}

//loopback packet for sending out robot state data
//
// Structure:
//   - header: 0xA2 (log snapshot identifier)
//   - robot_id: u8
//   - log_snapshot: LogSnapshot (serialized)
//
// Wire format (compact with i):
//   [len:1] [header:1] [robot_id:1] [t_ms:4] [17×i16:34] = 41 bytes total
//
// Wire format (f32):
//   [len:1] [header:1] [robot_id:1] [t_ms:4] [17×f32:68] = 75 bytes total
//
#[derive(Debug)]
pub struct StateLoopBackPacketF32 {
    pub header: u8,
    pub robot_id: u8,
    pub log_snapshot: LogSnapshot,
}

impl StateLoopBackPacketF32 {
    pub fn new(robot_id: u8, log_snapshot: LogSnapshot) -> Self {
        Self {
            header: 0xA2,
            robot_id,
            log_snapshot,
        }
    }

    /// Full precision f32 format (75 bytes total)
    pub fn to_bytes(&self) -> Vec<u8, 80> {
        let mut v: Vec<u8, 80> = Vec::new();
        let payload = self.log_snapshot.to_bytes_f32();
        // len = header(1) + robot_id(1) + payload
        v.push((2 + payload.len()) as u8).ok();
        v.push(self.header).ok();
        v.push(self.robot_id).ok();
        v.extend_from_slice(&payload).ok();
        v
    }

    /// Compact i16 fixed-point format (41 bytes total)
    /// Returns Vec<u8, 64> for direct use with UART_TX_CHANNEL
    pub fn to_bytes_compact(&self) -> Vec<u8, 64> {
        let mut v: Vec<u8, 64> = Vec::new();
        let payload = self.log_snapshot.to_bytes_compact();
        // len = header(1) + robot_id(1) + payload
        v.push((2 + payload.len()) as u8).ok();
        v.push(self.header).ok();
        v.push(self.robot_id).ok();
        v.extend_from_slice(&payload).ok();
        v
    }
}
