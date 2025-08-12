use defmt::warn;

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
