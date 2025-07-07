use crate::uart_irq::try_read_byte;
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
            return None;
        }

        Some(Self {
            header: u8::from_le_bytes([data[1]]),
            left_pwm_duty: u16::from_le_bytes([data[2], data[3]]),
            right_pwm_duty: u16::from_le_bytes([data[4], data[5]]),
            left_direction: u16::from_le_bytes([data[6], data[7]]),
            right_direction: u16::from_le_bytes([data[8], data[9]]),
        })
    }
}

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
            return None;
        }

        Some(Self {
            header: u8::from_le_bytes([data[1]]),
            left_pwm_duty: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            right_pwm_duty: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            left_direction: f32::from_le_bytes([data[10], data[11], data[12], data[13]]),
            right_direction: f32::from_le_bytes([data[14], data[15], data[16], data[17]]),
        })
    }
}

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
            return None;
        }

        Some(Self {
            header: u8::from_le_bytes([data[1]]),
            left_pwm_duty: f32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            right_pwm_duty: f32::from_le_bytes([data[6], data[7], data[8], data[9]]),
            left_direction: u16::from_le_bytes([data[10], data[11]]),
            right_direction: u16::from_le_bytes([data[12], data[13]]),
        })
    }
}

/// header of the bags
const SYNC_BYTE: u8 = 0x09;

pub enum CmdPacket {
    U16(CmdLegacyPacketU16),
    F32(CmdLegacyPacketF32),
    Mix(CmdLegacyPacketMix),
}

pub fn try_read_packet() -> Option<CmdPacket> {
    let mut buffer: Vec<u8, 20> = Vec::new();

    loop {
        let byte = try_read_byte()?;
        if byte == SYNC_BYTE {
            buffer.push(byte).ok()?;
            break;
        }
    }

    while buffer.len() < 2 {
        let byte = try_read_byte()?;
        buffer.push(byte).ok()?;
    }

    let header = buffer[1];

    let expected_len = match header {
        0x3c => 10, // U16
        0x3d => 18, // F32
        0x3e => 14, // Mix
        _ => return None,
    };

    while buffer.len() < expected_len {
        let byte = try_read_byte()?;
        buffer.push(byte).ok()?;
    }

    match header {
        0x3c => CmdLegacyPacketU16::from_bytes(&buffer).map(CmdPacket::U16), // U16
        0x3d => CmdLegacyPacketF32::from_bytes(&buffer).map(CmdPacket::F32), // F32
        0x3e => CmdLegacyPacketMix::from_bytes(&buffer).map(CmdPacket::Mix), // Mix
        _ => None,
    }
}
