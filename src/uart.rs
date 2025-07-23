use embassy_rp::uart::{Async, Uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::Vec;

use crate::packet::{CmdLegacyPacketF32, CmdLegacyPacketMix, CmdLegacyPacketU16};

pub type SharedUart<'a> = &'a Mutex<ThreadModeRawMutex, Uart<'a, Async>>;

#[embassy_executor::task]
pub async fn uart_receive_task(uart: SharedUart<'static>) {
    let mut buffer: Vec<u8, 32> = Vec::new();

    loop {
        let byte: u8 = {
            let mut uart = uart.lock().await;
            let mut b = [0u8; 1];
            match uart.read(&mut b).await {
                Ok(_) => b[0],
                Err(_) => continue,
            }
        };

        // The first byte ( seems to be the length of data packet)
        if buffer.is_empty() {
            // legal length of different packets
            if byte != 9 && byte != 13 && byte != 17 {
                continue;
            }
        }

        buffer.push(byte).ok();

        // here 0x3C is related to the channel and port num, to be modified if channel and port are changed
        if buffer.len() == 2 && buffer[1] != 0x3C {
            buffer.clear();
            continue;
        }

        let expected_len = buffer[0] as usize + 1;
        if buffer.len() == expected_len {
            match expected_len {
                10 => {
                    if let Some(pkt) = CmdLegacyPacketU16::from_bytes(&buffer) {
                        defmt::info!(
                            "U16 Packet: pwm L={} R={}, dir L={} R={}",
                            pkt.left_pwm_duty,
                            pkt.right_pwm_duty,
                            pkt.left_direction,
                            pkt.right_direction
                        );
                    } else {
                        defmt::warn!("Invalid U16 packet");
                    }
                }
                14 => {
                    if let Some(pkt) = CmdLegacyPacketMix::from_bytes(&buffer) {
                        defmt::info!(
                            "Mix Packet: pwm L={} R={}, dir L={} R={}",
                            pkt.left_pwm_duty,
                            pkt.right_pwm_duty,
                            pkt.left_direction,
                            pkt.right_direction
                        );
                    } else {
                        defmt::warn!("Invalid Mix packet");
                    }
                }
                18 => {
                    if let Some(pkt) = CmdLegacyPacketF32::from_bytes(&buffer) {
                        defmt::info!(
                            "F32 Packet: pwm L={} R={}, dir L={} R={}",
                            pkt.left_pwm_duty,
                            pkt.right_pwm_duty,
                            pkt.left_direction,
                            pkt.right_direction
                        );
                    } else {
                        defmt::warn!("Invalid F32 packet");
                    }
                }
                _ => {
                    defmt::warn!("Unsupported packet length: {}", expected_len);
                }
            }

            buffer.clear();
        }
    }
}
