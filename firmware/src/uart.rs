#[allow(unused_imports)]
use crate::packet::StateLoopBackPacketF32;
use embassy_futures::select::{Either, select};
use embassy_rp::uart::{Async, Uart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use heapless::Vec;

use crate::packet::{
    CmdLegacyPacketF32, CmdLegacyPacketMix, CmdLegacyPacketU16, MocapPosesPacketF32Test,
};

pub type SharedUart<'a> = &'a Mutex<ThreadModeRawMutex, Uart<'a, Async>>;

// ===================== Channels =====================
// TX: Other Task -> Low level UART HW Task
pub static UART_TX_CHANNEL: Channel<ThreadModeRawMutex, Vec<u8, 64>, 128> = Channel::new();
// RX: Low level UART HW Task -> decode Task
pub static UART_RX_CHANNEL: Channel<ThreadModeRawMutex, u8, 512> = Channel::new();

//
// ===================== Low level UART HW Task =====================
#[embassy_executor::task]
pub async fn uart_hw_task(uart: &'static Mutex<ThreadModeRawMutex, Uart<'static, Async>>) {
    // defmt::info!("uart_hw_task started");

    let mut rx_buf = [0u8; 1];

    loop {
        // 2 future：
        //   1) future 1 waits for TX channel
        //   2) future 2 waits for UART RX
        let tx_future = UART_TX_CHANNEL.receive();
        let rx_future = async {
            let mut u = uart.lock().await;
            let _ = u.read(&mut rx_buf).await;
            rx_buf[0]
        };

        match select(tx_future, rx_future).await {
            // ================ TX Ready ================
            Either::First(tx) => {
                let mut u = uart.lock().await;
                if let Err(_) = u.write(&tx).await {
                    defmt::error!("UART TX failed");
                } else {
                    // defmt::info!("UART TX: sent {} bytes", tx.len());
                }
            }

            // ================ RX Ready ================
            Either::Second(b) => {
                let _ = UART_RX_CHANNEL.send(b).await;
            }
        }
    }
}

// ===================== uart_receive_task =====================
#[embassy_executor::task]
pub async fn uart_receive_task() {
    let mut buffer: Vec<u8, 32> = Vec::new();

    defmt::info!("uart_receive_task started");

    loop {
        // Read 1 byte from UART_RX_Channel
        let byte: u8 = UART_RX_CHANNEL.receive().await;

        // The first byte ( seems to be the length of data packet)
        if buffer.is_empty() {
            // legal length of different packets
            if byte != 9 && byte != 13 && byte != 17 && byte != 18 {
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
        // let expected_len = 19;
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
                19 => {
                    if let Some(pkt) = MocapPosesPacketF32Test::from_bytes(&buffer) {
                        defmt::info!(
                            "Pose Packet: robot_id={}, PosX={} PosY={} Pos_Z={} Quat={}",
                            pkt.robot_id,
                            pkt.pos_x,
                            pkt.pos_y,
                            pkt.pos_z,
                            pkt.quat
                        );
                    } else {
                        defmt::warn!("Invalid Pose packet");
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

// ===================== TX Func =====================
pub fn uart_send(data: &[u8]) {
    let mut v: Vec<u8, 64> = Vec::new();
    v.extend_from_slice(data).ok();

    UART_TX_CHANNEL.try_send(v).ok();
}

