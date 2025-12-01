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
pub static UART_TX_CHANNEL: Channel<ThreadModeRawMutex, Vec<u8, 64>, 4> = Channel::new();
// RX: Low level UART HW Task -> decode Task
pub static UART_RX_CHANNEL: Channel<ThreadModeRawMutex, u8, 64> = Channel::new();

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
                    defmt::info!("UART TX: sent {} bytes", tx.len());
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

pub fn pack_state_loopback(pkt: &StateLoopBackPacketF32) -> Vec<u8, 64> {
    let mut v: Vec<u8, 64> = Vec::new();

    // total length = header(1) + robot_id(1) + 10 floats * 4 = 1 + 1 + 40 = 42
    v.push(1 + 1 + 4 * 10).ok();
    v.push(pkt.header).ok();
    v.push(pkt.robot_id).ok();

    for f in [
        pkt.pos_x, pkt.pos_y, pkt.pos_z, pkt.vel_x, pkt.vel_y, pkt.vel_z, pkt.qw, pkt.qx, pkt.qy,
        pkt.qz,
    ] {
        v.extend_from_slice(&f.to_le_bytes()).ok();
    }

    v
}

pub fn uart_send_state_loopback(pkt: &StateLoopBackPacketF32) {
    let data = pack_state_loopback(pkt);
    UART_TX_CHANNEL.try_send(data).expect("UART TX queue full");
}

// ==================== Robot State Sending Task ====================
pub static ROBOT_STATE_CH: Channel<ThreadModeRawMutex, StateLoopBackPacketF32, 4> = Channel::new();

pub fn update_robot_state(state: StateLoopBackPacketF32) {
    while ROBOT_STATE_CH.try_receive().is_ok() {}

    let _ = ROBOT_STATE_CH.try_send(state);
}

#[embassy_executor::task]
pub async fn state_loopback_task() {
    defmt::info!("state_loopback_task started");

    loop {
        let state: StateLoopBackPacketF32 = ROBOT_STATE_CH.receive().await;

        let pkt = StateLoopBackPacketF32 {
            header: state.header,
            robot_id: state.robot_id,
            pos_x: state.pos_x,
            pos_y: state.pos_y,
            pos_z: state.pos_z,
            vel_x: state.vel_x,
            vel_y: state.vel_y,
            vel_z: state.vel_z,
            qw: state.qw,
            qx: state.qx,
            qy: state.qy,
            qz: state.qz,
        };

        // 3. Send via UART
        uart_send_state_loopback(&pkt);
    }
}
