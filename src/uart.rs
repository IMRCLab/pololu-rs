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
use crate::parameter_sync::{
    CMD_SAVE_CONFIG_TO_SD, CommandRequest, ParameterWriteRequest, handle_parameter_write,
    request_save_config_to_sd, send_command_ack,
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

                match u.write(&tx).await {
                    Ok(_) => {
                        let _ = u.blocking_flush();
                    }
                    Err(_e) => {
                        defmt::error!("UART TX failed");
                    }
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

    loop {
        // Read 1 byte from UART_RX_Channel
        let byte: u8 = UART_RX_CHANNEL.receive().await;

        // The first byte is the length of data packet (excluding the length byte itself)
        if buffer.is_empty() {
            // legal length of different packets:
            // 2 = command packet, 8 = parameter packet, 9/13/17/18 = legacy packets
            if byte != 2 && byte != 8 && byte != 9 && byte != 13 && byte != 17 && byte != 18 {
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
                // Command packet: length(1) + header(1) + command_id(1) = 3
                3 => {
                    if let Some(cmd) = CommandRequest::from_bytes(&buffer) {
                        handle_command(cmd.command_id).await;
                    }
                }
                // Parameter write request: length(1) + header(1) + param_id(1) + value(4) + checksum(2) = 9
                9 => {
                    if let Some(req) = ParameterWriteRequest::from_bytes(&buffer) {
                        // Handle parameter write: update config and echo back
                        handle_parameter_write(req.param_id, req.value).await;
                    }
                }
                10 => {
                    let _ = CmdLegacyPacketU16::from_bytes(&buffer);
                }
                14 => {
                    let _ = CmdLegacyPacketMix::from_bytes(&buffer);
                }
                18 => {
                    let _ = CmdLegacyPacketF32::from_bytes(&buffer);
                }
                19 => {
                    let _ = MocapPosesPacketF32Test::from_bytes(&buffer);
                }
                _ => {}
            }

            buffer.clear();
        }
    }
}

/// Handle special commands from the dongle
async fn handle_command(command_id: u8) {
    match command_id {
        CMD_SAVE_CONFIG_TO_SD => {
            // Save current config to SD card
            let success = save_config_to_sd().await;
            send_command_ack(command_id, success).await;
        }
        _ => {
            // Unknown command, send negative ack
            send_command_ack(command_id, false).await;
        }
    }
}

/// Save the current robot configuration to SD card
/// Uses signal/channel to request the main task to perform the actual SD write
async fn save_config_to_sd() -> bool {
    // Request save via signal and wait for result
    request_save_config_to_sd().await
}

// ===================== TX Func =====================
pub fn uart_send(data: &[u8]) {
    if data.is_empty() || data.len() > 64 {
        return;
    }

    let mut v: Vec<u8, 64> = Vec::new();
    v.extend_from_slice(data).ok();

    let _ = UART_TX_CHANNEL.try_send(v);
}

pub fn pack_state_loopback(pkt: &StateLoopBackPacketF32) -> Vec<u8, 64> {
    let mut v: Vec<u8, 64> = Vec::new();

    // total length = header(1) + robot_id(1) + 10 floats * 4 = 1 + 1 + 40 = 42
    v.push(1 + 40).ok();
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
    let _ = UART_TX_CHANNEL.try_send(data);
}

// ==================== Robot State Sending Task ====================
pub static ROBOT_STATE_CH: Channel<ThreadModeRawMutex, StateLoopBackPacketF32, 4> = Channel::new();

pub fn update_robot_state(state: StateLoopBackPacketF32) {
    while ROBOT_STATE_CH.try_receive().is_ok() {}

    let _ = ROBOT_STATE_CH.try_send(state);
}

#[embassy_executor::task]
pub async fn state_loopback_task() {
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

/// Task to send robot parameters via UART upon startup.
/// This task runs once at startup, sends all parameters, then exits.
#[embassy_executor::task]
pub async fn uart_param_sync_task(config: crate::read_robot_config_from_sd::RobotConfig) {
    // Initialize the global config for runtime updates
    crate::parameter_sync::init_robot_config(config);

    // Wait a bit for UART to stabilize
    embassy_time::Timer::after_millis(500).await;

    // Send all parameters to the dongle
    let _ = crate::parameter_sync::send_robot_parameters_to_dongle(&config).await;
}
