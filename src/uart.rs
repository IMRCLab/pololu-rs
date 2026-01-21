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
// TX channel for sending data via UART
pub static UART_TX_CHANNEL: Channel<ThreadModeRawMutex, Vec<u8, 64>, 128> = Channel::new();
// RX: Low level UART HW Task -> decode Task
pub static UART_RX_CHANNEL: Channel<ThreadModeRawMutex, u8, 128> = Channel::new();

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
            defmt::info!("received a package of length {}", expected_len);
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
                        defmt::info!(
                            "UART RX: Parameter write request: id={}, value={}",
                            req.param_id,
                            req.value
                        );
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
/// note: not used yet
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
/// note: note implemented in the dongle yet
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

// NOTE: log_uart_task removed - use robotstate::uart_log_sending_task instead
// which polls mutexes periodically rather than consuming from a channel
