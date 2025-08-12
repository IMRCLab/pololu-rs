use core::cmp::min;
use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use heapless::Vec as HVec;

use crate::math::{decode_quat_i8x4, rpy_from_quaternion};
use crate::trajectory_signal::{FIRST_MESSAGE, LAST_STATE, PoseAbs, START_EVENT, STATE_SIG};
use crate::uart::SharedUart; // = Mutex<Raw, Uart<'static, Async>>

#[derive(Clone, Copy)]
pub struct UartCfg {
    pub robot_id: u8,
}

const FRAME_MAX: usize = 64;
const LEN_A: u8 = 0x18;
const LEN_B: u8 = 10;

#[embassy_executor::task]
pub async fn uart_motioncap_receiving_task(uart: SharedUart<'static>, cfg: UartCfg) {
    let mut seen_first = false;
    let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    loop {
        // Read Length Buffer Byte First
        let len: Option<u8> = {
            let read_len = async {
                let mut u = uart.lock().await;
                match u.read(&mut len_buf).await {
                    Ok(()) => Some(len_buf[0]), // read 1 byte
                    Err(_) => None,
                }
            };

            match select(read_len, Timer::after(Duration::from_millis(1))).await {
                Either::First(v) => v,     // Some(byte) or None
                Either::Second(_) => None, // Timeout
            }
        };

        let Some(_len) = len else {
            // TimeOut Or Error
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        let len = len_buf[0];
        if !(len == LEN_A || len == LEN_B) {
            continue; // illegal Length
        }

        // -------- read payload，until [len] bytes --------
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            // maximally read 32 bytes at a time
            let mut chunk = [0u8, 32];
            let take = min(need - got, chunk.len());

            let ok: bool = {
                let read_chunk = async {
                    let mut u = uart.lock().await;
                    u.read(&mut chunk[..take]).await.is_ok()
                };
                match select(read_chunk, Timer::after(Duration::from_millis(2))).await {
                    Either::First(ok) => ok,
                    Either::Second(_) => false,
                }
            };

            if !ok {
                // Current frame is lost, abandon and quit reading this frame
                Timer::after(Duration::from_micros(300)).await;
                got = 0;
                frame.clear();
                break;
            }

            // attach to frame
            for &b in &chunk[..take] {
                let _ = frame.push(b);
            }
            got += take;
        }

        if got != need {
            continue;
        }

        // -------- Decoding --------
        if is_start_event(&frame) {
            START_EVENT.signal(());
            info!("start event received");
            continue;
        }
        if let Some(pose) = decode_abs_pose(&frame, cfg.robot_id) {
            {
                let mut s = LAST_STATE.lock().await;
                *s = pose;
            }
            STATE_SIG.signal(pose);

            if !seen_first {
                FIRST_MESSAGE.signal(());
                seen_first = true;
                info!("first message set");
            }
        } else {
            // Unknown Frame, ignore.
        }
    }
}

fn is_start_event(payload: &[u8]) -> bool {
    payload.len() >= 2 && (payload[0] & 0xF3) == 0x80 && payload[1] == 0x05
}

fn decode_abs_pose(payload: &[u8], robot_id: u8) -> Option<PoseAbs> {
    // frame header check
    if !(payload.len() >= 24 && (payload[0] & 0xF3) == 0x61 && payload[1] == 0x09) {
        return None;
    }

    let s1 = if payload[2] == robot_id {
        &payload[2..13]
    } else if payload[13] == robot_id {
        &payload[13..24]
    } else {
        return None;
    };

    if s1.len() < 11 {
        return None;
    }

    // i16(mm) -> f32(m)
    let x = i16::from_le_bytes([s1[1], s1[2]]) as f32 * 0.001;
    let y = i16::from_le_bytes([s1[3], s1[4]]) as f32 * 0.001;
    let z = i16::from_le_bytes([s1[5], s1[6]]) as f32 * 0.001;

    // I8x4 -> Quaternion
    let raw = u32::from_le_bytes([s1[7], s1[8], s1[9], s1[10]]);
    let q = decode_quat_i8x4(raw);
    let (roll, pitch, yaw) = rpy_from_quaternion(&q);

    Some(PoseAbs {
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
    })
}
