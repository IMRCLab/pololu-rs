use core::cmp::min;
use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use heapless::Vec as HVec;

use crate::math::{quat_decompress, rpy_from_quaternion};
use crate::trajectory_signal::{
    FIRST_MESSAGE, LAST_STATE, PoseAbs, START_EVENT, STATE_SIG, TRAJECTORY_CONTROL_EVENT,
};
use crate::uart::SharedUart; // = Mutex<Raw, Uart<'static, Async>>

#[derive(Clone, Copy)]
pub struct UartCfg {
    pub robot_id: u8,
}

const FRAME_MAX: usize = 54;
const LEN_POSE: u8 = 18;
const LEN_START: u8 = 10;
const LEN_TRAJECTORY_CMD: u8 = 3;

#[embassy_executor::task]
pub async fn uart_motioncap_receiving_task(uart: SharedUart<'static>, cfg: UartCfg) {
    let mut seen_first = false;
    let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    loop {
        // Read Length Buffer Byte First
        //info!("Waiting for mocap UART data...");
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
        if !(len == LEN_POSE || len == LEN_START || len == LEN_TRAJECTORY_CMD) {
            // info!("here1");
            continue; // illegal Length
        }

        // -------- read payload，until [len] bytes --------
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            // maximally read 32 bytes at a time

            let mut chunk = [0u8; 18];
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
            let _ = frame.extend_from_slice(&chunk[..take]);
            // for &b in &chunk[..take] {
            //     let _ = frame.push(b);
            // }
            got += take;
        }

        if got != need {
            continue;
        }

        // -------- Decoding --------
        //info!("Received frame len={}", len);
        if len == LEN_TRAJECTORY_CMD {
            if let Some(start_trajectory) = decode_trajectory_command(&frame, cfg.robot_id) {
                TRAJECTORY_CONTROL_EVENT.signal(start_trajectory);
                info!("Trajectory control command received: {}", start_trajectory);
            }
            continue;
        }
        if len == LEN_START {
            if is_start_event(&frame) {
                START_EVENT.signal(());
                info!("start event received"); // This won't be triggered since the ros node is not sending this start event.
            }
            continue;
        }
        if len == LEN_POSE {
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
            }
            continue;
        }
    }
}

fn is_start_event(payload: &[u8]) -> bool {
    payload.len() == 10 && (payload[0] & 0xF3) == 0x80 && payload[1] == 0x05
}

fn decode_trajectory_command(payload: &[u8], robot_id: u8) -> Option<bool> {
    // Check frame format: should be 3 bytes with specific header
    if payload.len() != 3 || payload[0] != 0x3C {
        //related to channel and port number
        // PORT 3 identifier
        return None;
    }

    // Check if command is for this robot (255 = broadcast to all)
    if payload[1] != robot_id && payload[1] != 255 {
        return None;
    }

    // Return command: 1 = start trajectory, 0 = stop trajectory
    match payload[2] {
        1 => Some(true),  // Start trajectory
        0 => Some(false), // Stop trajectory
        _ => None,        // Invalid command
    }
}

fn decode_abs_pose(payload: &[u8], robot_id: u8) -> Option<PoseAbs> {
    // frame header check
    // info!("here3 {}", payload);

    if !(payload.len() == 18 && (payload[0] == 0x3C)) {
        //0x3C is related to channel and port number
        return None;
    }

    let s1 = if payload[1] == robot_id {
        &payload[2..18]
    } else {
        return None;
    };

    if s1.len() < 11 {
        return None;
    }

    // if let Some(pkt) = MocapPosesPacketF32Test::from_bytes(&buffer) {
    //                     defmt::info!(
    //                         "Pose Packet: robot_id={}, PosX={} PosY={} Pos_Z={} Quat={}",
    //                         pkt.robot_id,
    //                         pkt.pos_x,
    //                         pkt.pos_y,
    //                         pkt.pos_z,
    //                         pkt.quat
    //                     );
    //                 } else {
    //                     defmt::warn!("Invalid Pose packet");
    //                 }

    let x = f32::from_le_bytes([s1[0], s1[1], s1[2], s1[3]]);
    let y = f32::from_le_bytes([s1[4], s1[5], s1[6], s1[7]]);
    let z = f32::from_le_bytes([s1[8], s1[9], s1[10], s1[11]]);

    let raw = u32::from_le_bytes([s1[12], s1[13], s1[14], s1[15]]);
    let q = quat_decompress(raw);
    //info!("quat {} {} {} {}", q.x, q.y, q.z, q.w);
    let (roll, pitch, yaw) = rpy_from_quaternion(&q);

    // info!(
    //     "robot Id: {}, x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}",
    //     payload[1], x, y, z, roll, pitch, yaw,
    // );

    Some(PoseAbs {
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
    })
}
