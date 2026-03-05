use defmt::*;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec as HVec;

use crate::buzzer::beep_signal;
use crate::math::{quat_decompress, rpy_from_quaternion};
use crate::orchestrator_signal::{
    LEN_FUNC_SELECT_CMD, LEN_STOP_RESUME_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_MOCAP_UART_SIG,
    TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, decode_functionality_select_command,
};
use crate::trajectory_signal::{FIRST_MESSAGE, LAST_STATE, POSE_FRESH, PoseAbs, STATE_SIG};
use crate::uart::UART_RX_CHANNEL;

#[derive(Clone, Copy)]
pub struct UartCfg {
    pub robot_id: u8,
}

const FRAME_MAX: usize = 54;
const LEN_POSE: u8 = 18;
const LEN_START: u8 = 10;

#[embassy_executor::task]
pub async fn uart_motioncap_receiving_task(cfg: UartCfg) {
    let mut seen_first = false;
    // let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    loop {
        // Read Length Buffer Byte First
        let read_len_fut = UART_RX_CHANNEL.receive();
        let timeout_len_fut = Timer::after(Duration::from_millis(1));
        let stop_fut = STOP_MOCAP_UART_SIG.wait();

        let len_opt: Option<u8> = match select3(read_len_fut, timeout_len_fut, stop_fut).await {
            Either3::First(v) => Some(v),
            Either3::Second(_) => None,
            Either3::Third(_) => {
                info!("mocap uart: stop signal on len -> exit");
                return;
            }
        };

        let Some(len) = len_opt else {
            // TimeOut Or Error
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        // let len = len_buf[0];
        if !(len == LEN_POSE
            || len == LEN_START
            || len == LEN_FUNC_SELECT_CMD
            || len == LEN_STOP_RESUME_CMD)
        {
            continue; // illegal Length
        }

        // ================= Read PAYLOAD =================
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            let read_byte_fut = UART_RX_CHANNEL.receive();
            let timeout_fut = Timer::after(Duration::from_millis(2));
            let stop_fut = STOP_MOCAP_UART_SIG.wait();

            let byte_opt: Option<u8> = match select3(read_byte_fut, timeout_fut, stop_fut).await {
                Either3::First(b) => Some(b),
                Either3::Second(_) => None,
                Either3::Third(_) => {
                    info!("mocap uart: stop signal on payload -> exit");
                    return;
                }
            };

            let Some(b) = byte_opt else {
                frame.clear();
                got = 0;
                break;
            };

            frame.push(b).ok();
            got += 1;
        }

        if got != need {
            continue;
        }

        // -------- Decoding --------
        if len == LEN_FUNC_SELECT_CMD {
            info!("here_mocap");
            if let Some(sel) = decode_functionality_select_command(&frame, cfg.robot_id) {
                let target = match sel {
                    0 => Mode::Menu,
                    1 => Mode::TeleOp,
                    2 => Mode::TrajMocap,
                    3 => Mode::TrajDuty,
                    _ => Mode::Menu,
                };
                let _ = ORCH_CH.try_send(OrchestratorMsg::SwitchTo(target));
                return;
            }
            continue;
        }

        if len == LEN_STOP_RESUME_CMD {
            if let Some(start_trajectory) = decode_trajectory_command(&frame, cfg.robot_id) {
                beep_signal();
                info!("Trajectory control command received: {}", start_trajectory);

                if start_trajectory {
                    TRAJ_RESUME_SIG.signal(true);
                    info!("Trajectory following resume!!");
                } else {
                    TRAJ_PAUSE_SIG.signal(true);
                    info!("Trajectory following pause!!");
                }
            }
            continue;
        }

        if len == LEN_POSE {
            if let Some(pose) = decode_abs_pose(&frame, cfg.robot_id) {
                let stamped = PoseAbs {
                    stamp: Instant::now(),
                    ..pose
                };
                {
                    let mut s = LAST_STATE.lock().await;
                    *s = stamped;
                }
                POSE_FRESH.store(true, portable_atomic::Ordering::Release);
                STATE_SIG.signal(stamped);

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

fn decode_trajectory_command(payload: &[u8], robot_id: u8) -> Option<bool> {
    // Check frame format: should be 4 bytes with specific header
    // [0x3C 0x(address) 0xAA 0x00(0x01)]
    if payload.len() != 4 || payload[0] != 0x3C {
        //related to channel and port number
        // PORT 3 identifier
        return None;
    }

    // Check if command is for this robot (255 = broadcast to all)
    if payload[1] != robot_id && payload[1] != 255 && payload[2] != 0xAA {
        return None;
    }

    // Return command: 1 = start trajectory, 0 = stop trajectory
    match payload[3] {
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
    // Pose logging disabled for performance — enable only for debugging
    // let timestamp = Instant::now().as_millis();
    // defmt::info!(
    //     "Pose Abs: robot_id={}, x={} y={} z={} roll={} pitch={} yaw={} timestamp={}",
    //     payload[1], x, y, z, roll, pitch, yaw, timestamp
    // );
    Some(PoseAbs {
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        stamp: Instant::from_ticks(0), // caller will overwrite with Instant::now()
    })
}
