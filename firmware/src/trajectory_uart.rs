use defmt::*;
use embassy_time::Instant;
use heapless::Vec as HVec;

use crate::buzzer::beep_signal;
use crate::math::{quat_decompress, rpy_from_quaternion};
use crate::orchestrator_signal::{
    LEN_FUNC_SELECT_CMD, LEN_STOP_RESUME_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_MOCAP_UART_SIG,
    TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, decode_functionality_select_command,
};
use crate::robotstate::{MocapPose, TRAJECTORY_CONTROL_EVENT};
use crate::robotstate::MOCAP_SIG as STATE_SIG;
use crate::uart_parser::{RecvResult, receive_packet};

#[derive(Clone, Copy)]
pub struct UartCfg {
    pub robot_id: u8,
}

const FRAME_MAX: usize = 54;
const LEN_POSE: u8 = 18;
#[allow(dead_code)]
const LEN_START: u8 = 10;

#[embassy_executor::task]
pub async fn uart_motioncap_receiving_task(cfg: UartCfg) {
    let mut seen_first = false;
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    loop {
        let len = match receive_packet(&mut frame, &STOP_MOCAP_UART_SIG).await {
            RecvResult::Packet { len } => len,
            RecvResult::Stop => {
                info!("mocap uart: stop signal -> exit");
                return;
            }
        };

        // ---- Decode ----
        if len == LEN_FUNC_SELECT_CMD {
            if let Some(sel) = decode_functionality_select_command(&frame, cfg.robot_id) {
                let target = match sel {
                    0 => Mode::Menu,
                    1 => Mode::TeleOp,
                    2 => Mode::TrajMocap,
                    4 => Mode::CtrlAction,
                    5 => Mode::TrajOnboard,
                    6 => Mode::TrajOnboard2,
                    _ => Mode::Menu,
                };
                // Use blocking send so the command is never dropped
                ORCH_CH.send(OrchestratorMsg::SwitchTo(target)).await;
            }
            continue;
        }

        if len == LEN_STOP_RESUME_CMD {
            if let Some(start) = decode_trajectory_command(&frame, cfg.robot_id) {
                info!("Trajectory control command received: {}", start);
                TRAJECTORY_CONTROL_EVENT.signal(start);
                if start {
                    TRAJ_RESUME_SIG.signal(true);
                    beep_signal(b'b');
                    info!("Trajectory following resume!!");
                } else {
                    TRAJ_PAUSE_SIG.signal(true);
                    beep_signal(b's');
                    info!("Trajectory following pause!!");
                }
            }
            continue;
        }

        if len == LEN_POSE {
            if let Some(pose) = decode_abs_pose(&frame, cfg.robot_id) {
                let stamped = MocapPose { stamp: Instant::now(), ..pose };
                STATE_SIG.signal(stamped);
                if !seen_first {
                    seen_first = true;
                    info!("first mocap message seen");
                }
            }
            continue;
        }

        if len == 8 {
            if let Some(req) = crate::parameter_sync::ParameterWriteRequest::from_bytes(&frame) {
                crate::parameter_sync::handle_parameter_write(req.param_id, req.value).await;
            }
            continue;
        }
    }
}

fn decode_trajectory_command(payload: &[u8], robot_id: u8) -> Option<bool> {
    if payload.len() != 4 || payload[0] != 0x3C {
        return None;
    }
    if payload[1] != robot_id && payload[1] != 255 && payload[2] != 0xAA {
        return None;
    }
    match payload[3] {
        1 => Some(true),
        0 => Some(false),
        _ => None,
    }
}

fn decode_abs_pose(payload: &[u8], robot_id: u8) -> Option<MocapPose> {
    if !(payload.len() == 18 && payload[0] == 0x3C) {
        return None;
    }
    let s1 = if payload[1] == robot_id {
        &payload[2..18]
    } else {
        return None;
    };
    if s1.len() < 16 {
        return None;
    }
    let x = f32::from_le_bytes([s1[0], s1[1], s1[2], s1[3]]);
    let y = f32::from_le_bytes([s1[4], s1[5], s1[6], s1[7]]);
    let z = f32::from_le_bytes([s1[8], s1[9], s1[10], s1[11]]);
    let raw = u32::from_le_bytes([s1[12], s1[13], s1[14], s1[15]]);
    let q = quat_decompress(raw);
    let (roll, pitch, yaw) = rpy_from_quaternion(&q);
    Some(MocapPose {
        x, y, z, roll, pitch, yaw,
        stamp: Instant::from_ticks(0),
    })
}
