use core::cmp::min;
use defmt::*;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use heapless::HistoryBuffer;
use heapless::Vec as HVec;

use crate::math::{quat_decompress, rpy_from_quaternion};
use crate::trajectory_signal::{
    FIRST_MESSAGE, LAST_STATE, PoseAbs, RobotDistance, RobotInfo, RobotState, START_EVENT,
    STATE_SIG, TRAJECTORY_CONTROL_EVENT,
};
use crate::uart::SharedUart;
use libm::sqrtf;

#[derive(Clone, Copy)]
pub struct UartCfg {
    pub robot_id: u8,
}

const FRAME_MAX: usize = 54;
const LEN_POSE: u8 = 18;
const LEN_START: u8 = 10;
const LEN_TRAJECTORY_CMD: u8 = 3;

const DT: u32 = 100; //sending period in ms
const DISTANCE_HISTORY_SIZE: usize = 16;

// Local position tracking struct for distance computation
#[derive(Clone, Copy)]
pub struct LocalRobotPosition {
    pub distance_x: f32, // relative distance in x
    pub distance_y: f32, // relative distance in y
    pub angle_dist: f32, // yaw angle in rad
    pub timestamp: Instant,
}

pub struct LocalRobotState {
    pub position: LocalRobotPosition,
    pub velocity: (f32, f32),
}

pub async fn compute_relative_velocity(
    buffer: &HistoryBuffer<RobotDistance, DISTANCE_HISTORY_SIZE>,
) -> Option<(f32, f32)> {
    //or use low pass filtering with other coefficients
    let mut vals = buffer.iter().rev().take(3).map(|e| (e.x, e.y));
    let first = vals.next();
    let second = vals.next();
    let third = vals.next();

    if let (Some(a), Some(_b), Some(c)) = (first, second, third) {
        // a is newest, c is oldest because we iter().rev()
        // compute centered difference: (oldest - newest) / (2*DT)
        let dt_s = DT as f32 / 1000.0;

        // info!(
        //     "distance points used: newest ({}, {}), oldest ({}, {})",
        //     a.0, a.1, c.0, c.1
        // );
        //info!("DT used: {}", 2.0 * dt_s);
        let vel_xr = (a.0 - c.0) / (2.0 * dt_s);
        let vel_yr = (a.1 - c.1) / (2.0 * dt_s);

        // info!(
        //     "Computed relative velocity: vel_xr = {}, vel_yr = {}",
        //     vel_xr, vel_yr
        // );
        Some((vel_xr, vel_yr))
    } else {
        None
    }
}

pub async fn compute_angular_velocity(
    x_dist: f32,
    y_dist: f32,
    vel_xr: f32,
    vel_yr: f32,
) -> Option<f32> {
    //derivative of tan^(-1)
    let angular_vel = (y_dist * vel_xr - x_dist * vel_yr) / (x_dist * x_dist + y_dist * y_dist);
    Some(angular_vel)
}

pub async fn closing_speed(x_dist: f32, y_dist: f32, vel_xr: f32, vel_yr: f32) -> Option<f32> {
    //Annäherungsrate von vom anderen Roboter
    let closing_speed =
        (x_dist * vel_xr + y_dist * vel_yr) / sqrtf(x_dist * x_dist + y_dist * y_dist);
    Some(closing_speed)
}

async fn add_robot_info_from_buffer(
    buffer: &HistoryBuffer<RobotDistance, DISTANCE_HISTORY_SIZE>,
    robot_state: &mut RobotState,
) {
    if let Some(latest_distance) = buffer.recent() {
        let velocity = compute_relative_velocity(buffer).await;
        let (vel_x, vel_y) = velocity.unwrap_or((0.0, 0.0));
        let robot_info = RobotInfo {
            robot_id: latest_distance.id,
            distance_x: latest_distance.x,
            distance_y: latest_distance.y,
            vel_x,
            vel_y,
            angle: latest_distance.angle,
        };
        let _ = robot_state.other_robots.push(robot_info);
    }
}

#[embassy_executor::task]
pub async fn uart_motioncap_receiving_task(uart: SharedUart<'static>, cfg: UartCfg) {
    let mut seen_first = false;
    let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    // Local copy of own robot's pose - no mutex needed!
    let mut own_pose = PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    // Distance buffers for 3 robots (robot IDs will be mapped to these buffers)
    let mut robot1_buffer: HistoryBuffer<RobotDistance, DISTANCE_HISTORY_SIZE> =
        HistoryBuffer::new();
    let mut robot2_buffer: HistoryBuffer<RobotDistance, DISTANCE_HISTORY_SIZE> =
        HistoryBuffer::new();
    let mut robot3_buffer: HistoryBuffer<RobotDistance, DISTANCE_HISTORY_SIZE> =
        HistoryBuffer::new();

    //----------------------------------
    //------- ROBOT IDS -----------------
    // -----------------------------------
    let robot_ids = [7, 9, 10]; //pololu08 Robot IDs that will use buffers 1, 2, 3 respectively
    //let robot_ids = [8, 9, 10]; // pololu7's id list
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
                info!("Trajectory command received");
            }
            continue;
        }
        if len == LEN_START {
            if is_start_event(&frame) {
                START_EVENT.signal(());
                info!("Start event received");
            }
            continue;
        }
        if len == LEN_POSE {
            //search for all other robot IDs in the poses

            if let Some(pose) = decode_abs_pose(&frame) {
                {
                    //if own robot, update both LAST_STATE and local copy
                    // info!(
                    //     "Decoded pose for robot ID {}: ({}, {}, {})",
                    //     frame[1], pose.x, pose.y, pose.z
                    // );
                    //info!("My robot ID is {}", cfg.robot_id);
                    if frame[1] == cfg.robot_id {
                        own_pose = pose; //local copy for distance calculations

                        // Build complete new state first, then update atomically
                        let mut new_state = RobotState {
                            pose,
                            other_robots: HVec::new(),
                        };

                        //add robot info from buffers to new state
                        add_robot_info_from_buffer(&robot1_buffer, &mut new_state).await;
                        add_robot_info_from_buffer(&robot2_buffer, &mut new_state).await;
                        add_robot_info_from_buffer(&robot3_buffer, &mut new_state).await;

                        // Update LAST_STATE atomically
                        {
                            let mut s = LAST_STATE.lock().await;
                            *s = new_state; // Single atomic update
                        }

                        STATE_SIG.signal(pose);
                    }

                    //track other robots if pose from them incoming
                    if frame[1] != cfg.robot_id {
                        let dx = pose.x - own_pose.x;
                        let dy = pose.y - own_pose.y;
                        let distance = RobotDistance {
                            id: frame[1],
                            x: dx,
                            y: dy,
                            angle: pose.yaw,
                        };

                        //sort distance to that robot into appropriate buffer based on robot ID
                        match robot_ids.iter().position(|&id| id == frame[1]) {
                            //get robot id from position in array defining them
                            Some(0) => {
                                robot1_buffer.write(distance);
                            }
                            Some(1) => {
                                robot2_buffer.write(distance);
                            }
                            Some(2) => {
                                robot3_buffer.write(distance);
                            }
                            Some(_) => {
                                //this shouldn't happen with our array of size 3, but handle it just in case
                            }
                            None => {
                                //ignore if unknown ID
                            }
                        }
                    }
                }

                if !seen_first {
                    FIRST_MESSAGE.signal(());
                    seen_first = true;
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

//modify for the hackathon:
//Decode all pose packets, put the one from the robot id in LAST_STATE,
fn decode_abs_pose(payload: &[u8]) -> Option<PoseAbs> {
    // frame header check
    // info!("here3 {}", payload);

    if !(payload.len() == 18 && (payload[0] == 0x3C)) {
        //0x3C is related to channel and port number
        return None;
    }

    let s1 = &payload[2..18];
    // let s1 = if payload[1] == robot_id {
    //     &payload[2..18]
    // } else {
    //     return None;
    // };

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

    //calculate relative position if not own robot id
    Some(PoseAbs {
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
    })
}
