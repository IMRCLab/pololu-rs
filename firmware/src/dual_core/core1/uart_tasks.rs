//! Permanent Core 1 UART router.
//!
//! This is the only high-level task that consumes packets from the UART RX
//! channel. Mode-specific packet handlers are ordinary functions in this file.
use heapless::Vec as HVec;

use crate::{
    dual_core::{
        messages::{
            ControlCommand,
            RuntimeMode,
        },
        shared_state::{
            TeleopCommand,
            active_mode,
            publish_teleop_command,
            send_control_command,
            MocapSample,
            publish_mocap_sample,
        },
        metrics::record_uart_packet,
    },
    math::{
        quat_decompress,
        rpy_from_quaternion,
    },
    packet::CmdTeleopPacketMix,
    peripherals::sdcard::RobotConfig,
};
use super::uart_parser::receive_packet;

const FRAME_MAX: usize = 54;

const MODE_PACKET_LEN: u8 = 3;
const TRAJECTORY_COMMAND_PACKET_LEN: u8 = 4;
const PARAMETER_PACKET_LEN: u8 = 8;
const TELEOP_PACKET_LEN: u8 = 9;
const MOCAP_PACKET_LEN: u8 = 18;

#[derive(Clone, Copy)]
pub struct UartRouterConfig {
    pub robot_id: u8,
    pub max_speed: f32,
    pub max_omega: f32,
}

impl From<RobotConfig> for UartRouterConfig {
    fn from(config: RobotConfig) -> Self {
        Self {
            robot_id: config.robot_id,
            max_speed: config.max_speed,
            max_omega: config.max_omega,
        }
    }
}


fn decode_mode_selection(
    frame: &[u8],
    robot_id: u8,
) -> Option<RuntimeMode> {
    if frame.len() != MODE_PACKET_LEN as usize {
        return None;
    }

    if frame[0] != 0x3C {
        return None;
    }

    // 255 means broadcast.
    if frame[1] != robot_id && frame[1] != 255 {
        return None;
    }

    let selection = match frame[2] {
        b'q' => 0,
        b'T' => 1,
        b'M' => 2,
        b'A' => 4,
        b'F' => 5,
        b'G' => 6,
        _ => return None,
    };

    RuntimeMode::from_selection(selection)
}

async fn handle_mode_packet(
    frame: &[u8],
    robot_id: u8,
) {
    let Some(mode) =
        decode_mode_selection(frame, robot_id)
    else {
        return;
    };

    // Mode changes are reliable ordered events.
    send_control_command(
        ControlCommand::SwitchMode(mode),
    )
    .await;
}



fn map_teleop_command(
    packet: &CmdTeleopPacketMix,
    config: UartRouterConfig,
    stamp_us: u64,
) -> TeleopCommand {
    let raw_v = packet.linear_velocity;

    let v_sign = if raw_v >= 0.0 {
        1.0
    } else {
        -1.0
    };

    let normalized_v =
        ((raw_v.abs() - 1000.0) / 19000.0)
            .clamp(0.0, 1.0);

    let v = v_sign
        * config.max_speed
        * 0.5
        * (libm::expf(2.0 * normalized_v) - 1.0)
        / (libm::expf(2.0) - 1.0);

    let raw_omega =
        packet.steering_angle
            * core::f32::consts::PI
            / (180.0 * 0.15);

    let omega_sign = if raw_omega >= 0.0 {
        1.0
    } else {
        -1.0
    };

    // Intentionally not clamped, matching the current TeleOp behavior.
    let omega = omega_sign
        * config.max_omega
        * (
            libm::expf(
                raw_omega.abs() / config.max_omega
            ) - 1.0
        )
        / (libm::expf(1.0) - 1.0);

    TeleopCommand {
        v,
        omega,
        stamp_us,
    }
}

fn map_control_action(
    packet: &CmdTeleopPacketMix,
    stamp_us: u64,
) -> TeleopCommand {
    TeleopCommand {
        v: packet.linear_velocity,
        omega: packet.steering_angle,
        stamp_us,
    }
}


/// Decode the frame produced by `receive_packet()`.
///
/// The length byte itself is not included in `frame`, so an eight-byte
/// parameter frame is:
///
/// `[0x3C, param_id, value(4), checksum(2)]`
fn decode_parameter_write(
    frame: &[u8],
) -> Option<(u8, f32)> {
    if frame.len() != PARAMETER_PACKET_LEN as usize {
        return None;
    }

    if frame[0] != 0x3C {
        return None;
    }

    let parameter_id = frame[1];

    let value = f32::from_le_bytes([
        frame[2],
        frame[3],
        frame[4],
        frame[5],
    ]);

    Some((parameter_id, value))
}

async fn handle_parameter_packet(
    frame: &[u8],
) {
    let Some((parameter_id, value)) =
        decode_parameter_write(frame)
    else {
        return;
    };

    // Core 0 applies control parameters at a safe control-loop boundary.
    send_control_command(
        ControlCommand::ParameterWrite {
            parameter_id,
            value,
        },
    )
    .await;
}

fn handle_velocity_packet(
    frame: &[u8],
    stamp_us: u64,
    config: UartRouterConfig,
) {
    let Some(packet) =
        CmdTeleopPacketMix::from_bytes(frame)
    else {
        return;
    };

    let command = match active_mode() {
        RuntimeMode::TeleOp => {
            map_teleop_command(
                &packet,
                config,
                stamp_us,
            )
        }

        RuntimeMode::CtrlAction => {
            map_control_action(
                &packet,
                stamp_us,
            )
        }

        // Velocity packets are ignored in every other mode.
        _ => return,
    };

    publish_teleop_command(command);
}




// Trajectory Message Decode
fn decode_trajectory_command(
    frame: &[u8],
    robot_id: u8,
) -> Option<bool> {
    if frame.len()
        != TRAJECTORY_COMMAND_PACKET_LEN as usize
    {
        return None;
    }

    if frame[0] != 0x3C {
        return None;
    }

    if frame[1] != robot_id && frame[1] != 255 {
        return None;
    }

    if frame[2] != 0xAA {
        return None;
    }

    match frame[3] {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    }
}

async fn handle_trajectory_command(
    frame: &[u8],
    robot_id: u8,
) {
    // Ignore trajectory commands outside trajectory modes.
    match active_mode() {
        RuntimeMode::TrajMocap
        | RuntimeMode::TrajOnboard
        | RuntimeMode::TrajOnboard2 => {}

        _ => return,
    }

    let Some(resume) =
        decode_trajectory_command(frame, robot_id)
    else {
        return;
    };

    let command = if resume {
        ControlCommand::TrajectoryResume
    } else {
        ControlCommand::TrajectoryPause
    };

    send_control_command(command).await;
}



// Mocap Message Decode
fn decode_mocap_sample(
    frame: &[u8],
    robot_id: u8,
    stamp_us: u64,
) -> Option<MocapSample> {
    if frame.len() != MOCAP_PACKET_LEN as usize {
        return None;
    }

    if frame[0] != 0x3C {
        return None;
    }

    // Preserve current behavior: Mocap pose must target this robot exactly.
    if frame[1] != robot_id {
        return None;
    }

    let payload = &frame[2..18];

    let x = f32::from_le_bytes([
        payload[0],
        payload[1],
        payload[2],
        payload[3],
    ]);

    let y = f32::from_le_bytes([
        payload[4],
        payload[5],
        payload[6],
        payload[7],
    ]);

    let z = f32::from_le_bytes([
        payload[8],
        payload[9],
        payload[10],
        payload[11],
    ]);

    let compressed_quaternion = u32::from_le_bytes([
        payload[12],
        payload[13],
        payload[14],
        payload[15],
    ]);

    let quaternion =
        quat_decompress(compressed_quaternion);

    let (roll, pitch, yaw) =
        rpy_from_quaternion(&quaternion);

    if !x.is_finite()
        || !y.is_finite()
        || !z.is_finite()
        || !roll.is_finite()
        || !pitch.is_finite()
        || !yaw.is_finite()
    {
        return None;
    }

    // Preserve the validation bounds from the current firmware.
    if !(-10.0..=10.0).contains(&x)
        || !(-10.0..=10.0).contains(&y)
    {
        return None;
    }

    if !(-0.5..=1.0).contains(&z) {
        return None;
    }

    if !(-0.5..=0.5).contains(&roll)
        || !(-0.5..=0.5).contains(&pitch)
    {
        return None;
    }

    Some(MocapSample {
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        stamp_us,
    })
}

fn handle_mocap_packet(
    frame: &[u8],
    robot_id: u8,
    stamp_us: u64,
) {
    // Mocap is relevant to all current trajectory modes.
    match active_mode() {
        RuntimeMode::TrajMocap
        | RuntimeMode::TrajOnboard
        | RuntimeMode::TrajOnboard2 => {}

        _ => return,
    }

    let Some(sample) =
        decode_mocap_sample(
            frame,
            robot_id,
            stamp_us,
        )
    else {
        return;
    };

    publish_mocap_sample(sample);
}




/// Permanent high-level UART consumer running on Core 1.
///
/// `uart_hw_task` remains responsible for DMA/hardware access. This task
/// consumes framed packets and routes them according to packet type and active
/// mode.
#[embassy_executor::task]
pub async fn core1_uart_router_task(
    config: UartRouterConfig,
) {
    let mut frame: HVec<u8, FRAME_MAX> =
        HVec::new();

    defmt::info!("Core 1 UART router started");

    loop {
        let packet =
            receive_packet(&mut frame).await;
        record_uart_packet();

        let length = packet.length;
        let packet_stamp = packet.stamp;

        match length {
            MODE_PACKET_LEN => {
                handle_mode_packet(
                    &frame,
                    config.robot_id,
                )
                .await;
            }

            TRAJECTORY_COMMAND_PACKET_LEN => {
                handle_trajectory_command(
                    &frame,
                    config.robot_id,
                )
                .await;
            }

            PARAMETER_PACKET_LEN => {
                handle_parameter_packet(
                    &frame,
                )
                .await;
            }

            TELEOP_PACKET_LEN => {
                handle_velocity_packet(
                    &frame,
                    packet_stamp.as_micros(),
                    config,
                );
            }

            MOCAP_PACKET_LEN => {
                handle_mocap_packet(
                    &frame,
                    config.robot_id,
                    packet_stamp.as_micros(),
                );
            }

            _ => {}
        }
    }
}