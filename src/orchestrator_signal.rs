use defmt::info;
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex as Raw, channel::Channel, signal::Signal,
};

pub const FRAME_MAX: usize = 54;
pub const LEN_FUNC_SELECT_CMD: u8 = 3;
pub const LEN_STOP_RESUME_CMD: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Mode {
    Menu,
    TeleOp,
    TrajMocap,
    TrajDuty,
    CtrlAction,
}

#[derive(Clone, Copy, Debug)]
pub enum OrchestratorMsg {
    SwitchTo(Mode), // Switch back to certain mode（Menu means back to mode select interface）
}

// Global Command Channel: Any uart task can put commands into the channel
pub static ORCH_CH: Channel<Raw, OrchestratorMsg, 4> = Channel::new();

// Top level functionality selection task stop signal
pub static STOP_MENU_UART_SIG: Signal<Raw, ()> = Signal::new();

// Teleop stop signal
pub static STOP_TELEOP_UART_SIG: Signal<Raw, ()> = Signal::new();
pub static STOP_MOTOR_CTRL_SIG: Signal<Raw, ()> = Signal::new();

// Trajectory Following signal
pub static STOP_MOCAP_UART_SIG: Signal<Raw, ()> = Signal::new();
pub static STOP_MOCAP_UPDATE_SIG: Signal<Raw, ()> = Signal::new();
pub static STOP_WHEEL_INNER_SIG: Signal<Raw, ()> = Signal::new();
pub static STOP_TRAJ_OUTER_SIG: Signal<Raw, ()> = Signal::new();

pub static TRAJ_PAUSE_SIG: Signal<Raw, bool> = Signal::new();
pub static TRAJ_RESUME_SIG: Signal<Raw, bool> = Signal::new();

pub fn decode_functionality_select_command(payload: &[u8], robot_id: u8) -> Option<u8> {
    // Check frame format: should be 4 bytes with specific header
    if payload.len() != (LEN_FUNC_SELECT_CMD as usize) || payload[0] != 0x3C {
        // related to channel and port number
        // PORT 3 identifier

        return None;
    }

    // Check if command is for this robot (255 = broadcast to all)
    if payload[1] != robot_id && payload[1] != 255 {
        info!("payload {}", payload);
        return None;
    }

    info!("payload {}", payload);

    // Return command: 1 = start trajectory, 0 = stop trajectory
    match payload[2] {
        b'q' => Some(0), // Menu
        b'T' => Some(1), // Tele operation
        b'M' => Some(2), // Traj following
        b'D' => Some(3), // Traj following
        b'A' => Some(4), // Control Action
        _ => None,       // Invalid mode
    }
}
