//! Messages shared by the Core 0 control domain and Core 1 I/O domain.
//!
//! All types in this file must remain small and `Copy`, because they may be
//! transferred through bounded cross-core channels.

/// Runtime operating mode used by the dual-core firmware.
///
/// Values intentionally match the current UART mode-selection protocol.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RuntimeMode {
    Menu = 0,
    TeleOp = 1,
    TrajMocap = 2,
    CtrlAction = 4,
    TrajOnboard = 5,
    TrajOnboard2 = 6,
}

impl RuntimeMode {
    pub fn from_selection(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Menu),
            1 => Some(Self::TeleOp),
            2 => Some(Self::TrajMocap),
            4 => Some(Self::CtrlAction),
            5 => Some(Self::TrajOnboard),
            6 => Some(Self::TrajOnboard2),
            _ => None,
        }
    }

    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

impl Default for RuntimeMode {
    fn default() -> Self {
        Self::Menu
    }
}

/// Reliable commands sent from Core 1 to Core 0.
///
/// These commands must be delivered in order and must not use a latest-value mailbox.
#[derive(Clone, Copy)]
pub enum ControlCommand {
    SwitchMode(RuntimeMode),

    ParameterWrite {
        parameter_id: u8,
        value: f32,
    },

    TrajectoryPause,
    TrajectoryResume,
}

/// Commands sent to the single Core 1 logging service.
#[derive(Clone, Copy)]
pub enum LoggerCommand {
    /// Reset the shared logging state and open a new file.
    StartSession(RuntimeMode),

    /// Flush and close the current log file.
    StopSession,

    /// Flush without closing the current file.
    Flush,
}


/// Commands handled by the Core 1 UI task.
#[derive(Clone, Copy)]
pub enum UiCommand {
    ModeChanged(RuntimeMode),
    Error,
    CommandTimeout,
    Beep(u8),

    SetLed(bool),
}