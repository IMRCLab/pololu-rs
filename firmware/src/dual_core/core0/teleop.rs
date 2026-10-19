//! TeleOp and ControlAction target generation.
//!
//! This module converts the latest `(v, omega)` command into wheel angular
//! velocity targets. It does not access motors or encoders.

use crate::{
    dual_core::{
        messages::RuntimeMode,
        shared_state::TeleopCommand,
    },
    peripherals::sdcard::RobotConfig,
};

const COMMAND_TIMEOUT_US: u64 = 300_000;

#[derive(Clone, Copy)]
pub struct WheelTarget {
    pub omega_left: f32,
    pub omega_right: f32,
}

impl WheelTarget {
    pub const ZERO: Self = Self {
        omega_left: 0.0,
        omega_right: 0.0,
    };
}

#[derive(Clone, Copy)]
pub struct ControlReference {
    pub command_v: f32,
    pub command_omega: f32,
    pub wheel_target: WheelTarget,
    pub command_valid: bool,
}

impl ControlReference {
    pub const ZERO: Self = Self {
        command_v: 0.0,
        command_omega: 0.0,
        wheel_target: WheelTarget::ZERO,
        command_valid: false,
    };
}

pub fn resolve_reference(
    mode: RuntimeMode,
    command: TeleopCommand,
    now_us: u64,
    config: &RobotConfig,
) -> ControlReference {
    if !matches!(mode, RuntimeMode::TeleOp | RuntimeMode::CtrlAction) {
        return ControlReference::ZERO;
    }

    if command.stamp_us == 0
        || now_us.saturating_sub(command.stamp_us) > COMMAND_TIMEOUT_US
    {
        return ControlReference::ZERO;
    }

    let velocity_left = command.v - command.omega * config.wheel_base * 0.5;
    let velocity_right = command.v + command.omega * config.wheel_base * 0.5;

    ControlReference {
        command_v: command.v,
        command_omega: command.omega,
        wheel_target: WheelTarget {
            omega_left: velocity_left / config.wheel_radius,
            omega_right: velocity_right / config.wheel_radius,
        },
        command_valid: true,
    }
}