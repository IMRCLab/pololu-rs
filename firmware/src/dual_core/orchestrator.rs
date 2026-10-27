//! Dual-core mode orchestrator.
//!
//! This task handles low-frequency lifecycle events. It never owns motors and
//! never performs control calculations.

use crate::dual_core::{
    core0::trajectory_control::trajectory_following_task,
    messages::{ControlCommand, LoggerCommand, RuntimeMode, UiCommand},
    shared_state::{
        active_mode, clear_mode_inputs, receive_control_command, send_logger_command,
        send_parameter_update, send_ui_command, set_active_mode, set_trajectory_running,
    },
};
use crate::peripherals::sdcard::RobotConfig;
use embassy_executor::Spawner;

fn trajectory_robot_config(
    config: RobotConfig,
) -> crate::read_robot_config_from_sd::RobotConfig {
    crate::read_robot_config_from_sd::RobotConfig {
        robot_id: config.robot_id,
        joystick_control_dt_ms: config.joystick_control_dt_ms,
        traj_following_dt_s: config.traj_following_dt_s,
        sd_logging_dt_s: config.sd_logging_dt_s,
        wheel_radius: config.wheel_radius,
        wheel_base: config.wheel_base,
        motor_direction_left: config.motor_direction_left,
        motor_direction_right: config.motor_direction_right,
        motor_max_duty_left: config.motor_max_duty_left,
        motor_max_duty_right: config.motor_max_duty_right,
        k_clip: config.k_clip,
        kp_inner: config.kp_inner,
        ki_inner: config.ki_inner,
        kd_inner: config.kd_inner,
        kx_traj: config.kx_traj,
        ky_traj: config.ky_traj,
        ktheta_traj: config.ktheta_traj,
        gear_ratio: config.gear_ratio,
        encoder_cpr: config.encoder_cpr,
        max_speed: config.max_speed,
        max_omega: config.max_omega,
        wheel_max: config.wheel_max,
    }
}

fn is_trajectory_mode(mode: RuntimeMode) -> bool {
    matches!(
        mode,
        RuntimeMode::TrajMocap
            | RuntimeMode::TrajOnboard
            | RuntimeMode::TrajOnboard2
    )
}

async fn switch_mode(
    target: RuntimeMode,
    spawner: Spawner,
    robot_config: RobotConfig,
) {
    let current = active_mode();

    if current == target {
        return;
    }

    set_active_mode(RuntimeMode::Menu);

    clear_mode_inputs();

    send_logger_command(LoggerCommand::StopSession).await;

    if target != RuntimeMode::Menu {
        clear_mode_inputs();
        set_active_mode(target);

        send_logger_command(
            LoggerCommand::StartSession(target),
        )
        .await;

        if is_trajectory_mode(target)
            && spawner
                .spawn(trajectory_following_task(Some(
                    trajectory_robot_config(robot_config),
                )))
                .is_err()
        {
            defmt::error!("Failed to spawn trajectory-following task");
        }
    }

    send_ui_command(
        UiCommand::ModeChanged(target),
    )
    .await;

    defmt::info!(
        "Dual-core mode: {} -> {}",
        current.as_u8(),
        target.as_u8(),
    );
}


#[embassy_executor::task]
pub async fn core0_orchestrator_task(
    spawner: Spawner,
    robot_config: RobotConfig,
) {
    set_active_mode(RuntimeMode::Menu);
    clear_mode_inputs();

    send_logger_command(LoggerCommand::StopSession).await;
    send_ui_command(UiCommand::ModeChanged(RuntimeMode::Menu)).await;

    defmt::info!("Core 0 orchestrator started");

    loop {
        match receive_control_command().await {
            ControlCommand::SwitchMode(target) => {
                switch_mode(target, spawner, robot_config).await;
            }

            ControlCommand::ParameterWrite { parameter_id, value } => {
                // The control task applies this at the beginning of a control
                // iteration, never halfway through a PID update.
                send_parameter_update(parameter_id, value).await;
            }
            ControlCommand::TrajectoryPause => {
                set_trajectory_running(false);
                defmt::info!("Trajectory paused");
                send_ui_command(UiCommand::Beep(b's')).await;
            }

            ControlCommand::TrajectoryResume => {
                set_trajectory_running(true);
                defmt::info!("Trajectory resumed");
                send_ui_command(UiCommand::Beep(b'b')).await;
            }
        }
    }
}