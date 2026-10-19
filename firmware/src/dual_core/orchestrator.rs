//! Dual-core mode orchestrator.
//!
//! This task handles low-frequency lifecycle events. It never owns motors and
//! never performs control calculations.

use crate::dual_core::{
    messages::{ControlCommand, LoggerCommand, RuntimeMode, UiCommand},
    shared_state::{
        active_mode, clear_mode_inputs, receive_control_command, send_logger_command,
        send_parameter_update, send_ui_command, set_active_mode, set_trajectory_running,
    },
};

async fn switch_mode(target: RuntimeMode) {
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
pub async fn core0_orchestrator_task() {
    set_active_mode(RuntimeMode::Menu);
    clear_mode_inputs();

    send_logger_command(LoggerCommand::StopSession).await;
    send_ui_command(UiCommand::ModeChanged(RuntimeMode::Menu)).await;

    defmt::info!("Core 0 orchestrator started");

    loop {
        match receive_control_command().await {
            ControlCommand::SwitchMode(target) => {
                switch_mode(target).await;
            }

            ControlCommand::ParameterWrite { parameter_id, value } => {
                // The control task applies this at the beginning of a control
                // iteration, never halfway through a PID update.
                send_parameter_update(parameter_id, value).await;
            }

            ControlCommand::TrajectoryPause => {
                set_trajectory_running(false);
                send_ui_command(UiCommand::Beep(b's')).await;
            }

            ControlCommand::TrajectoryResume => {
                set_trajectory_running(true);
                send_ui_command(UiCommand::Beep(b'b')).await;
            }
        }
    }
}