//! Permanent Core 1 LED and buzzer task.

use embassy_time::{
    Duration,
    Timer,
};

use crate::{
    dual_core::{
        messages::{
            RuntimeMode,
            UiCommand,
        },
        shared_state::receive_ui_command,
    },
    peripherals::{
        buzzer::Buzzer,
        led::Led,
    },
};

async fn short_beep(
    buzzer: &mut Buzzer,
) {
    buzzer.tone(1200, 0.15);

    Timer::after(
        Duration::from_millis(80),
    )
    .await;

    buzzer.stop();

    Timer::after(
        Duration::from_millis(40),
    )
    .await;
}

async fn long_beep(
    buzzer: &mut Buzzer,
) {
    buzzer.tone(800, 0.4);

    Timer::after(
        Duration::from_millis(150),
    )
    .await;

    buzzer.stop();

    Timer::after(
        Duration::from_millis(50),
    )
    .await;
}

async fn timeout_beep(
    buzzer: &mut Buzzer,
) {
    buzzer.tone(1500, 0.05);

    Timer::after(
        Duration::from_millis(40),
    )
    .await;

    buzzer.stop();
}

async fn signal_mode(
    mode: RuntimeMode,
    buzzer: &mut Buzzer,
) {
    match mode {
        RuntimeMode::Menu => {
            short_beep(buzzer).await;
        }

        RuntimeMode::TeleOp => {
            short_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        RuntimeMode::TrajMocap => {
            short_beep(buzzer).await;
            short_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        RuntimeMode::CtrlAction => {
            short_beep(buzzer).await;
            long_beep(buzzer).await;
        }

        RuntimeMode::TrajOnboard => {
            long_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        RuntimeMode::TrajOnboard2 => {
            long_beep(buzzer).await;
            short_beep(buzzer).await;
            long_beep(buzzer).await;
        }
    }
}

async fn signal_code(
    code: u8,
    buzzer: &mut Buzzer,
) {
    match code {
        b'q' => {
            short_beep(buzzer).await;
        }

        b'T' => {
            short_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        b'M' => {
            short_beep(buzzer).await;
            short_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        b'A' => {
            short_beep(buzzer).await;
            long_beep(buzzer).await;
        }

        b'F' => {
            long_beep(buzzer).await;
            short_beep(buzzer).await;
        }

        b'G' => {
            long_beep(buzzer).await;
            short_beep(buzzer).await;
            long_beep(buzzer).await;
        }

        _ => {
            timeout_beep(buzzer).await;
        }
    }
}

/// The only task allowed to operate LED and buzzer hardware.
#[embassy_executor::task]
pub async fn core1_ui_task(
    mut led: Option<Led>,
    mut buzzer: Buzzer,
) {
    buzzer.stop();

    if let Some(led) = led.as_mut() {
        led.off();
    }

    defmt::info!("Core 1 UI task started");

    loop {
        match receive_ui_command().await {
            UiCommand::ModeChanged(mode) => {
                if let Some(led) = led.as_mut() {
                    led.set(mode != RuntimeMode::Menu);
                }

                signal_mode(
                    mode,
                    &mut buzzer,
                )
                .await;
            }

            UiCommand::Error => {
                if let Some(led) = led.as_mut() {
                    led.on();
                }

                long_beep(&mut buzzer).await;
                long_beep(&mut buzzer).await;
            }

            UiCommand::CommandTimeout => {
                timeout_beep(
                    &mut buzzer,
                )
                .await;
            }

            UiCommand::Beep(code) => {
                signal_code(
                    code,
                    &mut buzzer,
                )
                .await;
            }

            UiCommand::SetLed(enabled) => {
                if let Some(led) = led.as_mut() {
                    led.set(enabled);
                }
            }
        }
    }
}