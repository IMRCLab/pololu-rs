#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_rp::init;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Raw, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::Vec as HVec;

use pololu3pi2040_rs::init::{self, init_all};
use pololu3pi2040_rs::orchestrator_signal::{
    FRAME_MAX, LEN_FUNC_SELECT_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_MENU_UART_SIG,
    STOP_MOCAP_UART_SIG, STOP_MOCAP_UPDATE_SIG, STOP_MOTOR_CTRL_SIG, STOP_TELEOP_UART_SIG,
    STOP_TRAJ_OUTER_SIG, STOP_WHEEL_INNER_SIG, decode_functionality_select_command,
};
use pololu3pi2040_rs::{
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    joystick_control::{control_action_uart_task, teleop_motor_control_task, teleop_uart_task},
    led::LED_SHARED,
    sdlog::SDLOGGER_SHARED,
    trajectory_control::{
        ControlMode, diffdrive_outer_loop_command_controlled_traj_following_from_sdcard,
        mocap_update_task, wheel_speed_inner_loop,
    },
    trajectory_uart::{UartCfg, uart_motioncap_receiving_task},
    uart::{UART_RX_CHANNEL, uart_hw_task},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let mut devices = init_all(p);

    // === FEATURE DEBUG PRINTS ===
    #[cfg(feature = "zumo")]
    defmt::info!("RUNTIME: ZUMO feature detected!");

    #[cfg(feature = "three-pi")]
    defmt::info!("RUNTIME: THREE-PI feature detected!");

    #[cfg(not(any(feature = "zumo", feature = "three-pi")))]
    defmt::info!("RUNTIME: DEFAULT (no features) detected!");

    if let Some(_sd) = devices.sdlogger.as_mut() {
        defmt::info!("SD card is here!");
    } else {
        defmt::warn!("No SD card / SdLogger disabled, skip logging");
    }

    spawner
        .spawn(orchestrator(spawner, devices, UartCfg { robot_id: 10 }))
        .unwrap();
}

#[embassy_executor::task]
pub async fn functionality_mode_selection_uart_task(cfg: UartCfg) {
    // let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    loop {
        let read_len_fut = UART_RX_CHANNEL.receive();
        let timeout_fut = Timer::after(Duration::from_millis(1));
        let stop_fut = STOP_MENU_UART_SIG.wait();

        let len_opt: Option<u8> = match select3(read_len_fut, timeout_fut, stop_fut).await {
            Either3::First(b) => Some(b), // read len
            Either3::Second(_) => None,   // timeout, continue looping
            Either3::Third(_) => break,   // Stop top uart task signal
        };

        let Some(len) = len_opt else {
            // TimeOut Or Error
            Timer::after(Duration::from_micros(400)).await;
            continue;
        };

        if !(len == LEN_FUNC_SELECT_CMD) {
            continue; // illegal Length
        }

        // -------- read payload，until [len] bytes --------
        frame.clear();
        let need = len as usize;
        let mut got = 0usize;

        while got < need {
            let read_byte_fut = UART_RX_CHANNEL.receive();
            let timeout_fut = Timer::after(Duration::from_millis(2));
            let stop_fut = STOP_MENU_UART_SIG.wait();

            let byte_opt: Option<u8> = match select3(read_byte_fut, timeout_fut, stop_fut).await {
                Either3::First(b) => Some(b),
                Either3::Second(_) => None,
                Either3::Third(_) => {
                    return;
                }
            };

            let Some(b) = byte_opt else {
                // timeout or error → drop frame
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
        // info!("len {}", len);
        if len == LEN_FUNC_SELECT_CMD {
            if let Some(cmd) = decode_functionality_select_command(&frame, cfg.robot_id) {
                let target = match cmd {
                    0 => Mode::Menu,
                    1 => Mode::TeleOp,
                    2 => Mode::TrajMocap,
                    3 => Mode::TrajDuty,
                    4 => Mode::CtrlAction,
                    _ => continue,
                };
                let _ = ORCH_CH.try_send(OrchestratorMsg::SwitchTo(target));
            }
        }
    }
    defmt::info!("UART functionality_mode_selection task stopped.");
}

/* ======================== Functionality Selection Task ============================ */
#[embassy_executor::task]
pub async fn orchestrator(spawner: Spawner, mut devices: init::InitDevices<'static>, cfg: UartCfg) {
    if let Some(led_dev) = devices.led.take() {
        let mut g = LED_SHARED.lock().await;
        *g = Some(led_dev);
    }
    if let Some(sd) = devices.sdlogger.take() {
        let mut g = SDLOGGER_SHARED.lock().await;
        *g = Some(sd);
    }

    // ================ Spawn Encoder Task ==========================
    let EncoderPair {
        encoder_left,
        encoder_right,
    } = devices.encoders;
    let encoder_count_left = devices.encoder_counts.left;
    let encoder_count_right = devices.encoder_counts.right;
    spawner
        .spawn(encoder_left_task(encoder_left, encoder_count_left))
        .unwrap();
    spawner
        .spawn(encoder_right_task(encoder_right, encoder_count_right))
        .unwrap();

    // ============= spawn low level uart task ======================
    spawner.spawn(uart_hw_task(devices.uart)).unwrap();

    // ============== spawn func select task ========================
    spawner
        .spawn(functionality_mode_selection_uart_task(cfg))
        .unwrap();
    let mut mode = Mode::Menu;

    defmt::info!("Orchestrator Task Launched! Initial Mode = Menu");

    loop {
        // Wait for switching command sent by any task
        let msg = ORCH_CH.receive().await;

        match msg {
            OrchestratorMsg::SwitchTo(target) => {
                defmt::info!("Orchestrator received mode switch request");
                if target == mode {
                    info!("Already in target mode, ignoring");
                    continue;
                }

                // Stop current running tasks
                match mode {
                    Mode::Menu => {
                        STOP_MENU_UART_SIG.signal(());
                        // TASK_SELECT_UART_STOP_SIG.signal(());

                        Timer::after(Duration::from_millis(2)).await;

                        drain_signal(&STOP_MENU_UART_SIG, 2).await;
                    }
                    Mode::TeleOp | Mode::CtrlAction => {
                        defmt::info!("Stopping TeleOp/CtrlAction tasks...");
                        STOP_TELEOP_UART_SIG.signal(());
                        STOP_MOTOR_CTRL_SIG.signal(());

                        Timer::after(Duration::from_millis(50)).await; // Wait longer for tasks to exit

                        drain_signal(&STOP_TELEOP_UART_SIG, 2).await;
                        drain_signal(&STOP_MOTOR_CTRL_SIG, 2).await;
                        defmt::info!("TeleOp/CtrlAction tasks stopped");
                    }
                    Mode::TrajMocap | Mode::TrajDuty => {
                        STOP_MOCAP_UART_SIG.signal(());
                        STOP_MOCAP_UPDATE_SIG.signal(());
                        STOP_WHEEL_INNER_SIG.signal(());
                        STOP_TRAJ_OUTER_SIG.signal(());

                        Timer::after(Duration::from_millis(2)).await;

                        drain_signal(&STOP_MOCAP_UART_SIG, 2).await;
                        drain_signal(&STOP_MOCAP_UPDATE_SIG, 2).await;
                        drain_signal(&STOP_WHEEL_INNER_SIG, 2).await;
                        drain_signal(&STOP_TRAJ_OUTER_SIG, 2).await;
                    }
                }

                match target {
                    Mode::Menu => {
                        defmt::info!("Menu");
                        if let Err(_) = spawner.spawn(functionality_mode_selection_uart_task(cfg)) {
                            defmt::warn!("Menu task already running or failed to spawn");
                        }
                    }
                    Mode::TeleOp => {
                        defmt::info!("TELE-OPERATION Mode is selected!!!!!");

                        if let Err(_) = spawner.spawn(teleop_uart_task(cfg)) {
                            defmt::warn!("Teleop UART task already running or failed to spawn");
                        }

                        if let Err(_) = spawner.spawn(teleop_motor_control_task(
                            devices.motor,
                            encoder_count_left,
                            encoder_count_right,
                            devices.config,
                        )) {
                            defmt::warn!("Teleop motor control task already running or failed to spawn");
                        }
                    }
                    Mode::TrajMocap => {
                        defmt::info!("TRAJ-FOLLOWING Mode (With Mocap) is selected!!!!!");

                        spawner.spawn(uart_motioncap_receiving_task(cfg)).unwrap();
                        spawner.spawn(mocap_update_task()).unwrap();
                        spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .unwrap();
                        spawner
                            .spawn(
                                diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                                    ControlMode::WithMocapController,
                                    devices.config,
                                ),
                            )
                            .unwrap();
                    }
                    Mode::TrajDuty => {
                        defmt::info!("TRAJ-FOLLOWING Mode (Directduty) is selected!!!!!");

                        spawner.spawn(uart_motioncap_receiving_task(cfg)).unwrap();
                        spawner.spawn(mocap_update_task()).unwrap();
                        spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .unwrap();
                        spawner
                            .spawn(
                                diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                                    ControlMode::DirectDuty,
                                    devices.config,
                                ),
                            )
                            .unwrap();
                    }
                    Mode::CtrlAction => {
                        defmt::info!("CONTROL-ACTION Mode is selected!!!!!");

                        if let Err(_) = spawner.spawn(control_action_uart_task(cfg)) {
                            defmt::warn!("Control action UART task already running or failed to spawn");
                        }

                        if let Err(_) = spawner.spawn(teleop_motor_control_task(
                            devices.motor,
                            encoder_count_left,
                            encoder_count_right,
                            devices.config,
                        )) {
                            defmt::warn!("Teleop motor control task already running or failed to spawn");
                        }
                    }
                }
                mode = target;
            }
        }
    }
}
/* ================================================================================== */

pub async fn drain_signal_once(sig: &Signal<Raw, ()>) {
    match select(sig.wait(), Timer::after(Duration::from_micros(1))).await {
        Either::First(_) => {
            defmt::trace!("drained a stale stop signal");
        }
        Either::Second(_) => { /* nothing to drain */ }
    }
}

pub async fn drain_signal(sig: &Signal<Raw, ()>, repeats: u8) {
    for _ in 0..repeats {
        match select(sig.wait(), Timer::after(Duration::from_micros(1))).await {
            Either::First(_) => { /* drained one */ }
            Either::Second(_) => break,
        }
    }
}
