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
    STOP_MOCAP_UART_SIG, STOP_MOCAP_UPDATE_SIG, STOP_MOTOR_CTRL_SIG, STOP_ODOM_SIG,
    STOP_TELEOP_UART_SIG, STOP_TRAJ_OUTER_SIG, STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG,
    TRAJ_RESUME_SIG, decode_functionality_select_command, STOP_LOG_SENDING_SIG,
};
use pololu3pi2040_rs::{
    buzzer::{beep_signal, buzzer_beep_task},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    joystick_control::{control_action_uart_task, teleop_motor_control_task, teleop_uart_task},
    led::LED_SHARED,
    odometry::odometry_task,
    sdlog::SDLOGGER_SHARED,
    trajectory_control::{
        ControlMode, diffdrive_outer_loop_command_controlled_traj_following_from_sdcard,
        diffdrive_outer_loop_onboard_traj, diffdrive_outer_loop_onboard_traj2, mocap_update_task,
        wheel_speed_inner_loop,
    },
    robotstate::uart_log_sending_task,
    trajectory_signal::{LAST_STATE, POSE_FRESH, PoseAbs, TRAJECTORY_CONTROL_EVENT},
    trajectory_uart::{UartCfg, uart_motioncap_receiving_task},
    uart::{UART_RX_CHANNEL, uart_hw_task},
};
use portable_atomic::Ordering;

/// Drain any stale bytes from the UART RX channel.
/// Called between stopping old tasks and spawning new ones so that
/// duplicate command bytes (the dongle sends each command 3×) don't
/// corrupt the framing of the next UART task.
fn drain_uart_rx_channel() {
    while UART_RX_CHANNEL.try_receive().is_ok() {}
}

/// Reset shared mocap/pose state so that a new trajectory session
/// does not inherit stale data from a previous mode.
async fn reset_pose_state() {
    POSE_FRESH.store(false, Ordering::Release);
    let mut s = LAST_STATE.lock().await;
    *s = PoseAbs::default();
}

/// Clear any stale trajectory signals left from a previous session.
/// Without this, a stale TRAJ_PAUSE_SIG causes the outer loop to
/// enter the "PAUSE (idle)" branch immediately, swallowing the
/// first real start command.
fn drain_trajectory_signals() {
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();
    TRAJECTORY_CONTROL_EVENT.reset();
}

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
        .spawn(uart_log_sending_task(10, 100))
        .unwrap();

    spawner
        .spawn(orchestrator(spawner, devices, UartCfg { robot_id: 10 }))
        .unwrap();
}

#[embassy_executor::task]
pub async fn functionality_mode_selection_uart_task(cfg: UartCfg) {
    // let mut len_buf = [0u8; 1];
    let mut frame: HVec<u8, FRAME_MAX> = HVec::new();

    // Drain any stale bytes that arrived between orchestrator drain and task start
    drain_uart_rx_channel();

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

        // Only handle mode-select packets; for anything else,
        // wait briefly for the remaining payload bytes to arrive
        // from the HW UART task, then drain them so they don't
        // corrupt framing of the next packet.
        if len != LEN_FUNC_SELECT_CMD {
            Timer::after(Duration::from_millis(5)).await;
            drain_uart_rx_channel();
            continue;
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
                // timeout or error -> drop frame
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
                    5 => Mode::TrajOnboard,
                    6 => Mode::TrajOnboard2,
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

    // ============= spawn buzzer beep task ========================
    spawner.spawn(buzzer_beep_task(devices.buzzer)).unwrap();

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
                    beep_signal(b'R');
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
                        // Signal tasks to stop
                        STOP_TELEOP_UART_SIG.signal(());
                        STOP_MOTOR_CTRL_SIG.signal(());
                        STOP_LOG_SENDING_SIG.signal(());
                        defmt::warn!("Stop signals sent");

                        //wait for tasks to actually terminate (they run at 50Hz, so 40ms = 2 cycles)
                        Timer::after(Duration::from_millis(40)).await;

                        //safety: ensure motors are stopped after tasks terminate
                        devices.motor.set_speed(0.0, 0.0).await;
                        defmt::warn!("Tasks stopped, motors zeroed");

                        //drain signals to clear state
                        drain_signal(&STOP_TELEOP_UART_SIG, 2).await;
                        drain_signal(&STOP_MOTOR_CTRL_SIG, 2).await;
                        drain_signal(&STOP_LOG_SENDING_SIG, 2).await;
                    }
                    Mode::TrajMocap | Mode::TrajDuty | Mode::TrajOnboard | Mode::TrajOnboard2 => {
                        STOP_MOCAP_UART_SIG.signal(());
                        STOP_MOCAP_UPDATE_SIG.signal(());
                        STOP_WHEEL_INNER_SIG.signal(());
                        STOP_TRAJ_OUTER_SIG.signal(());
                        STOP_ODOM_SIG.signal(());
                        STOP_LOG_SENDING_SIG.signal(());

                        Timer::after(Duration::from_millis(40)).await;

                        devices.motor.set_speed(0.0, 0.0).await;

                        drain_signal(&STOP_MOCAP_UART_SIG, 2).await;
                        drain_signal(&STOP_MOCAP_UPDATE_SIG, 2).await;
                        drain_signal(&STOP_WHEEL_INNER_SIG, 2).await;
                        drain_signal(&STOP_TRAJ_OUTER_SIG, 2).await;
                        drain_signal(&STOP_ODOM_SIG, 2).await;
                        drain_signal(&STOP_LOG_SENDING_SIG, 2).await;
                    }
                }

                // Drain stale UART bytes left over from duplicate dongle sends
                // (dongle sends each command 3x). Without this, the next UART
                // task can read mid-frame bytes and lose framing permanently.
                drain_uart_rx_channel();

                // Reset shared mocap/pose state so new trajectory tasks
                // don't inherit stale position data from a previous session.
                reset_pose_state().await;

                // Drain stale trajectory start/pause/resume signals so a
                // previous session's leftovers don't confuse new tasks.
                drain_trajectory_signals();

                match target {
                    Mode::Menu => {
                        defmt::info!("Menu");
                        match spawner.spawn(functionality_mode_selection_uart_task(cfg)) {
                            Ok(_) => {
                                beep_signal(b'q'); // Quit to Menu
                            }
                            Err(_) => {
                                defmt::warn!("Menu task already running or failed to spawn");
                            }
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 100));
                    }
                    Mode::TeleOp => {
                        defmt::info!("TELE-OPERATION Mode is selected!!!!!");

                        let uart_ok = spawner.spawn(teleop_uart_task(cfg)).is_ok();
                        if !uart_ok {
                            defmt::warn!("Teleop UART task already running or failed to spawn");
                        }

                        let motor_ok = spawner
                            .spawn(teleop_motor_control_task(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        if !motor_ok {
                            defmt::warn!(
                                "Teleop motor control task already running or failed to spawn"
                            );
                        }

                        if uart_ok && motor_ok {
                            beep_signal(b'T');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
                    }
                    Mode::TrajMocap => {
                        defmt::info!("TRAJ-FOLLOWING Mode (With Mocap) is selected!!!!!");

                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let outer_ok = spawner
                            .spawn(
                                diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                                    ControlMode::WithMocapController,
                                    devices.config,
                                ),
                            )
                            .is_ok();

                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'M');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
                    }
                    Mode::TrajDuty => {
                        defmt::info!("TRAJ-FOLLOWING Mode (Directduty) is selected!!!!!");

                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let outer_ok = spawner
                            .spawn(
                                diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                                    ControlMode::DirectDuty,
                                    devices.config,
                                ),
                            )
                            .is_ok();

                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'D');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
                    }
                    Mode::CtrlAction => {
                        defmt::info!("CONTROL-ACTION Mode is selected!!!!!");

                        let uart_ok = spawner.spawn(control_action_uart_task(cfg)).is_ok();
                        if !uart_ok {
                            defmt::warn!(
                                "Control action UART task already running or failed to spawn"
                            );
                        }

                        let teleop_ok = spawner
                            .spawn(teleop_motor_control_task(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        if !teleop_ok {
                            defmt::warn!(
                                "Teleop motor control task already running or failed to spawn"
                            );
                        }

                        if uart_ok && teleop_ok {
                            beep_signal(b'A');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
                    }
                    Mode::TrajOnboard => {
                        defmt::info!("ONBOARD-TRAJ Mode (figure-8 etc.) is selected!!!!!");

                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let outer_ok = spawner
                            .spawn(diffdrive_outer_loop_onboard_traj(
                                ControlMode::DirectDuty,
                                devices.config,
                            ))
                            .is_ok();
                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'F');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
                    }
                    Mode::TrajOnboard2 => {
                        defmt::info!("ONBOARD-TRAJ-2 Mode (demo) is selected!!!!!");

                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                            ))
                            .is_ok();
                        let outer_ok = spawner
                            .spawn(diffdrive_outer_loop_onboard_traj2(
                                ControlMode::DirectDuty,
                                devices.config,
                            ))
                            .is_ok();

                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'G');
                        }
                        let _ = spawner.spawn(uart_log_sending_task(cfg.robot_id, 50));
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
