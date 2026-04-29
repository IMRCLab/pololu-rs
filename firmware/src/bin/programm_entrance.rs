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
    FRAME_MAX, LEN_FUNC_SELECT_CMD, Mode, ORCH_CH, OrchestratorMsg, STOP_LOG_SENDING_SIG,
    STOP_MENU_UART_SIG, STOP_MOCAP_UART_SIG, STOP_MOCAP_UPDATE_SIG, STOP_MOTOR_CTRL_SIG,
    STOP_ODOM_SIG, STOP_POSE_EST_SIG, STOP_TELEOP_UART_SIG, STOP_TRAJ_OUTER_SIG,
    STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG, decode_functionality_select_command,
};
use pololu3pi2040_rs::robotstate;
use pololu3pi2040_rs::{
    buzzer::{beep_signal, buzzer_beep_task},
    ekf::mocap_update_task,
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    inner_controller::wheel_speed_inner_loop,
    joystick_control::{control_action_uart_task, teleop_motor_control_task, teleop_uart_task},
    led::LED_SHARED,
    odometry::odometry_task,
    robotstate::TRAJECTORY_CONTROL_EVENT,
    robotstate::uart_log_sending_task,
    sdlog::{SDLOGGER_SHARED, sd_logging_task},
    trajectory_control::{
        diffdrive_outer_loop_command_controlled_traj_following_from_sdcard,
        diffdrive_outer_loop_onboard_traj, diffdrive_outer_loop_onboard_traj2,
    },
    trajectory_uart::{UartCfg, uart_motioncap_receiving_task},
    uart::{UART_RX_CHANNEL, uart_hw_task},
};

/// Drain any stale bytes from the UART RX channel.
/// Called between stopping old tasks and spawning new ones so that
/// duplicate command bytes (the dongle sends each command 3×) don't
/// corrupt the framing of the next UART task.
fn drain_uart_rx_channel() {
    while UART_RX_CHANNEL.try_receive().is_ok() {}
}

/// Reset all shared robotstate so that a new session
/// does not inherit stale data from a previous mode.
async fn reset_pose_state() {
    robotstate::reset_all().await;
}

/// Clear any stale trajectory signals left from a previous session.
/// Without this, a stale TRAJ_PAUSE_SIG causes the outer loop to
/// enter the "PAUSE (idle)" branch immediately, swallowing the
/// first real start command.
fn drain_trajectory_signals() {
    TRAJ_PAUSE_SIG.reset();
    TRAJ_RESUME_SIG.reset();
    TRAJECTORY_CONTROL_EVENT.reset();
    // Reset so a stale signal doesn't kill the next EKF task on its first select() poll.
    STOP_POSE_EST_SIG.reset();
}

async fn wait_for_ekf_init() -> robotstate::EkfInitMsg {
    match select(robotstate::MOCAP_SIG.wait(), Timer::after_millis(300)).await {
        Either::First(pose) => {
            defmt::info!(
                "EKF init from mocap: ({}, {}, {})",
                pose.x,
                pose.y,
                pose.yaw
            );
            robotstate::EkfInitMsg {
                x: pose.x,
                y: pose.y,
                yaw: pose.yaw,
            }
        }
        Either::Second(_) => {
            if let Some((x, y, yaw)) =
                pololu3pi2040_rs::setpoint::trajectory_start_pose().await
            {
                defmt::info!(
                    "EKF init from trajectory start: ({}, {}, {}) (no mocap)",
                    x,
                    y,
                    yaw
                );
                robotstate::EkfInitMsg { x, y, yaw }
            } else {
                defmt::warn!("EKF init fallback to origin (0,0,0)");
                robotstate::EkfInitMsg {
                    x: 0.0,
                    y: 0.0,
                    yaw: 0.0,
                }
            }
        }
    }
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

        // Only handle mode-select packets and parameter updates (len 8); for anything else,
        // wait briefly for the remaining payload bytes to arrive
        // from the HW UART task, then drain them so they don't
        // corrupt framing of the next packet.
        if len != LEN_FUNC_SELECT_CMD && len != 8 {
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
                    4 => Mode::CtrlAction,
                    5 => Mode::TrajOnboard,
                    6 => Mode::TrajOnboard2,
                    _ => continue,
                };
                let _ = ORCH_CH.try_send(OrchestratorMsg::SwitchTo(target));
            }
            continue;
        }

        if len == 8 {
            if let Some(req) =
                pololu3pi2040_rs::parameter_sync::ParameterWriteRequest::from_bytes(&frame)
            {
                pololu3pi2040_rs::parameter_sync::handle_parameter_write(req.param_id, req.value)
                    .await;
            }
            continue;
        }
    }
    defmt::info!("UART functionality_mode_selection task stopped.");
}

/* ======================== Functionality Selection Task ============================ */

// Task tick periods [ms] — single source of truth for all task frequencies
const ODOM_PERIOD_MS: u64 = 10;      // 100 Hz
const INNER_PERIOD_MS: u64 = 10;     // 100 Hz
const LOG_PERIOD_MS: u64 = 50;       // 20 Hz (Menu), 50 (control modes)
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
    // ============== Initialize Global Parameter Config ============
    let global_config = devices.config.unwrap_or_default();
    pololu3pi2040_rs::parameter_sync::init_robot_config(global_config.clone());

    defmt::info!("Orchestrator Task Launched! Initial Mode = Menu");
    // Optionally spawn a task to sync all parameters once on boot

    let mut mode = Mode::Menu;

    loop {
        // Wait for switching command sent by any task
        let msg = ORCH_CH.receive().await;
        match msg {
            OrchestratorMsg::SwitchTo(target) => {
                defmt::info!("Orchestrator: {} -> {}", mode, target);

                if target == mode {
                    info!("Already in target mode, ignoring");
                    beep_signal(b'R');
                    continue;
                }

                // Stop current running tasks
                match mode {
                    Mode::Menu => {
                        STOP_MENU_UART_SIG.signal(());
                        STOP_LOG_SENDING_SIG.signal(());

                        Timer::after(Duration::from_millis(2)).await;

                        drain_signal(&STOP_MENU_UART_SIG, 2).await;
                        drain_signal(&STOP_LOG_SENDING_SIG, 2).await;
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
                    Mode::TrajMocap | Mode::TrajOnboard | Mode::TrajOnboard2 => {
                        STOP_MOCAP_UART_SIG.signal(());
                        STOP_MOCAP_UPDATE_SIG.signal(());
                        STOP_WHEEL_INNER_SIG.signal(());
                        STOP_TRAJ_OUTER_SIG.signal(());
                        STOP_ODOM_SIG.signal(());
                        STOP_LOG_SENDING_SIG.signal(());
                        STOP_POSE_EST_SIG.signal(());

                        // 100 ms gives >=10 EKF cycles (10 ms each) -- enough time
                        // for the task to exit even if blocked inside a mutex.
                        Timer::after(Duration::from_millis(100)).await;

                        devices.motor.set_speed(0.0, 0.0).await;

                        drain_signal(&STOP_MOCAP_UART_SIG, 2).await;
                        drain_signal(&STOP_MOCAP_UPDATE_SIG, 2).await;
                        drain_signal(&STOP_WHEEL_INNER_SIG, 2).await;
                        drain_signal(&STOP_TRAJ_OUTER_SIG, 2).await;
                        drain_signal(&STOP_ODOM_SIG, 2).await;
                        drain_signal(&STOP_LOG_SENDING_SIG, 2).await;
                        drain_signal(&STOP_POSE_EST_SIG, 2).await;
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
                        pololu3pi2040_rs::parameter_sync::send_mode(0).await;
                        match spawner.spawn(functionality_mode_selection_uart_task(cfg)) {
                            Ok(_) => {
                                beep_signal(b'q'); // Quit to Menu
                            }
                            Err(_) => {
                                defmt::warn!("Menu task already running or failed to spawn");
                            }
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, 100)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                    }
                    Mode::TeleOp => {
                        defmt::info!("TELE-OPERATION Mode is selected!!!!!");
                        pololu3pi2040_rs::parameter_sync::send_mode(1).await;
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
                            defmt::info!("TeleOp: UART and Motor tasks active");
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, LOG_PERIOD_MS)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                    }
                    Mode::TrajMocap => {
                        defmt::info!("TRAJ-FOLLOWING Mode (With Mocap) is selected!!!!!");
                        pololu3pi2040_rs::parameter_sync::send_mode(2).await;
                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                ODOM_PERIOD_MS,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                INNER_PERIOD_MS,
                            ))
                            .is_ok();

                        // (The EKF is now inlined in the unified control loop)
                        // Resolve initial pose (<=300 ms wait). Outer loop spawned after
                        // so it sees a valid EKF_STATE, but inner/odometry tasks are
                        // no longer blocked during this window.
                        let init_msg = wait_for_ekf_init().await;
                        robotstate::write_ekf_state(robotstate::RobotPose {
                            x: init_msg.x,
                            y: init_msg.y,
                            yaw: init_msg.yaw,
                            stamp: embassy_time::Instant::now(),
                            ..robotstate::RobotPose::DEFAULT
                        }).await;

                        let outer_ok = spawner
                            .spawn(
                                diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                                    devices.config,
                                ),
                            )
                            .is_ok();

                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'M');
                            defmt::info!(
                                "TrajMocap: All tasks active (Uart, Mocap, Odo, Inner, Outer)"
                            );
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, LOG_PERIOD_MS)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                        if spawner.spawn(sd_logging_task(devices.config)).is_err() {
                            defmt::warn!("Failed to spawn sd_logging_task");
                        }
                    }
                    Mode::CtrlAction => {
                        defmt::info!("CONTROL-ACTION Mode is selected!!!!!");
                        pololu3pi2040_rs::parameter_sync::send_mode(4).await;
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
                            defmt::info!("CtrlAction: UART and Motor tasks active");
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, LOG_PERIOD_MS)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                    }
                    Mode::TrajOnboard => {
                        defmt::info!("ONBOARD-TRAJ Mode (figure-8 etc.) is selected!!!!!");
                        pololu3pi2040_rs::parameter_sync::send_mode(5).await;
                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                ODOM_PERIOD_MS,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                INNER_PERIOD_MS,
                            ))
                            .is_ok();

                        // let ekf_ok = spawner.spawn(ekf_estimator_task(devices.config, EKF_PERIOD_MS)).is_ok();

                        // Resolve initial pose (<=300 ms wait).
                        let init_msg = wait_for_ekf_init().await;
                        robotstate::write_ekf_state(robotstate::RobotPose {
                            x: init_msg.x,
                            y: init_msg.y,
                            yaw: init_msg.yaw,
                            stamp: embassy_time::Instant::now(),
                            ..robotstate::RobotPose::DEFAULT
                        }).await;

                        let outer_ok = spawner
                            .spawn(diffdrive_outer_loop_onboard_traj(devices.config))
                            .is_ok();
                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'F');
                            defmt::info!(
                                "TrajOnboard: All tasks active (Uart, Mocap, Odo, Inner, Outer)"
                            );
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, LOG_PERIOD_MS)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                        if spawner.spawn(sd_logging_task(devices.config)).is_err() {
                            defmt::warn!("Failed to spawn sd_logging_task");
                        }
                    }
                    Mode::TrajOnboard2 => {
                        defmt::info!("ONBOARD-TRAJ-2 Mode (demo) is selected!!!!!");
                        pololu3pi2040_rs::parameter_sync::send_mode(6).await;
                        let uart_ok = spawner.spawn(uart_motioncap_receiving_task(cfg)).is_ok();
                        let mocap_ok = spawner.spawn(mocap_update_task()).is_ok();
                        let odo_ok = spawner
                            .spawn(odometry_task(
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                ODOM_PERIOD_MS,
                            ))
                            .is_ok();
                        let inner_ok = spawner
                            .spawn(wheel_speed_inner_loop(
                                devices.motor,
                                encoder_count_left,
                                encoder_count_right,
                                devices.config,
                                INNER_PERIOD_MS,
                            ))
                            .is_ok();

                        // let ekf_ok = spawner.spawn(ekf_estimator_task(devices.config, EKF_PERIOD_MS)).is_ok();

                        // Resolve initial pose (<=300 ms wait).
                        let init_msg = wait_for_ekf_init().await;
                        robotstate::write_ekf_state(robotstate::RobotPose {
                            x: init_msg.x,
                            y: init_msg.y,
                            yaw: init_msg.yaw,
                            stamp: embassy_time::Instant::now(),
                            ..robotstate::RobotPose::DEFAULT
                        }).await;

                        let outer_ok = spawner
                            .spawn(diffdrive_outer_loop_onboard_traj2(devices.config))
                            .is_ok();

                        if uart_ok && mocap_ok && odo_ok && inner_ok && outer_ok {
                            beep_signal(b'G');
                            defmt::info!(
                                "TrajOnboard2: All tasks active (Uart, Mocap, Odo, Inner, Outer)"
                            );
                        }
                        if spawner.spawn(uart_log_sending_task(cfg.robot_id, LOG_PERIOD_MS)).is_err() {
                            defmt::warn!("Failed to spawn uart_log_sending_task");
                        }
                        if spawner.spawn(sd_logging_task(devices.config)).is_err() {
                            defmt::warn!("Failed to spawn sd_logging_task");
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
