use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Instant, Ticker};
use libm::PI;

use crate::control_types::{DiffdriveCascade, DiffdriveSetpointCascade, DiffdriveStateCascade};
use crate::math::SO2;
use crate::orchestrator_signal::{STOP_TRAJ_OUTER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG};
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate::{SETPOINT_CH, TRAJECTORY_CONTROL_EVENT};
use crate::setpoint::TRAJ_REF;
use crate::trajectory_reading::{Action, Pose};

#[embassy_executor::task]
pub async fn sd_card_provider_task(robot_cfg: RobotConfig, period_ms: u64) {
    let (states, actions) = {
        let g = TRAJ_REF.lock().await;
        let t = g.borrow();
        if t.is_none() {
            defmt::error!("SD Card Trajectory is not set!");
            return;
        }
        let tr = t.unwrap();
        (tr.states, tr.actions)
    };

    let len = core::cmp::min(states.len(), actions.len());
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);

    loop {
        match select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command {
                    break;
                }
            }
            Either3::Third(_) => {
                let pause_start = Instant::now();
                match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                    Either::First(_) => {
                        pause_offset += Instant::now() - pause_start;
                    }
                    Either::Second(_) => break,
                }
                continue;
            }
        }

        if crate::orchestrator_signal::STOP_ALL.load(core::sync::atomic::Ordering::Relaxed) {
            break;
        }

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;

        let mut i = (t_sec / robot_cfg.traj_following_dt_s) as usize;
        if i >= len {
            i = len - 1;
        }

        let Pose { x: x_d, y: y_d, yaw: theta_d } = states[i + 1];
        let Action { v: vd, omega: wd } = actions[i];

        let setpoint = DiffdriveSetpointCascade {
            des: DiffdriveStateCascade {
                x: x_d,
                y: y_d,
                theta: SO2::new(theta_d),
            },
            vdes: vd,
            wdes: wd,
        };

        // Send to outer loop via blackboard channel
        while SETPOINT_CH.try_receive().is_ok() {}
        let _ = SETPOINT_CH.try_send(setpoint);
        
        if i >= len - 1 {
            // Trajectory finished
            defmt::info!("SD Card Trajectory Finished.");
            // We could break here or just hold the last point. Let's hold the last point as v=0, w=0.
            // Wait, holding v=0, w=0 is better. We'll let the user/orchestrator stop it.
        }
    }
}

#[embassy_executor::task]
pub async fn figure8_provider_task(robot_cfg: RobotConfig, period_ms: u64) {
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);
    
    // figure-8 geometry from control_types
    let dur = 30.0;
    let ax = 0.5;
    let ay = 0.3;
    let (x0, y0, phi0) = (0.0, 0.0, 0.0);
    let robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );

    loop {
        match select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command { break; }
            }
            Either3::Third(_) => {
                let pause_start = Instant::now();
                match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                    Either::First(_) => { pause_offset += Instant::now() - pause_start; }
                    Either::Second(_) => break,
                }
                continue;
            }
        }

        if crate::orchestrator_signal::STOP_ALL.load(core::sync::atomic::Ordering::Relaxed) {
            break;
        }

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;

        let setpoint = robot.figure8_reference(t_sec, dur, ax, ay, x0, y0, phi0);

        while SETPOINT_CH.try_receive().is_ok() {}
        let _ = SETPOINT_CH.try_send(setpoint);
    }
}

#[embassy_executor::task]
pub async fn spin_provider_task(robot_cfg: RobotConfig, period_ms: u64) {
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));

    let start = Instant::now();
    let mut pause_offset = Duration::from_millis(0);
    let dur = 30.0; // spin for 30 seconds
    let robot = DiffdriveCascade::new(
        robot_cfg.wheel_radius,
        robot_cfg.wheel_base,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
        -robot_cfg.wheel_max,
        robot_cfg.wheel_max,
    );
    
    let max_spin_fraction: f32 = 0.8;
    let wd_spin = 2.0 * robot_cfg.wheel_radius * robot_cfg.wheel_max / robot_cfg.wheel_base * max_spin_fraction;

    // Start pos could be extracted dynamically, but standard approach sets x0, y0, th0 to 0
    let (x0, y0, phi0) = (0.0, 0.0, 0.0);

    loop {
        match select3(ticker.next(), TRAJECTORY_CONTROL_EVENT.wait(), TRAJ_PAUSE_SIG.wait()).await {
            Either3::First(_) => {}
            Either3::Second(command) => {
                if !command { break; }
            }
            Either3::Third(_) => {
                let pause_start = Instant::now();
                match select(TRAJ_RESUME_SIG.wait(), TRAJECTORY_CONTROL_EVENT.wait()).await {
                    Either::First(_) => { pause_offset += Instant::now() - pause_start; }
                    Either::Second(_) => break,
                }
                continue;
            }
        }

        if crate::orchestrator_signal::STOP_ALL.load(core::sync::atomic::Ordering::Relaxed) {
            break;
        }

        let t = (Instant::now() - start) - pause_offset;
        let t_sec = t.as_millis() as f32 / 1000.0;

        let setpoint = robot.spinning_at_wd(wd_spin, t_sec, x0, y0, phi0);

        while SETPOINT_CH.try_receive().is_ok() {}
        let _ = SETPOINT_CH.try_send(setpoint);
    }
}
