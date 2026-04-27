use crate::encoder::wheel_speed_from_counts_now;
use crate::motor::MotorController;
use crate::orchestrator_signal::{STOP_WHEEL_INNER_SIG, TRAJ_PAUSE_SIG, TRAJ_RESUME_SIG};
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate::{self, WheelCmd, WHEEL_CMD_CH};
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::{AtomicU32, Ordering};

static INNER_LOOP_TICK: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn wheel_speed_inner_loop(
    motor: MotorController,
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cfg: Option<RobotConfig>,
    period_ms: u64,
) {
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        robot_cfg = cfg.unwrap();
    } else {
        robot_cfg = RobotConfig::default();
    }

    let mut ticker = Ticker::every(Duration::from_millis(period_ms));
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32);
    let (kp, ki, kd) = (robot_cfg.kp_inner, robot_cfg.ki_inner, robot_cfg.kd_inner);

    // =========== Filter Parameters ==============
    let dt: f32 = period_ms as f32 / 1000.0;
    let fc_hz: f32 = 3.0;
    let tau: f32 = 1.0 / (2.0 * core::f32::consts::PI * fc_hz);
    let alpha: f32 = dt / (tau + dt);

    //need to get current state of the encoders, because it is potentially leading to v spikes when switching programs.
    let mut prev_l = *left_counter.lock().await;
    let mut prev_r = *right_counter.lock().await;

    let mut omega_l_lp: f32 = 0.0;
    let mut omega_r_lp: f32 = 0.0;

    let mut last_cmd = WheelCmd {
        omega_l: 0.0,
        omega_r: 0.0,
        stamp: Instant::now(),
    };

    loop {
        match select3(STOP_WHEEL_INNER_SIG.wait(), TRAJ_PAUSE_SIG.wait(), ticker.next()).await {
            Either3::First(_) => {
                motor.set_speed(0.0, 0.0).await;
                defmt::warn!("STOP inner loop -> exit");
                return;
            }
            Either3::Second(_) => {
                motor.set_speed(0.0, 0.0).await;
                match select(TRAJ_RESUME_SIG.wait(), STOP_WHEEL_INNER_SIG.wait()).await {
                    Either::First(_) => {},        // back to normal loop
                    Either::Second(_) => {
                        motor.set_speed(0.0, 0.0).await;
                        return;
                    }
                }
                //reset the controller errors before resuming to avoid v spikes.
                il = 0.0; ir = 0.0;
                prev_el = 0.0; prev_er = 0.0;
                omega_l_lp = 0.0; omega_r_lp = 0.0;
                prev_l = *left_counter.lock().await;
                prev_r = *right_counter.lock().await;
                last_cmd = WheelCmd { omega_l: 0.0, omega_r: 0.0, stamp: Instant::now() };
                continue;
            }
            Either3::Third(_) => {
                // Normal loop - ticker fired
            }
        }

        while let Ok(cmd) = WHEEL_CMD_CH.try_receive() {
            // defmt::info!("Inner Loop: New WheelCmd received (L:{}, R:{})", cmd.omega_l, cmd.omega_r);
            last_cmd = cmd; // drain the channel
        }

        // raw angular velocity of the wheel
        let ((omega_l_raw, omega_r_raw), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            robot_cfg.encoder_cpr,
            prev_l,
            prev_r,
            dt,
        ).await;
        prev_l = ln;
        prev_r = rn;

        // =========== Low Pass Filter ==============
        omega_l_lp = omega_l_lp + alpha * (omega_l_raw - omega_l_lp);
        omega_r_lp = omega_r_lp + alpha * (omega_r_raw - omega_r_lp);

        // error
        let el = last_cmd.omega_l - omega_l_lp;
        let er = last_cmd.omega_r - omega_r_lp;

        // error integration
        il = (il + ki * dt * el).clamp(-2.0, 2.0);
        ir = (ir + ki * dt * er).clamp(-2.0, 2.0);

        // error differentiation
        let dl = (el - prev_el) / dt;
        let dr = (er - prev_er) / dt;

        prev_el = el;
        prev_er = er;

        // PID (normally the D term is disabled, but just in case)
        let u_l = (kp * el + il + kd * dl).clamp(-1.0, 1.0);
        let u_r = (kp * er + ir + kd * dr).clamp(-1.0, 1.0);

        let duty_l = u_l * robot_cfg.motor_direction_left;
        let duty_r = u_r * robot_cfg.motor_direction_right;

        motor.set_speed(duty_l, duty_r).await;

        // Write motor duty and encoder readings to robotstate
        robotstate::write_motor(robotstate::MotorDuty {
            left: duty_l,
            right: duty_r,
        }).await;
        robotstate::write_encoder(robotstate::EncoderReading {
            omega_l: omega_l_lp,
            omega_r: omega_r_lp,
            stamp: Instant::now(),
        }).await;

        let n = INNER_LOOP_TICK.fetch_add(1, Ordering::Relaxed);
        if n % 100 == 0 {
            defmt::info!("InnerLoop [{}]: Target=(L:{}, R:{}), Duty=(L:{}, R:{})", 
                n, last_cmd.omega_l, last_cmd.omega_r, duty_l, duty_r);
        }
    }
}
