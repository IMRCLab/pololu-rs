use crate::math::SO2;
use crate::orchestrator_signal::STOP_ODOM_SIG;
use crate::read_robot_config_from_sd::RobotConfig;
use crate::robotstate;
use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};

use crate::encoder::wheel_speed_from_counts_now;

//robot state from wheel odometry
#[derive(Debug, Copy, Clone)]
pub struct OdometryData {
    pub x: f32,
    pub y: f32,
    pub theta: SO2,
    pub v: f32, // linear velocity  [m/s]
    pub w: f32, // angular velocity [rad/s]
    pub timestamp: Instant,
}

impl OdometryData {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: SO2::new(0.0),
            v: 0.0,
            w: 0.0,
            timestamp: Instant::now(),
        }
    }
}


//Odometry task: integrates encoder counts into (x, y, theta) at 100 Hz.
//Responds to `STOP_ODOM_SIG` to cleanly exit when switching modes.
#[embassy_executor::task]
pub async fn odometry_task(
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cfg: Option<RobotConfig>,
    period_ms: u64,
) {
    let robot_cfg = cfg.unwrap_or_default();

    let dt: f32 = period_ms as f32 / 1000.0;
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));
    // Use lock().await to guarantee a valid starting count (try_lock could return 0
    // if the encoder task holds the mutex, causing a spurious spike on the first tick).
    let mut prev_l: i32 = *left_counter.lock().await;
    let mut prev_r: i32 = *right_counter.lock().await;
    let mut odom = OdometryData::new();

    info!(
        "Odometry task started (wheel_r={}, wheel_base={}, cpr={})",
        robot_cfg.wheel_radius, robot_cfg.wheel_base, robot_cfg.encoder_cpr
    );

    loop {
        match select(ticker.next(), STOP_ODOM_SIG.wait()).await {
            Either::First(_) => { /* normal tick */ }
            Either::Second(_) => {
                defmt::info!("odometry_task stopped by STOP_ODOM_SIG");
                return;
            }
        }

        // Raw angular velocity of each wheel [rad/s]
        let ((omega_l, omega_r), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            robot_cfg.encoder_cpr,
            prev_l,
            prev_r,
            dt,
        ).await;
        prev_l = ln;
        prev_r = rn;

        // Differential-drive kinematics as measured (not from CMD_unicycle)
        let v = (robot_cfg.wheel_radius * (omega_r + omega_l)) / 2.0;
        let w = (robot_cfg.wheel_radius * (omega_r - omega_l)) / robot_cfg.wheel_base;

        // Integrate heading
        let dtheta = w * dt;
        odom.theta = odom.theta.add(dtheta);

        // Integrate position
        odom.x += v * odom.theta.cos() * dt;
        odom.y += v * odom.theta.sin() * dt;
        odom.v = v;
        odom.w = w;
        odom.timestamp = Instant::now();

        //TODO:
        // IMU gyro yaw correction
        // let imu_gz = read gyro z from shared signal/mutex;
        // let dtheta_imu = imu_gz * dt;
        // Complementary filter: trust gyro more for short-term heading
        // let alpha = 0.98;
        // let dtheta_fused = alpha * dtheta_imu + (1.0 - alpha) * dtheta;
        // odom.theta = odom.theta.add(dtheta_fused - dtheta + dtheta); // replace encoder heading

        // Publish to shared state
        robotstate::write_odom(robotstate::OdomPose {
            x: odom.x,
            y: odom.y,
            theta: odom.theta.rad(),
            v: odom.v,
            w: odom.w,
            stamp: Instant::now(),
        }).await;


    }
}
