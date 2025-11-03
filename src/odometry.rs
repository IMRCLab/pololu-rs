use crate::math::SO2;
use defmt::info;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};

//use crate::imu::IMU;

use crate::encoder::wheel_speed_from_counts_now;
use crate::robot_parameters_default::robot_constants::*;

//TODO: when tested, use speed calculation from the odometry loop for the inner loop speed control, so its not done twice.
//note: wheel odometer for robots running in flat space only without mocap system available.
#[derive(Debug, Copy, Clone)]
pub struct OdometryData {
    pub x: f32,
    pub y: f32,
    pub theta: SO2,
    pub v: f32, // linear velocity m/s
    pub w: f32, // angular velocity rad/s
    pub timestamp: Instant,
}

//init as zero, since there is no reference
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

//Channel for Odometry data
pub static ODOMETRY_CHANNEL: Channel<ThreadModeRawMutex, OdometryData, 4> = Channel::new();

#[embassy_executor::task]
pub async fn odometry_task(
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
) {
    // Use the constants directly, since they are imported from robot_constants

    let mut ticker = Ticker::every(Duration::from_millis(20));
    let dt: f32 = 0.02; // 20 ms
    let mut prev_l: i32 = 0; //needed for delta calculation in wheel speed
    let mut prev_r: i32 = 0;
    let mut odom = OdometryData::new();
    info!("Odometry task started");
    let sender = ODOMETRY_CHANNEL.sender();

    //let pose be updated here from the last known pose

    loop {
        ticker.next().await;
        // raw angular velocity of the wheel
        let ((omega_l, omega_r), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            ENCODER_CPR,
            prev_l,
            prev_r,
            dt,
        );
        prev_l = ln;
        prev_r = rn;

        //update the pose with robot dynamics:
        //note: difference to step in trajectory_control is that we are using the measured speed instead of propagating with control output for the motors
        let v = (WHEEL_RADIUS * (omega_r + omega_l)) / 2.0; // linear velocity
        let w = (WHEEL_RADIUS * (omega_r - omega_l)) / WHEEL_BASE; // angular velocity

        //heading
        let dtheta = w * dt;
        odom.theta = odom.theta.add(dtheta);

        odom.x += v * odom.theta.cos() * dt;
        odom.y += v * odom.theta.sin() * dt;
        odom.v = v;
        odom.w = w;
        odom.timestamp = Instant::now();

        //variant where the imu is used to correct the heading drift
        // let imu_angular_velocity = imu.get_angular_velocity_z(); // in rad/s
        // let w_imu = imu_angular_velocity;
        // let dtheta_imu = w_imu * dt;
        // odom.theta = odom.theta.add(dtheta_imu);

        //publish the newly derived odometry data
        let _ = sender.try_send(odom);
        // defmt::info!(
        //     "Odometry: x={}, y={}, theta={}, v={}, w={}",
        //     odom.x,
        //     odom.y,
        //     odom.theta.rad(),
        //     odom.v,
        //     odom.w
        // );
    }
}

//helper function for other modules to get the latest odometry data
pub async fn get_latest_odometry() -> Option<OdometryData> {
    let receiver = ODOMETRY_CHANNEL.receiver();
    receiver.try_receive().ok()
}

//helper function to wait for new odometry data - ensuring the data is fresh
pub async fn wait_for_odometry() -> OdometryData {
    let receiver = ODOMETRY_CHANNEL.receiver();
    receiver.receive().await
}
