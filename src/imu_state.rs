use embassy_time::{Duration, Instant, Timer};
use crate::trajectory_signal::{LAST_STATE, PoseAbsStamped};
use crate::imu::IMU_ANGLES_SHARED;
use crate::encoder::EncoderCounters;
use defmt::info;
use libm::{cosf, sinf, PI};

use crate::encoder::wheel_speed_from_counts_now;
use crate::robot_parameters_default::robot_constants::{ENCODER_CPR, WHEEL_RADIUS};

#[embassy_executor::task]
pub async fn imu_estimator_task(
    counters: EncoderCounters,
) {
    info!("IMU Estimator Task Started");

    let mut prev_left = 0;
    let mut prev_right = 0;

    loop {
        // 1. Get current state (Mocap might have updated this)
        let mut slammed_state = {
            let g = LAST_STATE.lock().await;
            *g 
        };

        let now = Instant::now();
        // Calculate dt from the last valid state time
        // This handles cases where Mocap injects a new state asynchronously.
        let dt = now.duration_since(slammed_state.timestamp).as_secs_f32();

        // 2. Get Yaw from IMU
        let (pitch, roll, yaw_deg) = {
            let g = IMU_ANGLES_SHARED.lock().await;
            *g
        };
        let yaw_rad = yaw_deg * PI / 180.0;

        // 3. Get Encoder Counts & Speed
        let ((omega_l, omega_r), (left_now, right_now)) = wheel_speed_from_counts_now(
            &counters.left,
            &counters.right,
            ENCODER_CPR,
            prev_left,
            prev_right,
            dt
        );
        prev_left = left_now;
        prev_right = right_now;

        let vl = omega_l * WHEEL_RADIUS;
        let vr = omega_r * WHEEL_RADIUS;
        let v_linear = (vl + vr) / 2.0;
        
        // 4. Integrate
        // Update yaw from IMU
        slammed_state.pose.yaw = yaw_rad; 
        
        // Dead reckoning for X/Y
        slammed_state.pose.x += v_linear * cosf(yaw_rad) * dt;
        slammed_state.pose.y += v_linear * sinf(yaw_rad) * dt;
        
        // 5. Write back
        // Update timestamp to NOW
        slammed_state.timestamp = now; 
        
        {
            let mut g = LAST_STATE.lock().await;
            *g = slammed_state;
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}