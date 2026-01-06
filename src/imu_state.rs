use embassy_time::{Duration, Instant, Timer};
use crate::trajectory_signal::{LAST_STATE, PoseAbsStamped};
use crate::imu::IMU_ANGLES_SHARED;
use crate::encoder::EncoderCounters;
use defmt::info;
use libm::{cosf, sinf, PI};

#[embassy_executor::task]
pub async fn imu_estimator_task(
    counters: EncoderCounters,
) {
    info!("IMU Estimator Task Started");
    loop {
        // 1. Get current state (Mocap might have updated this)
        let mut slammed_state = {
            let g = LAST_STATE.lock().await;
            *g 
        };

        let now = Instant::now();
        // Calculate dt from the last valid state time, not the last loop run.
        // This handles cases where Mocap injects a new state asynchronously.
        let dt = now.duration_since(slammed_state.timestamp).as_secs_f32();

        // 2. Get Yaw from IMU
        let (pitch, roll, yaw_deg) = {
            let g = IMU_ANGLES_SHARED.lock().await;
            *g
        };
        let yaw_rad = yaw_deg * PI / 180.0;

        // 3. Get Encoder Counts (Boilerplate placeholder for now)
        // Access counters.left and counters.right here if needed
        // let left_counts = *counters.left.lock().await;
        
        // 4. Integrate (Boilerplate math)
        // For now, just update the timestamp and maybe yaw
        slammed_state.pose.yaw = yaw_rad; // Trust IMU yaw
        // slammed_state.pose.x += 0.0 * dt; 
        
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