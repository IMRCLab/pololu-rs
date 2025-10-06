#![no_std]
#![no_main]

use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::odometry::odometry_task;
use pololu3pi2040_rs::trajectory_control::wheel_speed_inner_loop;
use pololu3pi2040_rs::trajectory_control_odometry::diffdrive_outer_loop_with_keyboard_control;
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};
use pololu3pi2040_rs::{
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    robot_parameters_default::robot_constants::WHEEL_RADIUS,
    trajectory_control_odometry::testing_odometry,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    Timer::after_millis(3000).await;
    defmt::info!("Starting odometry-based trajectory following test with keyboard control");

    // =========================== Start uart task for receiving commands from ROS ===========================
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 10 },
        ))
        .unwrap();
    // ================================================================================================

    // ========================================== Start encoder tasks =================================================
    // =========================== (will be accumulated to ENC_LEFT/RIGHT_DELTA) ======================================
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
    // ================================================================================================================

    // ============================================ Start odometry task ===============================================
    // ======================= (20ms, calculate pose from wheel encoders) ==========================================
    spawner
        .spawn(odometry_task(encoder_count_left, encoder_count_right))
        .unwrap();
    // ================================================================================================================

    // ============================================ Start inner loop ==================================================
    // ============================== (20ms, velocity control, track target velocity) =================================
    spawner
        .spawn(wheel_speed_inner_loop(
            devices.motor,
            encoder_count_left,
            encoder_count_right,
        ))
        .unwrap();
    // ================================================================================================================

    // ============================================= Start outer loop =================================================
    // =============== (trajectory control using odometry, triggered by keyboard) ======

    //test the keyboard functionality on the ros node.
    spawner
        .spawn(diffdrive_outer_loop_with_keyboard_control(
            devices.sdlogger,
            devices.led,
            devices.config,
        ))
        .unwrap();

    // Alternative: Uncomment this and comment the above to test odometry output only
    // spawner
    //     .spawn(test_odometry_only())
    //     .unwrap();
    // ================================================================================================================

    defmt::info!("All tasks spawned successfully. Robot ready for keyboard commands via ROS node!");
    defmt::info!("Press 't' in ROS terminal to start trajectory, 's' to stop.");
}
