#![no_std]
#![no_main]

use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_control::{
    ControlMode, diffdrive_outer_loop_command_controlled_traj_following_from_sdcard,
    mocap_update_task, wheel_speed_inner_loop,
};
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};
use pololu3pi2040_rs::{
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    trajectory_control::diffdrive_outer_loop_command_controlled_tuning,
};
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    Timer::after_millis(3000).await;

    // =========================== Start uart task for receiving pose information from UART ===========================
    // ===================================== (will update STATE_SIG and LAST_STATE) ===================================
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 10 },
        ))
        .unwrap();
    // ================================================================================================================

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

    // ======================================== Start Mocap update task ===============================================
    // ==================== (transfer the data in STATE_SIG to LAST_STATE, for outer loop to read) ====================
    spawner.spawn(mocap_update_task()).unwrap();
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

    // ============================================= Start gain tuning outer loop ====================================
    // =============== Semi - Automatically tests different gains with straight line trajectory ============================
    // spawner
    //     .spawn(diffdrive_outer_loop_command_controlled_tuning(
    //         //ControlMode::DirectDuty,
    //         ControlMode::WithMocapController,
    //         devices.sdlogger,
    //         devices.led,
    //         devices.config,
    //     ))
    //     .unwrap();

    spawner
        .spawn(
            diffdrive_outer_loop_command_controlled_traj_following_from_sdcard(
                //ControlMode::DirectDuty,
                ControlMode::WithMocapController,
                devices.sdlogger,
                devices.led,
                devices.config,
            ),
        )
        .unwrap();

    // ==================================== End of gain tuning outer loop ============================================
}
