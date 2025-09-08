#![no_std]
#![no_main]

use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use pololu3pi2040_rs::diffdrive_cascade::{
    ControlMode, diffdrive_outer_loop, mocap_update_task, wheel_speed_inner_loop,
};
use pololu3pi2040_rs::encoder::{EncoderPair, encoder_left_task, encoder_right_task};
use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};

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

    // ============================================= Start outer loop =================================================
    // =============== (100ms, trajectory control, give out wl/wr, send to inner loop via WHEEL_CMD_CH) ===============
    // spawner
    //     .spawn(diffdrive_outer_loop(
    //         ControlMode::WithMocapController,
    //         devices.sdlogger,
    //         devices.led,
    //     ))
    //     .unwrap();
    spawner
        .spawn(diffdrive_outer_loop(
            ControlMode::DirectDuty,
            devices.sdlogger,
            devices.led,
        ))
        .unwrap();
    // ================================================================================================================
}
