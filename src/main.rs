#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;
use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    imu::read_imu_task,
    init::init_all,
    joystick_control::teleop_motor_control_task,
    packet::StateLoopBackPacketF32,
    parameter_sync::{init_robot_config, send_robot_parameters_to_dongle},
    robotstate::LogSnapshot,
    sdlog::*,
    uart::{UART_TX_CHANNEL, uart_hw_task, uart_receive_task},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    let mut devices = init_all(p);

    // === LED Initialization ===
    let mut led = devices.led.unwrap();

    // === Buttons Task ===
    let buttons = devices.buttons;
    spawner.spawn(button_task_b(buttons.btn_b)).unwrap();
    spawner.spawn(button_task_c(buttons.btn_c)).unwrap();

    // === Encoder Task ===
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

    // === Motor Control Task ===
    // Start the PI controller task instead of direct motor control
    let motors = devices.motor;
    spawner
        .spawn(teleop_motor_control_task(
            motors,
            encoder_count_left,
            encoder_count_right,
            devices.config,
        ))
        .unwrap();

    // === IMU Task ===
    let imu = devices.imu;
    spawner.spawn(read_imu_task(imu)).unwrap();

    // === UART Task ===
    let uart_rec = devices.uart;
    spawner.spawn(uart_hw_task(uart_rec)).unwrap();
    spawner.spawn(uart_receive_task()).unwrap();

    // === UART Parameter Sync Task ===
    // Send robot parameters via UART upon startup
    // In main.rs - ohne separaten Task
    if let Some(config) = devices.config.clone() {
        init_robot_config(config);
        embassy_time::Timer::after_millis(500).await;
        let _ = send_robot_parameters_to_dongle(&config).await;
    }

    // === SD Logger Task ===
    // Spawn task to handle all SD card operations (logging, config save, etc.)
    spawner
        .spawn(sd_logger_task(devices.sdlogger.take()))
        .unwrap();

    // === Control Logic - Step Function Tests ===
    defmt::info!("Starting step function tests for wheel speed control");

    // Send dummy log snapshot packet for testing
    let log_snapshot = LogSnapshot {
        t_ms: 0,
        x: 1.2,
        y: 2.3,
        yaw: 3.4,
        x_des: 0.1,
        y_des: 0.2,
        yaw_des: 0.3,
        v_ff: 1.0,
        w_ff: 2.0,
        omega_l_cmd: 0.1,
        omega_r_cmd: 0.2,
        omega_l_meas: 3.0,
        omega_r_meas: 4.0,
        duty_l: 0.0,
        duty_r: 0.0,
        x_err: 1.0,
        y_err: 2.0,
        yaw_err: 3.0,
    };

    let sendback = StateLoopBackPacketF32::new(8, log_snapshot);

    loop {
        led.blink(100, 1).await;

        // Send test packet via UART - now directly compatible with channel
        let data = sendback.to_bytes_compact();
        let len = data.len();
        let _ = UART_TX_CHANNEL.try_send(data);

        defmt::info!("Sent test packet, {} bytes", len);
        //print the log snapshot here for verification
        let data = sendback.log_snapshot.to_bytes_compact();
        defmt::info!(
            "LogSnapshot: {:?}",
            defmt::Debug2Format(&sendback.log_snapshot)
        );
        defmt::info!("LogSnapshot (compact): {:?}", defmt::Debug2Format(&data));
        Timer::after_millis(4000).await;
    }
}
