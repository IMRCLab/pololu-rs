#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::init;

use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    init::init_all,
    sdlog::*,
    uart::uart_receive_task,
    joystick_control::{motor_control_task, robot_command_control_task},
};

/*
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    let devices = init_all(p);

    // === LED Initialization ===
    let mut led = devices.led;

    // === Buzzer Initialization ===
    let mut buzzer = devices.buzzer;
    buzzer.buzzer_warn(1000, 2).await;

    // === Buttons Task ===
    let buttons = devices.buttons;
    spawner.spawn(button_task_b(buttons.btn_b)).unwrap();
    spawner.spawn(button_task_c(buttons.btn_c)).unwrap();

    // === Motor Task ===
    let motors = devices.motor;

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

    // === IMU Task ===
    // let imu = devices.imu;
    // spawner.spawn(read_imu_task(imu)).unwrap();

    // === UART Task ===
    let uart_rec = devices.uart;
    spawner.spawn(uart_receive_task(uart_rec)).unwrap();

    // === SdLogger ===
    // let mut sdlogger = devices.sdlogger;
    // let motion = MotionLog {
    //     timestamp_ms: 12345,
    //     target_vx: 0.2,
    //     target_vy: 0.3,
    //     target_vz: 0.4,
    //     target_qw: 0.5,
    //     target_qx: 0.2,
    //     target_qy: 0.6,
    //     target_qz: 0.3,
    //     actual_vx: 0.18,
    //     actual_vy: 0.28,
    //     actual_vz: 0.38,
    //     actual_qw: 0.18,
    //     actual_qx: 0.28,
    //     actual_qy: 0.38,
    //     actual_qz: 0.48,
    //     roll: 1.5,
    //     pitch: 0.2,
    //     yaw: 90.0,
    //     motor_left: 1200,
    //     motor_right: 1300,
    // };

    // // sdlogger.write_csv_header();
    // sdlogger.log_motion_as_bin(&motion);
    // sdlogger.flush(); // This is super important!!!!!!
    // info!("Finish Sd card writing test!");

    // === Control Logic ===
    loop {
        led.blink(100, 3).await;

        motors.set_speed(0.5, 0.5).await; // forward
        Timer::after_millis(1000).await;

        motors.set_speed(-0.5, -0.5).await; // backward
        Timer::after_millis(1000).await;

        motors.set_speed(0.0, 0.0).await; // stop
        Timer::after_millis(1000).await;
    }
}

*/

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let devices = init_all(p);

    // === Start Receiving Command ===
    spawner
        .spawn(robot_command_control_task(devices.uart))
        .unwrap();

    // === Start Encoder Task ===
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

    // === Start Control Task ===
    spawner
        .spawn(motor_control_task(
            devices.motor,
            encoder_count_left,
            encoder_count_right,
        ))
        .unwrap();
}


