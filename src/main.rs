#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use embassy_executor::task;
use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    imu::read_imu_task,
    init::init_all,
    joystick_control::{CONTROL_CMD_UNICYCLE, ControlCommandUnicycle, teleop_motor_control_task},
    packet::StateLoopBackPacketF32,
    sdlog::*,
    uart::{uart_hw_task, uart_param_sync_task, uart_receive_task, uart_send_state_loopback},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    let mut devices = init_all(p);

    // === LED Initialization ===
    let mut led = devices.led.unwrap();

    // === Buzzer Initialization ===
    let buzzer = devices.buzzer;
    // play_twinkle(&buzzer).await;
    // play_jingle_bells(&buzzer).await;
    //play_startup_sound(&buzzer).await;

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
    if let Some(config) = devices.config.clone() {
        spawner.spawn(uart_param_sync_task(config)).unwrap();
    } else {
        defmt::warn!("No robot config available - skipping parameter sync");
    }

    // === SD Logger Task ===
    // Spawn task to handle all SD card operations (logging, config save, etc.)
    spawner
        .spawn(sd_logger_task(devices.sdlogger.take()))
        .unwrap();

    // === Control Logic - Step Function Tests ===
    defmt::info!("Starting step function tests for wheel speed control");

    // Helper function to set wheel speed commands
    let set_wheel_speed = |v: f32, omega: f32| async move {
        let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
        *lock = ControlCommandUnicycle { v, omega };
        defmt::info!("Set command: v={} m/s, omega={} rad/s", v, omega);
    };

    let sendback = StateLoopBackPacketF32 {
        header: 0xA1,
        robot_id: 1,
        pos_x: 1.2,
        pos_y: 2.3,
        pos_z: 3.4,
        vel_x: 0.1,
        vel_y: 0.2,
        vel_z: 0.3,
        qw: 1.0,
        qx: 0.0,
        qy: 0.0,
        qz: 0.0,
    };

    loop {
        led.blink(100, 1).await;

        // // Step 1: Keep speed at zero for 2 seconds
        // defmt::info!("=== Step 1: Zero speed for 2 seconds ===");
        // set_wheel_speed(0.0, 0.0).await;
        // Timer::after_millis(1000).await;

        // // Step 2: Step up to 0.4 m/s for 4 seconds
        // defmt::info!("=== Step 2: Step up to 0.4 m/s for 4 seconds ===");
        // set_wheel_speed(0.0, 0.0).await;
        // Timer::after_millis(1000).await;

        // set_wheel_speed(-0.0, 0.0).await;
        // Timer::after_millis(1000).await;

        // uart_send_state_loopback(&sendback);

        // // Step 3: Set back to zero for 4 seconds
        // defmt::info!("=== Step 3: Back to zero for 4 seconds ===");
        // set_wheel_speed(0.0, 0.0).await;
        // Timer::after_millis(1000).await;

        // defmt::info!("=== Test cycle complete, restarting... ===");
    }
}
