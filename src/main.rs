#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    // encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    imu::read_imu_task,
    init::init_all,
    joystick_control::{CONTROL_CMD_UNICYCLE, ControlCommandUnicycle, motor_control_task},
    sdlog::*,
    uart::uart_receive_task,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    let mut devices = init_all(p);

    // === LED Initialization ===
    let mut led = devices.led;

    // === Buzzer Initialization ===
    let mut buzzer = devices.buzzer;
    buzzer.buzzer_warn(1000, 2).await;

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
        .spawn(motor_control_task(
            motors,
            encoder_count_left,
            encoder_count_right,
        ))
        .unwrap();

    // === IMU Task ===
    let imu = devices.imu;
    spawner.spawn(read_imu_task(imu)).unwrap();

    // === UART Task ===
    let uart_rec = devices.uart;
    spawner.spawn(uart_receive_task(uart_rec)).unwrap();

    // === SdLogger ===
    let motion = MotionLog {
        timestamp_ms: 12345,
        target_vx: 0.2,
        target_vy: 0.3,
        target_vz: 0.4,
        target_qw: 0.5,
        target_qx: 0.2,
        target_qy: 0.6,
        target_qz: 0.3,
        actual_vx: 0.18,
        actual_vy: 0.28,
        actual_vz: 0.38,
        actual_qw: 0.18,
        actual_qx: 0.28,
        actual_qy: 0.38,
        actual_qz: 0.48,
        roll: 1.5,
        pitch: 0.2,
        yaw: 90.0,
        motor_left: 1200,
        motor_right: 1300,
    };

    if let Some(sd) = devices.sdlogger.as_mut() {
        sd.write_csv_header();
        defmt::info!("Start Sd card writing test!");
        sd.log_motion_as_bin(&motion);
        sd.flush(); // This is super important!!!!!!
        defmt::info!("Finish Sd card writing test!");
    } else {
        defmt::warn!("No SD card / SdLogger disabled, skip logging");
    }

    // === Control Logic - Step Function Tests ===
    defmt::info!("Starting step function tests for wheel speed control");

    // Helper function to set wheel speed commands
    let set_wheel_speed = |v: f32, omega: f32| async move {
        let mut lock = CONTROL_CMD_UNICYCLE.lock().await;
        *lock = ControlCommandUnicycle { v, omega };
        defmt::info!("Set command: v={} m/s, omega={} rad/s", v, omega);
    };

    loop {
        led.blink(100, 1).await;

        // Step 1: Keep speed at zero for 2 seconds
        defmt::info!("=== Step 1: Zero speed for 2 seconds ===");
        set_wheel_speed(0.0, 0.0).await;
        Timer::after_millis(1000).await;

        // Step 2: Step up to 0.4 m/s for 4 seconds
        defmt::info!("=== Step 2: Step up to 0.4 m/s for 4 seconds ===");
        set_wheel_speed(0.5, 0.0).await;
        Timer::after_millis(1000).await;

        set_wheel_speed(-0.4, 0.0).await;
        Timer::after_millis(1000).await;

        // Step 3: Set back to zero for 4 seconds
        defmt::info!("=== Step 3: Back to zero for 4 seconds ===");
        set_wheel_speed(0.0, 0.0).await;
        Timer::after_millis(1000).await;

        defmt::info!("=== Test cycle complete, restarting... ===");
    }
}
