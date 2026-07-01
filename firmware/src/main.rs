#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    // buzzer::play_startup_sound,
    encoder::start_encoder_irq,
    imu::read_imu_task,
    init::init_all,
    joystick_control::teleop_motor_control_task,

    robotstate,

    uart::{uart_hw_task, uart_receive_task},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());

    let mut devices = init_all(p);

    // === LED Initialization ===
    let mut led = devices.led.unwrap();

    // === Buzzer Initialization ===
    let _buzzer = devices.buzzer;
    // play_twinkle(&buzzer).await;
    // play_jingle_bells(&buzzer).await;
    // play_startup_sound(&buzzer).await;

    // === Buttons Task ===
    let buttons = devices.buttons;
    spawner.spawn(button_task_b(buttons.btn_b)).unwrap();
    spawner.spawn(button_task_c(buttons.btn_c)).unwrap();

    // === Encoder Task ===
    let encoder_count_left = devices.encoder_counts.left;
    let encoder_count_right = devices.encoder_counts.right;
    start_encoder_irq(devices.encoders);

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

    // === SdLogger ===
    if let Some(sd) = devices.sdlogger.as_mut() {
        sd.open_new_file();
        defmt::info!("Start Sd card logging initialization");
        sd.flush();
        defmt::info!("Finish Sd card logging initialization");
    } else {
        defmt::warn!("No SD card / SdLogger disabled, skip logging");
    }

    // === Control Logic - Step Function Tests ===
    defmt::info!("Starting step function tests for wheel speed control");

    // Helper function to set wheel speed commands
    let set_wheel_speed = |v: f32, omega: f32| async move {
        robotstate::write_unicycle_cmd(robotstate::UnicycleCmd {
            v,
            omega,
            stamp: embassy_time::Instant::now(),
        }).await;
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
        set_wheel_speed(0.0, 0.0).await;
        Timer::after_millis(1000).await;

        set_wheel_speed(-0.0, 0.0).await;
        Timer::after_millis(1000).await;

        // uart_send_state_loopback removed

        // Step 3: Set back to zero for 4 seconds
        defmt::info!("=== Step 3: Back to zero for 4 seconds ===");
        set_wheel_speed(0.0, 0.0).await;
        Timer::after_millis(1000).await;

        defmt::info!("=== Test cycle complete, restarting... ===");
    }
}
