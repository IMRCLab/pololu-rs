#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;
use embassy_time::Timer;

use pololu3pi2040_rs::{
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    init::init_all,
    joystick_control::{motor_control_task, robot_command_control_task},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    let mut led = devices.led;

    // === Uart Task ===
    spawner
        .spawn(robot_command_control_task(devices.uart))
        .unwrap();

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

    // === Motor Control ===
    spawner
        .spawn(motor_control_task(
            devices.motor,
            encoder_count_left,
            encoder_count_right,
        ))
        .unwrap();

    // ========================= blink LED ===============================
    let mut flag_led: bool = false;
    loop {
        Timer::after(embassy_time::Duration::from_secs(1)).await;
        if !flag_led {
            led.on();
            flag_led = true;
        } else {
            led.off();
            flag_led = false;
        }
    }
}
