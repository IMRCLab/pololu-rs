#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use embassy_time::Timer;

use pololu3pi2040_rs::{
    button::{button_task_b, button_task_c},
    encoder::{EncoderPair, encoder_left_task, encoder_right_task},
    init::init_all,
    motor::set_speed,
    uart::uart_receive_task,
};

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

    // === Encoder Task ===
    let EncoderPair {
        encoder_left,
        encoder_right,
    } = devices.encoders;
    spawner.spawn(encoder_left_task(encoder_left)).unwrap();
    spawner.spawn(encoder_right_task(encoder_right)).unwrap();

    // === UART Task ===
    let uart_rec = devices.uart;
    spawner.spawn(uart_receive_task(uart_rec)).unwrap();

    // === Control Logic ===
    loop {
        led.blink(100, 3).await;

        set_speed(0.5, 0.5); // forward
        Timer::after_millis(1000).await;

        set_speed(-0.5, -0.5); // backward
        Timer::after_millis(1000).await;

        set_speed(0.0, 0.0); // stop
        Timer::after_millis(1000).await;
    }
}
