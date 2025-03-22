//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::gpio;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::Timer;
use gpio::{Level, Output};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut c = Config::default();
    c.top = 6000 - 1;
    c.phase_correct = true;
    c.divider = 1.into();

    let mut direction_right = Output::new(p.PIN_10, Level::Low);
    let mut direction_left = Output::new(p.PIN_11, Level::Low);

    // let mut channel_right = Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, c.clone());
    // let mut channel_left = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c.clone());


    // channel_right.set_duty_cycle(3000).unwrap();
    // direction_right.set_high();

    // let mut channel_left = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c.clone());

    let mut channel = Pwm::new_output_ab(p.PWM_SLICE7, p.PIN_14, p.PIN_15, c.clone());

    // c.compare_a = 500; //right
    // c.compare_b = 250; // left
    // channel.set_config(&c);
    direction_right.set_high();
    direction_left.set_low();

    // let (channel_left, channel_right) = channel.split();

    // channel_right.unwrap().set_duty_cycle(3000).unwrap();
    // direction_right.set_high();

    // channel_left.unwrap().set_duty_cycle(1000).unwrap();
    // direction_left.set_low();

    loop {
        log::info!("High");

        led.set_high();

        c.compare_a = 2000; //right
        c.compare_b = 2000; // left
        channel.set_config(&c);

        Timer::after_secs(1).await;

        log::info!("Low");

        led.set_low();

        c.compare_a = 0; //right
        c.compare_b = 0; // left
        channel.set_config(&c);

        Timer::after_secs(1).await;
    }
}
