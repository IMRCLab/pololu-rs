use defmt::info;
use embassy_rp::gpio::Input;
use embassy_time::{Duration, Timer};

pub struct Buttons {
    // pub btn_a: Input<'static>,
    pub btn_b: Input<'static>,
    pub btn_c: Input<'static>,
}

/// If Button A is enabled, LED should be disabled.
#[embassy_executor::task]
pub async fn button_task_a(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        if button.is_low() {
            info!("Button A (PIN_25) pressed!");
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
pub async fn button_task_b(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        if button.is_low() {
            info!("Button B (PIN_1) pressed!");
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
pub async fn button_task_c(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        if button.is_low() {
            info!("Button C (PIN_0) pressed!");
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
