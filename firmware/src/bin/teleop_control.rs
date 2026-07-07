#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use pololu3pi2040_rs::{
    encoder::start_encoder_irq,
    init::init_all,
    joystick_control::{get_gear_ratio, teleop_motor_control_task, teleop_uart_task},
    trajectory_uart::UartCfg,
    uart::uart_hw_task,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    // === FEATURE DEBUG PRINTS ===
    #[cfg(feature = "zumo")]
    defmt::info!("RUNTIME: ZUMO feature detected!");

    #[cfg(feature = "three-pi")]
    defmt::info!("RUNTIME: THREE-PI feature detected!");

    #[cfg(not(any(feature = "zumo", feature = "three-pi")))]
    defmt::info!("RUNTIME: DEFAULT (no features) detected!");

    // === Robot Configuration Test ===
    // This will show the actual GEAR_RATIO being used at runtime
    let mut led = devices.led.unwrap();

    // Get the actual gear ratio being used
    let gear_ratio = get_gear_ratio();
    defmt::info!("RUNTIME: Actual gear ratio = {}", gear_ratio);

    // Flash LED pattern based on ACTUAL gear ratio value
    if gear_ratio > 70.0 {
        // GEAR_RATIO = 75.81 = Zumo (or default with Zumo constants)
        // 2 short flashes = Zumo/Default
        led.blink(100, 2).await;
        embassy_time::Timer::after_millis(500).await;
        led.blink(100, 2).await;
    } else if gear_ratio < 35.0 {
        // GEAR_RATIO = 29.86 = 3Pi
        // 3 short flashes = 3Pi
        led.blink(100, 3).await;
        embassy_time::Timer::after_millis(500).await;
        led.blink(100, 3).await;
    } else {
        // Unknown gear ratio
        // 5 fast flashes = Unknown
        for _ in 0..5 {
            led.blink(50, 1).await;
            embassy_time::Timer::after_millis(50).await;
        }
    }

    // === Start Receiving Command ===
    let robot_id = devices.config.map(|cfg| cfg.robot_id).unwrap_or(7);
    spawner.spawn(uart_hw_task(devices.uart)).unwrap();
    spawner
        .spawn(teleop_uart_task(UartCfg { robot_id }))
        .unwrap();

    // === Start Encoder Task ===
    let encoder_count_left = devices.encoder_counts.left;
    let encoder_count_right = devices.encoder_counts.right;
    start_encoder_irq(devices.encoders);

    // === Start Control Task ===
    spawner
        .spawn(teleop_motor_control_task(
            devices.motor,
            encoder_count_left,
            encoder_count_right,
            devices.config,
        ))
        .unwrap();
}
