#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;
use embassy_time::{Duration, Timer};

use pololu3pi2040_rs::diffdrive::diffdrive_control_task;
use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    let mut led = devices.led;

    // === Startup sound with LED blinking ===
    let mut buzzer = devices.buzzer;
    for _ in 0..3 {
        led.on();
        buzzer.buzzer_warn(800, 1).await;
        Timer::after(Duration::from_millis(150)).await;
        led.off();
        Timer::after(Duration::from_millis(100)).await;
    }

    Timer::after(Duration::from_secs(2)).await;

    // Signal that diffdrive trajectory is ready
    //DIFFDRIVE_TRAJECTORY_READY.signal(());

    // ===== start receiving position message from motion cap system =====
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 8 },
        ))
        .unwrap();

    // ==================== start diffdrive control task ===========================
    spawner
        .spawn(diffdrive_control_task(devices.motor))
        .unwrap();

    // // ========================= blink LED ===============================
    // let mut on = false;
    // loop {
    //     Timer::after(Duration::from_secs(1)).await;
    //     if on {
    //         led.off();
    //     } else {
    //         led.on();
    //     }
    //     on = !on;
    // }
}
