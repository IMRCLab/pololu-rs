#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;
use embassy_time::{Duration, Timer};

use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_control::control_task;
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    let mut led = devices.led;

    Timer::after(Duration::from_secs(5)).await;

    // ===== start receiving position message from motion cap system =====
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 8 },
        ))
        .unwrap();

    // ==================== start control task ===========================
    spawner
        .spawn(control_task(devices.motor, devices.config.unwrap()))
        .unwrap();

    // ========================= blink LED ===============================
    let mut on = false;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        if on {
            led.off();
        } else {
            led.on();
        }
        on = !on;
    }
}
