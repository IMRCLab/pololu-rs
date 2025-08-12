#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;
use embassy_time::{Duration, Timer};

use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_control::{CtrlCfg, Gains, control_task};
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    let mut led = devices.led;

    // ===== start receiving position message from motion cap system =====
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 1 },
        ))
        .unwrap();

    // ==================== start control task ===========================
    spawner
        .spawn(control_task(
            devices.motor,
            CtrlCfg {
                dt_s: 0.1,
                wheel_base_m: 0.05,
                gains: Gains {
                    kx: 1.0,
                    ky: 2.0,
                    ktheta: 1.5,
                },
            },
        ))
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
