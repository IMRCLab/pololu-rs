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
    let mut devices = init_all(p);

    let mut led = devices.led;
    let mut buzzer = devices.buzzer;
    // let mut buzzer = devices.buzzer;

    if let Some(_sd) = devices.sdlogger.as_mut() {
        //sd.write_csv_header();
        //defmt::info!("Start Sd card writing test!");
        //sd.log_motion_as_bin(&motion);
        //sd.flush(); // This is super important!!!!!!
        defmt::info!("SD card is here!");
    } else {
        defmt::warn!("No SD card / SdLogger disabled, skip logging");
    }

    #[cfg(feature = "zumo")]
    for _ in 0..2 {
        //led.on();
        buzzer.buzzer_warn(800, 1).await;
        Timer::after(Duration::from_millis(150)).await;
        //led.off();
        Timer::after(Duration::from_millis(100)).await;
    }
    #[cfg(feature = "three-pi")]
    for _ in 0..3 {
        //led.on();
        buzzer.buzzer_warn(800, 1).await;
        Timer::after(Duration::from_millis(150)).await;
        //led.off();
        Timer::after(Duration::from_millis(100)).await;
    }

    // === Startup sound with LED blinking ===
    //blink 2 times for zumo
    Timer::after(Duration::from_secs(2)).await;

    // Signal that diffdrive trajectory is ready
    //DIFFDRIVE_TRAJECTORY_READY.signal(());

    // ===== start receiving position message from motion cap system =====
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 7 },
            //pololu 3pi is 8, zumo is 7
        ))
        .unwrap();

    // ==================== start diffdrive control task ===========================
    defmt::info!("starting diffdrive control task...");
    spawner
        .spawn(diffdrive_control_task(devices.motor, devices.sdlogger, led))
        .unwrap();
    defmt::info!("diffdrive control task started");
    // ========================= blink LED ===============================
    for _ in 0..3 {
        //led.on();
        buzzer.buzzer_warn(1000, 1).await;
        Timer::after(Duration::from_millis(150)).await;
        //led.off();
        Timer::after(Duration::from_millis(100)).await;
    }
}
