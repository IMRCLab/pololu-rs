#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Executor;
use embassy_rp::{
    multicore::{spawn_core1, Stack},
    i2c::{self, Async, I2c},
    peripherals::I2C0,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker};
use portable_atomic::{AtomicU32, Ordering};
use static_cell::StaticCell;

use pololu3pi2040_rs::{
    imu::ImuPack,
    init::Irqs,
    sdlog::{init_sd_logger, SdLogger},
};


// Core 1 必须有自己的栈和executor
static CORE1_STACK: StaticCell<Stack<32768>> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();


static CORE0_COUNTER: AtomicU32 = AtomicU32::new(0);


// I2C driver 只在 Core 1 上初始化和使用
static I2C_CELL:StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, I2C0, Async>>> = StaticCell::new();
static IMU_SAMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);
static IMU_ERROR_COUNTER: AtomicU32 = AtomicU32::new(0);
static GYRO_Z_BITS: AtomicU32 = AtomicU32::new(0);


static SD_READY: AtomicU32 = AtomicU32::new(0);
static SD_RECORD_COUNTER: AtomicU32 = AtomicU32::new(0);
static IMU_MAX_GAP_US: AtomicU32 = AtomicU32::new(0);
static IMU_DEADLINE_MISSES: AtomicU32 = AtomicU32::new(0);


fn write_imu_record(
    logger: &mut SdLogger,
    timestamp_ms: u32,
    accel: [f32; 3],
    gyro: [f32; 3],
) {
    logger.write_buffered(&timestamp_ms.to_le_bytes());

    for value in accel {
        logger.write_buffered(&value.to_le_bytes());
    }

    for value in gyro {
        logger.write_buffered(&value.to_le_bytes());
    }
}


#[embassy_executor::task]
async fn core0_control_task() {
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        ticker.next().await;

        CORE0_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
}


/// Core 1: 真实读取IMU
#[embassy_executor::task]
async fn core1_imu_sd_task(
    i2c: &'static Mutex<ThreadModeRawMutex, I2c<'static, I2C0, Async>,>,
    mut sdlogger: Option<SdLogger>,
) {
    let mut imu = ImuPack::new(i2c);

    if imu.init().await.is_err() {
        IMU_ERROR_COUNTER.fetch_add(1, Ordering::Relaxed);
        defmt::error!("Core 1: IMU initialization failed");
        return;
    }

    defmt::info!("Core 1: IMU initialized");

    if let Some(logger) = sdlogger.as_mut() {
        logger.open_new_file();
        logger.flush();

        SD_READY.store(1, Ordering::Release);
        defmt::info!("Core 1: SD log file opened");
    } else {
        defmt::warn!("Core 1: SD unavailable, continue without logging");
    }

    let mut ticker = Ticker::every(Duration::from_millis(5));
    let mut last_sample = embassy_time::Instant::now();
    let mut records_since_flush = 0u32;

    loop {
        ticker.next().await;

        match imu.read_all().await {
            Ok((accel, gyro, _mag)) => {
                let now = embassy_time::Instant::now();
                let gap_us = now.duration_since(last_sample).as_micros() as u32;
                last_sample = now;

                IMU_MAX_GAP_US.fetch_max(gap_us, Ordering::Relaxed);

                // 10 ms 是正常周期。这里用 15 ms 作为 deadline miss。
                if gap_us > 15_000 {
                    IMU_DEADLINE_MISSES.fetch_add(1, Ordering::Relaxed);
                }


                GYRO_Z_BITS.store(gyro[2].to_bits(), Ordering::Relaxed);
                IMU_SAMPLE_COUNTER.fetch_add(1, Ordering::Relaxed);


                if let Some(logger) = sdlogger.as_mut() {
                    write_imu_record(
                        logger,
                        now.as_millis() as u32,
                        accel,
                        gyro,
                    );

                    SD_RECORD_COUNTER.fetch_add(1, Ordering::Relaxed);
                    records_since_flush += 1;

                    // 每 100 条，也就是大约每秒同步一次文件。
                    //
                    // flush 可能耗时数毫秒甚至更久，
                    // 但它只会阻塞 Core 1。
                    if records_since_flush >= 1000 {
                        logger.flush();
                        records_since_flush = 0;
                    }
                }
            }
            Err(_) => {
                IMU_ERROR_COUNTER.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}


/// Core 0：每秒读取并显示两个核心的状态。
#[embassy_executor::task]
async fn monitor_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    let mut previous_core0 = 0u32;
    let mut previous_imu_samples = 0u32;


    loop {
        ticker.next().await;

        let current_core0 = CORE0_COUNTER.load(Ordering::Relaxed);

        // Acquire 与 Core 1 的 Release 配对。
        let current_imu_samples =
            IMU_SAMPLE_COUNTER.load(Ordering::Acquire);

        let imu_errors =
            IMU_ERROR_COUNTER.load(Ordering::Relaxed);

        let gyro_z = f32::from_bits(
            GYRO_Z_BITS.load(Ordering::Relaxed)
        );

        let core0_updates =
            current_core0.wrapping_sub(previous_core0);

        let imu_updates =
            current_imu_samples.wrapping_sub(previous_imu_samples);

        let sd_ready = SD_READY.load(Ordering::Acquire);

        let sd_records =
            SD_RECORD_COUNTER.load(Ordering::Relaxed);

        let max_imu_gap_us =
            IMU_MAX_GAP_US.swap(0, Ordering::Relaxed);

        let deadline_misses =
            IMU_DEADLINE_MISSES.swap(0, Ordering::Relaxed);

        previous_core0 = current_core0;
        previous_imu_samples = current_imu_samples;

        defmt::info!(
            "Core0={}Hz IMU={}Hz gyro_z={} SD={} records={} max_gap={}us misses={} errors={}",
            core0_updates,
            imu_updates,
            gyro_z,
            sd_ready,
            sd_records,
            max_imu_gap_us,
            deadline_misses,
            imu_errors
        );
    }
}

#[embassy_executor::main]
async fn main(core0_spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    let core1 = p.CORE1;

    let i2c0 = p.I2C0;
    let scl = p.PIN_5;
    let sda = p.PIN_4;

    let spi0 = p.SPI0;
    let sd_sck = p.PIN_18;
    let sd_mosi = p.PIN_19;
    let sd_miso = p.PIN_20;
    let sd_cs = p.PIN_21;


    let core1_stack = CORE1_STACK.init(Stack::new());

    spawn_core1(core1, core1_stack, move || {
        let i2c = I2c::new_async(
            i2c0,
            scl,
            sda,
            Irqs,
            i2c::Config::default(),
        );

        let shared_i2c = I2C_CELL.init(Mutex::new(i2c));

        // SPI/SD 也完全由 Core 1 初始化和持有。
        let sdlogger = match init_sd_logger(
            spi0,
            sd_sck,
            sd_mosi,
            sd_miso,
            sd_cs,
        ) {
            Ok((logger, _config)) => Some(logger),

            Err(_) => {
                defmt::warn!("Core 1: SD initialization failed");
                None
            }
        };

        let executor = CORE1_EXECUTOR.init(Executor::new());

        executor.run(|spawner| {
            spawner.spawn(core1_imu_sd_task(shared_i2c, sdlogger)).unwrap();
        })
    });


    core0_spawner.spawn(core0_control_task()).unwrap();
    core0_spawner.spawn(monitor_task()).unwrap();

    core::future::pending::<()>().await;
}
