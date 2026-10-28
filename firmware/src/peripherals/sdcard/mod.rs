//! Core 1 SD card and filesystem driver.
//!
//! This module contains no Embassy logging task and no mode logic. The
//! `SdLogger` object is moved into and exclusively owned by the permanent
//! Core 1 logging task.

mod config;

use embassy_rp::{
    Peri,
    gpio::{
        Level,
        Output,
    },
    peripherals::{
        PIN_18,
        PIN_19,
        PIN_20,
        PIN_21,
        SPI0,
    },
    spi::{
        self,
        Spi,
    },
};
use embassy_time::Delay;
use embedded_hal_bus::spi::{
    ExclusiveDevice,
    NoDelay,
};
use embedded_sdmmc::{
    Directory,
    File,
    Mode,
    SdCard,
    ShortFileName,
    TimeSource,
    Timestamp,
    Volume,
    VolumeIdx,
    VolumeManager,
};
use heapless::String;
use static_cell::StaticCell;

pub use config::RobotConfig;

use config::load_robot_config_with_dir;
use crate::trajectory_reading::Trajectory;

/* ================= Filesystem types ================= */

pub type SpiDevice<'a> = ExclusiveDevice<
    Spi<'a, SPI0, spi::Blocking>,
    Output<'a>,
    NoDelay,
>;

pub type SdCardDevice<'a> =
    SdCard<SpiDevice<'a>, Delay>;

pub type Clock = DummyClock;

const MAX_DIRS: usize = 4;
const MAX_FILES: usize = 4;
const MAX_VOLUMES: usize = 1;

/// Robot configuration files are currently ~500 bytes
const CONFIG_SCRATCH_SIZE: usize = 1024;
const TRAJECTORY_SCRATCH_SIZE: usize = 48 * 1024;

pub type SdDir = Directory<
    'static,
    SdCardDevice<'static>,
    Clock,
    MAX_DIRS,
    MAX_FILES,
    MAX_VOLUMES,
>;

pub type SdFile = File<
    'static,
    SdCardDevice<'static>,
    Clock,
    MAX_DIRS,
    MAX_FILES,
    MAX_VOLUMES,
>;

/* ================= Clock ================= */

pub struct DummyClock;

impl TimeSource for DummyClock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/* ================= Static filesystem storage ================= */

static VOLUME_MANAGER: StaticCell<
    VolumeManager<
        SdCardDevice<'static>,
        Clock,
        MAX_DIRS,
        MAX_FILES,
        MAX_VOLUMES,
    >,
> = StaticCell::new();

static VOLUME: StaticCell<
    Volume<
        'static,
        SdCardDevice<'static>,
        Clock,
        MAX_DIRS,
        MAX_FILES,
        MAX_VOLUMES,
    >,
> = StaticCell::new();

static ROOT_DIRECTORY: StaticCell<SdDir> =
    StaticCell::new();

static CONFIG_SCRATCH: StaticCell<
    [u8; CONFIG_SCRATCH_SIZE],
> = StaticCell::new();

static TRAJECTORY_SCRATCH: StaticCell<
    [u8; TRAJECTORY_SCRATCH_SIZE],
> = StaticCell::new();

/* ================= Error type ================= */

#[derive(Debug, defmt::Format)]
pub enum SdError {
    NoCardOrInitFail,
    OpenRoot,
}

/// Read and parse the onboard trajectory while the caller owns the SD directory.
pub fn load_trajectory_with_dir(
    directory: &mut SdDir,
    scratch: &mut [u8],
) -> Result<Trajectory, &'static str> {
    defmt::info!("Entered load_trajectory_with_dir");
    
    let short_name = ShortFileName::create_from_str("TRJ0001.JSN")
        .map_err(|_| "trajectory filename invalid")?;

    let total = {
        let mut file = directory
            .open_file_in_dir(&short_name, Mode::ReadOnly)
            .map_err(|_| "trajectory file open failed")?;

        let file_length = file.length() as usize;
        if file_length > scratch.len() {
            return Err("trajectory exceeds scratch buffer");
        }

        let mut total = 0;
        let mut chunk = [0; 512];

        loop {
            let count = file
                .read(&mut chunk)
                .map_err(|_| "trajectory file read failed")?;
            if count == 0 {
                break;
            }
            
            if total + count > scratch.len() {
                return Err("trajectory exceeds scratch buffer");
            }
            defmt::info!("Read {} bytes from trajectory file", total);

            scratch[total..total + count]
                .copy_from_slice(&chunk[..count]);
            total += count;
        }

        total
    };
    crate::dual_core::trajectory_reading::from_json(&scratch[..total])
}

/* ================= Buffered logger ================= */

pub struct SdLogger {
    file: Option<SdFile>,
    directory: &'static mut SdDir,

    write_buffer: [u8; 512],
    write_length: usize,
}

impl SdLogger {
    pub fn open_new_file(&mut self) {
        // Safely finish a previous session first.
        self.close_file();

        // embedded-sdmmc ties the opened file lifetime to this directory
        // borrow. The directory itself is stored for the complete firmware
        // lifetime.
        let directory: &'static mut SdDir = unsafe {
            core::mem::transmute(
                &mut *self.directory
            )
        };

        let mut new_file = None;

        for index in 0..100 {
            let tens =
                b'0' + (index / 10) as u8;

            let ones =
                b'0' + (index % 10) as u8;

            let mut name = String::<8>::new();

            let _ = name.push('T');
            let _ = name.push('R');
            let _ = name.push(char::from(tens));
            let _ = name.push(char::from(ones));

            if let Ok(short_name) =
                ShortFileName::create_from_str(&name)
            {
                let already_exists = directory
                    .open_file_in_dir(
                        &short_name,
                        Mode::ReadOnly,
                    )
                    .is_ok();

                if !already_exists {
                    let file = directory
                        .open_file_in_dir(
                            &short_name,
                            Mode::ReadWriteCreateOrAppend,
                        )
                        .unwrap();

                    new_file = Some(file);
                    break;
                }
            }
        }

        let Some(mut file) = new_file else {
            defmt::error!(
                "SD logger: no free log file slot"
            );
            return;
        };

        // Legacy file magic retained so existing tools can still identify a
        // Pololu binary log. The dual-core logger appends its own schema header.
        let magic = [
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];

        if let Err(error) = file.write(&magic) {
            defmt::error!(
                "Failed to write log magic: {:?}",
                defmt::Debug2Format(&error),
            );

            return;
        }

        let _ = file.flush();

        self.file = Some(file);
        self.write_length = 0;

        defmt::info!(
            "SD logger: opened new binary log file"
        );
    }

    pub fn write_buffered(
        &mut self,
        data: &[u8],
    ) {
        let mut input_offset = 0usize;

        while input_offset < data.len() {
            let available =
                self.write_buffer.len()
                    - self.write_length;

            let amount = core::cmp::min(
                available,
                data.len() - input_offset,
            );

            self.write_buffer[
                self.write_length
                    ..self.write_length + amount
            ]
            .copy_from_slice(
                &data[
                    input_offset
                        ..input_offset + amount
                ],
            );

            self.write_length += amount;
            input_offset += amount;

            if self.write_length
                == self.write_buffer.len()
            {
                self.flush_buffer();
            }
        }
    }

    pub fn flush_buffer(&mut self) {
        if self.write_length == 0 {
            return;
        }

        if let Some(file) = self.file.as_mut() {
            if let Err(error) = file.write(
                &self.write_buffer[
                    ..self.write_length
                ],
            ) {
                defmt::error!(
                    "SD buffered write failed: {:?}",
                    defmt::Debug2Format(&error),
                );
            }
        }

        self.write_length = 0;
    }

    pub fn flush(&mut self) {
        self.flush_buffer();

        if let Some(file) = self.file.as_mut() {
            if let Err(error) = file.flush() {
                defmt::error!(
                    "SD flush failed: {:?}",
                    defmt::Debug2Format(&error),
                );
            }
        }
    }

    pub fn close_file(&mut self) {
        self.flush();

        if let Some(file) = self.file.take() {
            drop(file);

            defmt::info!(
                "SD logger: file closed"
            );
        }

        self.write_length = 0;
    }

    pub fn is_open(&self) -> bool {
        self.file.is_some()
    }
}

/* ================= Initialization ================= */

#[allow(clippy::too_many_arguments)]
pub fn init_sd_card(
    spi: Peri<'static, SPI0>,
    clock_pin: Peri<'static, PIN_18>,
    mosi_pin: Peri<'static, PIN_19>,
    miso_pin: Peri<'static, PIN_20>,
    chip_select_pin: Peri<'static, PIN_21>,
) -> Result<(SdLogger, RobotConfig, Option<Trajectory>), SdError> {
    // SD cards must be initialized at no more than 400 kHz.
    let mut slow_config =
        spi::Config::default();
    defmt::info!("Initializing SD card at 400 kHz");
    slow_config.frequency = 400_000;

    let spi = Spi::new_blocking(
        spi,
        clock_pin,
        mosi_pin,
        miso_pin,
        slow_config,
    );

    let chip_select = Output::new(
        chip_select_pin,
        Level::High,
    );

    let volume_manager =
        VOLUME_MANAGER.init_with(|| {
            let spi_device =
                ExclusiveDevice::new_no_delay(
                    spi,
                    chip_select,
                );

            let sd_card = SdCard::new(
                spi_device,
                Delay,
            );

            let mut fast_config =
                spi::Config::default();

            fast_config.frequency = 16_000_000;

            sd_card.spi(|device| {
                device
                    .bus_mut()
                    .set_config(&fast_config)
            });

            VolumeManager::new(
                sd_card,
                DummyClock,
            )
        });

    let volume = match volume_manager
        .open_volume(VolumeIdx(0))
    {
        Ok(volume) => volume,

        Err(_) => {
            defmt::warn!(
                "SD open_volume failed"
            );

            return Err(
                SdError::NoCardOrInitFail,
            );
        }
    };

    let volume = VOLUME.init(volume);

    let root_directory =
        match volume.open_root_dir() {
            Ok(directory) => directory,

            Err(_) => {
                defmt::warn!(
                    "SD open_root_dir failed"
                );

                return Err(SdError::OpenRoot);
            }
        };

    let root_directory =
        ROOT_DIRECTORY.init(root_directory);
    defmt::info!("Initializing config scratch");
    
    let config_scratch =
        CONFIG_SCRATCH.init_with(|| [0u8; CONFIG_SCRATCH_SIZE]);
    let config = load_robot_config_with_dir(
        root_directory,
        config_scratch,
    );
    // TODO hangs here, investigate stack size issues
    defmt::info!("Entering load_trajectory_with_dir");
    
    let traj_scratch =
    TRAJECTORY_SCRATCH.init_with(|| [0u8; TRAJECTORY_SCRATCH_SIZE]);
    
    // let trajectory = None;
    let trajectory = match load_trajectory_with_dir(root_directory, traj_scratch) {
        Ok(trajectory) => {
            defmt::info!(
                "Core 1 loaded onboard trajectory: states={}, actions={}",
                trajectory.states.len(),
                trajectory.actions.len(),
            );
            Some(trajectory)
        }
        Err(error) => {
            defmt::warn!(
                "Core 1 could not load onboard trajectory: {}",
                error,
            );
            None
        }
    };

    defmt::info!(
        "Core 1 SD card initialized"
    );

    Ok((
        SdLogger {
            file: None,
            directory: root_directory,
            write_buffer: [0u8; 512],
            write_length: 0,
        },
        config,
        trajectory,
    ))
}