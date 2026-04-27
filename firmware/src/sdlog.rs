use core::fmt::Write as FmtWrite;
use core::{mem, slice};
use defmt::*;
use embassy_rp::{
    Peri,
    gpio::{Level, Output},
    peripherals::{PIN_18, PIN_19, PIN_20, PIN_21, SPI0},
    spi::{self, Spi},
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as Raw;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{
    Directory, File, Mode, SdCard, ShortFileName, TimeSource, Timestamp, Volume, VolumeIdx,
    VolumeManager,
};
use heapless::String;
use static_cell::StaticCell;

use crate::read_robot_config_from_sd::{RobotConfig, load_robot_config_with_dir};
use crate::trajectory_control::{register_trajectory, store_trajectory};

pub type SpiDev<'a> = ExclusiveDevice<Spi<'a, SPI0, spi::Blocking>, Output<'a>, NoDelay>;
pub type Sd<'a> = SdCard<SpiDev<'a>, Delay>;
pub type Clock = DummyClock;

const MAX_DIRS: usize = 4;
const MAX_FILES: usize = 4;
const MAX_VOLUMES: usize = 1;

pub static SDLOGGER_SHARED: Mutex<Raw, Option<SdLogger>> = Mutex::new(None);

/// Convenience helper to run a closure with the shared SD logger.
/// Returns `None` if no SD card / logger is available.
/// Moved from trajectory_control.rs to consolidate logging logic.
pub async fn with_sdlogger<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut SdLogger) -> R,
{
    let mut g = SDLOGGER_SHARED.lock().await;
    if let Some(l) = g.as_mut() {
        Some(f(l))
    } else {
        defmt::warn!("SdLogger not available; skip.");
        None
    }
}

// === Time Resources ===
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

// === Static Resources ===
static VOLUME_MGR: StaticCell<
    VolumeManager<Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
> = StaticCell::new();
static VOLUME: StaticCell<Volume<'static, Sd<'static>, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>> =
    StaticCell::new();
static DIR: StaticCell<Directory<'static, Sd<'static>, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>> =
    StaticCell::new();
static SCRATCH_JSON: StaticCell<[u8; 48 * 1024]> = StaticCell::new();

// pub struct SdLogger<'a> {
//     // volume_mgr: &'a mut VolumeManager<Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
//     file: File<'static, Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
// }

pub struct SdLogger {
    file: File<'static, Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
}

impl SdLogger {
    /// write in string
    pub fn write(&mut self, data: &[u8]) {
        if let Err(e) = self.file.write(data) {
            defmt::error!("Write failed: {:?}", defmt::Debug2Format(&e));
        }
    }

    /// flush
    /// (CAUTION: This needs to be called when all writing task is finished!!!)
    pub fn flush(&mut self) {
        if let Err(e) = self.file.flush() {
            defmt::error!("Flush failed: {:?}", defmt::Debug2Format(&e));
        }
    }

    /// print everything in the file(log.txt)
    pub fn read_all(&mut self) {
        if let Err(e) = self.file.seek_from_start(0) {
            defmt::error!("Seek failed: {:?}", defmt::Debug2Format(&e));
            return;
        }

        let mut buf = [0u8; 27];
        while !self.file.is_eof() {
            match self.file.read(&mut buf) {
                Ok(n) => {
                    if n > 0 {
                        info!("{:a}", &buf[..n]);
                    }
                }
                Err(e) => {
                    defmt::warn!("Read error: {:?}", defmt::Debug2Format(&e));
                    break;
                }
            }
        }
    }
}

#[derive(Debug, defmt::Format)]
pub enum SdError {
    NoCardOrInitFail,
    OpenVolume,
    OpenRoot,
    FileCreate,
    NoFreeSlot,
    BadShortName,
}

/// Initialize SD card, Loading File system, open/create log.txt
pub fn init_sd_logger(
    spi: Peri<'static, SPI0>,
    sck: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_20>,
    cs: Peri<'static, PIN_21>,
) -> Result<(SdLogger, RobotConfig), SdError> {
    // SPI clock needs to be running at <= 400kHz during initialization
    let mut slow_cfg = spi::Config::default();
    slow_cfg.frequency = 400_000;
    let spi = Spi::new_blocking(spi, sck, mosi, miso, slow_cfg);
    let cs = Output::new(cs, Level::High);

    // Initialize volume manager in one step
    let volume_mgr = VOLUME_MGR.init_with(|| {
        let spi_dev = ExclusiveDevice::new_no_delay(spi, cs);
        let sdcard = SdCard::new(spi_dev, Delay);

        // Speed up for normal write/read
        let mut fast_config = spi::Config::default();
        fast_config.frequency = 16_000_000;
        sdcard.spi(|dev| dev.bus_mut().set_config(&fast_config));

        VolumeManager::new(sdcard, DummyClock)
    });

    // Safely open file
    let volume_res = { volume_mgr.open_volume(VolumeIdx(0)) };
    let volume = match volume_res {
        Ok(v) => v,
        Err(_e) => {
            defmt::warn!("open_volume failed!!");
            return Err(SdError::NoCardOrInitFail);
        }
    };
    let volume = VOLUME.init(volume);

    let dir_res = volume.open_root_dir();
    let dir = match dir_res {
        Ok(d) => d,
        Err(_e) => {
            defmt::warn!("open_root_dir failed!!");
            return Err(SdError::OpenRoot);
        }
    };
    let dir = DIR.init(dir);

    // === Read + Parse Trajectory（File name must be json, file name format must be: TRJ0001.JSN） ===
    let scratch = SCRATCH_JSON.init([0u8; 48 * 1024]);

    // check length
    match crate::trajectory_reading::file_len_with_dir::<
        Sd<'static>,
        Clock,
        { MAX_DIRS },
        { MAX_FILES },
        { MAX_VOLUMES },
    >(dir, "TRJ0001.JSN")
    {
        Ok(len) => {
            defmt::info!("Trajectory file len = {} bytes", len);
            if (len as usize) > scratch.len() {
                defmt::warn!(
                    "Trajectory too large for scratch ({} > {}), skip loading",
                    len,
                    scratch.len()
                );
            } else {
                match crate::trajectory_reading::load_trajectory_with_dir_count::<
                    Sd<'static>,
                    Clock,
                    { MAX_DIRS },
                    { MAX_FILES },
                    { MAX_VOLUMES },
                >(dir, "TRJ0001.JSN", &mut scratch[..])
                {
                    Ok((traj, n_s, n_a)) => {
                        defmt::info!("Trajectory parsed: states={}, actions={}", n_s, n_a);
                        let traj_ref = store_trajectory(traj);
                        register_trajectory(traj_ref);
                    }
                    Err(e) => {
                        defmt::warn!("Trajectory load/parse failed: {}", e);
                    }
                }
            }
        }
        Err(e) => {
            defmt::warn!("Trajectory file not found or open failed: {}", e);
        }
    }
    // === Read + Parse Trajectory（File name must be json, file name format must be: TRJ0001.JSN） ===

    // =============================== Read Robot Configuration file =================================
    let cfg = load_robot_config_with_dir::<
        Sd<'static>,
        Clock,
        { MAX_DIRS },
        { MAX_FILES },
        { MAX_VOLUMES },
    >(dir, scratch);

    // ============================= Prepare logging file for trajectory =============================
    let mut file = None;
    for i in 0..100 {
        // File name will be TR00....TR99, binary file
        let tens = b'0' + (i / 10) as u8;
        let ones = b'0' + (i % 10) as u8;

        let mut name = String::<8>::new();
        name.push('T').ok();
        name.push('R').ok();
        name.push(char::from(tens)).ok();
        name.push(char::from(ones)).ok();

        if let Ok(short_name) = ShortFileName::create_from_str(&name) {
            // drop the first borrow
            let exists = dir.open_file_in_dir(&short_name, Mode::ReadOnly).is_ok();

            if !exists {
                // first borrow is released, so the second borrow is safe.
                let new_file = dir
                    .open_file_in_dir(&short_name, Mode::ReadWriteCreateOrAppend)
                    .unwrap();
                file = Some(new_file);
                break;
            }
        }
    }

    let file = file.expect("No available log file slot");
    info!("SD logger initialized.");

    Ok((SdLogger { file }, cfg))
}

#[repr(C)]
pub struct TrajControlLog {
    pub timestamp_ms: u32,
    pub target_x: f32,
    pub target_y: f32,
    pub target_theta: f32,
    pub actual_x: f32,
    pub actual_y: f32,
    pub actual_theta: f32,
    pub target_vx: f32,
    pub target_vy: f32,
    pub target_vz: f32,
    pub actual_vx: f32,
    pub actual_vy: f32,
    pub actual_vz: f32,
    pub target_qw: f32,
    pub target_qx: f32,
    pub target_qy: f32,
    pub target_qz: f32,
    pub actual_qw: f32,
    pub actual_qx: f32,
    pub actual_qy: f32,
    pub actual_qz: f32,
    //control errors from
    pub xerror: f32,
    pub yerror: f32,
    pub thetaerror: f32,
    pub ul: f32,
    pub ur: f32,
    pub dutyl: f32,
    pub dutyr: f32,
}

#[repr(C)]
pub struct MotionLog {
    pub timestamp_ms: u32,
    pub target_vx: f32,
    pub target_vy: f32,
    pub target_vz: f32,
    pub target_qw: f32,
    pub target_qx: f32,
    pub target_qy: f32,
    pub target_qz: f32,
    pub actual_vx: f32,
    pub actual_vy: f32,
    pub actual_vz: f32,
    pub actual_qw: f32,
    pub actual_qx: f32,
    pub actual_qy: f32,
    pub actual_qz: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub motor_left: i16,
    pub motor_right: i16,
}

impl SdLogger {
    pub fn write_csv_header(&mut self) {
        let header = b"ts,target_vx,target_vy,target_vz,target_qw,target_qx,target_qy,target_qz,actual_vx,actual_vy,actual_vz,actual_qw,actual_qx,actual_qy,actual_qz,roll,pitch,yaw,motor_l,motor_r\n";
        let _ = self.file.write(header);
    }

    pub fn write_traj_control_header(&mut self) {
        let header = b"ts,target_x,target_y,target_theta,actual_x,actual_y,actual_theta,target_vx,target_vy,target_vz,actual_vx,actual_vy,actual_vz,target_qw,target_qx,target_qy,target_qz,actual_qw,actual_qx,actual_qy,actual_qz,xerror,yerror,thetaerror,ul,ur,dutyl,dutyr\n";
        let _ = self.file.write(header);
    }

    pub fn log_motion(&mut self, data: &MotionLog) {
        let raw: &[u8; core::mem::size_of::<MotionLog>()] = unsafe { core::mem::transmute(data) };

        if let Err(e) = self.file.write(raw) {
            defmt::error!("Write log failed: {:?}", defmt::Debug2Format(&e));
        }
    }

    pub fn log_traj_control_as_csv(&mut self, data: &TrajControlLog) {
        let mut line: String<512> = String::new();

        let _ = core::write!(
            &mut line,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            data.timestamp_ms,
            data.target_x,
            data.target_y,
            data.target_theta,
            data.actual_x,
            data.actual_y,
            data.actual_theta,
            data.target_vx,
            data.target_vy,
            data.target_vz,
            data.actual_vx,
            data.actual_vy,
            data.actual_vz,
            data.target_qw,
            data.target_qx,
            data.target_qy,
            data.target_qz,
            data.actual_qw,
            data.actual_qx,
            data.actual_qy,
            data.actual_qz,
            //errrs
            data.xerror,
            data.yerror,
            data.thetaerror,
            data.ul,
            data.ur,
            data.dutyl,
            data.dutyr,
        );
        let _ = self.file.write(line.as_bytes());
    }

    pub fn log_motion_as_csv(&mut self, log: &MotionLog) {
        let mut line: String<512> = String::new();

        let _ = core::write!(
            &mut line,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            log.timestamp_ms,
            log.target_vx,
            log.target_vy,
            log.target_vz,
            log.target_qw,
            log.target_qx,
            log.target_qy,
            log.target_qz,
            log.actual_vx,
            log.actual_vy,
            log.actual_vz,
            log.actual_qw,
            log.actual_qx,
            log.actual_qy,
            log.actual_qz,
            log.roll,
            log.pitch,
            log.yaw,
            log.motor_left,
            log.motor_right,
        );

        let _ = self.file.write(line.as_bytes());
    }

    pub fn log_motion_as_bin(&mut self, log: &MotionLog) {
        let size = mem::size_of::<MotionLog>();

        let ptr = log as *const MotionLog as *const u8;
        let bytes: &[u8] = unsafe { slice::from_raw_parts(ptr, size) };

        let _ = self.file.write(bytes);
    }
}
