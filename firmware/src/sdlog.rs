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
use crate::setpoint::{register_trajectory, store_trajectory};

pub type SpiDev<'a> = ExclusiveDevice<Spi<'a, SPI0, spi::Blocking>, Output<'a>, NoDelay>;
pub type Sd<'a> = SdCard<SpiDev<'a>, Delay>;
pub type Clock = DummyClock;

const MAX_DIRS: usize = 4;
const MAX_FILES: usize = 4;
const MAX_VOLUMES: usize = 1;

pub static SDLOGGER_SHARED: Mutex<Raw, Option<SdLogger>> = Mutex::new(None);

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BinaryLogRecord {
    pub t_ms: u32,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub x_des: f32,
    pub y_des: f32,
    pub yaw_des: f32,
    pub v_ff: f32,
    pub w_ff: f32,
    pub v_actual: f32,
    pub w_actual: f32,
    pub omega_l_cmd: f32,
    pub omega_r_cmd: f32,
    pub omega_l_meas: f32,
    pub omega_r_meas: f32,
    pub duty_l: f32,
    pub duty_r: f32,
    pub x_err: f32,
    pub y_err: f32,
    pub yaw_err: f32,
    // Raw mocap (not fused by EKF)
    pub x_raw: f32,
    pub y_raw: f32,
    pub yaw_raw: f32,
    // IMU
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}

impl BinaryLogRecord {
    /// Initialize all float fields to NaN ("no data yet").
    /// Used by the consumer to start a row assembler.
    pub fn nan_init() -> Self {
        Self {
            t_ms: 0,
            x: f32::NAN, y: f32::NAN, yaw: f32::NAN,
            x_des: f32::NAN, y_des: f32::NAN, yaw_des: f32::NAN,
            v_ff: f32::NAN, w_ff: f32::NAN,
            v_actual: f32::NAN, w_actual: f32::NAN,
            omega_l_cmd: f32::NAN, omega_r_cmd: f32::NAN,
            omega_l_meas: f32::NAN, omega_r_meas: f32::NAN,
            duty_l: f32::NAN, duty_r: f32::NAN,
            x_err: f32::NAN, y_err: f32::NAN, yaw_err: f32::NAN,
            x_raw: f32::NAN, y_raw: f32::NAN, yaw_raw: f32::NAN,
            acc_x: f32::NAN, acc_y: f32::NAN, acc_z: f32::NAN,
            gyro_x: f32::NAN, gyro_y: f32::NAN, gyro_z: f32::NAN,
        }
    }

    /// Update the relevant fields from a LogEvent.
    pub fn apply_event(&mut self, event: &crate::robotstate::LogEvent) {
        use crate::robotstate::LogEvent;
        match event {
            LogEvent::EkfState(p) => {
                self.x = p.x;
                self.y = p.y;
                self.yaw = p.yaw;
            }
            LogEvent::Setpoint(s) => {
                self.x_des = s.x_des;
                self.y_des = s.y_des;
                self.yaw_des = s.yaw_des;
                self.v_ff = s.v_ff;
                self.w_ff = s.w_ff;
            }
            LogEvent::WheelCmd(w) => {
                self.omega_l_cmd = w.omega_l;
                self.omega_r_cmd = w.omega_r;
            }
            LogEvent::TrackingError(e) => {
                self.x_err = e.x_err;
                self.y_err = e.y_err;
                self.yaw_err = e.yaw_err;
            }
            LogEvent::Motor(m) => {
                self.duty_l = m.left;
                self.duty_r = m.right;
            }
            LogEvent::Encoder(e) => {
                self.omega_l_meas = e.omega_l;
                self.omega_r_meas = e.omega_r;
            }
            LogEvent::Mocap(p) => {
                self.x_raw = p.x;
                self.y_raw = p.y;
                self.yaw_raw = p.yaw;
            }
            LogEvent::Odom(o) => {
                self.v_actual = o.v;
                self.w_actual = o.w;
            }
            LogEvent::Imu(i) => {
                self.acc_x = i.acc_x;
                self.acc_y = i.acc_y;
                self.acc_z = i.acc_z;
                self.gyro_x = i.gyro_x;
                self.gyro_y = i.gyro_y;
                self.gyro_z = i.gyro_z;
            }
        }
    }
}

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

pub type SdDir = Directory<'static, Sd<'static>, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;
pub type SdFile = File<'static, Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;

pub struct SdLogger {
    pub file: Option<SdFile>,
    pub dir: &'static mut SdDir,
}

impl SdLogger {
    pub fn open_new_file(&mut self) {
        if let Some(f) = self.file.take() {
            drop(f);
        }

        // We transmute self.dir to have a static lifetime so that the opened file also gets a static lifetime
        let dir: &'static mut SdDir = unsafe { core::mem::transmute(&mut *self.dir) };

        let mut file = None;
        for i in 0..100 {
            let tens = b'0' + (i / 10) as u8;
            let ones = b'0' + (i % 10) as u8;

            let mut name = String::<8>::new();
            name.push('T').ok();
            name.push('R').ok();
            name.push(char::from(tens)).ok();
            name.push(char::from(ones)).ok();

            if let Ok(short_name) = ShortFileName::create_from_str(&name) {
                let exists = dir.open_file_in_dir(&short_name, Mode::ReadOnly).is_ok();

                if !exists {
                    let new_file = dir
                        .open_file_in_dir(&short_name, Mode::ReadWriteCreateOrAppend)
                        .unwrap();
                    file = Some(new_file);
                    break;
                }
            }
        }

        if let Some(mut new_file) = file {
            let magic = [0xAAu8, 0xBBu8, 0xCCu8, 0xDDu8];
            if let Err(e) = new_file.write(&magic) {
                defmt::error!("Failed to write magic header: {:?}", defmt::Debug2Format(&e));
            }
            let _ = new_file.flush();
            self.file = Some(new_file);
            defmt::info!("SD Logger: opened new binary log file");
        } else {
            defmt::error!("SD Logger: No free file slots found!");
        }
    }

    pub fn close_file(&mut self) {
        if let Some(f) = self.file.take() {
            drop(f);
            defmt::info!("SD Logger: file closed.");
        }
    }

    /// write in string
    pub fn write(&mut self, data: &[u8]) {
        if let Some(ref mut file) = self.file {
            if let Err(e) = file.write(data) {
                defmt::error!("Write failed: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    /// flush
    /// (CAUTION: This needs to be called when all writing task is finished!!!)
    pub fn flush(&mut self) {
        if let Some(ref mut file) = self.file {
            if let Err(e) = file.flush() {
                defmt::error!("Flush failed: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    /// print everything in the file(log.txt)
    pub fn read_all(&mut self) {
        if let Some(ref mut file) = self.file {
            if let Err(e) = file.seek_from_start(0) {
                defmt::error!("Seek failed: {:?}", defmt::Debug2Format(&e));
                return;
            }

            let mut buf = [0u8; 27];
            while !file.is_eof() {
                match file.read(&mut buf) {
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

    // SD logging file is opened dynamically when starting trajectory/mode.
    info!("SD logger initialized.");

    Ok((SdLogger { file: None, dir }, cfg))
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
        if let Some(ref mut file) = self.file {
            let _ = file.write(header);
        }
    }



    pub fn log_motion(&mut self, data: &MotionLog) {
        let raw: &[u8; core::mem::size_of::<MotionLog>()] = unsafe { core::mem::transmute(data) };

        if let Some(ref mut file) = self.file {
            if let Err(e) = file.write(raw) {
                defmt::error!("Write log failed: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    pub fn log_snapshot_as_csv(&mut self, data: &crate::robotstate::LogSnapshot, v_actual: f32, w_actual: f32) {
        if let Some(ref mut file) = self.file {
            let mut line: String<512> = String::new();

            let _ = core::write!(
                &mut line,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                data.t_ms,
                data.x,
                data.y,
                data.yaw,
                data.x_des,
                data.y_des,
                data.yaw_des,
                data.v_ff,
                data.w_ff,
                v_actual,
                w_actual,
                data.omega_l_cmd,
                data.omega_r_cmd,
                data.omega_l_meas,
                data.omega_r_meas,
                data.duty_l,
                data.duty_r,
                data.x_err,
                data.y_err,
                data.yaw_err,
            );
            let _ = file.write(line.as_bytes());
        }
    }

    pub fn log_snapshot_as_bin(&mut self, record: &BinaryLogRecord) {
        if let Some(ref mut file) = self.file {
            let ptr = record as *const BinaryLogRecord as *const u8;
            let size = core::mem::size_of::<BinaryLogRecord>();
            let bytes = unsafe { core::slice::from_raw_parts(ptr, size) };
            if let Err(e) = file.write(bytes) {
                defmt::error!("Write binary log failed: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    pub fn log_motion_as_csv(&mut self, log: &MotionLog) {
        if let Some(ref mut file) = self.file {
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

            let _ = file.write(line.as_bytes());
        }
    }

    pub fn log_motion_as_bin(&mut self, log: &MotionLog) {
        if let Some(ref mut file) = self.file {
            let size = mem::size_of::<MotionLog>();

            let ptr = log as *const MotionLog as *const u8;
            let bytes: &[u8] = unsafe { slice::from_raw_parts(ptr, size) };

            let _ = file.write(bytes);
        }
    }
}

#[embassy_executor::task]
pub async fn sd_logging_task(_cfg: Option<RobotConfig>) {
    // Wait for everything to spin up
    embassy_time::Timer::after_millis(500).await;

    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(20)); // 50 Hz

    loop {
        match embassy_futures::select::select(
            ticker.next(),
            crate::orchestrator_signal::STOP_LOG_SENDING_SIG.wait(),
        ).await {
            embassy_futures::select::Either::First(_) => {
                // Drain all pending events in the channel, writing one row per event
                while let Ok(event_with_time) = crate::robotstate::LOG_EVENT_CH.try_receive() {
                    let mut row = BinaryLogRecord::nan_init();
                    row.t_ms = event_with_time.t_ms;
                    row.apply_event(&event_with_time.event);

                    with_sdlogger(|logger| {
                        logger.log_snapshot_as_bin(&row);
                    }).await;
                }
            }
            embassy_futures::select::Either::Second(_) => {
                defmt::info!("sd_logging_task stopped via STOP_LOG_SENDING_SIG");

                // Drain any remaining events in the queue, writing one row per event
                while let Ok(event_with_time) = crate::robotstate::LOG_EVENT_CH.try_receive() {
                    let mut row = BinaryLogRecord::nan_init();
                    row.t_ms = event_with_time.t_ms;
                    row.apply_event(&event_with_time.event);

                    with_sdlogger(|logger| {
                        logger.log_snapshot_as_bin(&row);
                    }).await;
                }

                with_sdlogger(|logger| {
                    logger.flush();
                }).await;
                break;
            }
        }
    }
}
