

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



pub type SdDir = Directory<'static, Sd<'static>, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;
pub type SdFile = File<'static, Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;

pub struct SdLogger {
    pub file: Option<SdFile>,
    pub dir: &'static mut SdDir,
    pub write_buf: [u8; 512],
    pub write_len: usize,
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

    pub fn write_buffered(&mut self, data: &[u8]) {
        let mut input_offset = 0;
        while input_offset < data.len() {
            let available = 512 - self.write_len;
            let to_copy = core::cmp::min(available, data.len() - input_offset);
            self.write_buf[self.write_len..self.write_len + to_copy]
                .copy_from_slice(&data[input_offset..input_offset + to_copy]);
            self.write_len += to_copy;
            input_offset += to_copy;
            
            if self.write_len == 512 {
                self.flush_buffer();
            }
        }
    }

    pub fn flush_buffer(&mut self) {
        if self.write_len > 0 {
            if let Some(ref mut file) = self.file {
                if let Err(e) = file.write(&self.write_buf[..self.write_len]) {
                    defmt::error!("Buffered write failed: {:?}", defmt::Debug2Format(&e));
                }
            }
            self.write_len = 0;
        }
    }

    /// flush
    /// (CAUTION: This needs to be called when all writing task is finished!!!)
    pub fn flush(&mut self) {
        self.flush_buffer();
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

    Ok((SdLogger { file: None, dir, write_buf: [0; 512], write_len: 0 }, cfg))
}



impl SdLogger {


    fn write_floats(&mut self, floats: &[f32]) {
        let ptr = floats.as_ptr() as *const u8;
        let size = floats.len() * 4;
        let bytes = unsafe { core::slice::from_raw_parts(ptr, size) };
        self.write_buffered(bytes);
    }

    pub fn log_event_compact(&mut self, t_ms: u32, event: &crate::robotstate::LogEvent) {
        use crate::robotstate::LogEvent;
        self.write_buffered(&t_ms.to_le_bytes());
        match event {
            LogEvent::EkfState(p) => {
                self.write_buffered(&[1]);
                self.write_floats(&[p.x, p.y, p.z, p.roll, p.pitch, p.yaw]);
            }
            LogEvent::Setpoint(s) => {
                self.write_buffered(&[2]);
                self.write_floats(&[s.x_des, s.y_des, s.yaw_des, s.v_ff, s.w_ff]);
            }
            LogEvent::WheelCmd(w) => {
                self.write_buffered(&[3]);
                self.write_floats(&[w.omega_l, w.omega_r]);
            }
            LogEvent::TrackingError(e) => {
                self.write_buffered(&[4]);
                self.write_floats(&[e.x_err, e.y_err, e.yaw_err]);
            }
            LogEvent::Motor(m) => {
                self.write_buffered(&[5]);
                self.write_floats(&[m.left, m.right]);
            }
            LogEvent::Encoder(e) => {
                self.write_buffered(&[6]);
                self.write_floats(&[e.omega_l, e.omega_r]);
            }
            LogEvent::Mocap(p) => {
                self.write_buffered(&[7]);
                self.write_floats(&[p.x, p.y, p.z, p.roll, p.pitch, p.yaw]);
            }
            LogEvent::Odom(o) => {
                self.write_buffered(&[8]);
                self.write_floats(&[o.x, o.y, o.theta, o.v, o.w]);
            }
            LogEvent::Imu(i) => {
                self.write_buffered(&[9]);
                self.write_floats(&[i.acc_x, i.acc_y, i.acc_z, i.gyro_x, i.gyro_y, i.gyro_z]);
            }
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
                    with_sdlogger(|logger| {
                        logger.log_event_compact(event_with_time.t_ms, &event_with_time.event);
                    }).await;

                    // Yield to the executor to allow other tasks (like UART RX) to run
                    embassy_futures::yield_now().await;
                }
            }
            embassy_futures::select::Either::Second(_) => {
                defmt::info!("sd_logging_task stopped via STOP_LOG_SENDING_SIG");

                // Drain any remaining events in the queue, writing one row per event
                while let Ok(event_with_time) = crate::robotstate::LOG_EVENT_CH.try_receive() {
                    with_sdlogger(|logger| {
                        logger.log_event_compact(event_with_time.t_ms, &event_with_time.event);
                    }).await;

                    // Yield to the executor
                    embassy_futures::yield_now().await;
                }

                with_sdlogger(|logger| {
                    logger.flush();
                }).await;
                break;
            }
        }
    }
}
