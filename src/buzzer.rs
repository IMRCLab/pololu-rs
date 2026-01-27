#![allow(dead_code)]

use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use static_cell::StaticCell;

pub struct Buzzer<'a> {
    pwm: Pwm<'a>,
    top: u16,
}

impl<'a> Buzzer<'a> {
    pub fn new(pwm: Pwm<'a>, top: u16) -> Self {
        Self { pwm, top }
    }

    /// Change PWM frequency to play a musical tone
    pub fn tone(&mut self, freq_hz: u32, duty_ratio: f32) {
        let top = self.top as u32;

        // divider = 125MHz / (freq * (top + 1))
        let divider_f = 125_000_000.0 / (freq_hz as f32 * (top + 1) as f32);

        // convert to U8.4 fixed point
        let divider_fixed = divider_f.to_fixed::<fixed::types::U12F4>();

        let mut cfg = PwmConfig::default();
        cfg.top = self.top;
        cfg.compare_b = (duty_ratio * self.top as f32) as u16;
        cfg.divider = divider_fixed;

        self.pwm.set_config(&cfg);
    }

    /// Stop buzzer
    pub fn stop(&mut self) {
        let mut cfg = PwmConfig::default();
        cfg.top = self.top;
        cfg.compare_b = 0;
        self.pwm.set_config(&cfg);
    }

    /// Beep the same tone multiple times
    pub async fn beep(&mut self, freq: u32, duty: f32, interval_ms: u64, count: usize) {
        for _ in 0..count {
            self.tone(freq, duty);
            Timer::after(Duration::from_millis(interval_ms)).await;
            self.stop();
            Timer::after(Duration::from_millis(interval_ms)).await;
        }
    }
}

// Wrap in Mutex
static BUZZER_CELL: StaticCell<Mutex<ThreadModeRawMutex, Buzzer<'static>>> = StaticCell::new();

pub fn init_buzzer(pwm: Pwm<'static>, top: u16) -> BuzzerController {
    let buzzer = BUZZER_CELL.init(Mutex::new(Buzzer::new(pwm, top)));
    BuzzerController::new(buzzer)
}

// Buzzer controller to be shared across tasks
#[derive(Copy, Clone)]
pub struct BuzzerController {
    buzzer: &'static Mutex<ThreadModeRawMutex, Buzzer<'static>>,
}

impl BuzzerController {
    pub fn new(buzzer: &'static Mutex<ThreadModeRawMutex, Buzzer<'static>>) -> Self {
        Self { buzzer }
    }

    /// Play a tone with given frequency and duty cycle
    pub async fn tone(&self, f: u32, d: f32) {
        self.buzzer.lock().await.tone(f, d);
    }

    /// Stop the buzzer
    pub async fn stop(&self) {
        self.buzzer.lock().await.stop();
    }

    /// Beep the buzzer
    pub async fn beep(&self, f: u32, d: f32, interval: u64, count: usize) {
        self.buzzer.lock().await.beep(f, d, interval, count).await;
    }
}
