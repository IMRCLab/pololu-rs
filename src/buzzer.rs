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

#[derive(Copy, Clone)]
pub struct BuzzerController {
    buzzer: &'static Mutex<ThreadModeRawMutex, Buzzer<'static>>,
}

impl BuzzerController {
    pub fn new(buzzer: &'static Mutex<ThreadModeRawMutex, Buzzer<'static>>) -> Self {
        Self { buzzer }
    }

    pub async fn tone(&self, f: u32, d: f32) {
        self.buzzer.lock().await.tone(f, d);
    }

    pub async fn stop(&self) {
        self.buzzer.lock().await.stop();
    }

    pub async fn beep(&self, f: u32, d: f32, interval: u64, count: usize) {
        self.buzzer.lock().await.beep(f, d, interval, count).await;
    }
}

// ======================
// Seven Notes (Hz)
// ======================
pub const C4: u32 = 262;
pub const D4: u32 = 294;
pub const E4: u32 = 330;
pub const F4: u32 = 349;
pub const G4: u32 = 392;
pub const A4: u32 = 440;
pub const B4: u32 = 494;
pub const C5: u32 = 523;

// Note duration in ms
pub const T: u64 = 350;

pub async fn play_startup_sound(buzzer: &BuzzerController) {
    for freq in (200..1200).step_by(40) {
        buzzer.tone(freq, 0.4).await;
        Timer::after(Duration::from_millis(12)).await;
    }

    buzzer.stop().await;
    Timer::after(Duration::from_millis(60)).await;

    let osc_freqs = [800, 1000, 600, 1200, 700, 1300];
    for &f in &osc_freqs {
        buzzer.tone(f, 0.4).await;
        Timer::after(Duration::from_millis(70)).await;
    }

    buzzer.stop().await;
    Timer::after(Duration::from_millis(80)).await;

    buzzer.tone(180, 0.5).await;
    Timer::after(Duration::from_millis(150)).await;

    for freq in [500, 800, 1200, 1600] {
        buzzer.tone(freq, 0.6).await;
        Timer::after(Duration::from_millis(60)).await;
    }

    buzzer.tone(2000, 0.8).await;
    Timer::after(Duration::from_millis(120)).await;

    buzzer.stop().await;
}

// ======================
// Play Twinkle Twinkle Little Star
// ======================
pub async fn play_twinkle(buzzer: &BuzzerController) {
    use embassy_time::{Duration, Timer};

    let melody = [
        C4, C4, G4, G4, A4, A4, G4, F4, F4, E4, E4, D4, D4, C4, G4, G4, F4, F4, E4, E4, D4, G4, G4,
        F4, F4, E4, E4, D4, C4, C4, G4, G4, A4, A4, G4, F4, F4, E4, E4, D4, D4, C4,
    ];

    for &note in melody.iter() {
        buzzer.tone(note, 0.5).await; // 50% duty
        Timer::after(Duration::from_millis(T)).await;

        buzzer.stop().await;
        Timer::after(Duration::from_millis(30)).await; // small gap
    }
}

// ======================
// Play Fast Jingle Bells
// ======================
pub async fn play_jingle_bells(buzzer: &BuzzerController) {
    use embassy_time::{Duration, Timer};

    // Faster tempo
    let bpm = 180;
    let beat = 60000 / bpm; // quater

    //
    let melody = [
        (E4, 0.8),
        (E4, 0.8),
        (E4, 1.6),
        (E4, 0.8),
        (E4, 0.8),
        (E4, 1.6),
        (E4, 0.8),
        (G4, 0.8),
        (C4, 0.8),
        (D4, 0.8),
        (E4, 2.4),
        (F4, 0.8),
        (F4, 0.8),
        (F4, 1.2),
        (F4, 0.6),
        (F4, 0.8),
        (E4, 0.8),
        (E4, 0.8),
        (E4, 1.2),
        (E4, 0.6),
        (E4, 0.8),
        (D4, 0.8),
        (D4, 0.8),
        (E4, 0.8),
        (D4, 1.6),
        (G4, 1.6),
    ];

    let gap = 25;

    for &(note, len) in melody.iter() {
        let dur = (beat as f32 * len) as u64;

        if note > 0 {
            buzzer.tone(note, 0.5).await;
        } else {
            buzzer.stop().await;
        }

        Timer::after(Duration::from_millis(dur)).await;

        buzzer.stop().await;
        Timer::after(Duration::from_millis(gap)).await;
    }
}
