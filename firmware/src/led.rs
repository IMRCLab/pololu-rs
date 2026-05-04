use embassy_rp::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as Raw;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

pub static LED_SHARED: Mutex<Raw, Option<Led>> = Mutex::new(None);

pub struct Led {
    pin: Output<'static>,
}

impl Led {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }

    pub fn on(&mut self) {
        self.pin.set_high();
    }

    pub fn off(&mut self) {
        self.pin.set_low();
    }

    pub async fn blink(&mut self, interval_ms: u64, count: usize) {
        for _ in 0..count {
            self.on();
            Timer::after(Duration::from_millis(interval_ms)).await;
            self.off();
            Timer::after(Duration::from_millis(interval_ms)).await;
        }
    }
}

/// Convenience helper to set the shared LED on/off from any task.
/// Moved from trajectory_control.rs to consolidate LED logic.
pub async fn led_set(on: bool) {
    let mut g = LED_SHARED.lock().await;
    if let Some(led) = g.as_mut() {
        if on { led.on(); } else { led.off(); }
    } else {
        defmt::warn!("LED not available; skip.");
    }
}
