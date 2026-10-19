//! Single GPIO LED driver.

use embassy_rp::gpio::Output;

pub struct Led {
    output: Output<'static>,
}

impl Led {
    pub fn new(output: Output<'static>) -> Self {
        Self {
            output,
        }
    }

    pub fn on(&mut self) {
        self.output.set_high();
    }

    pub fn off(&mut self) {
        self.output.set_low();
    }

    pub fn set(&mut self, enabled: bool) {
        if enabled {
            self.on();
        } else {
            self.off();
        }
    }
}