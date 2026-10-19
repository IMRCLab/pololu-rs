//! Raw button GPIO driver.
//!
//! Button behavior and mode policy belong to a Core 1 task, not this driver.

use embassy_rp::gpio::Input;

pub struct Buttons {
    button_b: Input<'static>,
    button_c: Input<'static>,
}

impl Buttons {
    pub fn new(
        button_b: Input<'static>,
        button_c: Input<'static>,
    ) -> Self {
        Self {
            button_b,
            button_c,
        }
    }

    pub async fn wait_for_button_b(
        &mut self,
    ) {
        self.button_b
            .wait_for_falling_edge()
            .await;
    }

    pub async fn wait_for_button_c(
        &mut self,
    ) {
        self.button_c
            .wait_for_falling_edge()
            .await;
    }

    pub fn button_b_is_pressed(&self) -> bool {
        self.button_b.is_low()
    }

    pub fn button_c_is_pressed(&self) -> bool {
        self.button_c.is_low()
    }
}