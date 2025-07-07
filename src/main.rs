//! # UART and PWM Example Project
//!
//! Use receive interrupt to receive data from crazyflie radio and control the pwm duty.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// GPIO traits
use embedded_hal::digital::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;
use panic_probe as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
// use rp_pico::Pins;

/// crates for logging data
use defmt::*;
use defmt_rtt as _;

/// crates for UART communication
use rp2040_hal::gpio::FunctionUart;
use uart_irq::init_uart_with_irq;
pub mod uart_irq;
pub mod uart_nrf52_pk_rec;
use uart_nrf52_pk_rec::{try_read_packet, CmdPacket};

/// crates for pwm motor drive
pub mod pwm_motor;
use embedded_hal::pwm::SetDutyCycle;
use pwm_motor::init_motor_pwm;

#[entry]
fn main() -> ! {
    info!("Pololu Start!!!");
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init LED
    let mut led_pin = pins.led.into_push_pull_output();

    // init UART
    let tx = pins
        .gpio28
        .into_pull_up_input()
        .into_function::<FunctionUart>();
    let rx = pins
        .gpio29
        .into_pull_up_input()
        .into_function::<FunctionUart>();
    init_uart_with_irq(pac.UART0, tx, rx, &mut pac.RESETS, &clocks);

    // Init the PWM pins for both motors
    let gpio14 = pins.gpio14;
    let gpio15 = pins.gpio15;
    let gpio10 = pins.gpio10;
    let gpio11 = pins.gpio11;
    let mut duty_left = 0;
    let mut duty_right = 0;

    let mut motor = init_motor_pwm(pac.PWM, &mut pac.RESETS, gpio14, gpio15, gpio10, gpio11);
    motor.left_channel.set_duty_cycle(duty_left).unwrap();
    motor.left_direction.set_high().unwrap();
    motor.right_channel.set_duty_cycle(duty_right).unwrap();
    motor.right_direction.set_high().unwrap();

    loop {
        /*
        uart0.read_full_blocking(&mut buffer).unwrap();
        if let Some(cmd) = CmdLegacyPacketU16::from_bytes(&buffer) {
            info!(
                "Header is {}, Left Duty is {}, right Duty is {}, left direction is {}, right direction is {}",
                cmd.header,
                cmd.left_pwm_duty,
                cmd.right_pwm_duty,
                cmd.left_direction,
                cmd.right_direction,
            );
            duty_left = cmd.left_pwm_duty;
            duty_right = cmd.right_pwm_duty;

            /*
            info!(
                "Header is {:#04x}, Left Duty is {:#06x}, right Duty is {:#06x}, left direction is {:#06x}, right direction is {:#06x}",
                cmd.header,
                cmd.left_pwm_duty,
                cmd.right_pwm_duty,
                cmd.left_direction,
                cmd.right_direction,
            );*/
            // info!("{:?}", &buffer);
        } else {
            info!("Invalid Package!!!!!!!!!!!");
        }
        */
        if let Some(packet) = try_read_packet() {
            match packet {
                CmdPacket::U16(pkt) => {
                    defmt::info!(
                        "U16 Packet: header={}, left_duty={}, right_duty={}",
                        pkt.header,
                        pkt.left_pwm_duty,
                        pkt.right_pwm_duty
                    );
                    duty_left = pkt.left_pwm_duty;
                    duty_right = pkt.right_pwm_duty;
                }
                CmdPacket::F32(pkt) => {
                    defmt::info!(
                        "F32 Packet: header={}, left_duty={}, right_duty={}",
                        pkt.header,
                        pkt.left_pwm_duty,
                        pkt.right_pwm_duty
                    );
                }
                CmdPacket::Mix(pkt) => {
                    defmt::info!(
                        "Mix Packet: header={}, left_duty={}, right_duty={}",
                        pkt.header,
                        pkt.left_pwm_duty,
                        pkt.right_pwm_duty
                    );
                }
            }
        }

        motor.left_channel.set_duty_cycle(duty_left).unwrap();
        motor.left_direction.set_high().unwrap();
        motor.right_channel.set_duty_cycle(duty_right).unwrap();
        motor.right_direction.set_low().unwrap();

        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
