// use embassy_futures::join::join;
use embassy_rp::gpio::Output;
use embassy_rp::pwm::{PwmOutput, SetDutyCycle};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

/// Motor struct for a single motor
pub struct Motor<'a> {
    pwm: PwmOutput<'a>,
    dir: Output<'a>,
    top: u16,
}

impl<'a> Motor<'a> {
    pub fn new(pwm: PwmOutput<'a>, dir: Output<'a>, top: u16) -> Self {
        Self { pwm, dir, top }
    }

    /// Set motor speed from -1.0 to 1.0, which means full reverse to full forward.
    pub fn set_speed(&mut self, speed: f32) {
        let clamped = speed.clamp(-1.0, 1.0);
        if clamped >= 0.0 {
            self.dir.set_high();
        } else {
            self.dir.set_low();
        }
        let duty = (clamped.abs() * self.top as f32) as u16;
        self.pwm.set_duty_cycle(duty).unwrap();
    }
}

// Shared mutex for left and right motors
type SharedMotor = &'static Mutex<ThreadModeRawMutex, Motor<'static>>;

// Combine Controller for both motors
#[derive(Copy, Clone)]
pub struct MotorController {
    left: SharedMotor,
    right: SharedMotor,
}

impl MotorController {
    pub fn new(left: SharedMotor, right: SharedMotor) -> Self {
        Self { left, right }
    }

    /// Set both motor speed from -1.0 to 1.0, which means full reverse to full forward.
    pub async fn set_speed(&self, left: f32, right: f32) {
        self.left.lock().await.set_speed(left);
        self.right.lock().await.set_speed(right);
    }
}

// Static memory storage (no global access, only used for initialization)
static LEFT_CELL: StaticCell<Mutex<ThreadModeRawMutex, Motor<'static>>> = StaticCell::new();
static RIGHT_CELL: StaticCell<Mutex<ThreadModeRawMutex, Motor<'static>>> = StaticCell::new();

/// Initialization for both motors
/// assigns PWM output pins and direction pins, and PWM top frequency
pub fn init_motor(
    pwm_left: PwmOutput<'static>,
    dir_left: Output<'static>,
    pwm_right: PwmOutput<'static>,
    dir_right: Output<'static>,
    top: u16,
) -> MotorController {
    let left = LEFT_CELL.init(Mutex::new(Motor::new(pwm_left, dir_left, top)));
    let right = RIGHT_CELL.init(Mutex::new(Motor::new(pwm_right, dir_right, top)));
    MotorController::new(left, right)
}
