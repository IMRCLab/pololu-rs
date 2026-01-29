use core::f32::consts::FRAC_1_SQRT_2;

const PI: f32 = core::f32::consts::PI;

use libm::{asinf, atan2f, cosf, sinf, sqrtf};

pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn normalized(mut self) -> Self {
        let n = sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n > 0.0 {
            self.w /= n;
            self.x /= n;
            self.y /= n;
            self.z /= n;
        }
        self
    }
}

/// Decompression the quaternion sent via radio
pub fn quat_decompress(comp: u32) -> Quaternion {
    let mut comp = comp;
    let mask: u32 = (1 << 9) - 1; // 9-bit magnitude -> 0..511

    let i_largest: usize = (comp >> 30) as usize;

    let mut q = [0.0f32; 4];
    let mut sum_squares: f32 = 0.0;

    for i in (0..=3).rev() {
        if i != i_largest {
            let mag: u32 = comp & mask;
            let negbit: u32 = (comp >> 9) & 0x1;
            comp >>= 10;

            let mut val = FRAC_1_SQRT_2 * (mag as f32) / (mask as f32);
            if negbit == 1 {
                val = -val;
            }
            q[i] = val;
            sum_squares += val * val;
        }
    }

    q[i_largest] = sqrtf(1.0f32 - sum_squares);

    Quaternion {
        w: q[3],
        x: q[0],
        y: q[1],
        z: q[2],
    }
}

/// Convert quaternion to roll, pitch, yaw (in radians)
pub fn rpy_from_quaternion(q: &Quaternion) -> (f32, f32, f32) {
    let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let roll = atan2f(sinr_cosp, cosr_cosp);

    let sinp = 2.0 * (q.w * q.y - q.z * q.x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * core::f32::consts::FRAC_PI_2
    } else {
        asinf(sinp)
    };

    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    let yaw = atan2f(siny_cosp, cosy_cosp);

    (roll, pitch, yaw)
}

/// SO2 reqresentation of yaw angle in radians
#[derive(Debug, Copy, Clone)]
pub struct SO2 {
    value: f32, // rad, [-pi, pi]
}

impl SO2 {
    pub fn zero() -> SO2 {
        SO2 { value: 0.0 }
    }
    pub fn value(&self) -> f32 {
        self.value
    }

    pub fn new(value: f32) -> SO2 {
        let mut v = SO2 { value };
        v.normalize();
        v
    }

    // Normalize radians to be in range [-pi,pi]
    // See https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
    fn normalize(&mut self) {
        // Copy the sign of the value in radians to the value of pi.
        let signed_pi = PI.copysign(self.value);
        // Set the value of difference to the appropriate signed value between pi and -pi.
        self.value = (self.value + signed_pi) % (2.0 * PI) - signed_pi;
    }

    // returns the current value in radians [-pi, pi]
    pub fn rad(&self) -> f32 {
        self.value
    }

    pub fn cos(&self) -> f32 {
        cosf(self.value)
    }

    pub fn sin(&self) -> f32 {
        sinf(self.value)
    }

    // smallest signed difference
    // see https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    pub fn error(target: SO2, source: SO2) -> f32 {
        let d = target.value - source.value;
        atan2f(sinf(d), cosf(d))
    }

    pub fn distance(a: SO2, b: SO2) -> f32 {
        assert!(a.value <= PI);
        assert!(a.value >= -PI);
        assert!(b.value <= PI);
        assert!(b.value >= -PI);

        let d = (a.value - b.value).abs();
        if d > PI {
            return 2.0 * PI - d;
        }
        d
    }
    pub fn add(&mut self, a: f32) -> SO2 {
        let mut v = SO2::new(self.value + a);
        v.normalize();
        v
    }
}
