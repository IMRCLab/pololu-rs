use core::f32::consts::FRAC_1_SQRT_2;

use libm::{asinf, atan2f, sqrtf};

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

pub fn decode_quat_i8x4(p: u32) -> Quaternion {
    let w = ((p >> 24) & 0xFF) as u8 as i8;
    let x = ((p >> 16) & 0xFF) as u8 as i8;
    let y = ((p >> 8) & 0xFF) as u8 as i8;
    let z = ((p >> 0) & 0xFF) as u8 as i8;
    let f = |v: i8| -> f32 { (v as f32) / 127.0 };

    Quaternion {
        w: f(w),
        x: f(x),
        y: f(y),
        z: f(z),
    }
}

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
