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

// ========================= 3×3 Linear Algebra =========================

/// Wrap angle to [−π, π].
pub fn wrap_angle(a: f32) -> f32 {
    atan2f(sinf(a), cosf(a))
}

/// 3×3 matrix stored in row-major order.
#[derive(Debug, Clone, Copy)]
pub struct Mat3 {
    pub data: [[f32; 3]; 3],
}

impl Mat3 {
    pub const fn zero() -> Self {
        Self {
            data: [[0.0; 3]; 3],
        }
    }

    pub const fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    pub const fn diag(a: f32, b: f32, c: f32) -> Self {
        Self {
            data: [
                [a, 0.0, 0.0],
                [0.0, b, 0.0],
                [0.0, 0.0, c],
            ],
        }
    }

    pub fn transpose(&self) -> Self {
        let d = &self.data;
        Self {
            data: [
                [d[0][0], d[1][0], d[2][0]],
                [d[0][1], d[1][1], d[2][1]],
                [d[0][2], d[1][2], d[2][2]],
            ],
        }
    }

    /// Matrix addition: self + other
    pub fn add(&self, other: &Self) -> Self {
        let mut out = Self::zero();
        for i in 0..3 {
            for j in 0..3 {
                out.data[i][j] = self.data[i][j] + other.data[i][j];
            }
        }
        out
    }

    /// Matrix subtraction: self - other
    pub fn sub(&self, other: &Self) -> Self {
        let mut out = Self::zero();
        for i in 0..3 {
            for j in 0..3 {
                out.data[i][j] = self.data[i][j] - other.data[i][j];
            }
        }
        out
    }

    /// Matrix multiplication: self * other
    pub fn mul(&self, other: &Self) -> Self {
        let mut out = Self::zero();
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0f32;
                for k in 0..3 {
                    sum += self.data[i][k] * other.data[k][j];
                }
                out.data[i][j] = sum;
            }
        }
        out
    }

    /// Matrix × vector: self * v
    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3 {
            data: [
                self.data[0][0] * v.data[0]
                    + self.data[0][1] * v.data[1]
                    + self.data[0][2] * v.data[2],
                self.data[1][0] * v.data[0]
                    + self.data[1][1] * v.data[1]
                    + self.data[1][2] * v.data[2],
                self.data[2][0] * v.data[0]
                    + self.data[2][1] * v.data[1]
                    + self.data[2][2] * v.data[2],
            ],
        }
    }

    /// Determinant (Sarrus' rule).
    fn det(&self) -> f32 {
        let d = &self.data;
        d[0][0] * (d[1][1] * d[2][2] - d[1][2] * d[2][1])
            - d[0][1] * (d[1][0] * d[2][2] - d[1][2] * d[2][0])
            + d[0][2] * (d[1][0] * d[2][1] - d[1][1] * d[2][0])
    }

    /// 3×3 inverse via Cramer's rule.  Returns `None` if singular.
    pub fn invert(&self) -> Option<Self> {
        let det = self.det();
        if det.abs() < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        let d = &self.data;

        Some(Self {
            data: [
                [
                    (d[1][1] * d[2][2] - d[1][2] * d[2][1]) * inv_det,
                    (d[0][2] * d[2][1] - d[0][1] * d[2][2]) * inv_det,
                    (d[0][1] * d[1][2] - d[0][2] * d[1][1]) * inv_det,
                ],
                [
                    (d[1][2] * d[2][0] - d[1][0] * d[2][2]) * inv_det,
                    (d[0][0] * d[2][2] - d[0][2] * d[2][0]) * inv_det,
                    (d[0][2] * d[1][0] - d[0][0] * d[1][2]) * inv_det,
                ],
                [
                    (d[1][0] * d[2][1] - d[1][1] * d[2][0]) * inv_det,
                    (d[0][1] * d[2][0] - d[0][0] * d[2][1]) * inv_det,
                    (d[0][0] * d[1][1] - d[0][1] * d[1][0]) * inv_det,
                ],
            ],
        })
    }
}

/// 3-element vector.
#[derive(Debug, Clone, Copy)]
pub struct Vec3 {
    pub data: [f32; 3],
}

impl Vec3 {
    pub const fn zero() -> Self {
        Self { data: [0.0; 3] }
    }

    pub const fn new(a: f32, b: f32, c: f32) -> Self {
        Self { data: [a, b, c] }
    }

    pub fn sub(&self, other: &Self) -> Self {
        Self {
            data: [
                self.data[0] - other.data[0],
                self.data[1] - other.data[1],
                self.data[2] - other.data[2],
            ],
        }
    }

    pub fn add(&self, other: &Self) -> Self {
        Self {
            data: [
                self.data[0] + other.data[0],
                self.data[1] + other.data[1],
                self.data[2] + other.data[2],
            ],
        }
    }
}
