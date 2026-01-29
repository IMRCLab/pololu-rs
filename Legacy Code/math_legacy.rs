/// Decompression the quaternion sent via radio
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
