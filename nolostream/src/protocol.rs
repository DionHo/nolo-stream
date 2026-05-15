use crate::btea;
use crate::pose::{DeviceId, Pose};

pub const NOLO_VID: u16 = 0x0483;
pub const NOLO_PID: u16 = 0x5750;

const NOLO_KEY: [u32; 4] = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51];

/// Bytes 1..=60 are 15 LE u32 words that need BTEA decryption (nolo-osvr: cryptoffset=1, cryptwords=15).
const CRYPTWORDS: usize = 15;

/// Controller block length in bytes (nolo-osvr: 3 + (3+4)*2 + 2 + 2 + 1 = 22).
/// Used to locate the right controller: buf[64 - CTRL_LEN] = buf[42].
const CTRL_LEN: usize = 22;

/// Decrypt the encrypted region of a 64-byte raw HID buffer, then parse it.
/// Returns an empty Vec on an unknown or invalid report.
pub fn parse_report(buf: &[u8]) -> Vec<Pose> {
    if buf.len() < 64 {
        return vec![];
    }
    let mut work = [0u8; 64];
    work.copy_from_slice(&buf[..64]);

    let mut words = [0u32; CRYPTWORDS];
    for (i, word) in words.iter_mut().enumerate() {
        let b = 1 + i * 4;
        *word = u32::from_le_bytes([work[b], work[b + 1], work[b + 2], work[b + 3]]);
    }
    btea::btea_decrypt(&mut words, 1, &NOLO_KEY);
    for (i, word) in words.iter().enumerate() {
        let b = 1 + i * 4;
        work[b..b + 4].copy_from_slice(&word.to_le_bytes());
    }

    parse_decrypted(&work)
}

/// Decrypt a 64-byte HID buffer in-place and return it (without parsing into Poses).
/// Useful for diagnostics.
pub fn decrypt_report(buf: &[u8]) -> Option<[u8; 64]> {
    if buf.len() < 64 {
        return None;
    }
    let mut work = [0u8; 64];
    work.copy_from_slice(&buf[..64]);
    let mut words = [0u32; CRYPTWORDS];
    for (i, word) in words.iter_mut().enumerate() {
        let b = 1 + i * 4;
        *word = u32::from_le_bytes([work[b], work[b + 1], work[b + 2], work[b + 3]]);
    }
    btea::btea_decrypt(&mut words, 1, &NOLO_KEY);
    for (i, word) in words.iter().enumerate() {
        let b = 1 + i * 4;
        work[b..b + 4].copy_from_slice(&word.to_le_bytes());
    }
    Some(work)
}

/// Parse a fully-decrypted 64-byte buffer into Pose values.
fn parse_decrypted(buf: &[u8]) -> Vec<Pose> {
    if buf.len() < 64 {
        return vec![];
    }
    // On Windows, hidapi gives report ID 0x10/0x11 at buf[0] instead of the raw
    // packet type 0xa5/0xa6 found on Linux hidraw. The encrypted region and all
    // block offsets within the decrypted payload are identical on both platforms.
    let pkt_type = match buf[0] {
        0xa5 | 0x10 => 0xa5u8,
        0xa6 | 0x11 => 0xa6u8,
        _ => return vec![],
    };
    match pkt_type {
        // Left-controller frame: newer firmware only places valid left controller data at buf[1].
        // buf[42] (old right-controller location) contains unrelated data in newer firmware.
        0xa5 => {
            let mut poses = Vec::with_capacity(2);
            if let Some(p) = parse_controller(buf, 1, DeviceId::LeftController) {
                poses.push(p);
            }
            if let Some(p) = parse_hmd(buf, 1) {
                poses.push(p);
            }
            poses
        }
        // Right-controller frame (newer firmware): device block at buf[1] contains right
        // controller data. HMD position is also embedded at the same base+24/26/28 offsets.
        0xa6 => {
            let mut poses = Vec::new();
            if let Some(p) = parse_controller(buf, 1, DeviceId::RightController) {
                poses.push(p);
            }
            if let Some(p) = parse_hmd(buf, 1) {
                poses.push(p);
            }
            poses
        }
        _ => vec![],
    }
}

/// Return 4 diagnostic i16s from a controller block used by --orient-debug.
/// Returns [IMU0, IMU2, IMU3, IMU4] at base+9, base+13, base+15, base+17.
/// Based on observations: base+9 ≈ accel axis, base+13/15 ≈ pitch/roll rate,
/// base+17 ≈ yaw rate OR buttons|touchID depending on IMU word count (TBD).
pub fn raw_orientation_bytes(buf: &[u8], base: usize) -> Option<[i16; 4]> {
    if buf.len() < base + 19 {
        return None;
    }
    if buf[base] == 0 && buf[base + 1] == 0 {
        return None;
    }
    Some([
        read_i16_be(buf, base + 9),   // AY
        read_i16_be(buf, base + 13),  // RX
        read_i16_be(buf, base + 15),  // RY
        read_i16_be(buf, base + 17),  // RZ
    ])
}

fn parse_hmd(buf: &[u8], base: usize) -> Option<Pose> {
    if buf.len() < base + 30 {
        return None;
    }
    let raw_x = read_i16_be(buf, base + 24);
    let raw_y = read_i16_be(buf, base + 26);
    let raw_z = read_i16_be(buf, base + 28);
    // All-zero means no HMD tracking data in this frame.
    if raw_x == 0 && raw_y == 0 && raw_z == 0 {
        return None;
    }
    let mut sensor_raw = [0i16; 19];
    if buf.len() >= base + 39 {
        for idx in 0..19usize {
            sensor_raw[idx] = read_i16_be(buf, base + 1 + idx * 2);
        }
    }
    Some(Pose {
        device: DeviceId::Headset,
        position: [raw_x as f32 * 0.0001, raw_y as f32 * 0.0001, raw_z as f32 * 0.0001],
        orientation: [1.0_f32, 0.0, 0.0, 0.0],
        sensor_raw,
        touch_x: 255,
        touch_y: 255,
        battery: 0,
        timestamp_ms: 0,
    })
}

fn parse_controller(buf: &[u8], base: usize, device: DeviceId) -> Option<Pose> {
    // Need at least position bytes (up to base+8).
    if buf.len() < base + 9 {
        return None;
    }
    // Skip all-zero blocks — device is likely off or not present.
    if buf[base] == 0 && buf[base + 1] == 0 {
        return None;
    }
    let position = parse_position(buf, base + 3);
    // Orientation set to identity: no fused quaternion found yet.
    // IMU data (accel+gyro) occupies base+9..18; exact word split is 4 or 5 words (TBD).
    // Confirmed field positions (newer firmware):
    //   base+3..8:  position (Y,Z,X raw order → remapped to X,Y,Z)
    //   base+9..16: IMU channels (accel X/Y/Z + gyro X/Y or similar 4-word layout)
    //   base+17..18: last IMU word OR buttons|touchID (overlaps with nolo-osvr buttons byte)
    //   base+19:    touch X (confirmed: 255=no touch, 127=center, increases swiping left)
    //   base+20:    touch Y (confirmed: 255=no touch, 127=center, increases swiping down)
    //   base+21:    battery (tentative, same offset as nolo-osvr)
    //   base+23:    rolling 1-byte counter (previously mistaken for 32-bit LE tick counter)
    //   base+24..25: HMD position X (i16 BE, ×0.0001 → m) — confirmed via movement test
    //   base+26..27: HMD position Y
    //   base+28..29: HMD position Z
    //   base+30+:   HMD IMU data (same layout as controller IMU at base+9)
    let orientation = [1.0_f32, 0.0, 0.0, 0.0];
    let touch_x = if buf.len() > base + 19 { buf[base + 19] } else { 255 };
    let touch_y = if buf.len() > base + 20 { buf[base + 20] } else { 255 };
    let battery  = if buf.len() > base + 21 { buf[base + 21] } else { 0 };
    // Collect 19 × i16 from base+1..base+38 for the graph.
    let mut sensor_raw = [0i16; 19];
    if buf.len() >= base + 39 {
        for idx in 0..19usize {
            sensor_raw[idx] = read_i16_be(buf, base + 1 + idx * 2);
        }
    }
    Some(Pose {
        device,
        position,
        orientation,
        sensor_raw,
        touch_x,
        touch_y,
        battery,
        timestamp_ms: 0,
    })
}

#[inline]
fn read_i16_be(buf: &[u8], offset: usize) -> i16 {
    i16::from_be_bytes([buf[offset], buf[offset + 1]])
}

/// 3× i16 big-endian, scaled by 0.0001 to give metres.
/// Raw device byte order: (Y, Z, X) in world frame → remap to (X, Y, Z).
#[inline]
fn parse_position(buf: &[u8], offset: usize) -> [f32; 3] {
    let raw_y = read_i16_be(buf, offset) as f32 * 0.0001;
    let raw_z = read_i16_be(buf, offset + 2) as f32 * 0.0001;
    let raw_x = read_i16_be(buf, offset + 4) as f32 * 0.0001;
    [raw_x, raw_y, raw_z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::btea::btea_encrypt;

    #[test]
    fn btea_roundtrip() {
        let original: [u32; 15] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let mut data = original;
        btea_encrypt(&mut data, 1, &NOLO_KEY);
        assert_ne!(data, original, "data must change after encryption");
        btea::btea_decrypt(&mut data, 1, &NOLO_KEY);
        assert_eq!(data, original, "decrypt(encrypt(x)) must equal x");
    }

    /// Build a synthetic pre-decrypted 0xa5 buffer and verify both controllers parse.
    /// Raw position byte order is (raw_y, raw_z, raw_x); parse_position remaps to (X, Y, Z).
    #[test]
    fn parse_controller_report() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa5;

        // --- Left controller block at buf[1] ---
        buf[1] = 2; // hwversion (any non-zero accepted)
        buf[2] = 1; // fwversion
        // raw_y=1000 raw_z=2000 raw_x=3000 → output pos=[0.3, 0.1, 0.2]
        buf[4] = 0x03; buf[5] = 0xE8; // raw_y=1000
        buf[6] = 0x07; buf[7] = 0xD0; // raw_z=2000
        buf[8] = 0x0B; buf[9] = 0xB8; // raw_x=3000
        // orientation at base+9=buf[10]: w=16384, i=j=k=0 → identity
        buf[10] = 0x40; buf[11] = 0x00; // w=16384

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 1);

        let left = &poses[0];
        assert!(matches!(left.device, DeviceId::LeftController));
        assert!((left.position[0] - 0.3).abs() < 1e-4);  // raw_x=3000 → X
        assert!((left.position[1] - 0.1).abs() < 1e-4);  // raw_y=1000 → Y
        assert!((left.position[2] - 0.2).abs() < 1e-4);  // raw_z=2000 → Z
        assert!((left.orientation[0] - 1.0).abs() < 1e-5); // identity
        assert!(left.orientation[1].abs() < 1e-5);
        assert!(left.orientation[2].abs() < 1e-5);
        assert!(left.orientation[3].abs() < 1e-5);
    }

    /// 0xa6 frame: newer firmware embeds right controller at buf[1..22].
    #[test]
    fn parse_a6_right_controller() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa6;

        buf[1] = 0x99; // hwversion (non-zero)
        buf[2] = 0xee; // fwversion
        // raw_y=5000 raw_z=-3000 raw_x=1000 → output pos=[0.1, 0.5, -0.3]
        buf[4] = 0x13; buf[5] = 0x88; // raw_y=5000
        buf[6] = 0xF4; buf[7] = 0x48; // raw_z=-3000
        buf[8] = 0x03; buf[9] = 0xE8; // raw_x=1000
        // orientation at BASE+11=buf[12]: w=16384, i=j=k=0 → identity
        buf[12] = 0x40; buf[13] = 0x00; // w=16384

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 1);

        let ctrl = &poses[0];
        assert!(matches!(ctrl.device, DeviceId::RightController));
        assert!((ctrl.position[0] - 0.1).abs() < 1e-4);
        assert!((ctrl.position[1] - 0.5).abs() < 1e-4);
        assert!((ctrl.position[2] - (-0.3)).abs() < 1e-4);
        assert!((ctrl.orientation[0] - 1.0).abs() < 1e-5);
        assert!(ctrl.orientation[1].abs() < 1e-5);
        assert!(ctrl.orientation[2].abs() < 1e-5);
        assert!(ctrl.orientation[3].abs() < 1e-5);
    }

    /// 0xa5 frame with HMD position bytes populated at base+24..29.
    #[test]
    fn parse_hmd_from_a5_frame() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa5;
        buf[1] = 2; // non-zero block header (controller present)
        buf[2] = 1;
        // Controller position
        buf[4] = 0x03; buf[5] = 0xE8; // raw_y=1000
        buf[6] = 0x07; buf[7] = 0xD0; // raw_z=2000
        buf[8] = 0x0B; buf[9] = 0xB8; // raw_x=3000
        // HMD position at base+24..29 (base=1 → buf[25..30])
        // X=1000, Y=2000, Z=-500 → [0.1, 0.2, -0.05]
        buf[25] = 0x03; buf[26] = 0xE8; // HMD X = 1000
        buf[27] = 0x07; buf[28] = 0xD0; // HMD Y = 2000
        buf[29] = 0xFE; buf[30] = 0x0C; // HMD Z = -500

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 2);

        let hmd = poses.iter().find(|p| matches!(p.device, DeviceId::Headset)).unwrap();
        assert!((hmd.position[0] - 0.1).abs() < 1e-4);
        assert!((hmd.position[1] - 0.2).abs() < 1e-4);
        assert!((hmd.position[2] - (-0.05)).abs() < 1e-4);
    }
}
