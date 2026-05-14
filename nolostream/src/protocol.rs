use crate::btea;
use crate::pose::{DeviceId, Pose};

pub const NOLO_VID: u16 = 0x0483;
pub const NOLO_PID: u16 = 0x5750;

const NOLO_KEY: [u32; 4] = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51];

/// Number of u32 words in the encrypted region: bytes 1..=60, little-endian.
/// Derived from C: (64 - 4) / 4 = 15  (byte 0 = report type; last 3 bytes unencrypted)
const CRYPTWORDS: usize = 15;

/// Decrypt the encrypted region of a 64-byte raw HID buffer, then parse it.
/// Returns an empty Vec on an unknown or invalid report.
pub fn parse_report(buf: &[u8]) -> Vec<Pose> {
    if buf.len() < 64 {
        return vec![];
    }
    let mut work = [0u8; 64];
    work.copy_from_slice(&buf[..64]);

    // Bytes 1..=60 are 15 u32s (little-endian) that need BTEA decryption.
    let mut words = [0u32; CRYPTWORDS];
    for i in 0..CRYPTWORDS {
        let b = 1 + i * 4;
        words[i] = u32::from_le_bytes([work[b], work[b + 1], work[b + 2], work[b + 3]]);
    }
    btea::btea_decrypt(&mut words, 1, &NOLO_KEY);
    for i in 0..CRYPTWORDS {
        let b = 1 + i * 4;
        work[b..b + 4].copy_from_slice(&words[i].to_le_bytes());
    }

    parse_decrypted(&work)
}

/// Parse a fully-decrypted 64-byte buffer into Pose values.
fn parse_decrypted(buf: &[u8]) -> Vec<Pose> {
    if buf.len() < 64 {
        return vec![];
    }
    match buf[0] {
        // Dual-controller frame: left at buf[1], right at buf[32]
        0xa5 => {
            let mut poses = Vec::with_capacity(2);
            if let Some(p) = parse_controller(buf, 1, DeviceId::LeftController) {
                poses.push(p);
            }
            if let Some(p) = parse_controller(buf, 32, DeviceId::RightController) {
                poses.push(p);
            }
            poses
        }
        // Headset + base-station frame: headset block at buf[0x15]
        0xa6 => {
            const BASE: usize = 0x15; // 21
            // orientation ends at BASE+16+7 = BASE+23; need at least BASE+24 bytes
            if buf.len() < BASE + 24 {
                return vec![];
            }
            if buf[BASE] != 2 {
                // unexpected hwversion
                return vec![];
            }
            let position = parse_position(buf, BASE + 3);
            // homeposition occupies BASE+9..BASE+15 (6 bytes) — skip it
            let orientation = parse_orientation(buf, BASE + 16);
            vec![Pose {
                device: DeviceId::Headset,
                position,
                orientation,
                timestamp_ms: 0,
            }]
        }
        _ => vec![],
    }
}

/// Parse one controller block starting at `base` in the decrypted buffer.
fn parse_controller(buf: &[u8], base: usize, device: DeviceId) -> Option<Pose> {
    // orientation ends at base+9+7 = base+16; need at least base+17 bytes
    if buf.len() < base + 17 {
        return None;
    }
    if buf[base] != 2 || buf[base + 1] != 1 {
        // unexpected hwversion / fwversion
        return None;
    }
    let position = parse_position(buf, base + 3);
    let orientation = parse_orientation(buf, base + 9);
    Some(Pose {
        device,
        position,
        orientation,
        timestamp_ms: 0,
    })
}

#[inline]
fn read_i16_be(buf: &[u8], offset: usize) -> i16 {
    i16::from_be_bytes([buf[offset], buf[offset + 1]])
}

/// 3× i16 big-endian, scaled by 0.0001 to give metres.
#[inline]
fn parse_position(buf: &[u8], offset: usize) -> [f32; 3] {
    [
        read_i16_be(buf, offset) as f32 * 0.0001,
        read_i16_be(buf, offset + 2) as f32 * 0.0001,
        read_i16_be(buf, offset + 4) as f32 * 0.0001,
    ]
}

/// 4× i16 big-endian, divided by 16384.0 to give a unit quaternion [w, i, j, k].
#[inline]
fn parse_orientation(buf: &[u8], offset: usize) -> [f32; 4] {
    [
        read_i16_be(buf, offset) as f32 / 16384.0,
        read_i16_be(buf, offset + 2) as f32 / 16384.0,
        read_i16_be(buf, offset + 4) as f32 / 16384.0,
        read_i16_be(buf, offset + 6) as f32 / 16384.0,
    ]
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
    #[test]
    fn parse_controller_report() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa5;

        // --- Left controller block at buf[1] ---
        buf[1] = 2; // hwversion
        buf[2] = 1; // fwversion
        // position bytes at buf[4..=9]: x=1000, y=2000, z=3000 (i16 big-endian)
        //   1000 = 0x03E8, 2000 = 0x07D0, 3000 = 0x0BB8
        buf[4] = 0x03; buf[5] = 0xE8;
        buf[6] = 0x07; buf[7] = 0xD0;
        buf[8] = 0x0B; buf[9] = 0xB8;
        // orientation bytes at buf[10..=17]: w=16384, i=j=k=0 (identity)
        buf[10] = 0x40; buf[11] = 0x00; // 16384
        // remaining orientation bytes already zero

        // --- Right controller block at buf[32] ---
        buf[32] = 2; // hwversion
        buf[33] = 1; // fwversion
        // position: x=-500=0xFE0C, y=1500=0x05DC, z=2500=0x09C4
        buf[35] = 0xFE; buf[36] = 0x0C;
        buf[37] = 0x05; buf[38] = 0xDC;
        buf[39] = 0x09; buf[40] = 0xC4;
        // orientation: w=i=j=k=8192=0x2000 → each component = 8192/16384 = 0.5
        buf[41] = 0x20; buf[42] = 0x00;
        buf[43] = 0x20; buf[44] = 0x00;
        buf[45] = 0x20; buf[46] = 0x00;
        buf[47] = 0x20; buf[48] = 0x00;

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 2);

        let left = &poses[0];
        assert!(matches!(left.device, DeviceId::LeftController));
        // 1000 * 0.0001 = 0.1, 2000 * 0.0001 = 0.2, 3000 * 0.0001 = 0.3
        assert!((left.position[0] - 0.1).abs() < 1e-4);
        assert!((left.position[1] - 0.2).abs() < 1e-4);
        assert!((left.position[2] - 0.3).abs() < 1e-4);
        assert!((left.orientation[0] - 1.0).abs() < 1e-5);
        assert!(left.orientation[1].abs() < 1e-5);
        assert!(left.orientation[2].abs() < 1e-5);
        assert!(left.orientation[3].abs() < 1e-5);

        let right = &poses[1];
        assert!(matches!(right.device, DeviceId::RightController));
        // -500 * 0.0001 = -0.05, 1500 * 0.0001 = 0.15, 2500 * 0.0001 = 0.25
        assert!((right.position[0] - (-0.05)).abs() < 1e-4);
        assert!((right.position[1] - 0.15).abs() < 1e-4);
        assert!((right.position[2] - 0.25).abs() < 1e-4);
        // 8192 / 16384 = 0.5 for each component
        assert!((right.orientation[0] - 0.5).abs() < 1e-5);
        assert!((right.orientation[1] - 0.5).abs() < 1e-5);
        assert!((right.orientation[2] - 0.5).abs() < 1e-5);
        assert!((right.orientation[3] - 0.5).abs() < 1e-5);
    }

    /// Build a synthetic pre-decrypted 0xa6 buffer and verify the headset parses.
    #[test]
    fn parse_headset_report() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa6;

        // Headset block at buf[21] (0x15)
        buf[21] = 2; // hwversion
        buf[22] = 1; // fwversion
        // position at buf[24..=29]: x=5000=0x1388, y=-3000=0xF448, z=1000=0x03E8
        buf[24] = 0x13; buf[25] = 0x88;
        buf[26] = 0xF4; buf[27] = 0x48;
        buf[28] = 0x03; buf[29] = 0xE8;
        // homeposition at buf[30..=35] — skip (already zero)
        // orientation at buf[37..=44]: w=16384=0x4000, i=j=k=0 (identity)
        buf[37] = 0x40; buf[38] = 0x00;

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 1);

        let headset = &poses[0];
        assert!(matches!(headset.device, DeviceId::Headset));
        // 5000 * 0.0001 = 0.5, -3000 * 0.0001 = -0.3, 1000 * 0.0001 = 0.1
        assert!((headset.position[0] - 0.5).abs() < 1e-4);
        assert!((headset.position[1] - (-0.3)).abs() < 1e-4);
        assert!((headset.position[2] - 0.1).abs() < 1e-4);
        assert!((headset.orientation[0] - 1.0).abs() < 1e-5);
        assert!(headset.orientation[1].abs() < 1e-5);
        assert!(headset.orientation[2].abs() < 1e-5);
        assert!(headset.orientation[3].abs() < 1e-5);
    }
}
