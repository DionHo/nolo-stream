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
        // Dual-controller frame: left at buf[1], right at buf[64 - 22] = buf[42].
        0xa5 => {
            let mut poses = Vec::with_capacity(2);
            if let Some(p) = parse_controller(buf, 1, DeviceId::LeftController) {
                poses.push(p);
            }
            if let Some(p) = parse_controller(buf, 64 - CTRL_LEN, DeviceId::RightController) {
                poses.push(p);
            }
            poses
        }
        // Headset + base-station frame: headset block at buf[0x15].
        0xa6 => {
            const BASE: usize = 0x15; // 21
            // orientation ends at BASE+16+7 = BASE+23; need at least BASE+24 bytes
            if buf.len() < BASE + 24 {
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
    // Skip all-zero blocks — device is likely off or not present.
    // (nolo-osvr checked for hwver==2 && fwver==1 here, but newer firmware uses
    // different version bytes; zero means "no device".)
    if buf[base] == 0 && buf[base + 1] == 0 {
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

/// 4× i16 big-endian raw order (w, i, j, k).
/// Reorder per nolo-osvr: output = [W=w, X=i, Y=k, Z=-j].
/// Normalized by actual magnitude to handle both reference firmware (scale≈16384)
/// and newer firmware variants (scale≈800–1050 or other).
#[inline]
fn parse_orientation(buf: &[u8], offset: usize) -> [f32; 4] {
    let w = read_i16_be(buf, offset) as f32;
    let i = read_i16_be(buf, offset + 2) as f32;
    let j = read_i16_be(buf, offset + 4) as f32;
    let k = read_i16_be(buf, offset + 6) as f32;
    // nolo-osvr reorder: W=w, X=i, Y=k, Z=-j
    let qw = w;
    let qx = i;
    let qy = k;
    let qz = -j;
    let mag = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
    if mag < 100.0 {
        return [1.0, 0.0, 0.0, 0.0]; // near-zero: identity fallback
    }
    [qw / mag, qx / mag, qy / mag, qz / mag]
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
        buf[1] = 2; // hwversion (any non-zero accepted)
        buf[2] = 1; // fwversion
        // position at buf[4..=9]: x=1000, y=2000, z=3000 (i16 big-endian)
        buf[4] = 0x03; buf[5] = 0xE8; // 1000
        buf[6] = 0x07; buf[7] = 0xD0; // 2000
        buf[8] = 0x0B; buf[9] = 0xB8; // 3000
        // orientation at buf[10..=17]: w=16384, i=j=k=0 → identity after reorder [W=w,X=i,Y=k,Z=-j]
        buf[10] = 0x40; buf[11] = 0x00; // w=16384

        // --- Right controller block at buf[42] = buf[64 - 22] ---
        buf[42] = 2; // hwversion
        buf[43] = 1; // fwversion
        // position at buf[45..=50]: x=-500, y=1500, z=2500
        buf[45] = 0xFE; buf[46] = 0x0C; // -500
        buf[47] = 0x05; buf[48] = 0xDC; // 1500
        buf[49] = 0x09; buf[50] = 0xC4; // 2500
        // orientation at buf[51..=58]: w=i=j=k=8192
        // reorder: W=w=8192, X=i=8192, Y=k=8192, Z=-j=-8192 → norm=16384 → [0.5, 0.5, 0.5, -0.5]
        buf[51] = 0x20; buf[52] = 0x00; // w=8192
        buf[53] = 0x20; buf[54] = 0x00; // i=8192
        buf[55] = 0x20; buf[56] = 0x00; // j=8192
        buf[57] = 0x20; buf[58] = 0x00; // k=8192

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 2);

        let left = &poses[0];
        assert!(matches!(left.device, DeviceId::LeftController));
        assert!((left.position[0] - 0.1).abs() < 1e-4);  // 1000 * 0.0001
        assert!((left.position[1] - 0.2).abs() < 1e-4);
        assert!((left.position[2] - 0.3).abs() < 1e-4);
        assert!((left.orientation[0] - 1.0).abs() < 1e-5); // identity
        assert!(left.orientation[1].abs() < 1e-5);
        assert!(left.orientation[2].abs() < 1e-5);
        assert!(left.orientation[3].abs() < 1e-5);

        let right = &poses[1];
        assert!(matches!(right.device, DeviceId::RightController));
        assert!((right.position[0] - (-0.05)).abs() < 1e-4); // -500 * 0.0001
        assert!((right.position[1] - 0.15).abs() < 1e-4);
        assert!((right.position[2] - 0.25).abs() < 1e-4);
        // w=i=j=k=8192: after reorder W=w=8192, X=i=8192, Y=k=8192, Z=-j=-8192; norm=16384
        assert!((right.orientation[0] - 0.5).abs() < 1e-5);
        assert!((right.orientation[1] - 0.5).abs() < 1e-5);
        assert!((right.orientation[2] - 0.5).abs() < 1e-5);
        assert!((right.orientation[3] - (-0.5)).abs() < 1e-5);
    }

    /// Build a synthetic pre-decrypted 0xa6 buffer and verify the headset parses.
    #[test]
    fn parse_headset_report() {
        let mut buf = [0u8; 64];
        buf[0] = 0xa6;

        // Headset block at buf[21] (0x15)
        buf[21] = 2; // hwversion
        buf[22] = 1; // fwversion
        // position at buf[24..=29]: x=5000, y=-3000, z=1000
        buf[24] = 0x13; buf[25] = 0x88; // 5000
        buf[26] = 0xF4; buf[27] = 0x48; // -3000
        buf[28] = 0x03; buf[29] = 0xE8; // 1000
        // homeposition at buf[30..=35] — leave zero
        // orientation at buf[37..=44]: w=16384, i=j=k=0 → identity
        buf[37] = 0x40; buf[38] = 0x00; // w=16384

        let poses = parse_decrypted(&buf);
        assert_eq!(poses.len(), 1);

        let headset = &poses[0];
        assert!(matches!(headset.device, DeviceId::Headset));
        assert!((headset.position[0] - 0.5).abs() < 1e-4);   // 5000 * 0.0001
        assert!((headset.position[1] - (-0.3)).abs() < 1e-4);
        assert!((headset.position[2] - 0.1).abs() < 1e-4);
        assert!((headset.orientation[0] - 1.0).abs() < 1e-5); // identity
        assert!(headset.orientation[1].abs() < 1e-5);
        assert!(headset.orientation[2].abs() < 1e-5);
        assert!(headset.orientation[3].abs() < 1e-5);
    }
}
