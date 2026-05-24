use serde::{Serialize, Deserialize};

fn touch_default() -> u8 { 255 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ControllerSide {
    Left,
    Right,
}

/// Decrypted data report from the Nolo controller.
///
/// Intermediate representation used before UKF filtering. Each HID packet is from
/// one controller side and also carries HMD position/orientation embedded in it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerReport {
    pub side: ControllerSide,
    pub position: [f32; 3],
    pub acceleration: [f32; 3],
    pub angular_velocity: [f32; 3],
    /// Bit 0x01=Pad, 0x02=Trigger, 0x04=Menu, 0x08=System, 0x10=Grip, 0x20=PadTouch.
    #[serde(default)]
    pub buttons: u8,
    /// Touch pad X. 255 = no touch, 127 = center, 0 = rightmost.
    #[serde(default = "touch_default")]
    pub touch_x: u8,
    /// Touch pad Y. 255 = no touch, 127 = center, 0 = topmost.
    #[serde(default = "touch_default")]
    pub touch_y: u8,
    #[serde(default)]
    pub battery: u8,
    /// HMD position [X Y Z] m.
    pub hmd_position: [f32; 3],
    /// HMD angular velocity [X Y Z] rad/s.
    pub hmd_angular_velocity: [f32; 3],
    /// HMD orientation quaternion (w, x, y, z).
    pub hmd_orientation: [f32; 4],
    /// Raw i16 sensor block: 32 values from block base+1, step 2.
    #[serde(default)]
    pub sensor_raw: [i16; 32],
    pub timestamp_ms: u64,
}

fn rd_i16(hid: &[u8], off: usize) -> f32 {
    i16::from_le_bytes([hid[off], hid[off + 1]]) as f32
}

impl ControllerReport {
    pub fn from_decrypted(hid: &[u8], timestamp_ms: u64) -> Option<ControllerReport> {
        if hid.len() < 64 {
            return None;
        }
        let side = match hid[0] {
            0xa5 | 0x10 => ControllerSide::Left,
            0xa6 | 0x11 => ControllerSide::Right,
            _ => return None,
        };
        let pscale = 0.0001_f32;
        let qscale = 1.0_f32 / 16384.0_f32;
        let gscale = crate::ahrs::DEFAULT_GYRO_SCALE;
        Some(ControllerReport {
            side,
            position: [
                rd_i16(hid, 1) * pscale,
                rd_i16(hid, 3) * pscale,
                rd_i16(hid, 5) * pscale,
            ],
            acceleration: [
                rd_i16(hid, 7)  * pscale,
                rd_i16(hid, 9)  * pscale,
                rd_i16(hid, 11) * pscale,
            ],
            angular_velocity: [
                rd_i16(hid, 13) * gscale,
                rd_i16(hid, 15) * gscale,
                rd_i16(hid, 17) * gscale,
            ],
            buttons:  hid[19],
            touch_x:  255 - hid[20],
            touch_y:  255 - hid[21],
            battery:  hid[22],
            hmd_position: [
                rd_i16(hid, 25) * pscale,
                rd_i16(hid, 27) * pscale,
                rd_i16(hid, 29) * pscale,
            ],
            hmd_angular_velocity: [
                rd_i16(hid, 37) * gscale,
                rd_i16(hid, 39) * gscale,
                rd_i16(hid, 41) * gscale,
            ],
            hmd_orientation: [
                rd_i16(hid, 49) * qscale,
                rd_i16(hid, 51) * qscale,
                rd_i16(hid, 55) * qscale,
                rd_i16(hid, 53) * qscale,
            ],
            sensor_raw: {
                let mut arr = [0i16; 32];
                for i in 0..31 {  // bytes 1-62 → 31 valid i16 pairs; byte 63 is unpaired
                    arr[i] = i16::from_le_bytes([hid[1 + i * 2], hid[2 + i * 2]]);
                }
                arr
            },
            timestamp_ms,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ahrs::DEFAULT_GYRO_SCALE;

    fn make_hid(report_type: u8) -> [u8; 64] {
        let mut hid = [0u8; 64];
        hid[0] = report_type;
        hid
    }

    /// Helper: write a little-endian i16 into a HID buffer at offset.
    fn write_i16(hid: &mut [u8; 64], off: usize, val: i16) {
        let bytes = val.to_le_bytes();
        hid[off]     = bytes[0];
        hid[off + 1] = bytes[1];
    }

    #[test]
    fn unknown_report_type_returns_none() {
        let hid = make_hid(0x00);
        assert!(ControllerReport::from_decrypted(&hid, 0).is_none());
    }

    #[test]
    fn too_short_buffer_returns_none() {
        let hid = [0xa5u8; 63];
        assert!(ControllerReport::from_decrypted(&hid, 0).is_none());
    }

    #[test]
    fn left_controller_report_type() {
        // Both 0xa5 and 0x10 map to Left
        let r1 = ControllerReport::from_decrypted(&make_hid(0xa5), 0).unwrap();
        let r2 = ControllerReport::from_decrypted(&make_hid(0x10), 0).unwrap();
        assert!(matches!(r1.side, ControllerSide::Left));
        assert!(matches!(r2.side, ControllerSide::Left));
    }

    #[test]
    fn right_controller_report_type() {
        let r1 = ControllerReport::from_decrypted(&make_hid(0xa6), 0).unwrap();
        let r2 = ControllerReport::from_decrypted(&make_hid(0x11), 0).unwrap();
        assert!(matches!(r1.side, ControllerSide::Right));
        assert!(matches!(r2.side, ControllerSide::Right));
    }

    #[test]
    fn position_parsed_correctly() {
        let mut hid = make_hid(0xa5);
        // Write 1000 (= 0.1 m after * 0.0001) at bytes 1-2
        write_i16(&mut hid, 1, 1000);
        write_i16(&mut hid, 3, -500);
        write_i16(&mut hid, 5, 0);
        let r = ControllerReport::from_decrypted(&hid, 0).unwrap();
        let eps = 1e-5;
        assert!((r.position[0] - 0.1).abs() < eps);
        assert!((r.position[1] - (-0.05)).abs() < eps);
        assert!(r.position[2].abs() < eps);
    }

    #[test]
    fn battery_byte_passed_through() {
        let mut hid = make_hid(0xa5);
        hid[22] = 75;
        let r = ControllerReport::from_decrypted(&hid, 0).unwrap();
        assert_eq!(r.battery, 75);
    }

    #[test]
    fn battery_255_passed_through() {
        let mut hid = make_hid(0xa5);
        hid[22] = 255;
        let r = ControllerReport::from_decrypted(&hid, 0).unwrap();
        assert_eq!(r.battery, 255, "255 should be preserved as-is (means no data)");
    }

    #[test]
    fn hmd_angular_velocity_parsed_from_bytes_37_41() {
        let mut hid = make_hid(0xa5);
        // 1000 * DEFAULT_GYRO_SCALE ≈ 1.065 rad/s
        write_i16(&mut hid, 37, 1000);
        write_i16(&mut hid, 39, -500);
        write_i16(&mut hid, 41, 200);
        let r = ControllerReport::from_decrypted(&hid, 0).unwrap();
        let eps = 1e-5;
        assert!((r.hmd_angular_velocity[0] - 1000.0 * DEFAULT_GYRO_SCALE).abs() < eps);
        assert!((r.hmd_angular_velocity[1] - (-500.0 * DEFAULT_GYRO_SCALE)).abs() < eps);
        assert!((r.hmd_angular_velocity[2] - 200.0 * DEFAULT_GYRO_SCALE).abs() < eps);
    }

    #[test]
    fn timestamp_propagated() {
        let hid = make_hid(0xa5);
        let r = ControllerReport::from_decrypted(&hid, 42_000).unwrap();
        assert_eq!(r.timestamp_ms, 42_000);
    }

    #[test]
    fn touch_inversion() {
        // Raw hid[20]=0 → touch_x = 255-0 = 255, hid[21]=100 → touch_y = 255-100 = 155
        let mut hid = make_hid(0xa5);
        hid[20] = 0;
        hid[21] = 100;
        let r = ControllerReport::from_decrypted(&hid, 0).unwrap();
        assert_eq!(r.touch_x, 255);
        assert_eq!(r.touch_y, 155);
    }
}
