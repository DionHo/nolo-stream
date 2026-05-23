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
                rd_i16(hid, 26) * pscale,
                rd_i16(hid, 28) * pscale,
                rd_i16(hid, 30) * pscale,
            ],
            hmd_angular_velocity: [0.0; 3],
            hmd_orientation: [
                rd_i16(hid, 49) * qscale,
                rd_i16(hid, 51) * qscale,
                rd_i16(hid, 53) * qscale,
                rd_i16(hid, 55) * qscale,
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
