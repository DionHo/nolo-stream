use serde::{Serialize, Deserialize};

use crate::ControllerState;

fn touch_default() -> u8 { 255 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ControllerSide {
    Left,
    Right,
}

/// Decrypted data report from the Nolo controller.
/// 
/// Intermediate representation used for transport and logging. Will be converted
/// to ControllerState with AHRS orientation filtering applied before publishing to clients.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerReport {
    pub side: ControllerSide,
    pub position: [f32; 3],
    pub acceleration: [f32; 3],
    pub angular_velocity: [f32; 3],
    /// Button bitmask (controllers only; 0 for headset).
    /// Bit 0x01=Pad, 0x02=Trigger, 0x04=Menu, 0x08=System, 0x10=Grip, 0x20=PadTouch.
    #[serde(default)]
    pub buttons: u8,
    /// Touch pad X. 255 = no touch, 127 = center, 0 = rightmost (confirmed: base+19).
    #[serde(default = "touch_default")]
    pub touch_x: u8,
    /// Touch pad Y. 255 = no touch, 127 = center, 0 = topmost (confirmed: base+20).
    #[serde(default = "touch_default")]
    pub touch_y: u8,
    /// Battery level 0–255 (tentative: base+21, same offset as nolo-osvr reference).
    #[serde(default)]
    pub battery: u8,
    /// HMD position [X Y Z] m. Filled by client-api path; zero on HID path.
    pub hmd_position: [f32; 3],
    /// HMD angular velocity [X Y Z] rad/s. Filled by client-api path; zero on HID path.
    pub hmd_angular_velocity: [f32; 3],
    /// HMD orientation (quaternion, w,x,y,z). Filled by client-api path; zero on HID path.
    pub hmd_orientation: [f32; 4],
    /// Raw i16 sensor block: 32 values from block base+1, step 2.
    #[serde(default)]
    pub sensor_raw: [i16; 32],
    /// Timestamp in milliseconds.
    pub timestamp_ms: u64,
}

impl ControllerReport {
    // IMU data (accel+gyro) occupies base+9..18; exact word split is 4 or 5 words (TBD).
    // Confirmed field positions (newer firmware):
    //   1..6  [0..2] :  position (X,Y,Z)
    //   7..18 [3..8] :  IMU channels (accel X/Y/Z + gyro X/Y/Z)
    //   19    [9] :     buttons (0: touchpad-pressed, 1: trigger, 2: menu, 3: system, 4: grip, 5: finger-on-touchpad)
    //   20    [9] :     touch X (confirmed: 255=no touch, 127=center, increases swiping left)
    //   21    [9] :     touch Y (confirmed: 255=no touch, 127=center, increases swiping down)
    //   22    [10] :    battery (tentative, same offset as nolo-osvr)
    //   23    [10] :    ???
    //   24    [11] :    ??? (rolling 1-byte counter, previously mistaken for 32-bit LE tick counter)
    //   25    [11] :    ???
    //   26..27[12] : HMD position X (i16 BE, ×0.0001 → m) — confirmed via movement test
    //   28..29[13] : HMD position Y
    //   30..31[14] : HMD position Z
    //   ...
    //     [18..20] : HMD IMU gyro (X,Y,Z)
    //     [24..27] : HMD IMU ORIENTATION quaternion (w,x,y,z)
    pub fn from_decrypted(hid: &[u8], timestamp_ms: u64) -> Option<ControllerReport> {
        if hid.len() < 64 {
            return None;
        }
        // On Windows, hidapi gives report ID 0x10/0x11 at buf[0] instead of the raw
        // packet type 0xa5/0xa6 found on Linux hidraw. The encrypted region and all
        // block offsets within the decrypted payload are identical on both platforms.
        let side = match hid[0] {
            0xa5 | 0x10 => ControllerSide::Left,
            0xa6 | 0x11 => ControllerSide::Right,
            _ => return None,
        };
        let pscale = 0.0001;
        let qscale = 1.0 / 16384.0;
        Some(ControllerReport {
            side,
            position: [
                f32::from_le_bytes(hid[1..3].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[3..5].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[5..7].try_into().unwrap()) * pscale,
            ],
            acceleration: [
                f32::from_le_bytes(hid[7..9].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[9..11].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[11..13].try_into().unwrap()) * pscale,
            ],
            angular_velocity: [
                f32::from_le_bytes(hid[13..15].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[15..17].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[17..19].try_into().unwrap()) * pscale,
            ],
            buttons: hid[19],
            touch_x: 255-hid[20],
            touch_y: 255-hid[21],
            battery: hid[22],
            hmd_position: [
                f32::from_le_bytes(hid[26..28].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[28..30].try_into().unwrap()) * pscale,
                f32::from_le_bytes(hid[30..32].try_into().unwrap()) * pscale,
            ],
            hmd_angular_velocity: [0.0; 3],
            hmd_orientation: [
                f32::from_le_bytes(hid[49..51].try_into().unwrap()) * qscale,
                f32::from_le_bytes(hid[51..53].try_into().unwrap()) * qscale,
                f32::from_le_bytes(hid[53..55].try_into().unwrap()) * qscale,
                f32::from_le_bytes(hid[55..57].try_into().unwrap()) * qscale,
            ],
            sensor_raw: {
                let mut arr = [0i16; 32];
                for i in 0..32 {
                    arr[i] = i16::from_le_bytes(hid[(1 + i*2)..(3 + i*2)].try_into().unwrap());
                }
                arr
            },
            timestamp_ms,
        })
    }

    pub fn to_states(&self, controller_pose_filter : Box<dyn ControllerStateFilterTrait>) -> Vec<ControllerState> {
        vec![controller_pose_filter.filter(self),ControllerState{
            device: crate::DeviceId::Headset,
            position: self.hmd_position,
            orientation: self.hmd_orientation,
            timestamp_ms: self.timestamp_ms,
            touch_x: 255,
            touch_y: 255,
            battery: 0,
            buttons: 0,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            state: 0,
        }]
    }
}


pub trait ControllerStateFilterTrait {
    fn filter(&self, state: &ControllerReport) -> ControllerState;
    
}
