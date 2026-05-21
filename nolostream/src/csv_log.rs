use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::controller_state::{DeviceId, ControllerState};

pub const CSV_HEADER: &str = "timestamp_ms,source_api,device,\
pos_x,pos_y,pos_z,\
rot_w,rot_x,rot_y,rot_z,\
buttons,touch_x,touch_y,battery,\
vel_x,vel_y,vel_z,\
ang_vel_x,ang_vel_y,ang_vel_z,\
state,\
hid_b00,hid_b01,hid_b02,hid_b03,hid_b04,hid_b05,hid_b06,hid_b07,\
hid_b08,hid_b09,hid_b10,hid_b11,hid_b12,hid_b13,hid_b14,hid_b15,\
hid_b16,hid_b17,hid_b18,hid_b19,hid_b20,hid_b21,hid_b22,hid_b23,\
hid_b24,hid_b25,hid_b26,hid_b27,hid_b28,hid_b29,hid_b30,hid_b31,\
hid_b32,hid_b33,hid_b34,hid_b35,hid_b36,hid_b37,hid_b38,hid_b39,\
hid_b40,hid_b41,hid_b42,hid_b43,hid_b44,hid_b45,hid_b46,hid_b47,\
hid_b48,hid_b49,hid_b50,hid_b51,hid_b52,hid_b53,hid_b54,hid_b55,\
hid_b56,hid_b57,hid_b58,hid_b59,hid_b60,hid_b61,hid_b62,hid_b63";

pub struct CsvLogger {
    writer: BufWriter<File>,
}

impl CsvLogger {
    pub fn create(path: &Path) -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(writer, "{CSV_HEADER}")?;
        Ok(CsvLogger { writer })
    }

    /// Write one CSV row. `hid_bytes` is the decrypted 64-byte HID buffer (None for client_api rows).
    pub fn write_pose(
        &mut self,
        source: &str,
        pose: &ControllerState,
        hid_bytes: Option<&[u8; 64]>,
    ) -> std::io::Result<()> {
        let device = match pose.device {
            DeviceId::LeftController  => "left",
            DeviceId::RightController => "right",
            DeviceId::Headset         => "hmd",
        };
        write!(
            self.writer,
            "{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}",
            source, device,
            pose.position[0], pose.position[1], pose.position[2],
            pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3],
            pose.buttons, pose.touch_x, pose.touch_y, pose.battery,
            pose.velocity[0], pose.velocity[1], pose.velocity[2],
            pose.angular_velocity[0], pose.angular_velocity[1], pose.angular_velocity[2],
            pose.state,
        )?;
        match hid_bytes {
            Some(bytes) => {
                for b in bytes.iter() {
                    write!(self.writer, ",{b}")?;
                }
            }
            None => {
                // 64 empty columns
                for _ in 0..64 {
                    write!(self.writer, ",")?;
                }
            }
        }
        writeln!(self.writer)?;
        self.writer.flush()?;
        Ok(())
    }
}
