use std::collections::HashMap;
use crate::ahrs::{ComplementaryFilter, DEFAULT_GYRO_SCALE};
use crate::csv_log::CsvLogger;
use crate::hid::{NoloDevice, NoloError};
use crate::transport::{Transport, TransportError};
use crate::pose::DeviceId;
use crate::teleop::{TeleopFrame, TeleopState};
use crate::Pose;

pub struct NoloStream {
    device: NoloDevice,
    transports: Vec<Box<dyn Transport>>,
    filters: HashMap<DeviceId, ComplementaryFilter>,
    gyro_scale: f32,
    teleop: TeleopState,
    csv_logger: Option<CsvLogger>,
}

impl NoloStream {
    pub fn new() -> Result<Self, NoloError> {
        Ok(NoloStream {
            device: NoloDevice::open()?,
            transports: Vec::new(),
            filters: HashMap::new(),
            gyro_scale: DEFAULT_GYRO_SCALE,
            teleop: TeleopState::new(),
            csv_logger: None,
        })
    }

    pub fn set_gyro_scale(&mut self, scale: f32) {
        self.gyro_scale = scale;
        self.filters.clear(); // reset filters so they pick up the new scale
    }

    pub fn set_csv_log(&mut self, logger: CsvLogger) {
        self.csv_logger = Some(logger);
    }

    pub fn add_transport(&mut self, t: Box<dyn Transport>) {
        self.transports.push(t);
    }

    /// Read one HID report, apply AHRS orientation filter, dispatch to all transports.
    /// Returns the parsed poses and any teleop delta frames produced this cycle.
    pub fn poll_once(&mut self) -> Result<(Vec<Pose>, Vec<TeleopFrame>), NoloError> {
        let (mut poses, raw_buf) = self.device.poll_with_raw()?;
        let gyro_scale = self.gyro_scale;
        for pose in &mut poses {
            let filter = self.filters
                .entry(pose.device.clone())
                .or_insert_with(|| ComplementaryFilter::new(gyro_scale));
            let accel = [pose.sensor_raw[3], pose.sensor_raw[4], pose.sensor_raw[5]];
            let gyro  = [pose.sensor_raw[6], pose.sensor_raw[7], pose.sensor_raw[8]];
            pose.orientation = filter.update(accel, gyro);
        }

        if let Some(ref mut logger) = self.csv_logger {
            for pose in &poses {
                if let Err(e) = logger.write_pose("hid", pose, raw_buf.as_ref()) {
                    eprintln!("csv-log write error: {e}");
                }
            }
        }

        let teleop_frames = self.teleop.update(&poses);

        self.transports.retain_mut(|t| match t.send(&poses) {
            Ok(()) => true,
            Err(TransportError::Disconnected) => false,
            Err(TransportError::Io(msg)) => {
                eprintln!("transport io error: {msg}");
                true
            }
        });

        if !teleop_frames.is_empty() {
            for t in &mut self.transports {
                if let Err(e) = t.send_teleop(&teleop_frames) {
                    eprintln!("teleop dispatch error: {e}");
                }
            }
        }

        Ok((poses, teleop_frames))
    }
}

#[cfg(test)]
mod tests {
    // Tests that call NoloStream::new() require real HID hardware and are marked #[ignore].
    // Transport dispatch logic is tested via MockTransport in transport.rs.

    use super::*;

    #[test]
    #[ignore = "requires NoloVR HID hardware"]
    fn new_opens_device() {
        NoloStream::new().expect("should open device");
    }

    #[test]
    #[ignore = "requires NoloVR HID hardware"]
    fn poll_once_returns_poses() {
        let mut ns = NoloStream::new().unwrap();
        let poses = ns.poll_once().unwrap();
        // May be empty on timeout, but should not error
        let _ = poses;
    }
}
