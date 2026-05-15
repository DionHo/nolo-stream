use std::collections::HashMap;
use crate::ahrs::{ComplementaryFilter, DEFAULT_GYRO_SCALE};
use crate::hid::{NoloDevice, NoloError};
use crate::transport::{Transport, TransportError};
use crate::pose::DeviceId;
use crate::Pose;

pub struct NoloStream {
    device: NoloDevice,
    transports: Vec<Box<dyn Transport>>,
    filters: HashMap<DeviceId, ComplementaryFilter>,
    gyro_scale: f32,
}

impl NoloStream {
    pub fn new() -> Result<Self, NoloError> {
        Ok(NoloStream {
            device: NoloDevice::open()?,
            transports: Vec::new(),
            filters: HashMap::new(),
            gyro_scale: DEFAULT_GYRO_SCALE,
        })
    }

    pub fn set_gyro_scale(&mut self, scale: f32) {
        self.gyro_scale = scale;
        self.filters.clear(); // reset filters so they pick up the new scale
    }

    pub fn add_transport(&mut self, t: Box<dyn Transport>) {
        self.transports.push(t);
    }

    /// Read one HID report, apply AHRS orientation filter, and dispatch to all transports.
    pub fn poll_once(&mut self) -> Result<Vec<Pose>, NoloError> {
        let mut poses = self.device.poll()?;
        let gyro_scale = self.gyro_scale;
        for pose in &mut poses {
            let filter = self.filters
                .entry(pose.device.clone())
                .or_insert_with(|| ComplementaryFilter::new(gyro_scale));
            let accel = [pose.sensor_raw[3], pose.sensor_raw[4], pose.sensor_raw[5]];
            let gyro  = [pose.sensor_raw[6], pose.sensor_raw[7], pose.sensor_raw[8]];
            pose.orientation = filter.update(accel, gyro);
        }
        self.transports.retain_mut(|t| match t.send(&poses) {
            Ok(()) => true,
            Err(TransportError::Disconnected) => false,
            Err(TransportError::Io(msg)) => {
                eprintln!("transport io error: {msg}");
                true
            }
        });
        Ok(poses)
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
