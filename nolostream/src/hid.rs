use hidapi::HidApi;

use crate::pose::Pose;
use crate::protocol::{parse_report, NOLO_PID, NOLO_VID};

#[derive(Debug)]
pub enum NoloError {
    DeviceNotFound,
    HidError(String),
    InvalidReport,
}

impl std::fmt::Display for NoloError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NoloError::DeviceNotFound => write!(f, "NoloVR device not found (VID={NOLO_VID:#06x} PID={NOLO_PID:#06x})"),
            NoloError::HidError(s) => write!(f, "HID error: {s}"),
            NoloError::InvalidReport => write!(f, "Invalid HID report"),
        }
    }
}

impl std::error::Error for NoloError {}

pub struct NoloDevice {
    device: hidapi::HidDevice,
}

impl NoloDevice {
    /// Open the first NoloVR device found by VID/PID.
    pub fn open() -> Result<Self, NoloError> {
        let api = HidApi::new().map_err(|e| NoloError::HidError(e.to_string()))?;
        let device = api
            .open(NOLO_VID, NOLO_PID)
            .map_err(|_| NoloError::DeviceNotFound)?;
        Ok(NoloDevice { device })
    }

    /// Read one raw HID report (up to 64 bytes), 100 ms timeout.
    /// Returns an empty Vec on timeout (0 bytes read).
    pub fn read_report(&self) -> Result<Vec<u8>, NoloError> {
        let mut buf = vec![0u8; 64];
        let n = self
            .device
            .read_timeout(&mut buf, 100)
            .map_err(|e| NoloError::HidError(e.to_string()))?;
        buf.truncate(n);
        Ok(buf)
    }

    /// Read one HID report and parse it into Pose values.
    pub fn poll(&self) -> Result<Vec<Pose>, NoloError> {
        let buf = self.read_report()?;
        Ok(parse_report(&buf))
    }
}
