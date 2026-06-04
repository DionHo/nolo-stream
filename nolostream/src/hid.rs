use hidapi::HidApi;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::controller_report::ControllerReport;
use crate::protocol::{NOLO_PID, NOLO_VID};

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

        // The USB device exposes multiple HID collections; only one of them sends
        // pose reports. Try each interface briefly (500 ms) and use the first that
        // returns data.  If none respond within the probe window but at least one
        // could be opened, keep that handle (the controllers may be off at startup).
        let paths: Vec<String> = api
            .device_list()
            .filter(|d| d.vendor_id() == NOLO_VID && d.product_id() == NOLO_PID)
            .map(|d| d.path().to_string_lossy().into_owned())
            .collect();

        eprintln!("Found {} NoloVR HID interfaces:", paths.len());
        for path in &paths {
            eprintln!("  {path}");
        }

        if paths.is_empty() {
            return Err(NoloError::DeviceNotFound);
        }

        // Find the first interface that delivers HID reports within 500 ms.
        // Keep track of the last successfully-opened device in case no interface
        // responds to the probe (common on Linux where the first read may be delayed).
        let mut fallback_dev: Option<(hidapi::HidDevice, String)> = None;
        for path in &paths {
            let cpath = std::ffi::CString::new(path.as_str())
                .map_err(|e| NoloError::HidError(e.to_string()))?;
            match api.open_path(cpath.as_ref()) {
                Ok(dev) => {
                    let mut buf = [0u8; 64];
                    match dev.read_timeout(&mut buf, 500) {
                        Ok(n) if n > 0 => {
                            eprintln!("NoloVR device opened (VID={NOLO_VID:#06x} PID={NOLO_PID:#06x}) path={path}");
                            return Ok(NoloDevice { device: dev });
                        }
                        Ok(_) => {
                            eprintln!("  {path}: opened OK, no data within 500 ms (probe timeout)");
                            // Keep the handle as a fallback — don't drop it and re-open.
                            fallback_dev = Some((dev, path.clone()));
                        }
                        Err(e) => {
                            eprintln!("  {path}: opened OK, read error: {e}");
                            // Still usable as fallback — read errors during probe may be transient.
                            fallback_dev = Some((dev, path.clone()));
                        }
                    }
                }
                Err(e) => {
                    eprintln!("  {path}: open failed: {e}");
                    eprintln!("    hint: on Linux, ensure you have permission to access the HID device.");
                    eprintln!("    See: https://github.com/DionHo/nolo-stream#linux-setup");
                }
            }
        }

        // Use the last successfully-opened device handle from the probe loop.
        if let Some((dev, path)) = fallback_dev {
            eprintln!("NoloVR device opened (VID={NOLO_VID:#06x} PID={NOLO_PID:#06x}) path={path} [no-data fallback]");
            return Ok(NoloDevice { device: dev });
        }

        // All open_path attempts failed; try the generic VID/PID open as last resort.
        match api.open(NOLO_VID, NOLO_PID) {
            Ok(device) => {
                eprintln!("NoloVR device opened (VID={NOLO_VID:#06x} PID={NOLO_PID:#06x}) [vid/pid fallback]");
                Ok(NoloDevice { device })
            }
            Err(e) => {
                eprintln!("Failed to open NoloVR device: {e}");
                eprintln!("hint: on Linux, ensure you have permission to access the HID device.");
                eprintln!("See: https://github.com/DionHo/nolo-stream#linux-setup");
                Err(NoloError::DeviceNotFound)
            }
        }
    }

    /// Open a NoloVR device by its specific HID path (for multi-interface enumeration).
    pub fn open_path(path: &str) -> Result<Self, NoloError> {
        let api = HidApi::new().map_err(|e| NoloError::HidError(e.to_string()))?;
        let device = api
            .open_path(std::ffi::CString::new(path).map_err(|e| NoloError::HidError(e.to_string()))?.as_ref())
            .map_err(|e| NoloError::HidError(e.to_string()))?;
        Ok(NoloDevice { device })
    }

    /// List all HID device paths for the NoloVR VID/PID.
    pub fn enumerate_paths() -> Result<Vec<String>, NoloError> {
        let api = HidApi::new().map_err(|e| NoloError::HidError(e.to_string()))?;
        let paths = api
            .device_list()
            .filter(|d| d.vendor_id() == NOLO_VID && d.product_id() == NOLO_PID)
            .map(|d| {
                let path = d.path().to_string_lossy().into_owned();
                let usage = d.usage();
                let usage_page = d.usage_page();
                let iface = d.interface_number();
                format!("{path}  (iface={iface} usage_page={usage_page:#06x} usage={usage:#06x})")
            })
            .collect();
        Ok(paths)
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

    /// Read one raw HID report and log the raw bytes to stderr (for diagnostics).
    pub fn read_report_raw(&self) -> Result<Vec<u8>, NoloError> {
        let buf = self.read_report()?;
        if !buf.is_empty() {
            let hex: Vec<String> = buf.iter().map(|b| format!("{b:02x}")).collect();
            eprintln!("[raw ] n={} bytes: {}", buf.len(), hex.join(" "));
        }
        Ok(buf)
    }

    /// Read one HID report and parse it into a ControllerReport.
    pub fn poll(&self) -> Result<Option<ControllerReport>, NoloError> {
        let buf = self.read_report()?;
        if buf.is_empty() {
            return Ok(None);
        }
        let timestamp_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        Ok(crate::protocol::generate_report(&buf, timestamp_ms))
    }

    /// Read one HID report, decrypt it, and return both the report and the decrypted 64-byte buffer.
    pub fn poll_with_raw(&self) -> Result<(Option<ControllerReport>, Option<[u8; 64]>), NoloError> {
        let buf = self.read_report()?;
        if buf.is_empty() {
            return Ok((None, None));
        }
        let timestamp_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        Ok(crate::protocol::generate_report_with_raw(&buf, timestamp_ms))
    }
}
