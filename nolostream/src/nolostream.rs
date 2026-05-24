use crate::controller_filter_ukf::ControllerFilterUkf;
use crate::controller_report::{ControllerReport, ControllerSide};
use crate::controller_state::DeviceId;
use crate::csv_log::CsvLogger;
use crate::hid::{NoloDevice, NoloError};
use crate::teleop::{TeleopFrame, TeleopState};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

pub struct NoloStream {
    device:     Option<NoloDevice>,
    transports: Vec<Box<dyn Transport>>,
    ukf_left:   ControllerFilterUkf,
    ukf_right:  ControllerFilterUkf,
    teleop:     TeleopState,
    csv_logger: Option<CsvLogger>,
    gyro_scale: f32,
    last_raw:   Option<[u8; 64]>,
}

impl Default for NoloStream {
    fn default() -> Self { Self::new() }
}

impl NoloStream {
    /// Create a NoloStream, attempting to open the HID device immediately.
    /// Does not fail if the device is absent — call `is_device_connected()` to check,
    /// and `try_reconnect()` to retry opening.
    pub fn new() -> Self {
        NoloStream {
            device:     NoloDevice::open().ok(),
            transports: Vec::new(),
            ukf_left:   ControllerFilterUkf::new(),
            ukf_right:  ControllerFilterUkf::new(),
            teleop:     TeleopState::new(),
            csv_logger: None,
            gyro_scale: crate::ahrs::DEFAULT_GYRO_SCALE,
            last_raw:   None,
        }
    }

    /// Returns `true` if a HID device is currently open and available.
    pub fn is_device_connected(&self) -> bool {
        self.device.is_some()
    }

    /// Try to open the HID device if not already connected.
    /// Returns `true` if the device is now connected (whether it was already or just reconnected).
    pub fn try_reconnect(&mut self) -> bool {
        if self.device.is_some() {
            return true;
        }
        match NoloDevice::open() {
            Ok(dev) => {
                self.device = Some(dev);
                true
            }
            Err(_) => false,
        }
    }

    /// Mark the device as disconnected so the reconnect loop will reopen it.
    /// Call when external logic determines the device stopped responding.
    pub fn force_disconnect(&mut self) {
        self.device = None;
    }

    /// Return the last decrypted 64-byte HID report, if any.
    pub fn last_raw_report(&self) -> Option<[u8; 64]> {
        self.last_raw
    }

    pub fn set_gyro_scale(&mut self, scale: f32) {
        self.gyro_scale = scale;
        // Reset UKF filters so they start fresh with the new scale assumption
        self.ukf_left  = ControllerFilterUkf::new();
        self.ukf_right = ControllerFilterUkf::new();
    }

    pub fn set_csv_log(&mut self, logger: CsvLogger) {
        self.csv_logger = Some(logger);
    }

    pub fn add_transport(&mut self, t: Box<dyn Transport>) {
        self.transports.push(t);
    }

    /// Replace all current transports with a new set (used by GUI on config change).
    pub fn replace_transports(&mut self, transports: Vec<Box<dyn Transport>>) {
        self.transports = transports;
    }

    /// Read one HID report, apply UKF orientation filter, dispatch to all transports.
    /// Returns the parsed poses and any teleop delta frames produced this cycle.
    /// Returns empty poses (no error) when the device is absent or on read failure.
    pub fn poll_once(&mut self) -> Result<(Vec<ControllerState>, Vec<TeleopFrame>), NoloError> {
        let poll_result = self.device.as_ref().map(|dev| dev.poll_with_raw());
        let (report_opt, raw_buf) = match poll_result {
            None => return Ok((vec![], vec![])),
            Some(Ok(result)) => result,
            Some(Err(e)) => {
                eprintln!("HID device error: {e}, marking disconnected");
                self.device = None;
                return Ok((vec![], vec![]));
            }
        };

        let poses = match report_opt {
            None => vec![],
            Some(ref report) => self.report_to_poses(report),
        };

        self.last_raw = raw_buf;

        if let Some(ref mut logger) = self.csv_logger {
            for pose in &poses {
                if let Err(e) = logger.write_pose("hid", pose, raw_buf.as_ref()) {
                    eprintln!("csv-log write error: {e}");
                }
            }
        }

        let teleop_frames = {
            let teleop_target_msgs: Vec<_> = self.transports.iter_mut()
                .flat_map(|t| t.recv_teleop_target_msgs())
                .collect();
            let update = self.teleop.update(&poses, &teleop_target_msgs);

            self.transports.retain_mut(|t| match t.send(&poses) {
                Ok(()) => true,
                Err(TransportError::Disconnected) => false,
                Err(TransportError::Io(msg)) => {
                    eprintln!("transport io error: {msg}");
                    true
                }
            });

            if !update.frames.is_empty() {
                for t in &mut self.transports {
                    if let Err(e) = t.send_teleop(&update.frames) {
                        eprintln!("teleop dispatch error: {e}");
                    }
                }
            }

            if let Some(ref handover) = update.handover_out {
                for t in &mut self.transports {
                    let _ = t.send_handover(handover);
                }
            }

            update.frames
        };

        Ok((poses, teleop_frames))
    }

    fn report_to_poses(&mut self, report: &ControllerReport) -> Vec<ControllerState> {
        let controller_state = match report.side {
            ControllerSide::Left  => self.ukf_left.filter(report),
            ControllerSide::Right => self.ukf_right.filter(report),
        };

        let hmd_state = ControllerState {
            device:           DeviceId::Headset,
            position:         report.hmd_position,
            orientation:      report.hmd_orientation,
            timestamp_ms:     report.timestamp_ms,
            touch_x:          255,
            touch_y:          255,
            battery:          0,
            buttons:          0,
            velocity:         [0.0; 3],
            angular_velocity: [0.0; 3],
            state:            0,
        };

        vec![controller_state, hmd_state]
    }
}
