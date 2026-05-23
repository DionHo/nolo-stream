use crate::controller_filter_ukf::ControllerFilterUkf;
use crate::controller_report::{ControllerReport, ControllerSide};
use crate::controller_state::DeviceId;
use crate::csv_log::CsvLogger;
use crate::hid::{NoloDevice, NoloError};
use crate::teleop::{TeleopFrame, TeleopState};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

pub struct NoloStream {
    device:     NoloDevice,
    transports: Vec<Box<dyn Transport>>,
    ukf_left:   ControllerFilterUkf,
    ukf_right:  ControllerFilterUkf,
    teleop:     TeleopState,
    csv_logger: Option<CsvLogger>,
    gyro_scale: f32,
}

impl NoloStream {
    pub fn new() -> Result<Self, NoloError> {
        Ok(NoloStream {
            device:     NoloDevice::open()?,
            transports: Vec::new(),
            ukf_left:   ControllerFilterUkf::new(),
            ukf_right:  ControllerFilterUkf::new(),
            teleop:     TeleopState::new(),
            csv_logger: None,
            gyro_scale: crate::ahrs::DEFAULT_GYRO_SCALE,
        })
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

    /// Read one HID report, apply UKF orientation filter, dispatch to all transports.
    /// Returns the parsed poses and any teleop delta frames produced this cycle.
    pub fn poll_once(&mut self) -> Result<(Vec<ControllerState>, Vec<TeleopFrame>), NoloError> {
        let (report_opt, raw_buf) = self.device.poll_with_raw()?;

        let poses = match report_opt {
            None => vec![],
            Some(ref report) => self.report_to_poses(report),
        };

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
