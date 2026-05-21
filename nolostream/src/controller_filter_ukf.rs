use serde::{Serialize, Deserialize};

use crate::ControllerReport;
use crate::ControllerState;
use crate::controller_report::ControllerSide;
use crate::controller_report::ControllerStateFilterTrait;


/// UKF-based orientation filter for Nolo controller reports. Converts raw IMU data to filtered orientation estimates.
/// Importantly, this filter has the capability to estimate the controller's absolute yaw angle, 
/// not directly observable from the IMU data alone. This is achieved by leveraging the 
///  * measured position change of the controller vs the
///  * calculated position change based on acceleration data and yaw estimates.
#[derive(Debug, Serialize, Deserialize)]
pub struct ControllerFilterUkf {
    // UKF state and parameters would go here. For simplicity, this is a placeholder.
}

impl ControllerFilterUkf {
    pub fn new() -> Self {
        Self {}
    }
    pub fn filter(&self, report: &ControllerReport) -> ControllerState {
        // Placeholder implementation: in a real implementation, this would apply the UKF algorithm
        // to the raw IMU data in the report and return a ControllerState with filtered orientation.
        ControllerState {
            device: match report.side {
                ControllerSide::Left => DeviceId::ControllerLeft,
                ControllerSide::Right => DeviceId::ControllerRight,
                _ => panic!("ControllerFilter received non-controller report"),
            },
            position: report.position,
            orientation: report.orientation,
            timestamp_ms: report.timestamp_ms,
            touch_x: report.touch_x,
            touch_y: report.touch_y,
            battery: report.battery,
            buttons: report.buttons,
            velocity: [0.0; 3], // Velocity estimation would be part of the UKF state update
            angular_velocity: [0.0; 3], // Angular velocity estimation would also be part of the UKF state update
            state: 0, // Additional state flags could be set here based on button states or other conditions
        }
    }
}


#[derive(Debug, Serialize, Deserialize)]
pub struct ControllersFilterUkf {
    filter_algorithm: [ControllerFilterUkf; 2],
}


impl ControllerStateFilterTrait for ControllersFilterUkf {
    /// Apply the UKF filter to a raw ControllerReport, producing a ControllerState with filtered orientation.
    fn filter(&self, report: &ControllerReport) -> ControllerState {
        match report.side {
            ControllerSide::Left => self.filter_algorithm[0].filter(report),
            ControllerSide::Right => self.filter_algorithm[1].filter(report),
            _ => panic!("ControllerFilter received non-controller report"),
        }
    }
}