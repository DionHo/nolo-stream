use crate::hid::{NoloDevice, NoloError};
use crate::transport::{Transport, TransportError};
use crate::Pose;

pub struct NoloStream {
    device: NoloDevice,
    transports: Vec<Box<dyn Transport>>,
}

impl NoloStream {
    pub fn new() -> Result<Self, NoloError> {
        Ok(NoloStream {
            device: NoloDevice::open()?,
            transports: Vec::new(),
        })
    }

    pub fn add_transport(&mut self, t: Box<dyn Transport>) {
        self.transports.push(t);
    }

    /// Read one HID report and dispatch the resulting poses to all transports.
    /// Disconnected transports are removed. Io errors are logged but the transport is kept.
    pub fn poll_once(&mut self) -> Result<Vec<Pose>, NoloError> {
        let poses = self.device.poll()?;
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
