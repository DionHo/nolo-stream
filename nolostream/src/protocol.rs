use crate::controller_report::ControllerStateFilterTrait;
use crate::{ControllerReport, btea};
use crate::controller_state::{DeviceId, ControllerState};

pub const NOLO_VID: u16 = 0x0483;
pub const NOLO_PID: u16 = 0x5750;

const NOLO_KEY: [u32; 4] = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51];

/// Bytes 1..=60 are 15 LE u32 words that need BTEA decryption (nolo-osvr: cryptoffset=1, cryptwords=15).
const CRYPTWORDS: usize = 15;

/// Controller block length in bytes (nolo-osvr: 3 + (3+4)*2 + 2 + 2 + 1 = 22).
/// Used to locate the right controller: buf[64 - CTRL_LEN] = buf[42].
const CTRL_LEN: usize = 22;

/// Decrypt the encrypted region of a 64-byte raw HID buffer, then parse it.
/// Returns an empty Vec on an unknown or invalid report.
pub fn generate_report(buf: &[u8], timestamp_ms: u64, controller_filter: Box<dyn ControllerStateFilterTrait>) -> Vec<ControllerState> {
    if let Some(dec) = decrypt_report(buf) {
        parse_decrypted(&dec, timestamp_ms, controller_filter)
    } else {
        vec![]
    }
}

/// Decrypt and parse in one pass, returning both the decrypted buffer and parsed Poses.
pub fn generate_report_with_raw(buf: &[u8], timestamp_ms: u64, controller_filter: Box<dyn ControllerStateFilterTrait>) -> (Vec<ControllerState>, Option<[u8; 64]>) {
    if let Some(dec) = decrypt_report(buf) {
        let poses = parse_decrypted(&dec, timestamp_ms, controller_filter);
        (poses, Some(dec))
    } else {
        (vec![], None)
    }
}

/// Decrypt a 64-byte HID buffer in-place and return it (without parsing into Poses).
/// Useful for diagnostics.
pub fn decrypt_report(buf: &[u8]) -> Option<[u8; 64]> {
    if buf.len() < 64 {
        return None;
    }
    let mut work = [0u8; 64];
    work.copy_from_slice(&buf[..64]);
    let mut words = [0u32; CRYPTWORDS];
    for (i, word) in words.iter_mut().enumerate() {
        let b = 1 + i * 4;
        *word = u32::from_le_bytes([work[b], work[b + 1], work[b + 2], work[b + 3]]);
    }
    btea::btea_decrypt(&mut words, 1, &NOLO_KEY);
    for (i, word) in words.iter().enumerate() {
        let b = 1 + i * 4;
        work[b..b + 4].copy_from_slice(&word.to_le_bytes());
    }
    Some(work)
}

/// Parse a fully-decrypted 64-byte buffer into Pose values.
fn parse_decrypted(buf: &[u8], timestamp_ms: u64, controller_filter: Box<dyn ControllerStateFilterTrait>) -> Vec<ControllerState> {
    if let Some(report) = ControllerReport::from_decrypted(buf, timestamp_ms) {
        report.to_states(controller_filter)
    } else {
        vec![]
    }
}
