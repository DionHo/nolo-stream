use crate::{ControllerReport, btea};

pub const NOLO_VID: u16 = 0x0483;
pub const NOLO_PID: u16 = 0x5750;

const NOLO_KEY: [u32; 4] = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51];

const CRYPTWORDS: usize = 15;

/// Decrypt and parse a 64-byte HID buffer into a ControllerReport.
/// Returns None on unknown/invalid report type.
pub fn generate_report(buf: &[u8], timestamp_ms: u64) -> Option<ControllerReport> {
    decrypt_report(buf).and_then(|dec| ControllerReport::from_decrypted(&dec, timestamp_ms))
}

/// Decrypt and parse, returning both the report and the decrypted buffer.
pub fn generate_report_with_raw(buf: &[u8], timestamp_ms: u64) -> (Option<ControllerReport>, Option<[u8; 64]>) {
    match decrypt_report(buf) {
        Some(dec) => (ControllerReport::from_decrypted(&dec, timestamp_ms), Some(dec)),
        None => (None, None),
    }
}

/// Decrypt the 64-byte HID buffer and return it (for diagnostics).
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
