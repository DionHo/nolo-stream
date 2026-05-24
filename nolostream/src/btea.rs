//! Block TEA (BTEA / XXTEA) decryption, ported from nolo-osvr C source.
//! All arithmetic uses wrapping operations to match C unsigned-integer semantics.
//!
//! The C signature is `btea_decrypt(data, n_words, base_rounds, key)` where
//! total rounds = base_rounds + 52/n_words. We mirror that behaviour here.

const DELTA: u32 = 0x9e3779b9;

#[inline]
fn mx(z: u32, y: u32, sum: u32, key: &[u32; 4], p: usize, e: u32) -> u32 {
    let a = ((z >> 5) ^ (y << 2)).wrapping_add((y >> 3) ^ (z << 4));
    let b = (sum ^ y).wrapping_add(key[((p as u32 & 3) ^ e) as usize] ^ z);
    a ^ b
}

/// Decrypt `data` in-place. `base_rounds` is the base parameter; total rounds = base + 52/n.
pub fn btea_decrypt(data: &mut [u32], base_rounds: u32, key: &[u32; 4]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    let rounds = base_rounds + 52 / n as u32;
    let mut sum = DELTA.wrapping_mul(rounds);
    let mut y = data[0];
    loop {
        let e = (sum >> 2) & 3;
        let mut p = n - 1;
        while p > 0 {
            let z = data[p - 1];
            data[p] = data[p].wrapping_sub(mx(z, y, sum, key, p, e));
            y = data[p];
            p -= 1;
        }
        let z = data[n - 1];
        data[0] = data[0].wrapping_sub(mx(z, y, sum, key, 0, e));
        y = data[0];
        sum = sum.wrapping_sub(DELTA);
        if sum == 0 {
            break;
        }
    }
}

/// BTEA encryption — inverse of btea_decrypt, used only in tests.
#[cfg(test)]
pub(crate) fn btea_encrypt(data: &mut [u32], base_rounds: u32, key: &[u32; 4]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    let rounds = base_rounds + 52 / n as u32;
    let mut sum: u32 = 0;
    let mut z = data[n - 1];
    for _ in 0..rounds {
        sum = sum.wrapping_add(DELTA);
        let e = (sum >> 2) & 3;
        for p in 0..(n - 1) {
            let y = data[p + 1];
            data[p] = data[p].wrapping_add(mx(z, y, sum, key, p, e));
            z = data[p];
        }
        let y = data[0];
        data[n - 1] = data[n - 1].wrapping_add(mx(z, y, sum, key, n - 1, e));
        z = data[n - 1];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encrypt then decrypt a known block and verify we get back the original.
    #[test]
    fn encrypt_decrypt_roundtrip() {
        let original = [0xdeadbeef_u32, 0xcafebabe, 0x12345678, 0x87654321];
        let key = [0x875bcc51_u32, 0xa7637a66, 0x50960967, 0xf8536c51];
        let mut data = original;
        btea_encrypt(&mut data, 1, &key);
        assert_ne!(data, original);
        btea_decrypt(&mut data, 1, &key);
        assert_eq!(data, original);
    }

    /// Roundtrip for a 15-word block matching the actual HID report crypto size.
    #[test]
    fn encrypt_decrypt_roundtrip_15_words() {
        let key = [0x875bcc51_u32, 0xa7637a66, 0x50960967, 0xf8536c51];
        let original: [u32; 15] = [
            0x01020304, 0x05060708, 0x090a0b0c, 0x0d0e0f10,
            0x11121314, 0x15161718, 0x191a1b1c, 0x1d1e1f20,
            0x21222324, 0x25262728, 0x292a2b2c, 0x2d2e2f30,
            0x31323334, 0x35363738, 0x393a3b3c,
        ];
        let mut data = original;
        btea_encrypt(&mut data, 1, &key);
        assert_ne!(data, original);
        btea_decrypt(&mut data, 1, &key);
        assert_eq!(data, original);
    }

    /// A single-word slice should be a no-op for both directions.
    #[test]
    fn single_word_is_noop() {
        let key = [1u32, 2, 3, 4];
        let mut data = [0xaaaa_u32];
        btea_decrypt(&mut data, 1, &key);
        assert_eq!(data, [0xaaaa]);
        btea_encrypt(&mut data, 1, &key);
        assert_eq!(data, [0xaaaa]);
    }
}
