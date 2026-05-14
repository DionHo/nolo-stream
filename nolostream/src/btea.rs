/// Block TEA (BTEA / XXTEA) decryption, ported from nolo-osvr C source.
/// All arithmetic uses wrapping operations to match C unsigned-integer semantics.

const DELTA: u32 = 0x9e3779b9;

#[inline]
fn mx(z: u32, y: u32, sum: u32, key: &[u32; 4], p: usize, e: u32) -> u32 {
    let a = ((z >> 5) ^ (y << 2)).wrapping_add((y >> 3) ^ (z << 4));
    let b = (sum ^ y).wrapping_add(key[((p as u32 & 3) ^ e) as usize] ^ z);
    a ^ b
}

pub fn btea_decrypt(data: &mut [u32], rounds: u32, key: &[u32; 4]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
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
pub(crate) fn btea_encrypt(data: &mut [u32], rounds: u32, key: &[u32; 4]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
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
