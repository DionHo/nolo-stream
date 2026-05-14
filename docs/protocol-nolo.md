# NoloVR HID Protocol

Reverse-engineered from [lonetech/nolo-osvr](https://github.com/lonetech/nolo-osvr) (reference C++ implementation) and extended with observations from a Windows + newer-firmware device.

---

## USB HID Interface

| Parameter | Value |
|---|---|
| VID | `0x0483` |
| PID | `0x5750` |
| Report size | 64 bytes |
| Transfer type | Interrupt |

On **Linux** (hidraw), `hid_read` returns the raw 64-byte report. The first byte is the packet-type discriminator (see Frame Types).

On **Windows** (hidapi), the first byte of the 64-byte buffer is the Windows HID report ID, not the raw packet type byte. The encrypted region occupies the same byte positions (1–60) on both platforms.

---

## BTEA Decryption

The encrypted region of every report is bytes 1–60 (15 × 32-bit little-endian words).

```
key     = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]
n       = 15  (words)
base    = 1   (base_rounds parameter, same as nolo-osvr)
rounds  = base + 52/n = 1 + 3 = 4
```

Algorithm: BTEA / XXTEA (`btea_decrypt` from `btea.c` in nolo-osvr). Byte order for words is little-endian (same on both platforms).

**What is NOT decrypted**: byte 0 (frame-type / report-ID), bytes 61–63 (tail).

---

## Frame Types

| Linux buf[0] | Windows buf[0] | Contents |
|---|---|---|
| `0xa5` | `0x10` | Dual-controller frame |
| `0xa6` | `0x11` | Headset + base-station frame |

> **Linux vs Windows**: On Linux the raw packet-type byte appears at `buf[0]`. On Windows the HID driver maps it to a sequential report ID (`0xa5→0x10`, `0xa6→0x11`). The encrypted region and all offsets within the decrypted payload are identical.

---

## `0xa5` / `0x10` — Controller Frame Layout

After decryption, the 64-byte buffer contains:

| Byte range | Content |
|---|---|
| `[0]` | Frame type (`0xa5` Linux / `0x10` Windows) — unencrypted |
| `[1..23]` | Left controller block (22 bytes, see below) |
| `[24..28]` | 32-bit LE device timestamp (tick counter, ~1 kHz) — **not in nolo-osvr** |
| `[42..63]` | Right controller block (22 bytes at `buf[64 - 22]`) |
| `[61..63]` | Tail (unencrypted); `buf[59]` = monotonic frame counter |

### Controller Block Layout (22 bytes, relative to block start)

```
[0]       hwversion  — nolo-osvr checks for 2; see Firmware Notes
[1]       fwversion  — nolo-osvr checks for 1; see Firmware Notes
[2]       reserved
[3..8]    position    (3 × i16 big-endian)
[9..16]   orientation (4 × i16 big-endian, raw order w,i,j,k)
[17]      buttons     (bitmask)
[18]      touch ID
[19]      touch X     (0–255)
[20]      touch Y     (0–255)
[21]      battery     (0–255)
```

Block offsets in the full decrypted buffer:
- Left controller: `buf[1]`
- Right controller: `buf[42]`  (= `buf[64 - 22]`, nolo-osvr: `buf + 64 - controllerLength`)

---

## `0xa6` / `0x11` — Headset + Base-Station Frame Layout

| Byte range | Content |
|---|---|
| `[0]` | Frame type (`0xa6` / `0x11`) — unencrypted |
| `[21..44]` | Headset marker block (starts at `0x15`) |
| `[24..28]` | 32-bit LE device timestamp (tick counter) — **not in nolo-osvr** |
| `[54..56]` | Base station block (starts at `0x36`) |
| `[49..57]` | Observed base-station orientation bytes (constant, stationary unit quaternion) |
| `[59]` | Frame counter |

### Headset Marker Block (relative to block start = `buf[21]`)

```
[0]       hwversion  — nolo-osvr checks for 2
[1]       fwversion  — nolo-osvr checks for 1
[2]       reserved
[3..8]    position       (3 × i16 big-endian)
[9..14]   home position  (3 × i16 big-endian; reference/anchor point)
[15]      reserved
[16..23]  orientation    (4 × i16 big-endian, raw order w,i,j,k)
```

### Base Station Block (relative to block start = `buf[54]`)

```
[0]   hwversion — nolo-osvr checks for 2
[1]   fwversion — nolo-osvr checks for 1
[2]   battery   (0–255)
```

---

## Position Encoding

3 × signed 16-bit big-endian integers, multiply by `0.0001` to get metres.

```rust
x = i16_be(buf[offset], buf[offset+1]) as f32 * 0.0001
y = i16_be(buf[offset+2], buf[offset+3]) as f32 * 0.0001
z = i16_be(buf[offset+4], buf[offset+5]) as f32 * 0.0001
```

---

## Orientation Encoding

4 × signed 16-bit big-endian integers in raw order `(w, i, j, k)`.

### Reference firmware (nolo-osvr, hwver=2/fwver=1)

Fixed-point scale: divide by `16384.0`. Raw values should have `||(w,i,j,k)|| ≈ 16384`.

### Reordering (from nolo-osvr `decodeOrientation`)

The raw `(w, i, j, k)` values are not directly `(W, X, Y, Z)` in the output quaternion:

```
output W = w
output X = i
output Y = k     ← note: j and k are SWAPPED relative to raw order
output Z = -j    ← j is negated, not k
```

### Normalization (firmware-agnostic)

Newer firmware may scale values differently. Normalizing by the Euclidean magnitude is safe for both firmware variants:

```rust
let mag = (w*w + i*i + j*j + k*k).sqrt();
[w/mag, i/mag, k/mag, -j/mag]   // after reorder: W, X, Y, Z
```

Use identity `[1, 0, 0, 0]` as fallback when `mag < threshold`.

---

## Firmware Observations

### Reference firmware (nolo-osvr era, Linux)

- `buf[0]` = `0xa5` / `0xa6`
- Controller: `hwver=2`, `fwver=1`
- Headset: `hwver=2`, `fwver=1`
- Orientation scale: `16384` (values in `[-16384, +16384]`)

### Newer firmware (observed on Windows, 2024+)

- `buf[0]` = `0x10` / `0x11` (Windows report IDs replacing `0xa5`/`0xa6`)
- Controller: `hwver≈0x52–0x56` (82–86), `fwver≈0xee` (238) — fails nolo-osvr version check
- Headset: `hwver≈0x99–0x9e` (153–158), `fwver≈0xf5` (245)
- **hwver/fwver drift slightly over time** — these may not be version bytes at all; exact semantics unknown
- Orientation values at `block[9..17]`: magnitude `≈ 800–1050` (not `16384`). May be different fixed-point scale or IMU data. Normalizing by actual magnitude produces a reasonable unit quaternion.
- Additional bytes not present in nolo-osvr: 32-bit timestamp at `buf[24..28]`, frame counter at `buf[59]`, base-station orientation at `buf[49..57]`.

### Linux behavior with newer firmware

Unknown — the device was only tested on Windows. If `buf[0]` appears as `0x10`/`0x11` on Linux too (possible if the device firmware changed its report ID), the same code path handles it. If Linux still returns `0xa5`/`0xa6`, the existing mapping covers that as well.

---

## Wire Format (TCP / UDP / WebSocket)

Each message is a **newline-terminated JSON array** of pose objects, one per HID poll:

```json
[{"device":"left_controller","position":[x,y,z],"orientation":[w,x,y,z],"timestamp_ms":N}]
```

| Field | Type | Description |
|---|---|---|
| `device` | `"headset"` \| `"left_controller"` \| `"right_controller"` | Source device |
| `position` | `[x, y, z]` f32 metres | World-space position |
| `orientation` | `[w, x, y, z]` f32 unit quaternion | Orientation after reorder + normalize |
| `timestamp_ms` | u64 ms | Host UNIX epoch time at poll |

A `0xa5` poll yields up to 2 poses (both controllers). A `0xa6` poll yields 1 pose (headset). The Nolo base station is not streamed (no pose — static anchor).

---

## Open Questions

- Exact semantics of `hwver`/`fwver` bytes in newer firmware (drift suggests they may not be version numbers).
- Orientation format in newer firmware: fixed-point with smaller scale (~1000), IMU angular velocity, or something else? Normalizing works empirically but the meaning is uncertain.
- Whether `buf[0]` appears as `0x10`/`0x11` or `0xa5`/`0xa6` on Linux with newer firmware.
- Home-position field in headset block: purpose unclear (reference/anchor position?).
