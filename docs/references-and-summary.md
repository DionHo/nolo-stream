# References and summary

the subdirectory `reference` contains files from two different sources:
* `reference/nolo-osvr/` is a git clone of [lonetech/nolo-osvr](https://github.com/lonetech/nolo-osvr)
* `reference/NoloDeviceSDK/HmdDriver1_0_13Sample/driver_Sample/` the unzipped `driver_Sample` directory of the HmdDriver sample found in [Nolo Device SDK](https://github.com/NOLOVR/NoloDeviceSDK/blob/master/HmdDriver1_0_13Sample/HmdDriver1_0_13Sample.zip).
* `reference/NoloDeviceSDK/NoloClient/` — SDK headers (`NoloClientAPI/`) plus prebuilt DLLs and import libs for both x86 and x64 (`lib32/`, `lib64/`).

---

## Nolo OSVR protocol reverse-engineering insights

`com_osvr_Nolo.cpp` is the ground truth for the HID + BTEA decode path. Everything below is derived directly from its code.

### BTEA decrypt (exact parameters)

```
cryptoffset = 1          // first encrypted byte in the 64-byte buffer
cryptwords  = (64-4)/4   // = 15 words
key         = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]
btea_decrypt(data, n=15, base=1, key)  // rounds = base + 52/n = 1+3 = 4
```

Words are assembled **little-endian** from `buf[1..61]` before calling decrypt, then written back in the same byte order. Bytes 0 and 61–63 are **not** encrypted.

### Deduplication

Two 64-byte shadow buffers indexed by `buf[0] - 0xa5` (slot 0 for `0xa5`, slot 1 for `0xa6`). A full `memcmp(shadow, buf, 64)` is done **before** decryption. If identical, the frame is dropped without decrypt. On change, `memcpy` updates the shadow and processing continues.

### Frame routing

```
buf[0] == 0xa5  →  decodeControllerCV1(0, buf+1)          // left
                    decodeControllerCV1(1, buf+64-22)      // right (buf+42)
buf[0] == 0xa6  →  decodeHeadsetMarkerCV1(buf+0x15)       // buf+21
                    decodeBaseStationCV1(buf+0x36)         // buf+54
```

`controllerLength = 3 + (3+4)*2 + 2 + 2 + 1 = 22`

### Controller block field layout (22 bytes, reference firmware hwver=2/fwver=1)

All offsets are relative to the block start (`buf+1` for left, `buf+42` for right).

```
[0]      hwversion   — skip block unless == 2
[1]      fwversion   — skip block unless == 1
[2]      reserved
[3..8]   position    (3 × i16 BE, scale 0.0001 → metres)
[9..16]  orientation (4 × i16 BE, order w,i,j,k; scale 1/16384 → unit quat)
[17]     buttons     (bitmask, bits 0–5, see below)
[18]     touch_id    (non-zero = touch active)
[19]     touch_x     (0–255)
[20]     touch_y     (0–255)
[21]     battery     (0–255 → divide by 255.0 for 0..1)
```

### Headset marker block layout (relative to `buf+21`)

Offsets computed from `enum MarkerOffsets` in source:
```
orientation = 3 + 2*3*2 + 1 = 16
```

```
[0]      hwversion    — skip unless == 2
[1]      fwversion    — skip unless == 1
[2]      reserved
[3..8]   position     (3 × i16 BE, scale 0.0001)
[9..14]  homeposition (3 × i16 BE, scale 0.0001 — anchor/reference point)
[15]     reserved     (the +1 in the offset formula above)
[16..23] orientation  (4 × i16 BE, order w,i,j,k; scale 1/16384)
```

### Base station block layout (relative to `buf+54`)

```
[0]  hwversion — skip unless == 2
[1]  fwversion — skip unless == 1
[2]  battery   (0–255 raw, reported as analog channel 8 in OSVR)
```

### decodeOrientation — exact reorder

```rust
w = i16_be(&data[0..2]) as f64 / 16384.0;
i = i16_be(&data[2..4]) as f64 / 16384.0;
j = i16_be(&data[4..6]) as f64 / 16384.0;
k = i16_be(&data[6..8]) as f64 / 16384.0;
// output quaternion (W, X, Y, Z):
out_w = w;
out_x = i;
out_y = k;   // j and k are SWAPPED
out_z = -j;  // j is negated, not k
```

Normalize by magnitude to handle both firmware variants safely (newer firmware has smaller raw magnitudes ~800–1050 rather than 16384).

### Button bitmask (bit → button name)

Cross-validated between nolo-osvr OSVR JSON semantic mapping and SDK `EControlerButtonType`:

```
bit 0  (0x01) = Trackpad click     (ePadBtn)
bit 1  (0x02) = Trigger click      (eTriggerBtn)
bit 2  (0x04) = Menu / App button  (eMenuBtn)
bit 3  (0x08) = System / Home      (eSystemBtn)
bit 4  (0x10) = Grip               (eGripBtn)
bit 5  (0x20) = Trackpad touch     (ePadTouch)
```

`touch_id` at block[18] is a secondary touch-active flag, identical to bit 5 of buttons (the source notes "next byte is touch ID bitmask (identical to buttons bit 5)"). Only emit touchpad X/Y analog values when `touch_id != 0`.

### Touchpad normalization (exact from source)

Both axes are independently normalized then **both inverted**:

```rust
// Only when touch_id != 0:
axis_x = (2.0 * touch_x as f64 / 255.0 - 1.0) * -1.0;   // range -1..1, inverted
axis_y = (2.0 * touch_y as f64 / 255.0 - 1.0) * -1.0;   // range -1..1, inverted
```

Trigger is **digital only** in this reference: emit `0.0` or `1.0` from bit 1 of buttons.

### Vibration HID write protocol

```
data = [0xaa, 0x66, left_intensity, right_intensity]
hid_write(dev, data, 4)   // intensity: u8, 0–255
```

### HID device enumeration

Filter by VID `0x0483` + PID `0x5750`, then additionally check:
- `manufacturer_string == L"LYRobotix"`
- `product_string == L"NOLO"`

Only the first matching device is opened (single-instance plugin).

### Version guard and newer firmware

The version check `hwver == 2 && fwver == 1` gates all decode paths. Newer firmware reports `hwver ≈ 82–86` / `fwver ≈ 238` and will silently skip without patching this check. See `protocol-nolo.md` for the newer firmware packet layout.

### OSVR output channel mapping (from `com_osvr_Nolo.json`)

```
Trackers:  0 = home anchor (position only, no orientation)
           1 = HMD pose
           2 = left controller
           3 = right controller

Analogs:   idx*4+0 = touchpad X    (-1..1)
           idx*4+1 = touchpad Y    (-1..1)
           idx*4+2 = trigger       (0..1, digital)
           idx*4+3 = battery       (0..1)
           channel 8 = base station battery (0..255 raw)

Buttons:   idx*6+0 = trackpad click
           idx*6+1 = trigger click
           idx*6+2 = menu
           idx*6+3 = system
           idx*6+4 = grip
           idx*6+5 = touchpad touch
```

(`idx` = 0 for left controller, 1 for right controller)

---

## Nolo Device SDK insights

### Architecture

The SDK uses a two-process model: **NoloServer.exe** (in `NoloDeviceSDK/NoloServer/`) owns the USB HID connection and broadcasts data. **NoloClientLib.dll** connects to it via ZeroMQ (`libzmq-64.dll`). This means using the SDK path and the direct HID path simultaneously is not possible — they would compete for the same USB device.

Required DLL co-location for the SDK path:
- `NoloClientLib.dll` (x64: `lib64/`, x86: `lib32/`)
- `libzmq-64.dll` / `libzmq-32.dll` (same directory)

### Init sequence (from `NoloDeviceManager.cpp`)

```
1. StartNoloServer(L"")          // optional — launches NoloServer.exe if not running
2. RegisterCallBack(type, fn)    // preferred over SetEventListener for Rust (see below)
3. OpenNoloZeroMQ()              // → bool; returns false if server unreachable
4. [in OnZMQConnected callback]:
       SetHmdCenter(NVector3(0.0, 0.09, 0.07))  // sets tracking origin relative to HMD
5. Receive OnNewData callbacks with full NOLOData structs
6. CloseNoloZeroMQ()             // on shutdown
```

### Rust FFI: use `RegisterCallBack`, not `SetEventListener`

`SetEventListener` takes a pointer to a C++ virtual class (`INOLOZMQEvent`). Implementing a C++ vtable from Rust requires unsafe vtable construction that is fragile and ABI-specific. Use `RegisterCallBack` with plain C function pointers instead:

```rust
extern "C" fn on_new_data(data: *const NOLOData) {
    // safe to call from SDK thread
}
// call once before OpenNoloZeroMQ():
RegisterCallBack(EClientCallBackTypes::eOnNewData as i32, on_new_data as *const c_void);
```

The full callback type list from `EClientCallBackTypes`:
```
eOnZMQConnected    = 0  → pfnVoidCallBack   = fn()
eOnZMQDisConnected = 1  → pfnVoidCallBack   = fn()
eOnButtonDoubleClicked = 2  → pfnKeyEvent   = fn(ENoloDeviceType, u8)
eOnKeyPressEvent   = 3  → pfnKeyEvent
eOnKeyReleaseEvent = 4  → pfnKeyEvent
eOnNewData         = 5  → pfnDataCallBack   = fn(*const NOLOData)
eOnNoloDevVersion  = 6  → pfnVoidIntCallBack = fn(i32)
```

### `NoloClientLib` exported functions (`__cdecl`, `extern "C"`)

```c
bool     StartNoloServer(const wchar_t *path)    // "" = look next to DLL
void     SetEventListener(INOLOZMQEvent *)        // C++ vtable — avoid from Rust
void     RegisterCallBack(int type, void *fn)
void     SetHmdCenter(const NVector3 *center)
void     SetBCellingMode(bool ceiling)
bool     OpenNoloZeroMQ()
void     CloseNoloZeroMQ()
void     TriggerHapticPulse(int device, int intensity)  // intensity: 50–100
Controller  GetLeftControllerData()
Controller  GetRightControllerData()
HMD         GetHMDData()
NOLOData    GetNoloData()
void     SendUIComand(const char *json)          // max 60 chars
```

`TriggerHapticPulse` takes `ENoloDeviceType` as int: `eLeftController=1`, `eRightController=2`. Each call fires one ~16 ms pulse.

### SDK struct layouts (`#pragma pack(1)` — no padding anywhere)

```
NVector2  (8 bytes):  f32 x, f32 y
NVector3 (12 bytes):  f32 x, f32 y, f32 z
NQuaternion (16 bytes): f32 x, f32 y, f32 z, f32 w   ← NOTE: x,y,z,w — NOT w,x,y,z
```

```
Controller (56 bytes):
  +0   VersionID   i32
  +4   Position    NVector3   (x, y, z)
  +16  Rotation    NQuaternion (x, y, z, w)
  +32  Buttons     u32        (EControlerButtonType bitmask)
  +36  Touched     i32        (non-zero = touch active)
  +40  TouchAxis   NVector2   (x, y, already normalized –1..1 by SDK)
  +48  Battery     i32        (0–255)
  +52  State       i32

HMD (52 bytes):
  +0   HMDVersionID          i32
  +4   HMDPosition           NVector3
  +16  HMDInitPosition       NVector3   (anchor / home reference)
  +28  HMDTwoPointDriftAngle u32
  +32  HMDRotation           NQuaternion (x, y, z, w)
  +48  HMDState              i32

BaseStation (8 bytes):
  +0   BaseStationVersionID  i32
  +4   BaseStationPower      i32

NoloSensorData (72 bytes):
  +0   vecLVelocity        NVector3
  +12  vecLAngularVelocity NVector3
  +24  vecRVelocity        NVector3
  +36  vecRAngularVelocity NVector3
  +48  vecHVelocity        NVector3
  +60  vecHAngularVelocity NVector3

NOLOData:
  +0    leftData      Controller   (56 bytes)
  +56   rightData     Controller   (56 bytes)
  +112  hmdData       HMD          (52 bytes)
  +164  bsData        BaseStation  (8 bytes)
  +172  expandData    [u8; 64]
  +236  NoloSensorData NoloSensorData (72 bytes)
  +308  leftPackNumber  u8
  +309  rightPackNumber u8
  +310  FixedEyePosition NVector3   (12 bytes)
  total ≈ 322 bytes
```

### EControlerButtonType bitmask (same values as nolo-osvr bit indices)

```
0x01 = ePadBtn      (trackpad click)
0x02 = eTriggerBtn  (trigger)
0x04 = eMenuBtn     (menu / app)
0x08 = eSystemBtn   (system / home)
0x10 = eGripBtn     (grip)
0x20 = ePadTouch    (trackpad touch)
```

Check: `controller.Buttons & 0x02 != 0` → trigger pressed. `TouchAxis` is already in –1..1; only valid when `Touched != 0`.

### Haptic pulse amplitude

Range is **50–100**, not 0–100. The lower bound of 50 is the minimum the hardware responds to. The sample driver maps OpenVR's 0–1 amplitude as:

```rust
let intensity = 50 + (50.0 * openvr_amplitude) as i32;  // clamp to 50..100
```

Each `TriggerHapticPulse` call produces approximately one 16 ms burst.

### Coordinate space (from `NOLOController.cpp`)

The SDK delivers poses in Nolo-native space (left-handed, Z-forward, consistent with `protocol-nolo.md`). The commented-out OpenVR conversion code shows exactly what negations are needed to reach a right-handed Y-up –Z-forward frame:

```cpp
// from NOLOController.cpp (commented out but authoritative):
pose.vecPosition[2] = -ctrData.Position.z;   // negate Z position
pose.qRotation.w    = -ctrData.Rotation.w;   // negate quaternion W
pose.qRotation.z    = -ctrData.Rotation.z;   // negate quaternion Z
// x and y of both position and rotation are passed through unchanged
```

For a left-handed output (e.g. Babylon.js without `useRightHandedSystem`), no negation is needed — use the SDK values directly.

### Double-click gestures (built-in SDK behavior)

`OnKeyDoubleClicked` fires when a button is double-tapped. The sample driver maps:
- Double System → `RecenterHmd()` (resets tracking origin to current pose using both controllers)
- Double Menu → `TurnAroundHmd()` (180° yaw flip for facing-backward tracking)

These arrive via `eOnButtonDoubleClicked` callback; the raw `eOnKeyPressEvent` / `eOnKeyReleaseEvent` callbacks deliver individual button events.

### Rust CLI-switch strategy

Structure the crate with two backends behind a feature flag or runtime argument:

```
--backend hid       uses hidapi + BTEA decrypt (protocol-nolo.md)
--backend nolo-sdk  loads NoloClientLib.dll at runtime via libloading
```

Using `libloading` (dynamic load) rather than a static `.lib` link lets the binary run on systems without the Nolo SDK installed when `--backend hid` is selected. Resolve all function pointers at startup only when `--backend nolo-sdk` is active.

The `NOLOData` struct must be declared `#[repr(C, packed)]` in Rust to match the SDK's `#pragma pack(1)` layout. Verify the total size at compile time with `assert_eq!(std::mem::size_of::<NOLOData>(), 322)` (or the actual measured value).
