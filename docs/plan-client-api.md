# Plan: NoloClientLib Integration

## What We Learned About the API

### NoloClientLib.dll (ZMQ-based client library)

**Initialization flow:**
1. Call `OpenNoloZeroMQ()` — connects to NoloServer via ZMQ. Returns `true` on success.
   NoloServer must already be running; its socket listens on a well-known port.
2. Optionally call `SetHmdCenter(NVector3)` after connection to set the HMD centre offset.
3. Either poll `GetNoloData()` for a snapshot, or register callbacks:
   - `SetEventListener(INOLOZMQEvent*)` — C++ virtual-dispatch listener (not suitable for Rust FFI)
   - `RegisterCallBack(EClientCallBackTypes, void*)` — C-style function pointers (Rust-friendly)
     - `eOnZMQConnected` / `eOnZMQDisConnected` → `pfnVoidCallBack = void(__cdecl*)()`
     - `eOnNewData` → `pfnDataCallBack = void(__cdecl*)(const NOLOData&)`
4. Call `CloseNoloZeroMQ()` on shutdown.

**Polling approach (chosen for simplicity):**
- `GetNoloData() → NOLOData` — returns the latest snapshot by value.
  The driver sample calls `RunFrame()` every ~16 ms; we use the same interval.
- `GetLeftControllerData() → Controller` and `GetRightControllerData() → Controller` also exist
  but `GetNoloData()` gives all three devices in one call.

**Starting NoloServer:**
- `StartNoloServer(wchar_t* path)` — can launch `NoloServer.exe` from the DLL.
  The driver sample comments this out; we let the PS1 script manage it instead.

**Haptics:** `TriggerHapticPulse(ENoloDeviceType, int intensity)` — intensity 50–100, min 16 ms gap.

### Data Structures (`#pragma pack(push,1)`)

All compound structs use `#pragma pack(push,1)` (no padding between fields).
NVector2/NVector3/NQuaternion are defined outside the pack pragma and use natural alignment,
but all their fields are 32-bit floats so natural and packed layouts are identical.

| Struct          | Size  | Key fields |
|-----------------|-------|------------|
| NVector2        | 8     | x, y (f32) |
| NVector3        | 12    | x, y, z (f32) |
| NQuaternion     | 16    | x, y, z, w (f32) |
| Controller      | 56    | VersionID(i32) + Position(12) + Rotation(16) + Buttons(u32) + Touched(i32) + TouchAxis(8) + Battery(i32) + State(i32) |
| HMD             | 52    | HMDVersionID(i32) + HMDPosition(12) + HMDInitPosition(12) + HMDTwoPointDriftAngle(u32) + HMDRotation(16) + HMDState(i32) |
| BaseStation     | 8     | BaseStationVersionID(i32) + BaseStationPower(i32) |
| NoloSensorData  | 72    | 6 × NVector3 (velocities + angular velocities) |
| NOLOData        | 322   | left(56) + right(56) + hmd(52) + bs(8) + expand\[64\] + sensor(72) + leftPack(u8) + rightPack(u8) + FixedEye(12) |

**Critical layout note:** `NOLOData` has two `u8` fields (offsets 308, 309) immediately before
the final `NVector3 FixedEyePosition` (offset 310). With `pack(1)` this is 322 bytes;
without it, natural alignment would insert 2 bytes of padding → 324 bytes. This means the
Rust binding must use `#[repr(C, packed)]` for `NoloDataRaw`.

### Quaternion convention

`NQuaternion` fields are `x, y, z, w`. The existing `Pose.orientation` uses `[w, x, y, z]`
(Hamilton convention, identity = `[1, 0, 0, 0]`). Mapping: `[q.w, q.x, q.y, q.z]`.

### DLL dependencies

`NoloClientLib.dll` requires `libzmq-64.dll` to be loadable (same directory or PATH).
Both are provided in `docs/reference/NoloDeviceSDK/NoloClient/lib64/`.

### Symbol names

All exported functions use `extern "C"` linkage with `__cdecl` calling convention.
Despite being inside `namespace NOLOVR`, `extern "C"` suppresses name mangling on MSVC,
so DLL exports are unmangled: `OpenNoloZeroMQ`, `CloseNoloZeroMQ`, `GetNoloData`, etc.

---

## Implementation Plan

### Step 1 — Write this plan document ✓

### Step 2 — Add `libloading` dependency

In `nolostream/Cargo.toml`, add under `[target.'cfg(windows)'.dependencies]`:
```toml
libloading = "0.8"
```

### Step 3 — Create `nolostream/src/client_api.rs`

- Define `#[repr(C)]` FFI structs for NVector2/3, NQuaternion, ControllerData, HmdData, etc.
- Define `#[repr(C, packed)]` `NoloDataRaw` (the full 322-byte layout).
- Add compile-time size assertions for each struct.
- `pub struct NoloClientApi` holding the loaded `libloading::Library` and raw function pointers.
- `NoloClientApi::open()` — loads the DLL from the exe's directory, calls `OpenNoloZeroMQ()`.
  Retries up to 5 × 500 ms if the first call returns false.
- `NoloClientApi::get_data()` — calls `GetNoloData()`, returns `NoloDataRaw`.
- `pub fn nolo_data_to_poses(data: &NoloDataRaw) -> Vec<Pose>` — converts snapshot to `Vec<Pose>`.
  Uses `ptr::read_unaligned` to copy sub-structs out of the packed struct safely.
  Always emits left controller, right controller, and HMD poses.

### Step 4 — Update `nolostream/src/lib.rs`

Add `#[cfg(windows)] pub mod client_api;` and re-export `NoloClientApi`/`nolo_data_to_poses`.

### Step 5 — Update `nolostream/src/bin/server.rs`

- Add `#[arg(long)] client_api: bool` to `Args`.
- Extract transport-creation logic into a `build_transports(args: &Args) -> Vec<Box<dyn Transport>>` helper.
- When `--client-api`: call `NoloClientApi::open()`, set up transports, run a `loop` that calls
  `api.get_data()`, converts to poses, dispatches to transports, sleeps 16 ms.

### Step 6 — Write `test/debug-session-client-api.ps1`

Parameters: `-WsPort` (default 12345), `-NoloServerPath` (default to SDK reference path),
`-NoBuild`.

Steps:
1. Kill stale processes (nolostream_server, miniviz, NoloServer).
2. Copy `NoloClientLib.dll` + `libzmq-64.dll` to `target\release\` (creates dir if absent).
3. Optionally build with `cargo build --release`.
4. Start `NoloServer.exe` from `-NoloServerPath`; wait 2 s for it to initialise.
5. Start `nolostream_server --client-api --ws-listen-at $WsPort --debug`.
6. Wait 800 ms, start `miniviz --connect ws://127.0.0.1:$WsPort`.
7. Tail both logs with labels; stop on Ctrl-C.

### Step 7 — Copy DLLs to `target/release/`

Copy `NoloClientLib.dll` and `libzmq-64.dll` from the reference SDK directory.
