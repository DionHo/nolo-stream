# NoloStream

Stream 6DOF pose data from a NoloVR headset and controllers to other applications over TCP, UDP, or WebSocket. Core device protocol reverse-engineered from [lonetech/nolo-osvr](https://github.com/lonetech/nolo-osvr).

## Status

All features implemented and tested:
- HID device discovery, BTEA decryption, and report parsing (headset + dual controllers)
- NoloClientLib.dll integration via `--client-api` (requires NoloServer.exe running)
- Five transport modes: TCP listen, TCP teleop outbound (per-controller), UDP push, WebSocket listen
- Miniviz: 3D Babylon.js viewer with touchpad, button, battery, velocity display and command panel
- Teleop: yaw calibration + cumulative-offset streaming in robotics (Z-up) coordinates

## Tech Stack

| Layer | Technology |
|---|---|
| Language | Rust (2021 edition) |
| HID / USB | `hidapi` 2 (hidraw backend on Linux) |
| NoloClientLib | `libloading` 0.8 — runtime DLL load (Windows only, `--client-api`) |
| WebSocket | `tungstenite` 0.24 (sync) |
| CLI | `clap` ~4.4 (derive macros) |
| Serialization | `serde` + `serde_json` |
| Miniviz HTTP | `tiny_http` 0.12 |
| Miniviz 3D | Babylon.js (CDN) via Vanilla JS + WebSocket |
| CI / Binaries | GitHub Actions → Linux x64 + Windows x64 |

## Protocol

### Device (USB HID)

- **VID/PID**: `0x0483` / `0x5750`
- **Report size**: 64 bytes
- **Encryption**: BTEA (XXTEA) over bytes 1–60, key `[0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]`
- **Report `0xa5`**: dual-controller frame — left at offset 1, right at offset 32
- **Report `0xa6`**: headset frame — headset block at offset 21

Position is encoded as 3 × i16 big-endian, scaled by `0.0001` (metres).  
Orientation is encoded as 4 × i16 big-endian (W, X, Y, Z), divided by `16384.0`, with Z negated per nolo-osvr convention.

### Wire Format (TCP / WebSocket / UDP)

Each message is a **newline-terminated JSON array** of pose objects:

```json
[
  {
    "device": "left_controller",
    "position": [0.12, 1.05, -0.30],
    "orientation": [0.99, 0.01, 0.02, -0.01],
    "timestamp_ms": 1715702400000,
    "touch_x": 180, "touch_y": 100,
    "battery": 200,
    "buttons": 2,
    "velocity": [0.01, -0.05, 0.02],
    "angular_velocity": [0.1, 0.0, -0.2],
    "state": 0
  }
]
```

**Pose fields:**

| Field | Type | Description |
|---|---|---|
| `device` | string | `"headset"`, `"left_controller"`, or `"right_controller"` |
| `position` | `[x,y,z]` f32, metres | World-space position |
| `orientation` | `[w,x,y,z]` f32, unit quaternion | World-space rotation (Hamilton convention) |
| `timestamp_ms` | u64 | Host time at poll, ms since UNIX epoch |
| `touch_x` | u8 | Touchpad X: 0–254 (127=center); **255=no touch** |
| `touch_y` | u8 | Touchpad Y: 0–254 (127=center); **255=no touch** |
| `battery` | u8 | Battery 0–255 (controllers only; 0=unknown) |
| `buttons` | u32 | Button bitmask (controllers only) — see table below |
| `velocity` | `[x,y,z]` f32, m/s | Linear velocity — client-api path only; zeros on HID |
| `angular_velocity` | `[x,y,z]` f32, rad/s | Angular velocity — client-api path only; zeros on HID |
| `state` | i32 | Driver tracking state (0=OK) — client-api path only; 0 on HID |
| `sensor_raw` | `[i16; 19]` | Raw ADC values — HID path only; zeros on client-api |

**Button bits** (`buttons` field, controllers only):

| Bit | Name | Button |
|---|---|---|
| `0x01` | `ePadBtn` | Touchpad click |
| `0x02` | `eTriggerBtn` | Trigger |
| `0x04` | `eMenuBtn` | Menu |
| `0x08` | `eSystemBtn` | System |
| `0x10` | `eGripBtn` | Grip |
| `0x20` | `ePadTouch` | Touchpad touched (no click) |

### Client Commands (WebSocket only)

Clients may send JSON objects back to the server over the WebSocket to control the hardware:

```json
{"cmd": "haptic",         "device": "left_controller", "intensity": 75}
{"cmd": "haptic",         "device": "right_controller","intensity": 50}
{"cmd": "set_hmd_center", "x": 0.0, "y": 0.09, "z": 0.07}
{"cmd": "ceiling_mode",   "enabled": true}
{"cmd": "ui_command",     "content": "{\"action\":\"recenter\"}"}
```

| `cmd` | Extra fields | Action |
|---|---|---|
| `haptic` | `device` (string), `intensity` (50–100) | Trigger haptic pulse via `TriggerHapticPulse` |
| `set_hmd_center` | `x`, `y`, `z` (f32, metres) | Set HMD origin offset via `SetHmdCenter` |
| `ceiling_mode` | `enabled` (bool) | Toggle ceiling-mount mode via `SetBCellingMode` |
| `ui_command` | `content` (string) | Forward raw JSON to NoloServer via `SendUIComand` |

Commands are only forwarded to hardware on the `--client-api` path (ignored on HID path).

A single HID poll yields a `0xa5` report (2 controller poses) and/or a `0xa6` report (1 headset pose). All poses from one poll are batched into a single JSON array.

## Teleop

Teleop streams the **cumulative pose offset since handover** in **robotics coordinates (Z up, right-handed)** for use as robot remote-control input.

### Yaw calibration

Press the **Menu button** on either controller (both must be tracked). The horizontal vector from the left to the right controller becomes the **+X axis** for all subsequent teleop output.

### Offset streaming

Hold the **Trigger button** on a controller while handover is active. The server emits the **cumulative offset since handover** (reset on each `handover_active`) over **separate TCP connections** (one per controller):

```json
{"type":"relative","pose_mm-deg":[12.0,0.0,-8.0,1.5,0.0,-3.0],"timestamp_ms":1715702400123}
```

The coordinate system is Z-up right-handed with X calibrated to the left→right controller direction.  
NoloStream acts as TCP **client** — specify the robot's listening addresses with `--teleop-left-to` and `--teleop-right-to`.  
See [docs/teleop.md](docs/teleop.md) for the full handshake protocol, coordinate transform math, and robot-side application.

### Miniviz

Miniviz acts as a dual TeleopTarget for testing. Two wireframe target boxes (green = left, yellow = right) appear in the scene and move with teleop input. Use **START L** / **START R** to activate handover per controller. Press **RESET** to return targets to the origin.

## Project Layout

```
NoloStream/
├── nolostream/src/
│   ├── btea.rs          # BTEA/XXTEA decryption
│   ├── protocol.rs      # HID report parser (0xa5, 0xa6)
│   ├── hid.rs           # Device open + read loop
│   ├── pose.rs          # Pose struct + DeviceId enum
│   ├── teleop.rs        # TeleopState + TeleopFrame (yaw cal + offset streaming)
│   ├── client_api.rs    # NoloClientLib.dll wrapper (Windows, --client-api)
│   ├── command.rs       # Command enum for WS client→server messages
│   ├── transport.rs     # Transport trait
│   ├── transports/      # tcp_listener, tcp_stream, udp_stream, ws_listener
│   ├── nolostream.rs    # Orchestration: device + transports + poll loop
│   └── bin/server.rs    # CLI entry point
├── miniviz/src/main.rs  # HTTP server that injects WS URL into index.html
├── miniviz/web/         # index.html — Babylon.js 3D scene + control panel
├── dist/                # Pre-built binaries (from CI)
└── docs/
    ├── TODO.md          # Original implementation plan
    └── teleop.md        # Teleop API reference
```

## Usage

```bash
# Listen for WebSocket connections on port 12345
./nolostream_server --ws-listen-at 12345

# Use NoloClientLib.dll instead of direct HID (requires NoloServer.exe running)
./nolostream_server --ws-listen-at 12345 --client-api

# Listen for TCP connections on port 12345
./nolostream_server --tcp-listen-at 12345

# Push to a remote host via UDP
./nolostream_server --udp-stream-to 192.168.1.100:12345

# Connect to a robot's teleop endpoints (per controller, NoloStream is the TCP client)
./nolostream_server --teleop-left-to 192.168.1.100:9001 --teleop-right-to 192.168.1.100:9002

# Combine modes (e.g. WS server + teleop simultaneously)
./nolostream_server --ws-listen-at 12345 --teleop-left-to 127.0.0.1:9001 --teleop-right-to 127.0.0.1:9002
```

Then open miniviz to visualize and act as a dual TeleopTarget:

```bash
# miniviz starts HTTP + teleop TCP listeners and opens your browser
./miniviz --connect ws://127.0.0.1:12345
# (NoloStream should use --teleop-left-to 127.0.0.1:9001 --teleop-right-to 127.0.0.1:9002)
```

## Linux Setup

On Linux, HID devices require either root access or a udev rule to allow user-space access.
Without this, `nolostream_server` can *see* the NoloVR device in enumeration but cannot open it
(you'll see repeated "Found N NoloVR HID interfaces" messages without a successful connection).

Install the included udev rule:

```bash
sudo cp dist/99-nolovr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **unplug and replug** the NoloVR base station (or reboot). After this, any user can access the device.

## Building from Source

Requires Rust toolchain + `libhidapi-dev` and `libudev-dev` (Linux) or a Windows HID driver.

```bash
cargo build --release
```

Binaries will be in `target/release/nolostream_server` and `target/release/miniviz`.
