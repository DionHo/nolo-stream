# NoloStream

Stream 6DOF pose data from a NoloVR headset and controllers to other applications over TCP, UDP, or WebSocket. Core device protocol reverse-engineered from [lonetech/nolo-osvr](https://github.com/lonetech/nolo-osvr).

## Status

All features implemented and tested:
- HID device discovery, BTEA decryption, and report parsing (headset + dual controllers)
- Four transport modes: TCP listen, TCP push, UDP push, WebSocket listen
- Miniviz: 3D Babylon.js viewer that connects to the WebSocket server

## Tech Stack

| Layer | Technology |
|---|---|
| Language | Rust (2021 edition) |
| HID / USB | `hidapi` 2 (hidraw backend on Linux) |
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

Each message is a **newline-terminated JSON array** containing one or more pose objects:

```json
[
  {
    "device": "headset",
    "position": [0.0, 1.6, 0.0],
    "orientation": [1.0, 0.0, 0.0, 0.0],
    "timestamp_ms": 1715702400000
  }
]
```

| Field | Type | Description |
|---|---|---|
| `device` | `"headset"` \| `"left_controller"` \| `"right_controller"` | Source device |
| `position` | `[x, y, z]` (f32, metres) | World-space position |
| `orientation` | `[w, x, y, z]` (f32, unit quaternion) | World-space rotation |
| `timestamp_ms` | u64 | Host time at poll, ms since UNIX epoch |

A single HID poll yields a `0xa5` report (2 poses) and/or a `0xa6` report (1 pose). All poses from one poll are batched into a single JSON array before transmission.

## Project Layout

```
NoloStream/
├── nolostream/src/
│   ├── btea.rs          # BTEA/XXTEA decryption
│   ├── protocol.rs      # HID report parser (0xa5, 0xa6)
│   ├── hid.rs           # Device open + read loop
│   ├── pose.rs          # Pose struct + DeviceId enum
│   ├── transport.rs     # Transport trait
│   ├── transports/      # tcp_listener, tcp_stream, udp_stream, ws_listener
│   ├── nolostream.rs    # Orchestration: device + transports + poll loop
│   └── bin/server.rs    # CLI entry point
├── miniviz/src/main.rs  # HTTP server that injects WS URL into index.html
├── miniviz/web/         # index.html — Babylon.js 3D scene
├── dist/                # Pre-built binaries (from CI)
└── docs/TODO.md         # Original implementation plan
```

## Usage

```bash
# Listen for WebSocket connections on port 12345
./nolostream_server --ws-listen-at 12345

# Listen for TCP connections on port 12345
./nolostream_server --tcp-listen-at 12345

# Push to a remote host via TCP or UDP
./nolostream_server --tcp-stream-to 192.168.1.100:12345
./nolostream_server --udp-stream-to 192.168.1.100:12345

# Combine modes (e.g. WS server + UDP push simultaneously)
./nolostream_server --ws-listen-at 12345 --udp-stream-to 192.168.1.100:9000
```

Then open miniviz to visualize in a browser:

```bash
# miniviz starts an HTTP server and opens your browser automatically
./miniviz --connect ws://127.0.0.1:12345
```

## Building from Source

Requires Rust toolchain + `libhidapi-dev` and `libudev-dev` (Linux) or a Windows HID driver.

```bash
cargo build --release
```

Binaries will be in `target/release/nolostream_server` and `target/release/miniviz`.
