# NoloStream – Implementation Plan

**Target:** Rust library + server binary that streams NoloVR pose data over TCP / UDP / WebSocket.  
**Platforms:** Windows x64, Linux x64  
**Wire format:** JSON (newline-delimited)  
**Miniviz:** Rust binary serving an embedded HTML+Babylon.js page; the JS connects directly to nolostream_server via WebSocket.

---

## Phase 1 – Project Scaffolding

- [ ] Create root `Cargo.toml` (Cargo workspace with members: `src`, `miniviz`)
- [ ] Create `src/Cargo.toml` (lib crate `nolostream` + `[[bin]] nolostream_server`)
- [ ] Create `miniviz/Cargo.toml` (bin crate `miniviz`)
- [ ] Add `.gitignore` entries for `target/`, `dist/*.exe`, `dist/*.elf`
- [ ] Verify: `cargo check --workspace` succeeds with empty stubs

## Phase 2 – Core Library (`src/`)

### 2a. Pose Data Structures
- [ ] Define `Pose` struct: `device_id: u8`, `position: [f32; 3]`, `orientation: [f32; 4]` (quaternion), `timestamp_ms: u64`
- [ ] Derive `serde::Serialize / Deserialize` on all public types
- [ ] Unit-test round-trip JSON serialization

### 2b. NoloVR HID Driver
- [ ] Add `hidapi` crate dependency
- [ ] Implement device discovery: find NoloVR base station by USB vendor/product ID (reference: lonetech/nolo-osvr)
- [ ] Implement HID report parsing → `Pose` values for HMD, left controller, right controller
- [ ] Unit-test parser against captured raw HID byte sequences

### 2c. NoloStream Core Struct
- [ ] Define `NoloStream` struct holding HID handle + list of active transports
- [ ] Implement `NoloStream::new()` and `NoloStream::poll() -> Result<Vec<Pose>>`
- [ ] Implement `NoloStream::add_transport(Transport)` for runtime configuration

## Phase 3 – Transport Layer (`src/transport/`)

Each transport receives a `Vec<Pose>` and serializes it as a newline-terminated JSON object.

- [ ] **TCP Listener** (`--tcp-listen-at <port>`): bind port, accept clients into a shared list, broadcast each poll result to all connected clients
- [ ] **TCP Stream-to** (`--tcp-stream-to <addr:port>`): maintain outbound TCP connection, reconnect on drop, stream pose JSON
- [ ] **UDP Stream-to** (`--udp-stream-to <addr:port>`): fire-and-forget UDP datagrams (one JSON line per datagram)
- [ ] **WebSocket Listener** (`--ws-listen-at <port>`): upgrade HTTP to WS (use `tokio-tungstenite` + `axum`), broadcast pose JSON to all connected WS clients
- [ ] Integration-test each transport: server sends N pose frames, client verifies receipt and round-trip parse

## Phase 4 – Server Binary (`src/bin/server.rs`)

- [ ] CLI parsing with `clap` — flags: `--tcp-listen-at`, `--ws-listen-at`, `--tcp-stream-to`, `--udp-stream-to` (all optional, at least one required)
- [ ] Validate: at least one flag provided, addresses parse correctly
- [ ] Spin up requested transports, instantiate `NoloStream`, enter polling loop
- [ ] Graceful shutdown on Ctrl-C (flush buffers, close sockets)
- [ ] Smoke-test: start server with `--ws-listen-at 19000`, connect a WS client, verify JSON frames arrive

## Phase 5 – Miniviz (`miniviz/`)

- [ ] CLI: `--connect <ws-url>` (e.g. `ws://127.0.0.1:12345`)
- [ ] Embed web assets (HTML + JS) via `include_str!` at compile time
- [ ] Serve embedded assets on a random free localhost HTTP port via `axum`
- [ ] Pass `--connect` URL as a query param or injected JS variable to the page
- [ ] Open browser automatically (`open` crate)
- [ ] Web page (`miniviz/web/index.html`):
  - [ ] Connect to nolostream_server WebSocket using the injected URL
  - [ ] Parse incoming JSON pose frames
  - [ ] Render HMD + two controllers as 3D meshes using Babylon.js (CDN)
  - [ ] Update mesh position/rotation each frame
  - [ ] Display device label overlay (HMD / L / R)

## Phase 6 – Dev Container (`.devcontainer/`)

- [ ] `devcontainer.json` based on `mcr.microsoft.com/devcontainers/rust:latest`
- [ ] Install `libhidapi-dev`, `libudev-dev`, `pkg-config` (Linux HID dependencies)
- [ ] Install `mingw-w64` toolchain for Windows cross-compilation target (`x86_64-pc-windows-gnu`)
- [ ] Add VS Code extensions: `rust-analyzer`, `CodeLLDB`

## Phase 7 – CI / CD (`.github/`)

- [ ] `build.yml`: on push/PR — `cargo test --workspace`, `cargo clippy --workspace -- -D warnings`
- [ ] `release.yml`: on tag `v*` — cross-compile for `x86_64-unknown-linux-gnu` and `x86_64-pc-windows-gnu`, upload binaries to GitHub Release and copy to `dist/`
- [ ] Cache Rust target dir in both workflows

## Phase 8 – Distribution (`dist/`)

- [ ] Document expected `dist/` contents in README (already partially done)
- [ ] Add `dist/.gitkeep` so the directory is tracked
- [ ] Verify release workflow produces `nolostream_server` (Linux + Windows) and `miniviz` (Linux + Windows)

---

## Key Dependencies

| Crate | Purpose |
|---|---|
| `tokio` (full) | Async runtime |
| `serde` + `serde_json` | JSON serialization |
| `hidapi` | USB HID communication |
| `clap` | CLI argument parsing |
| `tokio-tungstenite` | WebSocket (server + client) |
| `axum` | HTTP server (WS upgrade, miniviz asset serving) |
| `open` | Open browser from Rust |

---

## Reference

- Protocol reverse-engineering: https://github.com/lonetech/nolo-osvr  
- NoloVR USB VID/PID: to be confirmed from nolo-osvr source
