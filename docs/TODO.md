# NoloStream – Implementation Plan

**Target:** Rust library + server binary that streams NoloVR pose data over TCP / UDP / WebSocket.  
**Platforms:** Windows x64, Linux x64  
**Wire format:** JSON (newline-delimited)  
**Miniviz:** Rust binary serving an embedded HTML+Babylon.js page; the JS connects directly to nolostream_server via WebSocket.

---

## Phase 1 – Project Scaffolding ✅

- [x] Create root `Cargo.toml` (Cargo workspace with members: `nolostream`, `miniviz`)
- [x] Create `nolostream/Cargo.toml` (lib crate `nolostream` + `[[bin]] nolostream_server`)
- [x] Create `miniviz/Cargo.toml` (bin crate `miniviz`)
- [x] Add `.gitignore` entries for `target/`, `dist/nolostream_server`, `dist/miniviz`, `dist/*.exe`
- [x] Verify: `cargo check --workspace` succeeds with empty stubs

## Phase 2 – Core Library (`nolostream/`) ✅

### 2a. Pose Data Structures ✅
- [x] Define `Pose` struct: `device: DeviceId`, `position: [f32; 3]`, `orientation: [f32; 4]` (quaternion w,x,y,z), `timestamp_ms: u64`
- [x] Derive `serde::Serialize / Deserialize` on all public types; DeviceId uses snake_case
- [x] Unit-test round-trip JSON serialization (3 tests)

### 2b. NoloVR HID Driver ✅
- [x] BTEA decryption (wrapping arithmetic, key `[0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]`)
- [x] HID device discovery by USB VID=0x0483 PID=0x5750 (`hidapi` crate)
- [x] Protocol parser for report 0xa5 (dual controller) and 0xa6 (headset)
- [x] Z-axis quaternion negation per nolo-osvr convention
- [x] Timestamp stamped at poll time (not inside parser)
- [x] Unit-test parser with synthetic HID byte sequences

### 2c. NoloStream Core Struct ✅
- [x] `Transport` trait (`send(&mut self, poses: &[Pose]) -> Result<(), TransportError>`), `Send` bound
- [x] `NoloStream` struct: `new()`, `add_transport()`, `poll_once()` with `retain_mut` lifecycle
- [x] `poll_once()` removes Disconnected transports, logs Io errors, returns poses

## Phase 3 – Transport Layer (`nolostream/src/transports/`) ✅

- [x] **TCP Listener**: non-blocking accept loop, JSON+newline broadcast to all clients
- [x] **TCP Stream-to**: lazy connect, 1s connect timeout, 500ms write timeout, silent reconnect
- [x] **UDP Stream-to**: `bind("0.0.0.0:0")`, fire-and-forget `send_to`
- [x] **WebSocket Listener**: non-blocking listener, 50ms read timeout on handshake, `tungstenite::Message::Text`
- [x] Integration tests: TCP listener + UDP round-trip parse (2 tests)

## Phase 4 – Server Binary (`nolostream/src/bin/server.rs`) ✅

- [x] CLI with `clap ~4.4`: `--tcp-listen-at`, `--ws-listen-at`, `--tcp-stream-to`, `--udp-stream-to`
- [x] Validate: at least one flag provided (exit(1) with error message)
- [x] Transport bind failures exit(1) with error message
- [x] Polling loop, stderr status messages

## Phase 5 – Miniviz (`miniviz/`) ✅

- [x] CLI: `--connect <ws-url>` (e.g. `ws://127.0.0.1:12345`)
- [x] Embed web assets via `include_str!("../web/index.html")`
- [x] Serve on ephemeral port via `tiny_http`, pass URL as `?ws=` query param
- [x] Open browser automatically (`open` crate)
- [x] `miniviz/web/index.html`: Babylon.js (CDN), ArcRotateCamera, dark background, grid floor
- [x] Three meshes: white box (HMD), blue box (left), red box (right)
- [x] Quaternion mapping: `[w,x,y,z]` → `Babylon.Quaternion(x,y,z,w)`
- [x] Text overlay labels (HMD / L / R), connection state indicator

## Phase 6 – Dev Container (`.devcontainer/`) ✅

- [x] `devcontainer.json` using `mcr.microsoft.com/devcontainers/rust:1-bullseye`
- [x] `postCreateCommand`: install `libhidapi-dev libudev-dev pkg-config`
- [x] VS Code extensions: `rust-lang.rust-analyzer`, `vadimcn.vscode-lldb`

## Phase 7 – CI / CD (`.github/workflows/`) ✅

- [x] `build.yml` (push + PR to main):
  - `test` job: `cargo test --workspace` + `cargo clippy -- -D warnings` on ubuntu-latest
  - `build-linux` job (push only, after test): `cargo build --release`, upload `nolostream_server` + `miniviz` as `linux-binaries` artifact (14-day retention)
  - `build-windows` job (push only, after test): same on windows-latest, uploads `*.exe` as `windows-binaries` artifact
- [x] `release.yml` (on `v*` tag): build Linux+Windows, rename binaries, create GitHub Release with all four attached; `permissions: contents: write`
- [x] Cargo cache in all jobs

## Phase 8 – Distribution (`dist/`) ✅

- [x] `dist/.gitkeep` — directory tracked
- [x] `.gitignore` excludes actual binaries but not `.gitkeep`
- [x] README already documents `dist/` usage

---

## Key Dependencies (as implemented)

| Crate | Purpose |
|---|---|
| `serde` + `serde_json` | JSON serialization |
| `hidapi` (linux-static-hidraw) | USB HID communication |
| `tungstenite` | WebSocket sync (no async needed) |
| `clap ~4.4` (derive) | CLI argument parsing |
| `tiny_http` | HTTP server for miniviz |
| `open` | Open browser from Rust |
| `urlencoding` | URL-encode WS URL for query param |

---

## Reference

- Protocol reverse-engineering: https://github.com/lonetech/nolo-osvr  
- NoloVR USB VID=0x0483, PID=0x5750, BTEA key confirmed from nolo-osvr source
