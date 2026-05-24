# Plan: GUI, Robustness & Architecture Refactor

**Scope:** Three interrelated improvements to `nolostream_server`:
1. egui-based GUI replacing the mandatory CLI flags
2. Retry/reconnect loop (HID + transports + startup wait)
3. Clean isolation of `client-api` feature to ease future removal

---

## 1. Current Architecture (as-is)

```
nolostream_server (bin)
├── clap Args  ──►  build_transports()  ──►  NoloStream::add_transport()
├── NoloDevice::open()  (fails hard on not found)
├── polling loop  (no reconnect, exits on first HID error after setup)
└── client-api feature
    └── NoloClientApi  (FFI to NoloClientLib.dll)
        └── Mixed into server main() via cfg(feature) blocks
```

**Problems:**
- User must pass CLI flags; no discovery or defaults
- Any HID disconnect crashes the server
- `client-api` FFI code mixed into `server.rs` main() logic via `#[cfg]` guards — hard to delete later
- No GUI; configuration requires terminal knowledge

---

## 2. Target Architecture

```
nolostream_server (bin)
├── mode: CLI (--no-ui + explicit flags, existing behaviour)
│   └── RunConfig built from clap Args
│
└── mode: GUI (default when no transport flags given)
    └── egui window
        ├── DeviceStatus panel  (searching / connected / error)
        ├── Transport panel  (TCP listen / WS listen / TCP push / UDP push)
        ├── Teleop panel  (enable toggle, target address)
        └── Log panel  (recent events)
        └── RunConfig built from GUI state

RunConfig
└── drives AppState (shared Arc<Mutex<AppState>>)
    ├── worker thread: HID + transport loop with reconnect
    └── GUI thread:    reads AppState for display, writes config changes

nolostream (lib)
├── NoloStream  (unchanged public API)
├── HID backend (hidapi, always compiled)
└── client-api feature (feature-gated, zero coupling to lib internals)
    └── NoloClientApi lives entirely in client_api.rs
    └── No references from NoloStream struct or nolostream.rs
```

---

## 3. Implementation Plan

### Phase A — Decouple client-api from core (prerequisite, low risk)

**Current issue:** `server.rs` contains `#[cfg(feature = "client-api")]` blocks that duplicate
the polling loop. The lib itself (`lib.rs`, `nolostream.rs`) already only references `client_api`
through the feature gate, but `server.rs` main() has a long `cfg`-guarded code path that mirrors
the HID path.

**Change:**
- Extract the client-api polling path in `server.rs` into a helper function
  `run_client_api_loop(config, transports)` in a new file
  `nolostream/src/bin/client_api_runner.rs` (also `#[cfg(feature = "client-api")]`).
- `server.rs` main() calls it as a single branch, keeping the else branch as the HID path.
- Result: deleting the `client-api` feature later requires removing one file and one `if` branch —
  no grep-and-pray surgery.

**Files changed:** `nolostream/src/bin/server.rs`, new `nolostream/src/bin/client_api_runner.rs`

---

### Phase B — Retry/reconnect loop in NoloStream

**Current issue:** `NoloStream::new()` returns `Err` on device-not-found and the server exits.
`poll_once()` returns `Err(HidError)` on read failure and the server exits.

**Change:**
- Add `NoloStream::try_open_with_retry(interval: Duration) -> NoloStream` — blocks until a device
  is found, printing status every `interval`. The GUI can call this on a background thread and
  update a status flag.
- Add reconnect logic inside `poll_once()`:
  - On `HidError` (read failure), set `self.device = None` and return `Ok((vec![], vec![]))`.
  - In the polling loop (in the worker thread), if `self.device.is_none()`, attempt
    `NoloDevice::open()` once. If it fails, sleep `interval` and continue.
- Transport reconnect: already partially handled (`retain_mut` removes `Disconnected` transports).
  Extend: `TcpStreamTransport` and `WsListenerTransport` track `needs_reconnect` flag; polling
  loop re-adds them from the saved `RunConfig` when `needs_reconnect` is set.

**Files changed:** `nolostream/src/nolostream.rs`, `nolostream/src/hid.rs`,
`nolostream/src/transports/tcp_stream.rs`, `nolostream/src/transports/ws_listener.rs`

---

### Phase C — RunConfig struct (shared between CLI and GUI)

**New struct** in `nolostream/src/bin/run_config.rs`:

```rust
pub struct TransportConfig {
    pub tcp_listen_port: Option<u16>,
    pub ws_listen_port:  Option<u16>,
    pub tcp_stream_to:   Option<SocketAddr>,
    pub udp_stream_to:   Option<SocketAddr>,
}

pub struct RunConfig {
    pub transport: TransportConfig,
    pub gyro_scale: f32,
    pub debug: bool,
    pub csv_log: Option<PathBuf>,
    // teleop
    pub teleop_enabled: bool,
    pub teleop_target: Option<SocketAddr>,
}
```

Both CLI and GUI build a `RunConfig`. The worker thread reads it (via `Arc<Mutex<RunConfig>>`),
rebuilding transports when the config changes (version counter or `dirty` flag).

---

### Phase D — GUI binary mode (egui)

Add `eframe`/`egui` as an optional dependency gated on a `gui` feature flag.

```toml
[features]
gui = ["eframe"]

[dependencies]
eframe = { version = "0.27", optional = true }
```

**Startup logic in `main()`:**
```
if no transport flags given && !--no-ui:
    run_gui()   // blocks until window closed
else:
    run_headless(config)
```

**GUI layout (single window, ~600×500px):**

```
┌─────────────────────────────────────────────┐
│  NoloStream                    [●] Connected │
├──────────────┬──────────────────────────────┤
│ DEVICE       │ TRANSPORTS                   │
│              │  [✓] TCP listen  port: [8123] │
│ Status: OK   │  [✓] WS listen   port: [8765] │
│ VID/PID      │  [ ] TCP push    addr: [    ] │
│ 0483:5750    │  [ ] UDP push    addr: [    ] │
│              │                              │
├──────────────┼──────────────────────────────┤
│ TELEOP       │ LOG                          │
│ [✓] Enabled  │ [13:42:01] HID connected     │
│ target: [  ] │ [13:42:02] WS client joined  │
│              │ [13:42:15] HID read error     │
│              │ [13:42:16] Reconnecting...   │
└──────────────┴──────────────────────────────┘
```

- Changes to checkboxes/fields take effect immediately (worker thread detects dirty flag).
- Log panel shows last N lines from a ring buffer shared via `Arc<Mutex<VecDeque<String>>>`.
- Device status line shows spinner while searching, green dot when connected, red on error.

**Worker thread:**
```
loop {
    // reconnect HID if needed (1s retry)
    // if config dirty: rebuild transports
    // poll_once()
    // sleep ~2ms
}
```

---

### Phase E — Testing

1. **Unit tests (no hardware):** Mock `NoloDevice` returning errors to validate retry logic.
2. **Build test:** `cargo build --features gui` (Windows only; eframe is Windows/Linux/macOS).
3. **Integration:** Run headless mode (`--ws-listen-at 8765 --no-ui`) and verify existing WebSocket
   transport test still passes.
4. **Manual GUI test:** Launch without flags, confirm GUI appears, enable WS on 8765, connect
   miniviz.

---

## 4. File Change Summary

| File | Change |
|---|---|
| `nolostream/Cargo.toml` | Add `gui = ["eframe"]` feature; add `eframe` dep |
| `nolostream/src/nolostream.rs` | Add retry-on-disconnect logic inside `poll_once` |
| `nolostream/src/hid.rs` | `NoloDevice` open result becomes `Option` for hot-unplug |
| `nolostream/src/bin/server.rs` | `main()` selects GUI vs headless; defines `RunConfig` |
| `nolostream/src/bin/run_config.rs` | New: `RunConfig`, `TransportConfig` structs |
| `nolostream/src/bin/gui.rs` | New: egui app, `AppState`, worker thread |
| `nolostream/src/bin/client_api_runner.rs` | New: isolated client-api polling path |

**Unchanged:** library public API (`lib.rs`, `transport.rs`, all transport impls), miniviz, tests.

---

## 5. Removal Path for client-api (future)

When `NoloClientLib` is no longer needed:
1. `cargo rm libloading` (or remove from `[target.cfg(windows).dependencies]`)
2. Delete `nolostream/src/client_api.rs`
3. Delete `nolostream/src/bin/client_api_runner.rs`
4. Remove one `if args.client_api { … }` branch in `server.rs`
5. Remove `client-api` feature from `Cargo.toml`
6. Remove `#[cfg(feature = "client-api")]` lines in `lib.rs` and `server.rs`

All removals are localised — no library core changes needed.

---

## 6. Decisions & Rationale

| Decision | Rationale |
|---|---|
| GUI in same binary (`--no-ui` for headless) | Avoids a third crate; CI/deploy stays simple |
| `gui` Cargo feature for egui | eframe pulls ~30 deps; keep them optional |
| `Arc<Mutex<AppState>>` over channels | Simpler for bidirectional GUI↔worker; low throughput |
| Retry in polling loop, not in `NoloStream::new()` | Keeps library API simple; retry policy is application concern |
| client-api stays feature-flagged | Removal is a future one-liner; no urgency |
