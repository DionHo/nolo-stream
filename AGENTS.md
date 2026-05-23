# AGENTS.md – AI Agent Guidelines for NoloStream

This file helps AI coding agents be immediately productive in the NoloStream codebase. It captures project-specific conventions, architecture decisions, and recommended workflows.

## Project Overview

**NoloStream** is a Rust library + server that streams 6DOF pose data from a NoloVR headset and controllers to other applications over TCP, UDP, or WebSocket. The core protocol was reverse-engineered from NOLO hardware using HID inspection and comparison against the closed-source NoloClientLib.dll.

**Key characteristics:**
- Multi-platform (Windows, Linux)
- Real-time sensor data streaming with multiple transport backends
- Complex sensor fusion (UKF-based orientation filter with gyro bias calibration)
- Interactive hardware debugging workflows requiring user confirmation
- Reference/ground-truth comparison against NoloClientLib for validation

**Workspace structure:**
- `nolostream/` — core library + server binary
- `miniviz/` — 3D visualization frontend (Babylon.js)
- `test/` — test scripts and analysis tools (PowerShell, Python)
- `docs/` — protocol specs, implementation plans, debug workflows

## Build & Test Commands

```bash
# Full workspace
cargo test --workspace          # Run all tests + clippy checks (CI pipeline)
cargo build --release           # Release binaries for Windows/Linux

# Individual crates
cargo test -p nolostream        # Library unit tests
cargo test -p miniviz           # Miniviz tests
cargo build -p miniviz --release

# With server running (Windows PowerShell)
.\test\debug-session.ps1              # Compare HID vs NoloClientLib output
.\test\capture_sync.ps1 -Duration 60  # Synchronized hardware capture to JSONL

# Linting
cargo clippy -- -D warnings     # Strict lint checks (CI)
```

**First-time setup (Windows):**
1. Install Rust (stable, 2021 edition)
2. Verify HID device: NoloVR headset/controllers must be connected and visible as USB VID=0x0483 PID=0x5750
3. For NoloClientLib testing: ensure `NoloServer.exe` is running and `NoloClientLib.dll` is accessible

## Architecture & Key Decisions

### Core Data Structures

- **`Pose`**: device ID (enum), [f32; 3] position, [f32; 4] quaternion (w,x,y,z), timestamp_ms
  - All public types are serializable (serde JSON)
  - Z-axis quaternion negation applied per nolo-osvr convention

- **`Transport` trait**: `send(&mut self, poses: &[Pose]) -> Result<(), TransportError>`
  - Transports must be `Send` (used in multi-threaded context)
  - Bidirectional: TCP/UDP push and listen, WebSocket listen

- **`NoloStream` core**: aggregates HID polling + multiple transports
  - `poll_once()` collects all active device frames in one call
  - Disconnected transports are auto-removed via `retain_mut`

### Sensor Fusion (UKF)

- Located in `nolostream/src/controller_filter_ukf.rs`
- Known issues documented in [docs/plan-ukf-debug.md](docs/plan-ukf-debug.md):
  - Gravity world-frame assumption (Y-up vs Z-up)
  - Accel/gyro axis mapping to body frame
  - Gyro bias calibration drift when still
  - Quaternion convention alignment with NoloClientLib
- **Validation approach**: synchronized capture + replay loop, compared against NoloClientLib reference

### Transport Backends

1. **TCP Listener** – non-blocking accept loop, JSON+newline to all clients
2. **TCP Stream** – direct TCP connection, push mode
3. **UDP Stream** – connectionless UDP push
4. **WebSocket Listener** – tungstenite (sync), used by miniviz

See [protocol-nolo.md](docs/protocol-nolo.md) for wire format details.

## Interactive Hardware Workflows

**This project includes multi-step hardware debugging sessions that require user confirmation and manual input.** When implementing such workflows (e.g., debug scripts, capture sequences, calibration steps), use the **AskQuestions tool** for:

- **Confirmation before state transitions**: "Ready to start Recording, confirm that hardware is running."
- **User input during captures**: "Keep sensors still for drift and noise calculation (press Enter when ready)"
- **Decision points in analysis**: "The correlation maps are shown in diagram xy.png … what conclusion to make (A/B/C)?"
- **Hardware state validation**: "Move the controller slowly in a circle – confirm motion detected in output"

**Example pattern:**
```rust
// Before starting capture
vscode_askQuestions([
    {
        header: "hardware_ready",
        question: "Is the NoloVR headset connected and tracked?",
        options: [{ label: "Yes" }, { label: "No" }]
    }
]);

// Between phases
vscode_askQuestions([
    {
        header: "still_phase",
        message: "Keep the controller completely still for 30 seconds (calibrating gyro bias).\nPress Enter when complete.",
        question: "Ready?"
    }
]);
```

This ensures multi-step workflows are transparent and traceable, with user confirmation at each step.

## Common Development Patterns

### Sensor Report Parsing

- HID report size: 64 bytes
- Report types: `0xa5` (dual controller), `0xa6` (headset)
- Encryption: BTEA (XXTEA) over bytes 1–60 only; header and padding not encrypted
- Position: i16 big-endian, scale 0.0001 (metres)
- Quaternion: i16 big-endian (W,X,Y,Z), divide by 16384.0

See [protocol-nolo.md](docs/protocol-nolo.md) and `nolostream/src/protocol.rs` for examples.

### NoloClientLib Integration

- FFI wrapper in `nolostream/src/client_api.rs` via `libloading` (runtime DLL load)
- Provides ground truth for sensor validation
- Both HID and ClientLib can be captured simultaneously in [capture_sync workflow](docs/plan-ukf-debug.md#part-a--synchronized-capture-script)

### Testing & Validation

- Unit tests live next to implementation (`#[cfg(test)]` modules)
- Integration tests in `nolostream/tests/` test full transport pipelines
- Comparison analysis via Python (`test/analyze_comparison.py`) for sensor fusion validation
- JSONL format for replay-able capture sessions

## Known Constraints & Pitfalls

1. **Large fixed-size arrays in Rust structs**: Default trait may not derive automatically (see user memory). Provide explicit `Default` impl if needed.

2. **Quaternion convention mismatches**: NoloClientLib uses `{x,y,z,w}` order; Rust implementation uses `[w,x,y,z]`. Always document conversion points.

3. **World frame ambiguity**: Codebase assumes Y-up convention in some places; hardware may differ. See [plan-ukf-debug.md](docs/plan-ukf-debug.md#known-issues-to-diagnose) for ongoing investigation.

4. **Windows-specific features**: NoloClientLib DLL is Windows-only. CI builds work on both Windows and Linux; feature-gate Windows-specific code.

5. **Hardware timing sensitivity**: Capture/replay workflows are time-critical. Use monotonic clocks and document any frame-sync assumptions.

## External References

- [nolo-osvr GitHub](https://github.com/lonetech/nolo-osvr) — original reverse-engineering reference
- [docs/protocol-nolo.md](docs/protocol-nolo.md) — full wire protocol spec
- [docs/plan-ukf-debug.md](docs/plan-ukf-debug.md) — sensor fusion debugging workflow
- [docs/teleop.md](docs/teleop.md) — frame coordinate transformations
- [CLAUDE.md](CLAUDE.md) — behavioral guidelines for AI agents (simplicity, surgical changes, goal-driven execution)

## Suggested Enhancements

For future AI agent customizations, consider creating specialized skills for:
- **Hardware debugging workflows** (`/create-skill hardware-debugging`) — automate capture/replay/compare loops
- **Sensor fusion validation** (`/create-skill ukf-validation`) — guide through quaternion/axis alignment issues
- **Transport layer testing** (`/create-skill transport-testing`) — multi-transport integration test patterns
