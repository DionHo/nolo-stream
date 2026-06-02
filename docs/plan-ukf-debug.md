# Plan: UKF Debug & Fix Workflow

**Goal:** Fix the orientation filter (UKF) in `nolostream/src/controller_filter_ukf.rs` by
building a capture→replay→compare loop that uses NoloClientLib as ground truth.
The recording is done once; replay + analysis are re-run after each Rust code change — no
new hardware session needed per iteration.

---

## Known Issues to Diagnose

1. **Gravity / world-frame convention wrong** — `gravity_world()` assumes Y-up; may need to
   match NoloClientLib's actual world frame (could be Z-up, right-handed vs left-handed).
2. **Accel/gyro axes not mapped to body frame** — `ControllerReport::from_decrypted` field
   offsets / sign conventions for `acceleration` and `angular_velocity` may be wrong.
3. **UKF drifts when still** — bias calibration not converging; gyro bias estimate is off.
4. **HMD quaternion wrong in HID path** — `qscale`, field order, and possible axis negation
   are all unknown for current firmware; must be discovered by comparison.
5. **Possible quaternion convention mismatch** — `NoloClientLib` stores `NQuaternion` as
   `{x,y,z,w}`; Rust UKF output is `[w,x,y,z]`. Any alignment error in the comparison
   layer would corrupt results.

---

## Infrastructure Already in Place

| File | What it provides |
|---|---|
| `nolostream/src/client_api.rs` | `NoloDataRaw` FFI, ZMQ callback integration |
| `nolostream/src/csv_log.rs` | CSV logger (processed state + 64 raw HID bytes) |
| `nolostream/src/protocol.rs` | `generate_report_with_raw` — decrypt + parse, returns both |
| `nolostream/src/controller_filter_ukf.rs` | Full multiplicative UKF |
| `nolostream/src/controller_report.rs` | `ControllerReport` with `sensor_raw` ([i16; 32]) |
| `test/debug-session-api-compare.ps1` | Two-server compare (uses processed HID, not raw) |
| `test/analyze_comparison.py` | Axis correlation + button-byte analysis |

The new plan extends this by:
- Recording **raw HID bytes + NoloClientLib frames** in a single JSONL file (one capture per
  hardware session, reused across all replay iterations).
- Adding a **`nolostream_replay`** Rust binary that re-runs the full pipeline on recorded bytes.
- Adding a **Python analysis tool** that compares replay output to reference and produces
  actionable diagnostics.

---

## Part A — Synchronized Capture Script

**File:** `test/capture_sync.ps1`

### What it does

Runs a single `nolostream_server` instance with a new `--capture-jsonl <path>` flag that
opens **both** the HID device and the NoloClientLib simultaneously, writing all frames to one
JSONL file. Because both sources live in the same process, their timestamps share the same
monotonic clock — eliminating the alignment error present in the two-process approach.

### Script behaviour

```
.\test\capture_sync.ps1 [-Out <path>] [-Duration <seconds>] [-NoloServerPath <path>] [-NoBuild]
```

1. Kill stale `nolostream_server` / `NoloServer` processes.
2. Copy `NoloClientLib.dll` + `libzmq-64.dll` next to the release binary (same as existing script).
3. `cargo build --release` (skippable with `-NoBuild`).
4. Start `NoloServer.exe` and wait 2 s.
5. Start `nolostream_server --capture-jsonl <path>` — this new mode opens both sources.
6. Tail stderr for status messages; stop after `-Duration` seconds or Ctrl-C.
7. Print path to the JSONL file.

### Expected recording session

Recommended capture sequence (annotate manually in the JSONL via a companion annotation file or
by pressing a key that emits a marker record):

| Phase | Duration | Purpose |
|---|---|---|
| **Standstill A** | 30 s | Gyro bias + noise baseline |
| **Slow rotation** yaw/pitch/roll | 60 s | Axis mapping validation |
| **Fast motion** (shake, tilt) | 30 s | Dynamic response |
| **Standstill B** | 30 s | Drift quantification post-motion |
| **Button presses** | 20 s | (Optional) button/touch alignment |

---

## Part B — Server Capture Mode (Rust)

**Changes to `nolostream/src/bin/server.rs` and `nolostream/src/nolostream.rs`:**

### New CLI flag

```
--capture-jsonl <path>   Write raw HID bytes + client-api frames to JSONL; implies --client-api.
```

### JSONL record schema

Two interleaved record types, one JSON object per line:

```jsonc
// Raw HID frame (before decryption)
{
  "type": "hid_raw",
  "ts_ms": 1234567890,
  "raw": [165, 0, 3, ...]   // 64 bytes as u8 array
}

// NoloClientLib frame
{
  "type": "client_api",
  "ts_ms": 1234567891,
  "left":  { "pos": [x,y,z], "rot": [x,y,z,w], "buttons": 0, "touch": [x,y], "battery": 80,
             "state": 0 },
  "right": { ... same fields ... },
  "hmd":   { "pos": [x,y,z], "rot": [x,y,z,w], "state": 0 },
  "sensor":{ "l_vel": [x,y,z], "l_ang_vel": [x,y,z],
             "r_vel": [x,y,z], "r_ang_vel": [x,y,z],
             "h_vel": [x,y,z], "h_ang_vel": [x,y,z] }
}
```

**Implementation sketch:**

- Add `pub fn run_capture(jsonl_path: &Path) -> Result<(), Box<dyn Error>>` in a new module
  `nolostream/src/capture.rs`.
- Open HID device and `NoloClientApi` in the same thread.
- Main loop at ~60 Hz:
  - Poll HID via `device.poll_with_raw()` → if `Some(raw)`, write `hid_raw` record.
  - Read latest client_api data via callback flag (`DATA_READY` atomic) → if fresh, write
    `client_api` record.
- All timestamps: `SystemTime::now()` in milliseconds (same clock, same process).
- Flush after every write to avoid data loss on Ctrl-C.

> **Note on threading:** The ZMQ callback fires on its own thread; the existing atomic
> `DATA_READY`/`NOLO_BYTES` pair already handles this safely. No additional locking needed.

---

## Part C — Replay Binary (`nolostream_replay`)

**New file:** `nolostream/src/bin/replay.rs`  
**New entry in `nolostream/Cargo.toml`:** `[[bin]] name = "nolostream_replay"`

### Purpose

Read a JSONL capture file, re-run the current Rust protocol pipeline on every `hid_raw` frame,
and emit a JSONL result file that Python can analyse. Because no hardware is needed, this can be
re-run instantly after each code change.

### CLI

```
nolostream_replay --input <capture.jsonl> --output <replay.jsonl>
                  [--init-from-client-api]   # seed UKF q_hat from first client_api frame
                  [--device left|right|both] # default: both
                  [--emit-covariance]         # include 9x9 P matrix in output (verbose)
```

### Processing pipeline per `hid_raw` frame

```
raw[64] → decrypt_report() → ControllerReport::from_decrypted()
        → ControllerFilterUkf::filter() → ControllerState
```

### Output JSONL schema

```jsonc
// Computed (Rust pipeline)
{
  "type": "computed",
  "ts_ms": 1234567890,
  "device": "left_controller",
  "pos":  [x,y,z],
  "rot":  [w,x,y,z],          // UKF orientation estimate
  "vel":  [x,y,z],
  "ang_vel": [x,y,z],
  "bias": [x,y,z],             // UKF gyro bias estimate
  "p_trace": 0.123,            // tr(P) for convergence monitoring
  "p_full": [[...9x9...]]      // only if --emit-covariance
}

// Reference (pass-through from capture)
{
  "type": "reference",
  "ts_ms": 1234567891,
  "device": "left_controller",
  "pos":  [x,y,z],
  "rot":  [x,y,z,w],           // NQuaternion order from NoloClientLib
  "vel":  [x,y,z],
  "ang_vel": [x,y,z]
}
```

### `--init-from-client-api`

When set, the replay binary waits for the first `client_api` frame before processing `hid_raw`
frames. It uses that frame to seed `ControllerFilterUkf.q_hat` with the reference orientation
(converting from NoloClientLib's `{x,y,z,w}` order to nalgebra's `UnitQuaternion`). This
removes initial transient error from the comparison so axis mapping issues are visible clearly.

---

## Part D — Python Analysis Tool

**File:** `test/analyze_ukf.py`

### CLI

```
python test/analyze_ukf.py <replay.jsonl>
        [--out-dir test/analysis]
        [--device left|right|both]          # default: left (adjust once we know which works)
        [--standstill-ts 0-30,120-150]      # manual override: comma-separated ms ranges
        [--standstill-gyro-thresh 0.05]     # rad/s RMS threshold for auto-detection
        [--standstill-pos-thresh 0.005]     # m stddev threshold for auto-detection
```

### Step 1 — Load & separate records

Load JSONL into two Pandas DataFrames: `computed` and `reference`.  
Align on nearest timestamp within 50 ms (`pd.merge_asof`).

### Step 2 — Standstill detection

Auto-detect from `reference` data (NoloClientLib, most reliable source):
- Rolling window (1 s, 50% overlap) over `ang_vel` RMS and `pos` stddev.
- Mark windows where both are below threshold.
- Merge contiguous windows with gap < 2 s.
- Report detected windows to stdout; allow manual override via `--standstill-ts`.

### Step 3 — Noise & drift estimation (standstill sections)

From **standstill** windows:

| Metric | From computed | From reference |
|---|---|---|
| Gyro bias | `bias` field mean ± std | `ang_vel` mean ± std |
| Orientation drift | rate of change of quaternion error angle | reference ang_vel RMS |
| Position drift | linear fit slope on `pos` | — |

Print a table and write to `analysis_report.txt`.

### Step 4 — Axis correlation (full recording)

For each axis pair `(computed.pos_X, reference.pos_Y)` for X, Y ∈ {x, y, z}:
- Pearson correlation coefficient and its negated counterpart.
- Best mapping (e.g. `computed_x → +reference_z`) reported as a 3×3 table.

Repeat for velocity and angular velocity.

### Step 5 — Orientation accuracy

Convert both quaternions to the same convention (`[w,x,y,z]` Hamilton).  
Per timestep, compute quaternion error:

$$e = q_{\text{ref}} \otimes q_{\text{computed}}^{-1}$$

Extract angle $\theta = 2 \arccos(|e_w|)$. Plot as time series and histogram.

### Step 6 — Plots (saved to `--out-dir`)

| Filename | Content |
|---|---|
| `pos_correlation.png` | 3×3 scatter matrix: computed vs reference position axes |
| `orientation_error.png` | Quaternion error angle over time (full + zoomed standstill) |
| `bias_drift.png` | UKF bias estimate over time |
| `p_trace.png` | `p_trace` over time (convergence indicator) |
| `angular_velocity_corr.png` | Computed vs reference angular velocity, per axis |
| `velocity_corr.png` | Computed vs reference linear velocity, per axis |
| `rmse_table.png` | RMSE table (position, orientation angle, velocity) per axis |

### Step 7 — Report

Write `analysis_report.txt`:
- Standstill windows used.
- Noise / drift estimates.
- RMSE per signal / axis.
- Top-3 axis mapping candidates for position, velocity, angular velocity.
- Flagged issues (e.g. "orientation error > 30° even in standstill → gravity frame mismatch likely").

---

## Part E — Iterative Fix Workflow

Each iteration follows this loop:

```
Record  →  Replay  →  Analyse  →  Read report  →  Fix Rust code  →  Replay again (no new record)
```

The Rust code changes in each iteration are guided by the analysis output.  
No new hardware session is needed until the current capture is exhausted or a structural
assumption changes (e.g. a new firmware).

### Suggested fix order

1. **HMD quaternion** — verify `qscale` (currently `1/16384`) and field order against the
   reference. Fix the obvious ones first; the HMD is often still while controllers move.
2. **Accel/gyro axis mapping** — use axis correlation table (Part D §4) to determine which
   HID sensor fields map to which body-frame axes and whether any need negation.
3. **Gravity vector** — once accel axes are correct, verify the gravity measurement in standstill
   (`accel` should equal `gravity_world()` in magnitude and direction); update `gravity_world()`
   if needed.
4. **Bias calibration** — use standstill section noise estimates to set tighter `CAL_MAX_WAIT`
   and threshold values; check bias estimate vs reference `ang_vel` mean.
5. **UKF noise parameters** (`Q_GYRO`, `Q_BIAS`, `Q_VEL`, `R_ACCEL`, `R_VEL`) — once geometry
   is correct, tune using standstill variance and dynamic RMSE as objective.

### AskQuestions checkpoints

Use the `AskQuestions` tool at these points:

- **After first analysis run:** present the axis correlation table and ask whether the suggested
  mapping matches physical expectations.
- **After each fix:** confirm whether the orientation error trend improved before proceeding to
  the next issue.
- **When `gravity_world()` ambiguity cannot be resolved from data alone:** ask the user to
  hold the controller flat on a table and describe which face points up.
- **Before changing quaternion conventions:** confirm current convention assumptions to avoid
  double-fix confusion.

---

## File Locations Summary

| File | Role |
|---|---|
| `test/capture_sync.ps1` | **New** — synchronized capture script |
| `nolostream/src/capture.rs` | **New** — dual-source capture loop |
| `nolostream/src/bin/replay.rs` | **New** — replay binary entry point |
| `test/analyze_ukf.py` | **New** — full analysis + plots |
| `nolostream/src/bin/server.rs` | **Modified** — add `--capture-jsonl` flag |
| `nolostream/Cargo.toml` | **Modified** — add `[[bin]] nolostream_replay` |

---

## Success Criteria

- [ ] `capture_sync.ps1` produces a JSONL file with both `hid_raw` and `client_api` records
  interleaved, timestamp monotonically increasing.
- [ ] `nolostream_replay` processes the entire JSONL and emits a result file; no panics.
- [ ] `analyze_ukf.py` runs without errors and produces all 7 plots + report.
- [ ] Quaternion error angle < 5° RMS in a standstill section (post-fix target).
- [ ] Axis correlation for position: each computed axis has |r| > 0.9 with exactly one reference
  axis.
- [ ] Bias estimate converges within 10 s of standstill (visible in `bias_drift.png`).
