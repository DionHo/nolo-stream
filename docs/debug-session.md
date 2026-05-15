# NoloStream Protocol Debug Session

Tracking iterative protocol fixes. Each iteration: run script → analyze logs → ask user for UI observations → adjust code → repeat.

**Setup:** Windows, newer firmware (hwver≈0x52–0x56, fwver≈0xee for controllers; hwver≈0x99–0x9e for headset). Running `test/debug-session.ps1`.

---

## Iteration 1 — 2026-05-14

### Starting state (pre-session observations from user)
- White box (HMD): always spinning/flipping; moves at constant speed while data arrives
- Red box (right controller): moves in one axis when headset-receiver is touched; unrelated to controller
- Blue box (left controller): moves with controller 1, but axes aligned wrong
- Controller 2 (right controller): does nothing

### Changes applied before this session
1. **`scene.useRightHandedSystem = true`** in `miniviz/web/index.html`
   - Hypothesis: Nolo/OSVR data is right-handed; Babylon.js default is left-handed → axis mismatch
2. **Numerical debug overlay** in `miniviz/web/index.html` — shows `pos=[x,y,z]  q=[w,x,y,z]` live
3. **`--debug` flag** in `nolostream_server` — logs latest parsed pose per device every 1 s to stderr

### Protocol audit vs reference (nolo-osvr C++)
All offsets, decryption, and quaternion reorder match the reference exactly:
- BTEA: 15 words from offset 1 ✓
- Left ctrl: buf[1], pos=+3, ori=+9 ✓
- Right ctrl: buf[42], pos=+3, ori=+9 ✓
- Headset block: buf[21], pos=+3, ori=+16 ✓
- Orientation reorder: W=w, X=i, Y=k, Z=−j ✓

---

## Iteration 2 — 2026-05-15

### Changes applied
- **Right controller source**: In newer firmware, 0xa5 frame at buf[42] contains garbage (not right ctrl).
  Right controller now parsed from 0xa6 frame at buf[1]. 0xa5 → left only.
- **Position axis remap**: Empirically determined device stores (Y, Z, X) not (X, Y, Z):
  `parse_position` reads raw_y=buf[3..4], raw_z=buf[5..6], raw_x=buf[7..8] → outputs [raw_x, raw_y, raw_z]
- **Orientation reorder** (applied, matches nolo-osvr reference): W=w, X=i, Y=k, Z=−j

### User observations after these changes
- Right controller: glitchy snap to ~[0.54, 0, 0] → **FIXED** by moving right ctrl to 0xa6 frame
- Y/Z positions: roughly correct for both controllers
- X position: ~7 cm separation between L/R, plausible
- Orientation: "jitters a lot, completely wrong" — no physical rotation maps to any displayed axis

### Diagnostic tool added: `--orient-debug` flag
Prints raw i16 bytes [A, B, C, D] at 200 ms intervals for 10 s.

---

## Iteration 3 — 2026-05-15

### Changes applied
- **ABCD graph** added to miniviz: real-time scrolling chart of raw orientation i16 bytes for L and R controllers
- `orientation_raw: [i16; 4]` added to `Pose` struct, sent over WebSocket alongside computed quaternion
- **Orientation reorder corrected** to exactly match nolo-osvr reference: `W=w, X=i, Y=k, Z=−j`
  (previous code had X=−j, Y=i — swapped from reference)

### User observations from ABCD graph
| Raw component | Byte offset (from orientation base) | Physical axis correlation |
|---|---|---|
| A | ori+0..1 | No clear single-axis change (scalar / w) |
| B | ori+2..3 | **No correlation with any axis** |
| C | ori+4..5 | **Correlates with PITCH** |
| D | ori+6..7 | **Correlates with ROLL** |
| — | — | **No value correlates with YAW** |

### Analysis

#### Current memory layout (controller block, base=1 for both L and R)

| Offset from block base | Bytes | Label | Current interpretation |
|---|---|---|---|
| +0 | 1 | hwver | hardware version |
| +1 | 1 | fwver | firmware version |
| +2 | 1 | — | unknown |
| +3..4 | 2 | pos[0] | → world **Y** (×0.0001 m) |
| +5..6 | 2 | pos[1] | → world **Z** (×0.0001 m) |
| +7..8 | 2 | pos[2] | → world **X** (×0.0001 m) |
| +9..10 | 2 | **A** | raw w → quat **W** |
| +11..12 | 2 | **B** | raw i → quat **X** (no axis observed) |
| +13..14 | 2 | **C** | raw j → quat **−Z** (correlates with **PITCH**) |
| +15..16 | 2 | **D** | raw k → quat **Y** (correlates with **ROLL**) |
| +17 | 1 | buttons | bitmask |
| +18 | 1 | touchid | |
| +19 | 1 | touchx | |
| +20 | 1 | touchy | |
| +21 | 1 | battery | |
| +22..62 | 41 | **???** | **unexplored remainder of 64-byte frame** |

#### Inferred reference layout (from nolo-osvr com_osvr_Nolo.cpp)

| Offset from block base | Bytes | Label | Reference interpretation |
|---|---|---|---|
| +0 | 1 | hwver | must = 2 (older firmware) |
| +1 | 1 | fwver | must = 1 (older firmware) |
| +2 | 1 | — | unknown |
| +3..4 | 2 | pos[0] | world **X** (×0.0001 m) — no remap in reference |
| +5..6 | 2 | pos[1] | world **Y** (×0.0001 m) |
| +7..8 | 2 | pos[2] | world **Z** (×0.0001 m, unsigned in ref — likely bug) |
| +9..10 | 2 | quat w | scale ×1/16384 → W |
| +11..12 | 2 | quat i | → X |
| +13..14 | 2 | quat j | → −Z |
| +15..16 | 2 | quat k | → Y |
| +17 | 1 | buttons | |
| +18..21 | 4 | touch / battery | |
| *(0xa5 only)* | | buf[42] = buf[64−22] | **right controller block** (older firmware) |
| *(0xa6 only)* | | buf[21] | **headset marker block** (pos+3, homepos+9, ori+16) |
| *(0xa6 only)* | | buf[54] | base station (battery only) |

#### Key discrepancies vs reference
1. **Position axis order**: Reference uses (X,Y,Z) direct; we use (Y,Z,X)→remap. User confirmed ours is
   closer to correct for newer firmware.
2. **Quaternion mapping (current)**: W=A, X=B, Y=D, Z=−C
   - C (j) maps to −Z but correlates with **pitch** → pitch should be on **X**
   - D (k) maps to Y but correlates with **roll** → roll should be on **Z**
   - B (i) maps to X but **shows no axis** → expected to be the yaw (Y) component
3. **Yaw absent from quaternion**: The device appears to report a **gravity-referenced tilt quaternion
   only** (pitch + roll from IMU), with no absolute heading component. This is consistent with a
   6-axis IMU (no magnetometer) where yaw = optical-only. Yaw would need to be derived from
   position relative to the base station.

#### Proposed fix for pitch/roll (to try next): remap to W=A, X=C, Y=B, Z=D
Currently C→Z and D→Y. If pitch should be X and roll should be Z:
```
qw = A (w)
qx = C (j)   ← was -Z, now X → pitch
qy = B (i)   ← was X, now Y → yaw (may be near-zero)
qz = D (k)   ← was Y, now Z → roll
```
Signs of qx and qz may need flipping depending on observed rotation direction.

---

---

## Iteration 4 — 2026-05-15

### Hypothesis: quaternion W is at base+9 (reference position); i/j/k shifted 2 bytes to base+13..17

From iteration 3, when ABCD was read at base+9..16:
- **A** (base+9): "no axis correlation" — consistent with **quaternion scalar W** (changes with all rotations, not one)
- **B** (base+11): "no axis correlation" — likely **2 unknown bytes** added in newer firmware
- **C** (base+13): pitch ✓
- **D** (base+15): roll ✓
- Yaw: absent (not visible in 4-byte window starting at base+9)

After shifting window to base+11..18 (iteration 3):
- **A** (base+11) = old B: used as W → caused jitter (was the unknown bytes, not W)
- **B** (base+13) = old C: pitch ✓
- **C** (base+15) = old D: roll ✓
- **D** (base+17): **yaw ✓** (newly exposed)
- Conclusion: all three rotation axes present, but W was wrong

### Change applied
- W read from **base+9** (reference offset, "A" from old ABCD — no-axis-correlation behavior matches scalar)
- i,j,k read from **base+13, 15, 17** (non-contiguous; skip 2 unknown bytes at base+11..12)
- nolo-osvr reorder applied: world [W, X, Y, Z] = [w, i, k, −j]

#### Updated memory layout (controller block, base=1)

| Offset from block base | Bytes | Label | Interpretation |
|---|---|---|---|
| +0 | 1 | hwver | hardware version |
| +1 | 1 | fwver | firmware version |
| +2 | 1 | — | unknown |
| +3..4 | 2 | pos[0] | → world Y (×0.0001 m) |
| +5..6 | 2 | pos[1] | → world Z (×0.0001 m) |
| +7..8 | 2 | pos[2] | → world X (×0.0001 m) |
| +9..10 | 2 | **quat W** | quaternion scalar |
| +11..12 | 2 | **???** | unknown 2 bytes (new in newer firmware) |
| +13..14 | 2 | **quat i** | → world X (pitch) |
| +15..16 | 2 | **quat j** | → world −Z (roll) |
| +17..18 | 2 | **quat k** | → world Y (yaw) |
| +19..62 | 44 | buttons/touch/battery + remainder | unknown layout shift |

ABCD graph now shows [W, i, j, k] = [base+9, base+13, base+15, base+17].

### Open questions
- Is orientation absolute (AHRS) or relative (gyro delta)? Hold still and observe drift.
- What are bytes base+11..12? (Possibly: status, confidence, extra sensor data)
- Buttons/touch/battery: their offsets shifted +2 in newer firmware (now at +19..23 instead of +17..21).
  Not yet verified.

---

## Iteration 5 — 2026-05-15

### Changes applied
- **miniviz graph panel** rebuilt: 2 rows × 4 graphs (Pos, Accel, AngVel, Unknowns) per controller (8 total).
  - Reads `sensor_raw` (19 i16s) instead of old `orientation_raw` (4 i16s).
  - Row 1: L Pos [P0 P1 P2] | L Accel [AX AY AZ] | L AngVel [RX RY RZ] | L Unknowns [E0..E9]
  - Row 2: same for R controller.
  - sensor_raw layout: indices 0..2 = pos raw, 3..5 = accel, 6..8 = angvel, 9..18 = unknowns.

### User observations
- **Position coordinate system**: P0/P1/P2 form a **left-handed** Y-up frame.
  - Y = up-axis; Z = from base-station outward (toward user). X = right.
  - This matches DirectX / Babylon.js default convention (not OpenGL right-handed).
- **Accelerometer (AX/AY/AZ)**: confirmed accelerometer. Controller body-fixed axes:
  - **+Y** = from controller center toward the top sensor bubble.
  - **+Z** = from center toward the face where the buttons are.
- **Angular velocity (RX/RY/RZ)**: confirmed gyroscope (same body-fixed axes as accel).
- **Unknowns (E0..E4 active, E5..E9 ≈ 0)**:
  - E0: tentatively **touchpad X-axis** position.
  - E2: tentatively a **monotonic counter** or secondary touch field.
  - E5..E9 are zero — these read beyond the ~26-byte controller block into inter-block padding.

### Cross-reference: OSVR reference → newer firmware input field offsets

In older firmware (nolo-osvr), the 4-word quaternion (base+9..16) is replaced in newer firmware
by 6 IMU words (3 × accel + 3 × gyro = 12 bytes), shifting input fields by +4:

| Field | Old offset (nolo-osvr) | New offset (newer firmware) | sensor_raw index | E-label |
|---|---|---|---|---|
| buttons | base+17 | **base+21** | [10] high byte | E1 hi |
| touch ID | base+18 | **base+22** | [10] low byte | E1 lo |
| touch X | base+19 | **base+23** | [11] high byte | E2 hi |
| touch Y | base+20 | **base+24** | [11] low byte | E2 lo |
| battery | base+21 | **base+25** | [12] high byte | E3 hi |
| block end | base+21 (old) | **base+25** (new, if same 22-byte size minus 4+4 = same) | — | — |

sensor_raw[9] (base+19..20): two bytes just before buttons — unknown; possibly an extra IMU word.

**Remaining questions**
- [ ] Verify button/touch/battery decode: press buttons and touch pad, watch labeled unk channels in new graph.
- [ ] E0 (base+19): what is it exactly? (counter? 6th IMU word? padding?)
- [ ] Coordinate system: decide whether to flip Z to produce right-handed output.
- [ ] Orientation (quaternion): no fused quaternion found yet. Options: AHRS from accel+gyro, or search elsewhere in packet.
- [ ] Headset tracking: 0xa6 frame parsing incomplete — headset pose not yet extracted.
- [ ] Timestamp: device provides its own tick counter (buf[24..27]) — switch to it for accurate dt.

---

## Iteration 6 — 2026-05-15

### User observations (from updated Unknowns graph with per-byte labels)

| Label | Byte offset | Observation |
|---|---|---|
| tX (was ?19) | base+19 | touch X: 255=no touch, ~127=center, increases when swiping LEFT |
| tY (was ?20) | base+20 | touch Y: 255=no touch, ~127=center, increases when swiping DOWN |
| cnt (was tx) | base+23 | continuously counts up while controller is connected → device tick counter LSB |

### Deductions

**touchX/touchY are at the same offsets as old firmware** (base+19/20, no shift from nolo-osvr).
This rules out the "+4-byte IMU expansion" shift hypothesis.

**IMU word count is likely 4.** Since input fields start at base+19 (not base+23), and the
4-word quaternion in old firmware ended at base+16, a 4-word IMU replacement also ends at base+16.
The word at base+17..18 (sensor_raw[8], currently labeled "AngVel RZ") is therefore either:
  - Still the old buttons|touchID bytes (base+17=buttons, base+18=touchID — same as nolo-osvr), OR
  - A 5th IMU axis (RY if accel is AX,AY,AZ,RX and gyro only has RX,RY)

When no button is pressed, buttons=0 → sensor_raw[8]≈0, indistinguishable from a stationary gyro.
**The user confirmed sensor_raw[8] as "RZ" during rotation-only testing.** Needs a dedicated
button-press test to distinguish these two possibilities.

**Device tick counter** at base+23..26 = buf[24..27] for left controller block (base=1).
Only the LSB (base+23 = "cnt") was visibly changing in the short test; higher bytes are slower.

**Revised inferred field layout (newer firmware):**

| Byte offset | Field | Status |
|---|---|---|
| base+3..8 | position (Y,Z,X raw → remapped) | confirmed |
| base+9..16 | IMU words 0..3 (accel+gyro, 4 words) | confirmed direction-correlated |
| base+17..18 | buttons\|touchID OR 5th IMU word | **ambiguous — needs button test** |
| base+19 | touch X (255=no touch, 127=center) | **confirmed** |
| base+20 | touch Y (255=no touch, 127=center) | **confirmed** |
| base+21 | battery (0–255) | tentative (nolo-osvr same offset) |
| base+22 | unknown | |
| base+23..26 | 32-bit LE device tick counter | **confirmed (LSB)** |

### Next steps
- [ ] **Button test**: press trigger/home/touchpad while watching AngVel RZ AND all unk columns.
  - If AngVel RZ spikes → buttons are at base+17..18 (IMU is 4 words ending at base+16).
  - If a unk column changes instead → buttons shifted beyond base+18.
- [ ] Implement touch X/Y from confirmed offsets (base+19, base+20) in Rust.
- [ ] Implement button parsing once offset confirmed.
- [ ] Battery from base+21 (tentative).
- [ ] Orientation: no quaternion found. Consider AHRS complementary filter from accel+gyro.
- [ ] Headset: parse 0xa6 headset block (buf[21..44]).
