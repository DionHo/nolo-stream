"""
Guided timed capture for Nolo CV1 button mapping.

Runs through timed phases (idle, then each button) automatically.
All decrypted frames are saved to test/button_capture.json for analysis.

Usage:
    test/.venv310/Scripts/python test/hid_button_finder.py

Follow the on-screen countdown — no interactive typing needed.
"""

import sys
import time
import struct
import json
import queue
import os
import pywinusb.hid as hid

NOLO_VID = 0x0483
NOLO_PID = 0x5750
NOLO_KEY = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]
OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "button_capture.json")

BUTTONS = [
    ("IDLE — keep ALL buttons released",        "idle"),
    ("TRIGGER — press and HOLD the trigger",    "trigger"),
    ("GRIP — press and HOLD the grip/side btn", "grip"),
    ("MENU — press and HOLD the menu button",   "menu"),
    ("TOUCHPAD CLICK — press the pad down",     "touchpad_click"),
    ("SYSTEM — press and HOLD system button",   "system"),
]
PHASE_SECONDS = 5
PREP_SECONDS  = 3


def _u32(x):
    return x & 0xFFFFFFFF


def _mx(z, y, s, key, p, e):
    a = _u32((_u32(z >> 5) ^ _u32(y << 2)) + (_u32(y >> 3) ^ _u32(z << 4)))
    b = _u32((_u32(s ^ y)) + (_u32(key[(p & 3) ^ e] ^ z)))
    return _u32(a ^ b)


def btea_decrypt(data, base_rounds=1):
    n = len(data)
    if n <= 1:
        return
    DELTA = 0x9E3779B9
    rounds = base_rounds + 52 // n
    s = _u32(DELTA * rounds)
    y = data[0]
    while True:
        e = (s >> 2) & 3
        p = n - 1
        while p > 0:
            z = data[p - 1]
            data[p] = _u32(data[p] - _mx(z, y, s, NOLO_KEY, p, e))
            y = data[p]
            p -= 1
        z = data[n - 1]
        data[0] = _u32(data[0] - _mx(z, y, s, NOLO_KEY, 0, e))
        y = data[0]
        s = _u32(s - DELTA)
        if s == 0:
            break


def decrypt_report(raw):
    if len(raw) < 64:
        return None
    buf = bytearray(raw[:64])
    words = list(struct.unpack_from("<15I", buf, 1))
    btea_decrypt(words, 1)
    struct.pack_into("<15I", buf, 1, *words)
    return list(buf)


def open_nolo(report_q):
    devs = [d for d in hid.find_all_hid_devices()
            if d.vendor_id == NOLO_VID and d.product_id == NOLO_PID]
    if not devs:
        print(f"ERROR: No Nolo device found (VID={NOLO_VID:#06x} PID={NOLO_PID:#06x})")
        sys.exit(1)
    print(f"Found {len(devs)} Nolo HID interface(s):")
    for d in devs:
        print(f"  {d.device_path}")

    def raw_handler(data):
        raw = bytes(data)
        payload = raw[1:] if len(raw) == 65 else raw
        dec = decrypt_report(payload)
        # if dec and dec[0] in (0xa5, 0x10, 0xa6, 0x11):
        try:
            report_q.put_nowait(dec)
        except queue.Full:
            pass

    dev = devs[0]
    dev.open()
    dev.set_raw_data_handler(raw_handler)
    return dev


def capture_phase(report_q, label, key, duration):
    """Capture all decrypted frames for `duration` seconds and return them."""
    print(f"\n{'='*60}")
    print(f"  PHASE: {label}")

    # prep countdown
    for i in range(PREP_SECONDS, 0, -1):
        print(f"  Starting in {i}...", end="\r", flush=True)
        time.sleep(1)
    print(f"  >>> RECORDING for {duration}s <<<         ", flush=True)

    frames = []
    deadline = time.time() + duration
    while time.time() < deadline:
        remaining = deadline - time.time()
        try:
            dec = report_q.get(timeout=0.05)
            frames.append(dec)
        except queue.Empty:
            pass
        print(f"  {remaining:.1f}s remaining, {len(frames)} frames captured", end="\r", flush=True)

    print(f"\n  Captured {len(frames)} frames for '{key}'")
    return frames


def analyse(captures):
    """Print which bytes differ most between idle and each button phase."""
    idle_frames = captures.get("idle", [])
    if not idle_frames:
        print("No idle frames — cannot analyse.")
        return

    # Build per-packet-type idle average
    idle_by_type = {}
    for f in idle_frames:
        t = f[0]
        idle_by_type.setdefault(t, [[] for _ in range(64)])
        for i, b in enumerate(f):
            idle_by_type[t][i].append(b)
    idle_avg = {t: [round(sum(col)/len(col)) for col in cols]
                for t, cols in idle_by_type.items()}

    print("\n" + "="*60)
    print("ANALYSIS: bytes that change per button")
    print("="*60)

    for _, key in BUTTONS:
        if key == "idle":
            continue
        frames = captures.get(key, [])
        if not frames:
            print(f"\n[{key}] — no frames captured")
            continue

        print(f"\n[{key}]  ({len(frames)} frames)")
        change_counts = {}
        for f in frames:
            t = f[0]
            baseline = idle_avg.get(t)
            if not baseline:
                continue
            for i in range(64):
                if f[i] != baseline[i]:
                    change_counts[i] = change_counts.get(i, 0) + 1

        if not change_counts:
            print("  No byte differences found.")
            continue

        # Sort by frequency, show top candidates
        top = sorted(change_counts.items(), key=lambda x: -x[1])[:15]
        total_frames = len(frames)
        for off, cnt in top:
            pct = cnt / total_frames * 100
            # Show typical changed values
            vals = set()
            for f in frames:
                t = f[0]
                bl = idle_avg.get(t)
                if bl and f[off] != bl[off]:
                    vals.add(f"{bl[off]:#04x}→{f[off]:#04x}")
            print(f"  offset {off:2d} (0x{off:02x})  {pct:5.1f}% of frames  values: {', '.join(list(vals)[:6])}")


def main():
    os.system("")  # enable ANSI on Windows
    print("="*60)
    print("  Nolo CV1 button capture — guided recording session")
    print("="*60)
    print(f"\nOutput will be saved to: {OUTPUT_FILE}")
    print(f"\nPhases: {len(BUTTONS)} × ({PREP_SECONDS}s prep + {PHASE_SECONDS}s record)")
    print(f"Total time: ~{len(BUTTONS) * (PREP_SECONDS + PHASE_SECONDS)}s\n")
    print("IMPORTANT: Use only ONE controller (whichever has the buttons).")
    print("Keep the HMD off or stationary to reduce noise.\n")

    input("Press Enter to start when you're ready...")

    report_q = queue.Queue(maxsize=64)
    device = open_nolo(report_q)

    captures = {}
    try:
        for label, key in BUTTONS:
            frames = capture_phase(report_q, label, key, PHASE_SECONDS)
            captures[key] = frames
    finally:
        device.close()

    # Save raw capture
    with open(OUTPUT_FILE, "w") as f:
        json.dump(captures, f)
    print(f"\nRaw capture saved to {OUTPUT_FILE}")

    analyse(captures)

    print("\nDone. Share the analysis above (or the JSON file) to map buttons to offsets.")


if __name__ == "__main__":
    main()
