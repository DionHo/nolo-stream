"""
Real-time HID bit visualizer with auto-reconnect for Nolo CV1.

Displays each byte's bits as blocks: white (0) or green (1).
Automatically reconnects if device connection is lost.

Usage:
    test/.venv/Scripts/python test/hid_bit_viewer.py

Press Ctrl+C to exit.
"""

import sys
import time
import struct
import queue
import os
import threading
import pywinusb.hid as hid

NOLO_VID = 0x0483
NOLO_PID = 0x5750
NOLO_KEY = [0x875bcc51, 0xa7637a66, 0x50960967, 0xf8536c51]

RECONNECT_RETRY_INTERVAL = 2.0
RECONNECT_MAX_ATTEMPTS = 10

# ANSI color codes
COLOR_WHITE = "\033[47m \033[0m"  # white block
COLOR_GREEN = "\033[42m \033[0m"  # green block
COLOR_RESET = "\033[0m"
CLEAR_SCREEN = "\033[2J\033[H"


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


def find_nolo_device():
    """Find Nolo HID device."""
    devs = [d for d in hid.find_all_hid_devices()
            if d.vendor_id == NOLO_VID and d.product_id == NOLO_PID]
    return devs[0] if devs else None


def open_nolo(report_q):
    """Open Nolo device and set up raw data handler."""
    device = find_nolo_device()
    if not device:
        return None

    def raw_handler(data):
        raw = bytes(data)
        payload = raw[1:] if len(raw) == 65 else raw
        dec = decrypt_report(payload)
        if dec:
            try:
                report_q.put_nowait(dec)
            except queue.Full:
                pass

    try:
        device.open()
        device.set_raw_data_handler(raw_handler)
        return device
    except Exception as e:
        print(f"Failed to open device: {e}")
        return None


def visualize_bits(data):
    """Convert byte data to visual bit representation with byte offset labels."""
    lines = []
    
    # Header with byte offsets
    header = "Byte:  "
    for i in range(16):
        header += f"   {i:02x}    | "
    lines.append(header)
    lines.append("")
    
    for row in range(0, len(data), 16):
        chunk = data[row:row+16]
        byte_visuals = []
        for col, byte_val in enumerate(chunk):
            bits = "".join(COLOR_GREEN if (byte_val >> bit) & 1 else COLOR_WHITE
                          for bit in range(8))
            byte_visuals.append(bits)
        
        offset_str = f"0x{row:02x} | "
        line = offset_str + " | ".join(byte_visuals)
        lines.append(line)
    
    return "\n".join(lines)


def format_time():
    """Format current timestamp."""
    return time.strftime("%H:%M:%S")


def format_selected_fields(data):
    """Format selected byte fields for quick inspection."""
    if len(data) <= 0x19:
        return "Fields: battery=N/A | counter1=N/A | counter2=N/A"

    battery = data[0x16]
    counter1 = data[0x17]
    counter2 = data[0x18]
    counter3 = data[0x19]
    counter1x = data[0x17] | (data[0x18] << 8)  # little-endian [0x17, 0x18]
    counter2x = data[0x18] | (data[0x19] << 8)  # little-endian [0x18, 0x19]
    return (
        f"Fields: battery={battery} (0x{battery:02x}) | "
        f"counter1={counter1} (0x{counter1:02x}) | "
        f"counter2={counter2} (0x{counter2:02x}) | "
        f"counter3={counter3} (0x{counter3:02x}) | "
        f"counter1x={counter1x} (0x{counter1x:04x}) | "
        f"counter2x={counter2x} (0x{counter2x:04x})"
    )


def run_viewer():
    """Main viewer loop with auto-reconnect."""
    os.system("")  # enable ANSI on Windows

    report_q = queue.Queue(maxsize=64)
    device = None
    attempt = 0
    last_display_time = time.time()
    # Per-type state: type_byte -> {"data": [...], "fps": float, "count": int, "last_time": float}
    type_frames = {}
    display_interval = 0.1  # Update display at 10 Hz to reduce flicker

    print(CLEAR_SCREEN)
    print("="*70)
    print("  Nolo CV1 HID Bit Viewer (white=0, green=1)")
    print("="*70)
    print("Connecting...\n")

    try:
        while True:
            # Try to connect if not connected
            if device is None:
                print(f"[{format_time()}] Attempting connection ({attempt + 1}/{RECONNECT_MAX_ATTEMPTS})...")
                device = open_nolo(report_q)
                if device:
                    attempt = 0
                    last_display_time = time.time()
                    print(f"[{format_time()}] ✓ Connected!\n")
                else:
                    attempt += 1
                    if attempt >= RECONNECT_MAX_ATTEMPTS:
                        print(f"ERROR: Failed to connect after {RECONNECT_MAX_ATTEMPTS} attempts.")
                        sys.exit(1)
                    time.sleep(RECONNECT_RETRY_INTERVAL)
                    continue

            # Try to get a frame (non-blocking with short timeout)
            try:
                dec = report_q.get(timeout=0.01)
                report_type = dec[0]
                now = time.time()

                prev = type_frames.get(report_type)
                if prev is not None:
                    dt = now - prev["last_time"]
                    fps = 1.0 / dt if dt > 0.001 else 0.0
                else:
                    fps = 0.0

                type_frames[report_type] = {
                    "data": dec,
                    "fps": fps,
                    "count": (prev["count"] + 1) if prev else 1,
                    "last_time": now,
                }

            except queue.Empty:
                pass

            # Update display at fixed interval to reduce flicker
            now = time.time()
            if now - last_display_time >= display_interval:
                last_display_time = now

                if type_frames:
                    print(CLEAR_SCREEN)
                    print("="*70)
                    print("  Nolo CV1 HID Bit Viewer (white=0, green=1)")
                    print("="*70)
                    print(f"Time: {format_time()}")
                    for report_type in sorted(type_frames):
                        state = type_frames[report_type]
                        print(f"\n--- Type: 0x{report_type:02x} | Frame #{state['count']} | FPS: {state['fps']:.1f} ---\n")
                        print(format_selected_fields(state["data"]))
                        print("")
                        print(visualize_bits(state["data"]))
                    print("\n" + "="*70)
                    print("Press Ctrl+C to exit")
                else:
                    print(f"[{format_time()}] Waiting for first frame...", end="\r", flush=True)

    except KeyboardInterrupt:
        print("\n\nShutting down...")
        if device:
            try:
                device.close()
            except:
                pass
        print("Goodbye!")
        sys.exit(0)

    except Exception as e:
        print(f"\nFatal error: {e}")
        if device:
            try:
                device.close()
            except:
                pass
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nShutting down...")
        if device:
            try:
                device.close()
            except:
                pass
        print("Goodbye!")
        sys.exit(0)

    except Exception as e:
        print(f"\nFatal error: {e}")
        if device:
            try:
                device.close()
            except:
                pass
        sys.exit(1)


if __name__ == "__main__":
    run_viewer()
