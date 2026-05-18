"""
HID device monitor: polls every 2s, detects new devices after a baseline,
then waits 5s and shows all new devices with full info.

Usage: test/.venv310/Scripts/python test/hid_device_monitor.py
Requires: pip install pywinusb
"""

import time
import pywinusb.hid as hid


def enumerate_devices() -> dict[str, object]:
    """Return {device_path: device} for all currently visible HID devices."""
    return {dev.device_path: dev for dev in hid.find_all_hid_devices()}


def print_device(dev) -> None:
    print(f"  Path        : {dev.device_path}")
    print(f"  VID         : {dev.vendor_id:#06x}  PID: {dev.product_id:#06x}")
    print(f"  Vendor      : {dev.vendor_name}")
    print(f"  Product     : {dev.product_name}")
    print(f"  Serial      : {dev.serial_number}")
    print(f"  Version     : {dev.version_number:#06x}")
    print(f"  Instance ID : {dev.instance_id}")
    print()


def main() -> None:
    print("Taking baseline snapshot of HID devices...")
    baseline = enumerate_devices()
    print(f"Baseline: {len(baseline)} device(s) registered.\n")
    print("Polling every 2 seconds. Plug in your Nolo device now.\n")

    while True:
        time.sleep(2)
        current = enumerate_devices()
        new_paths = set(current) - set(baseline)

        if new_paths:
            print(f"[+] {len(new_paths)} new device(s) detected! Waiting 5s for enumeration to settle...")
            time.sleep(5)
            current = enumerate_devices()
            new_paths = set(current) - set(baseline)

            print(f"\n=== {len(new_paths)} new HID device(s) relative to baseline ===\n")
            for path in sorted(new_paths):
                print_device(current[path])

            print("Continuing to monitor for further changes...\n")


if __name__ == "__main__":
    main()
