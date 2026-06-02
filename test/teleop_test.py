"""
Teleop protocol test script.
Simulates robot side: listens on two TCP ports (left/right controller),
accepts the NoloStream server connection, sends {"type":"handover_active"},
then prints received frames.
Press Ctrl-C to stop.

Usage:
  python teleop_test.py [--left 127.0.0.1:9001] [--right 127.0.0.1:9002]
"""
import argparse
import json
import socket
import threading
import time


def controller_thread(name: str, host: str, port: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[{name}] Listening on {host}:{port} (waiting for NoloStream server)...")
    srv.settimeout(30)
    try:
        sock, addr = srv.accept()
    except socket.timeout:
        print(f"[{name}] Timed out waiting for connection")
        srv.close()
        return
    srv.close()
    print(f"[{name}] Accepted connection from {addr}. Sending handover_active in 1s...")
    time.sleep(1.0)

    msg = json.dumps({"type": "handover_active"}) + "\n"
    sock.sendall(msg.encode())
    print(f"[{name}] Sent handover_active")

    buf = ""
    sock.settimeout(0.1)
    frame_count = 0
    last_print = time.time()
    try:
        while True:
            try:
                data = sock.recv(4096).decode(errors="replace")
                if not data:
                    print(f"[{name}] Server closed connection")
                    break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                        frame_count += 1
                        now = time.time()
                        if now - last_print >= 1.0:
                            print(f"[{name}] Received {frame_count} frames. Last: {obj}")
                            last_print = now
                    except json.JSONDecodeError:
                        print(f"[{name}] Non-JSON line: {line!r}")
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[{name}] Total frames received: {frame_count}")
        sock.close()


def parse_addr(s: str):
    host, port = s.rsplit(":", 1)
    return host, int(port)


def main():
    parser = argparse.ArgumentParser(description="NoloStream teleop test")
    parser.add_argument("--left",  default="127.0.0.1:9001", help="Left controller endpoint")
    parser.add_argument("--right", default="127.0.0.1:9002", help="Right controller endpoint")
    args = parser.parse_args()

    lh, lp = parse_addr(args.left)
    rh, rp = parse_addr(args.right)

    threads = [
        threading.Thread(target=controller_thread, args=("LEFT",  lh, lp), daemon=True),
        threading.Thread(target=controller_thread, args=("RIGHT", rh, rp), daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[main] Stopped.")


if __name__ == "__main__":
    main()
