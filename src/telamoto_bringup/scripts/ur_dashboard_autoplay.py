#!/usr/bin/env python3
"""Connect to the UR Dashboard Server and load + play a program.

Usage: ur_dashboard_autoplay.py <robot_ip> [program_name]
Default program: ext.urp
"""
import socket
import sys
import time


def main() -> None:
    ip   = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.2"
    prog = sys.argv[2] if len(sys.argv) > 2 else "ext.urp"

    try:
        print(f"[dashboard_autoplay] Connecting to {ip}:29999 ...", flush=True)
        s = socket.create_connection((ip, 29999), timeout=10)
        s.settimeout(5)

        welcome = s.recv(1024)
        print(f"[dashboard_autoplay] {welcome.decode().strip()}", flush=True)

        s.sendall(f"load {prog}\n".encode())
        time.sleep(0.3)
        resp = s.recv(1024)
        print(f"[dashboard_autoplay] load: {resp.decode().strip()}", flush=True)

        s.sendall(b"play\n")
        time.sleep(0.3)
        resp = s.recv(1024)
        print(f"[dashboard_autoplay] play: {resp.decode().strip()}", flush=True)

        s.close()
        print(f"[dashboard_autoplay] {prog} started on robot", flush=True)
    except Exception as exc:
        print(f"[dashboard_autoplay] failed: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
