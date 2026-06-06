#!/usr/bin/env python3
"""Wait for ur_robot_driver to listen on port 50001, then play ext.urp via Dashboard Server.

Usage: ur_dashboard_autoplay.py <robot_ip> [program_name] [reverse_port]
Defaults: robot_ip=192.168.10.2  program=ext.urp  reverse_port=50001
"""
import socket
import sys
import time


def wait_for_driver(port: int, timeout: float = 60.0) -> bool:
    """Block until ur_robot_driver opens its reverse-connection port."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=1.0)
            s.close()
            return True
        except OSError:
            time.sleep(1.0)
    return False


def dashboard_play(ip: str, prog: str) -> None:
    s = socket.create_connection((ip, 29999), timeout=10)
    s.settimeout(5)
    welcome = s.recv(1024)
    print(f"[autoplay] connected: {welcome.decode().strip()}", flush=True)

    s.sendall(f"load {prog}\n".encode())
    time.sleep(0.3)
    resp = s.recv(1024).decode().strip()
    print(f"[autoplay] load → {resp}", flush=True)
    if "error" in resp.lower() or "failed" in resp.lower():
        raise RuntimeError(f"load failed: {resp}")

    s.sendall(b"play\n")
    time.sleep(0.3)
    resp = s.recv(1024).decode().strip()
    print(f"[autoplay] play → {resp}", flush=True)
    s.close()


def main() -> None:
    ip           = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.2"
    prog         = sys.argv[2] if len(sys.argv) > 2 else "ext.urp"
    reverse_port = int(sys.argv[3]) if len(sys.argv) > 3 else 50001

    print(f"[autoplay] waiting for driver on port {reverse_port} ...", flush=True)
    if not wait_for_driver(reverse_port, timeout=60.0):
        print(
            f"[autoplay] ERROR: ur_robot_driver never opened port {reverse_port} after 60 s.\n"
            "           Check that on_configure() succeeded (look for recipe_id errors above).",
            file=sys.stderr, flush=True,
        )
        sys.exit(1)

    print(f"[autoplay] driver ready — playing {prog} on {ip}", flush=True)
    try:
        dashboard_play(ip, prog)
        print(f"[autoplay] {prog} started", flush=True)
    except Exception as exc:
        print(f"[autoplay] ERROR: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
