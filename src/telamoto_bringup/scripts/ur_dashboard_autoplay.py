#!/usr/bin/env python3
"""Play ext.urp via the Dashboard Server (port 29999) once the script-sender is up.

Manual convenience: replaces pressing Play on the pendant. Waits until the
local script-sender (ur_servo_controller.py's reverse port) is listening so the
External Control URCap has something to fetch, then loads + plays the program.

Usage: ur_dashboard_autoplay.py <robot_ip> [program_name] [reverse_port]
Defaults: robot_ip=192.168.10.2  program=ext.urp  reverse_port=50001
"""
import socket
import sys
import time


def wait_for_driver(port: int, timeout: float = 60.0) -> bool:
    """Block until the controller's script-sender opens its reverse port."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=1.0)
            s.close()
            return True
        except OSError:
            time.sleep(1.0)
    return False


def _recv_line(s: socket.socket) -> str:
    """Read one LF-terminated line from *s*, byte-by-byte, decoded as UTF-8.

    Prevents partial TCP segments from bleeding into the next command's response.
    """
    buf = bytearray()
    while True:
        ch = s.recv(1)
        if not ch:
            break
        buf += ch
        if ch == b"\n":
            break
    return buf.decode("utf-8", errors="replace").strip()


def _send_cmd(s: socket.socket, cmd: str) -> str:
    s.sendall((cmd + "\n").encode("utf-8"))
    return _recv_line(s)


def dashboard_play(ip: str, prog: str) -> None:
    s = socket.create_connection((ip, 29999), timeout=10)
    s.settimeout(5)
    welcome = _recv_line(s)
    print(f"[autoplay] connected: {welcome}", flush=True)

    resp = _send_cmd(s, f"load {prog}")
    print(f"[autoplay] load → {resp}", flush=True)
    if "error" in resp.lower() or "failed" in resp.lower():
        raise RuntimeError(f"load failed: {resp}")

    resp = _send_cmd(s, "play")
    print(f"[autoplay] play → {resp}", flush=True)
    s.close()


def main() -> None:
    ip           = sys.argv[1] if len(sys.argv) > 1 else "192.168.10.2"
    prog         = sys.argv[2] if len(sys.argv) > 2 else "ext.urp"
    reverse_port = int(sys.argv[3]) if len(sys.argv) > 3 else 50001

    print(f"[autoplay] waiting for the script-sender on port {reverse_port} ...", flush=True)
    if not wait_for_driver(reverse_port, timeout=60.0):
        print(
            f"[autoplay] ERROR: nothing listening on port {reverse_port} after 60 s.\n"
            "           Is the bringup (ur_servo_controller.py) running?",
            file=sys.stderr, flush=True,
        )
        sys.exit(1)

    print(f"[autoplay] script-sender ready — playing {prog} on {ip}", flush=True)
    try:
        dashboard_play(ip, prog)
        print(f"[autoplay] {prog} started", flush=True)
    except Exception as exc:
        print(f"[autoplay] ERROR: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
