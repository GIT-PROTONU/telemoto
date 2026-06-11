#!/usr/bin/env python3
"""E2E responsiveness test for the adaptive jog lease (laggy-link compensation).

Run manually, no robot or bringup needed (ports 8093/50091 must be free):

    pixi run python src/telamoto_bringup/test/test_jog_lag_e2e.py

Instantiates the REAL URServoController and drives its real WebSocket with
controlled frame timing, verifying:
  A. clean 30 Hz stream  -> scale x1.00, lease 100 ms (LAN behavior unchanged)
  B. keydown->command and keyup->zero latency (instant start/stop, sub-ms)
  C. jittery stream      -> lease stretches, speed scales, and the blind-travel
                            invariant scale*lease = 100 ms holds
  D. instant stop still works while degraded
  E. clean stream again  -> peak-hold decays back to x1.00 in ~14 s
"""
import base64
import importlib.util
import json
import os
import socket
import sys
import time
import urllib.request

_SCRIPT = os.path.join(os.path.dirname(__file__), os.pardir,
                       "scripts", "ur_servo_controller.py")
spec = importlib.util.spec_from_file_location("usc", _SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

import rclpy
# Full isolation from a live bringup: own ROS domain (a real stack publishes
# /joint_states at 125 Hz and would beat the test helper — and the test must
# never publish onto the live graph), and own web/reverse ports (an open
# web-UI tab reconnects to :8080 every second; its 30 Hz keepalive would
# pollute the link-jitter estimator under test).
os.environ["ROS_DOMAIN_ID"] = "77"
rclpy.init(args=["--ros-args", "-p", "web_port:=8093", "-p", "reverse_port:=50091"])
node = usc.URServoController()
time.sleep(0.7)                                   # let the web thread bind

urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8093/api/jogmode?on=1", method="POST"), timeout=2)

# ── minimal RFC 6455 client ─────────────────────────────────────────────────
s = socket.create_connection(("127.0.0.1", 8093), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:8093\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
buf = b""
while b"\r\n\r\n" not in buf:
    buf += s.recv(1024)
assert b" 101 " in buf.split(b"\r\n")[0] + b" ", buf[:80]

def cbor_jog(lx, ly, lz):
    def ci(n):
        m, u = (0x20, -1 - n) if n < 0 else (0x00, n)
        return bytes([m | u]) if u < 24 else bytes([m | 24, u])
    return b"\x83" + ci(lx) + ci(ly) + ci(lz)

def ws_send(payload):
    mask = os.urandom(4)
    s.sendall(bytes([0x82, 0x80 | len(payload)]) + mask
              + bytes(c ^ mask[i & 3] for i, c in enumerate(payload)))

def state():
    return json.loads(urllib.request.urlopen(
        "http://127.0.0.1:8093/api/state", timeout=2).read())

def stream(seconds, vec=(0, 0, 0), stall_every=0, stall=0.15):
    n, end = 0, time.monotonic() + seconds
    while time.monotonic() < end:
        ws_send(cbor_jog(*vec))
        n += 1
        time.sleep(stall if stall_every and n % stall_every == 0 else 1 / 30)

def wait_jog(pred, timeout=1.0):
    t0 = time.perf_counter()
    while not pred(node._jog) and time.perf_counter() - t0 < timeout:
        time.sleep(0.0002)
    return (time.perf_counter() - t0) * 1000.0

fails = 0
def check(name, cond, detail):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:34s} {detail}", flush=True)

# A ── clean 30 Hz: LAN behavior must be bit-identical ────────────────────────
stream(3.0)
st = state()
check("A clean-LAN scale/lease", st["linkscale"] == 1.0 and st["lease"] == 100,
      f"scale=x{st['linkscale']} lease={st['lease']}ms gap_pk={st['linkgap']}ms")

# A2 ── de-twitch: ONE isolated spike (browser GC) must not stretch the lease ──
stream(1.0)
ws_send(cbor_jog(0, 0, 0)); time.sleep(0.18); ws_send(cbor_jog(0, 0, 0))
stream(1.0)
st = state()
check("A2 isolated spike ignored", st["lease"] == 100 and st["linkscale"] == 1.0,
      f"lease={st['lease']}ms scale=x{st['linkscale']} after one 180ms gap")

# B ── start/stop latency at full speed ───────────────────────────────────────
ws_send(cbor_jog(1, 0, 0))
lat = wait_jog(lambda j: j[0] != 0.0)
check("B keydown latency", lat < 10.0, f"{lat:.2f} ms WS frame -> twist target")
check("B full speed on LAN", abs(node._jog[0] - node._jog_speed) < 1e-12,
      f"_jog[0]={node._jog[0]:.4f} == jog_speed={node._jog_speed:.4f} m/s")
ws_send(cbor_jog(0, 0, 0))
lat = wait_jog(lambda j: j[0] == 0.0)
check("B keyup latency", lat < 10.0, f"{lat:.2f} ms WS zero -> twist zeroed")

# C ── jittery link: every 10th frame delayed 150 ms ──────────────────────────
stream(4.0, stall_every=10)
st = state()
check("C lease stretches", 250 <= st["lease"] <= 400,
      f"lease={st['lease']}ms gap_pk={st['linkgap']}ms scale=x{st['linkscale']}")
check("C blind-travel invariant", abs(st["linkscale"] * st["lease"] - 100) < 3,
      f"scale*lease={st['linkscale'] * st['lease']:.0f}ms (target 100)")
ws_send(cbor_jog(1, 0, 0))
wait_jog(lambda j: j[0] != 0.0)
check("C speed scaled with lease",
      abs(node._jog[0] - node._jog_speed * node._link_scale) < 1e-12,
      f"_jog[0]={node._jog[0]*1000:.1f}mm/s = jog_speed x{node._link_scale:.2f}")

# D ── instant stop while degraded ────────────────────────────────────────────
ws_send(cbor_jog(0, 0, 0))
lat = wait_jog(lambda j: j[0] == 0.0)
check("D instant stop while degraded", lat < 10.0, f"{lat:.2f} ms")

# K ── idle KEEPALIVE (CBOR empty array, 0x80): feeds the estimator but must
#      never overwrite an active jog command nor renew its lease — a stuck
#      client sending only keepalives must still stop on lease expiry, and a
#      second idle tab must not fight the active tab (the choppiness bug).
ws_send(cbor_jog(1, 0, 0))
wait_jog(lambda j: j[0] != 0.0)
lease0 = node._jog_lease
tick0 = node._link_last_t
for _ in range(8):
    time.sleep(0.033)
    ws_send(b"\x80")
time.sleep(0.05)
check("K keepalive feeds estimator", node._link_last_t > tick0,
      f"last_t advanced {node._link_last_t - tick0:.2f}s")
check("K keepalive never moves/holds", node._jog[0] != 0.0 and node._jog_lease == lease0,
      f"_jog[0]={node._jog[0]:.3f} kept, lease NOT renewed (expired "
      f"{time.monotonic() - lease0:.2f}s ago)")
ws_send(cbor_jog(0, 0, 0))
wait_jog(lambda j: j[0] == 0.0)

# T ── analog (tilt) frames: CBOR float32 axes in [-1,1] scale the speed; a
#      NaN must NOT slip through _clamp (NaN comparisons are false) to full
#      speed — it zeroes the twist.
import struct as _st
def cbor_f32(*vals):
    return bytes([0x80 | len(vals)]) + b"".join(
        b"\xfa" + _st.pack(">f", v) for v in vals)
ws_send(cbor_f32(0.5, 0.0, 0.0))
wait_jog(lambda j: j[0] != 0.0)
check("T analog float frame half speed",
      abs(node._jog[0] - 0.5 * node._jog_speed * node._link_scale) < 1e-6,
      f"_jog[0]={node._jog[0]*1000:.1f}mm/s = 0.5 x jog_speed x{node._link_scale:.2f}")
ws_send(cbor_f32(float("nan"), 0.0, 0.0))
time.sleep(0.05)
check("T NaN frame zeroes the twist", node._jog[0] == 0.0,
      f"_jog[0]={node._jog[0]}")

# E ── recovery: clean stream decays the peak-hold back to x1.00 ──────────────
stream(14.0)
st = state()
check("E recovers to full speed", st["linkscale"] >= 0.95,
      f"scale=x{st['linkscale']} lease={st['lease']}ms after 14s clean")

print("RESULT:", "ALL PASS" if fails == 0 else f"{fails} FAILURES", flush=True)
sys.stdout.flush()
os._exit(fails)            # daemon threads (web/server loops) won't join
