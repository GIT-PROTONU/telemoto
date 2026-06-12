#!/usr/bin/env python3
"""E2E test for the WebRTC (UDP) jog transport.

Run manually, no robot or bringup needed (ports 8097/50097 must be free);
isolated on ROS_DOMAIN_ID=72 — safe to run beside a live bringup:

    pixi run python src/telamoto_bringup/test/test_rtc_jog_e2e.py

Instantiates the REAL URServoController and connects a REAL aiortc peer
(unordered/no-retransmit datachannel, loopback host candidates), verifying:
  R0. garbage offer -> 503, server stays up
  R1. signaling: POST /api/rtc answers and the "jog" datachannel opens
  R2. jog frame over the channel -> twist target at full speed (low latency)
  R3. zero frame stops
  R4. seq guard: a REORDERED stale frame (older mod-2^16 seq) is dropped —
      motion must not resurrect after the keyup zero — and newer seqs resume
  R5. keepalive (empty array) feeds the link estimator, never the twist
  R6. server-side jog-mode gate applies to the RTC path too
  R7. peer disconnect zeroes the jog and reaps the peer connection
"""
import asyncio
import importlib.util
import json
import os
import struct
import sys
import threading
import time
import urllib.request

_SCRIPT = os.path.join(os.path.dirname(__file__), os.pardir,
                       "scripts", "ur_servo_controller.py")
spec = importlib.util.spec_from_file_location("usc", _SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

from aiortc import RTCPeerConnection, RTCSessionDescription

import rclpy
os.environ["ROS_DOMAIN_ID"] = "72"
# rtc_stun '' = host candidates only: an unreachable STUN server must not
# stall the loopback answer.
rclpy.init(args=["--ros-args", "-p", "web_port:=8097", "-p", "reverse_port:=50097",
                 "-p", "rtc_stun:=''"])
node = usc.URServoController()
time.sleep(0.7)                                   # let the web thread bind

BASE = "http://127.0.0.1:8097"

def post(path, data=None):
    return urllib.request.urlopen(
        urllib.request.Request(BASE + path, data=data, method="POST"), timeout=10)

post("/api/jogmode?on=1")

def ci(n):
    m, u = (0x20, -1 - n) if n < 0 else (0x00, n)
    if u < 24:
        return bytes([m | u])
    if u < 256:
        return bytes([m | 24, u])
    return bytes([m | 25]) + struct.pack(">H", u)

def cbor_jog(lx, ly, lz, seq=None):
    if seq is None:
        return b"\x83" + ci(lx) + ci(ly) + ci(lz)
    return (b"\x87" + ci(lx) + ci(ly) + ci(lz)
            + ci(0) + ci(0) + ci(0) + ci(seq))

def wait(pred, timeout=1.0):
    t0 = time.perf_counter()
    while not pred() and time.perf_counter() - t0 < timeout:
        time.sleep(0.0002)
    return (time.perf_counter() - t0) * 1000.0

fails = 0
def check(name, cond, detail):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:34s} {detail}", flush=True)

# R0 ── malformed offer must 503, never kill the server ───────────────────────
try:
    post("/api/rtc", b"not json")
    code = 200
except urllib.error.HTTPError as e:
    code = e.code
check("R0 garbage offer -> 503", code == 503, f"HTTP {code}")

# R1 ── real aiortc peer: offer/answer + datachannel open ─────────────────────
async def connect():
    pc = RTCPeerConnection()           # no STUN: loopback host candidates
    ch = pc.createDataChannel("jog", ordered=False, maxRetransmits=0)
    opened = asyncio.Event()
    ch.on("open", opened.set)
    await pc.setLocalDescription(await pc.createOffer())
    body = json.dumps({"sdp": pc.localDescription.sdp,
                       "type": pc.localDescription.type}).encode()
    loop = asyncio.get_running_loop()
    resp = await loop.run_in_executor(None, lambda: post("/api/rtc", body).read())
    await pc.setRemoteDescription(RTCSessionDescription(**json.loads(resp)))
    await asyncio.wait_for(opened.wait(), 10)
    return pc, ch

# The client loop must run CONTINUOUSLY (like a browser's): aiortc's SCTP
# transmit queue and retransmit timers live on it — a start/stop loop delivers
# frames late, in bursts, with scrambled timing.
loop = asyncio.new_event_loop()
threading.Thread(target=loop.run_forever, daemon=True).start()
def run(coro, timeout=15):
    return asyncio.run_coroutine_threadsafe(coro, loop).result(timeout)

t0 = time.perf_counter()
pc, ch = run(connect())
check("R1 datachannel opens", ch.readyState == "open",
      f"signaling+ICE+DTLS+SCTP in {(time.perf_counter()-t0)*1000:.0f} ms")

def send(payload):
    loop.call_soon_threadsafe(ch.send, payload)

# R2/R3 ── start/stop over UDP ────────────────────────────────────────────────
send(cbor_jog(1, 0, 0, seq=1))
lat = wait(lambda: node._jog[0] != 0.0)
check("R2 keydown latency (rtc)", lat < 50.0, f"{lat:.2f} ms frame -> twist target")
check("R2 full speed", abs(node._jog[0] - node._jog_speed * node._link_scale) < 1e-9,
      f"_jog[0]={node._jog[0]:.4f} = jog_speed x{node._link_scale:.2f}")
send(cbor_jog(0, 0, 0, seq=2))
lat = wait(lambda: node._jog[0] == 0.0)
check("R3 keyup latency (rtc)", lat < 50.0, f"{lat:.2f} ms zero -> twist zeroed")

# R4 ── reordering: a stale pre-release twist must never resurrect motion ─────
send(cbor_jog(0, 0, 0, seq=10))                   # the keyup zero
time.sleep(0.05)
send(cbor_jog(1, 0, 0, seq=9))                    # late, out-of-order move frame
time.sleep(0.15)
check("R4 stale frame dropped", node._jog[0] == 0.0,
      f"_jog[0]={node._jog[0]} after seq 9 arrived behind seq 10")
send(cbor_jog(1, 0, 0, seq=11))
lat = wait(lambda: node._jog[0] != 0.0)
check("R4 newer seq resumes", lat < 50.0, f"{lat:.2f} ms")
# mod-2^16 wraparound: a real client INCREMENTS through the wrap, so walk the
# guard forward in <32768 hops (a direct 11 -> 65535 jump is serially OLDER
# and is rightly dropped), then 65535 -> 3 is forward arithmetic and must pass
for hop in (32000, 64000, 65535):
    send(cbor_jog(0, 0, 0, seq=hop))
    time.sleep(0.01)
wait(lambda: node._jog[0] == 0.0)
send(cbor_jog(1, 0, 0, seq=3))
lat = wait(lambda: node._jog[0] != 0.0)
check("R4 seq wraparound forward", node._jog[0] != 0.0 and lat < 50.0,
      f"65535 -> 3 accepted in {lat:.2f} ms")
send(cbor_jog(0, 0, 0, seq=4))
wait(lambda: node._jog[0] == 0.0)

# R5 ── keepalive: estimator only, never the twist ────────────────────────────
tick0 = node._link_last_t
send(b"\x80")
wait(lambda: node._link_last_t > tick0, 0.5)
check("R5 keepalive feeds estimator", node._link_last_t > tick0,
      f"last_t advanced {node._link_last_t - tick0:.3f}s")
check("R5 keepalive never moves", node._jog[0] == 0.0, f"_jog[0]={node._jog[0]}")

# R6 ── server-side jog-mode gate covers the RTC path ─────────────────────────
post("/api/jogmode?on=0")
send(cbor_jog(1, 0, 0, seq=20))
time.sleep(0.15)
check("R6 jog-mode gate (rtc)", node._jog[0] == 0.0,
      f"_jog[0]={node._jog[0]} with jog mode OFF")
post("/api/jogmode?on=1")

# R7 ── disconnect: twist zeroed, peer connection reaped ──────────────────────
send(cbor_jog(1, 0, 0, seq=21))
wait(lambda: node._jog[0] != 0.0)
run(pc.close())
lat = wait(lambda: node._jog[0] == 0.0, 10.0)
check("R7 disconnect zeroes jog", node._jog[0] == 0.0, f"{lat:.0f} ms after close")
wait(lambda: len(node._rtc_pcs) == 0, 10.0)
check("R7 peer connection reaped", len(node._rtc_pcs) == 0,
      f"{len(node._rtc_pcs)} pcs left")

print("RESULT:", "ALL PASS" if fails == 0 else f"{fails} FAILURES", flush=True)
sys.stdout.flush()
os._exit(fails)            # daemon threads (web/server loops) won't join
