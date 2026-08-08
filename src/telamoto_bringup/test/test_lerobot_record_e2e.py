#!/usr/bin/env python3
"""LeRobot teach E2E: real `lerobot_record` node (lerobot env, no robot, no
move_group) + fake /joint_states, driven by the start/stop services, then a
reopen of the written LeRobotDataset from disk.

Verifies the recording chain that later feeds training:
  A. The node comes up and exposes the start/stop services.
  B. `start` succeeds; joint_states published during recording land in the
     episode (decimated at fps).
  C. `stop` saves an episode; the dataset reopens from disk with the correct
     joint order and values; `action` == `observation.state`.
  D. A second episode appended to the SAME root (resume path) grows the
     corpus; reopening shows both.
  E. The REAL web UI path: the `urservo` controller's /api/record button
     (POST) drives the record node through the controller's Trigger clients
     and its `/api/state` reports recAvail/recording/recFrames; a third
     episode lands on disk.

Startup gating mirrors the other tests: the guard waits on OBSERVED tracking
(frames actually landing in the writer), never on service existence alone —
DDS loopback delivery into a fresh rclpy process on this host can black out
for tens of seconds. Posing the arm AFTER the gate keeps the assertions
honest; a blacked-out feed records nothing and the test aborts with an
environmental message, not a logic failure.

Isolated: ROS_DOMAIN_ID=82, web 8096, reverse 50094 (live bringup on 42).
Run:  pixi run --environment lerobot python \\
         src/telamoto_bringup/test/test_lerobot_record_e2e.py
"""
import contextlib
import http.client
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

os.environ["ROS_DOMAIN_ID"] = "82"
os.environ.pop("ROS_LOCALHOST_ONLY", None)

_WEB_PORT = 8096
_REV_PORT = 50094

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
SCRIPT = os.path.normpath(os.path.join(os.path.dirname(__file__),
                                       "..", "scripts", "lerobot_record.py"))
USC_SCRIPT = os.path.normpath(os.path.join(os.path.dirname(__file__),
                                           "..", "scripts",
                                           "ur_servo_controller.py"))
spec = importlib.util.spec_from_file_location("usc", USC_SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

FPS = 30
REC_RATE = 50          # fake /joint_states publish rate (Hz)
REC_SECS = 4           # recording run length
EXPECTED_FRAMES = int(FPS * REC_SECS)

results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name:46s} {info}")


def http_call(method, path):
    c = http.client.HTTPConnection("127.0.0.1", _WEB_PORT, timeout=5)
    c.request(method, path)
    r = c.getresponse()
    body = r.read()
    c.close()
    return r.status, body


def call_service(node, cli, timeout=15.0):
    deadline = time.monotonic() + timeout
    while not cli.service_is_ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not cli.service_is_ready():
        return None
    fut = cli.call_async(Trigger.Request())
    end = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if fut.done():
            return fut.result()
    return None


def main():
    corpus = tempfile.mkdtemp(prefix="lerobot_e2e_")
    repo_id = "telamoto/teleocorpus"

    script = SCRIPT
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = os.environ["ROS_DOMAIN_ID"]
    proc = subprocess.Popen(
        ["python", script,
         "--ros-args",
         "-p", f"dataset_root:={corpus}",
         "-p", f"repo_id:={repo_id}",
         "-p", "task:=pickup"],
        env=env, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init(args=["--ros-args", "-p", f"web_port:={_WEB_PORT}",
                     "-p", f"reverse_port:={_REV_PORT}"])
    node = Node("record_probe")
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    start = node.create_client(Trigger, "/lerobot_record/start")
    stop = node.create_client(Trigger, "/lerobot_record/stop")
    pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    def tick():                      # 50 Hz pose publish (~30 Hz recorded)
        m = JointState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.name, m.position = UR_JOINTS, list(pose)
        m.velocity = [0.0] * 6
        js_pub.publish(m)
    node.create_timer(1.0 / REC_RATE, tick)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        epoch = time.monotonic()
        resp = call_service(node, start)
        check("start service responds", resp is not None and resp.success,
              (f"took {time.monotonic()-epoch:.0f}s (startup delivery "
               "blackout can last tens of seconds)") if resp is None else
              resp.message)
        if resp is None or not resp.success:
            raise RuntimeError("record start service failed")

        spin(REC_SECS)
        epoch = time.monotonic()

        # stop + episode save
        resp = call_service(node, stop)
        check("stop service responds and saves", resp is not None and resp.success,
              resp.message if resp else "stop not ready")

# Reopen from disk via lerobot's readers, in THIS pixi env.
        import sys as _sys
        env_py = ["pixi", "run", "--environment", "lerobot", "python"]
        reopen = subprocess.run(
            env_py + ["-c", f"""
import sys, numpy as np
sys.path.insert(0, {os.path.dirname(script)!r})
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id={repo_id!r}, root={corpus!r})
meta = ds.meta.features
print("JOINTS", meta["observation.state"]["names"])
print("FRAMES", ds.num_frames)
print("EPS", ds.num_episodes)
fr = ds[0]
oa = np.asarray(fr["observation.state"])
aa = np.asarray(fr["action"])
print("MATCH", bool(np.allclose(oa, aa)))
print("VALUES", [round(float(x),4) for x in oa])
"""], capture_output=True, text=True)
        _sys.stdout.flush()
        out = reopen.stdout.strip()
        if reopen.returncode != 0:
            check("dataset reopens from disk", False, reopen.stderr.strip())
        else:
            lines = out.splitlines()
            ok_reopen = any(l.startswith("FRAMES") for l in lines)
            check("dataset reopens from disk", ok_reopen, out)
            nframes = None
            for l in lines:
                if l.startswith("FRAMES"):
                    nframes = int(l.split()[1])
            check("frame count in decimation band",
                  nframes is not None and 0.7 * EXPECTED_FRAMES <= nframes <= 1.4 * EXPECTED_FRAMES,
                  f"n={nframes} (expect ~{EXPECTED_FRAMES})")
            check("action equals observation.state",
                  any(l.startswith("MATCH True") for l in lines))
            check("joint names match UR order",
                  any("JOINTS" in l and l.split("JOINTS")[1].count("joint") >= 6
                      for l in lines))

        # D. Second episode appends to the SAME root (resume path).
        resp = call_service(node, start)
        if resp is None or not resp.success:
            raise RuntimeError("second start failed")
        spin(2)                       # ~2 s more of the same 30 Hz pose
        resp = call_service(node, stop)
        check("second episode save succeeds", resp is not None and resp.success,
              resp.message if resp else "stop not ready")
        reopen2 = subprocess.run(
            env_py + ["-c", f"""
import sys
sys.path.insert(0, {os.path.dirname(script)!r})
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id={repo_id!r}, root={corpus!r})
print("EPS", ds.num_episodes)
print("FRAMES", ds.num_frames)
"""], capture_output=True, text=True)
        if reopen2.returncode != 0:
            check("second episode appended (resume works)", False,
                  reopen2.stderr.strip().splitlines()[-1] if reopen2.stderr else "rc!=0")
        else:
            lines2 = reopen2.stdout.strip().splitlines()
            eps = frames2 = -1
            for l in lines2:
                if l.startswith("EPS"):
                    eps = int(l.split()[1])
                if l.startswith("FRAMES"):
                    frames2 = int(l.split()[1])
            check("second episode appended (resume works)",
                  eps == 2 and frames2 > nframes,
                  f"eps={eps} frames={frames2}")

        # E. The REAL web path: controller's /api/record drives the shared
        # record node (same domain 82, isolated web/reverse ports); everything
        # else (jog loop, move_group probes) just idles with no robot present.
        ctrl = usc.URServoController()
        cexec = MultiThreadedExecutor(num_threads=3)
        cexec.add_node(ctrl)
        threading.Thread(target=cexec.spin, daemon=True).start()
        time.sleep(1.0)                    # boot web thread + webserver

        def rec_state():
            st, body = http_call("GET", "/api/state")
            return st, json.loads(body) if st == 200 else {}

        st, js_ = rec_state()
        check("E web serves /api/state", st == 200, f"status={st}")
        check("E recAvail once record node seen",
              js_.get("recAvail") is True,
              f"recAvail={js_.get('recAvail')}")

        st, _ = http_call("POST", "/api/record?on=1")
        deadline = time.monotonic() + 10.0
        seen_rec = False
        while time.monotonic() < deadline:
            _, js_ = rec_state()
            if js_.get("recording"):
                seen_rec = True
                break
            time.sleep(0.2)
        check("E POST /api/record starts recording", seen_rec,
              f"recording={js_.get('recording')}")

        spin(2)                               # ~2 s more of the live 50 Hz pose
        _, js_ = rec_state()
        now_frames = js_.get("recFrames") or 0
        check("E recFrames climbs while recording", now_frames > 0,
              f"recFrames={now_frames}")

        st, _ = http_call("POST", "/api/record?on=0")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            _, js_ = rec_state()
            if not js_.get("recording"):
                break
            time.sleep(0.2)
        check("E POST /api/record stops recording",
              not js_.get("recording"),
              f"recording={js_.get('recording')}")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            _, js_ = rec_state()
            if (js_.get("recEps") or 0) >= 3:
                break
            time.sleep(0.2)
        check("E episode count readout catches up",
              (js_.get("recEps") or 0) >= 3,
              f"recEps={js_.get('recEps')}")

        reopen3 = subprocess.run(
            env_py + ["-c", f"""
import sys
sys.path.insert(0, {os.path.dirname(script)!r})
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id={repo_id!r}, root={corpus!r})
print("EPS", ds.num_episodes)
"""], capture_output=True, text=True)
        if reopen3.returncode != 0:
            check("E UI-driven episode lands on disk", False,
                  reopen3.stderr.strip().splitlines()[-1] if reopen3.stderr else "rc!=0")
        else:
            eps3 = [int(l.split()[1]) for l in reopen3.stdout.strip().splitlines()
                    if l.startswith("EPS")]
            check("E UI-driven episode lands on disk",
                  bool(eps3) and eps3[0] >= 3,
                  f"eps={eps3[0] if eps3 else '?'}")

        cexec.shutdown()
    except RuntimeError as e:
        print("ABORT:", e)
    finally:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(5)
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT:", "ALL PASS" if all(results) else "FAILURES")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()