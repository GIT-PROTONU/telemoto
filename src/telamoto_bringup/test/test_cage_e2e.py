#!/usr/bin/env python3
"""E2E test for the collision cage (scene_wall.py).

Run manually, no robot or bringup needed:

    pixi run python src/telamoto_bringup/test/test_cage_e2e.py

Instantiates the REAL SceneWall node on an isolated ROS domain with a fake
move_group /get_planning_scene service, and verifies the floor/column-ACM
safety contract:
  A. before the ACM is fetched: 5 walls published, NO floor, NO base column,
     NO ACM (either object without its ACM exclusions would pin Servo's
     collision monitor at permanent contact = jog bricked)
  B. once the service answers: floor + column added, ACM extended (floor
     allowed against base/shoulder; column additionally against upper-arm/
     forearm; both STILL checked against the wrist), original entries
     preserved; column geometry = square 2×column_r, floor→top, on the axis
  C. pose persistence: poses load from pose_file at startup (unknown ids
     skipped), and a drag (POSE_UPDATE + MOUSE_UP feedback) saves all poses
"""
import importlib.util
import os
import sys
import threading
import time

import yaml

# Full isolation from a live bringup: scene diffs published here must never
# reach the real move_group/servo planning scene.
os.environ["ROS_DOMAIN_ID"] = "76"

_SCRIPT = os.path.join(os.path.dirname(__file__), os.pardir,
                       "scripts", "scene_wall.py")
spec = importlib.util.spec_from_file_location("sw", _SCRIPT)
sw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sw)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene
from moveit_msgs.srv import GetPlanningScene

# Pre-seed the (test-local!) pose file: wall_front moved, an unknown id that
# must be skipped. Never point this at the default ~/.ros file.
POSE_FILE = "/tmp/cage_e2e_poses.yaml"
with open(POSE_FILE, "w") as f:
    yaml.safe_dump({"wall_front": {"position": [0.55, 0.1, 0.7]},
                    "bogus_wall": {"position": [9.0, 9.0, 9.0]}}, f)

rclpy.init(args=["--ros-args", "-p", f"pose_file:={POSE_FILE}"])
node = sw.SceneWall()
helper = Node("cage_test")
scenes = []
helper.create_subscription(PlanningScene, "/planning_scene",
                           lambda m: scenes.append(m), 10)
ex = MultiThreadedExecutor(num_threads=3)
ex.add_node(node)
ex.add_node(helper)
threading.Thread(target=ex.spin, daemon=True).start()

fails = 0
def check(name, cond, detail=""):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:36s} {detail}", flush=True)

WALLS5 = {"wall_front", "wall_back", "wall_left", "wall_right", "wall_top"}

# A ── no ACM service yet: floor AND column must be withheld ──────────────────
time.sleep(3.0)                                   # a few keepalives
ids = {o.id for s in scenes for o in s.world.collision_objects}
check("A 5 walls, floor+column withheld", ids == WALLS5, str(sorted(ids)))
check("A no ACM republished pre-fetch",
      all(not s.allowed_collision_matrix.entry_names for s in scenes))
front = next(o for s in scenes for o in s.world.collision_objects
             if o.id == "wall_front")
check("A persisted pose loaded for front",
      abs(front.primitive_poses[0].position.x - 0.55) < 1e-9
      and abs(front.primitive_poses[0].position.y - 0.1) < 1e-9
      and abs(front.primitives[0].dimensions[0] - 0.05) < 1e-9,
      f"x={front.primitive_poses[0].position.x} (pose_file, not the 0.825 default)")
back = next(o for s in scenes for o in s.world.collision_objects
            if o.id == "wall_back")
check("A unlisted wall keeps its default",
      abs(back.primitive_poses[0].position.x + 0.825) < 1e-9,
      f"x={back.primitive_poses[0].position.x}")

# B ── fake move_group ACM (SRDF-ish): only base_link↔base_link_inertia allowed
NAMES = ["base_link", "base_link_inertia", "shoulder_link", "forearm_link",
         "wrist_1_link"]
ROWS = [[False, True, False, False, False],
        [True, False, False, False, False],
        [False, False, False, False, False],
        [False, False, False, False, False],
        [False, False, False, False, False]]

def srv_cb(req, res):
    acm = res.scene.allowed_collision_matrix
    acm.entry_names = list(NAMES)
    acm.entry_values = [AllowedCollisionEntry(enabled=r) for r in ROWS]
    return res

helper.create_service(GetPlanningScene, "/get_planning_scene", srv_cb)
got = None
deadline = time.monotonic() + 8.0                 # fetch timer ticks at 2 s
while time.monotonic() < deadline and got is None:
    got = next((s for s in scenes
                if any(o.id == "wall_floor" for o in s.world.collision_objects)),
               None)
    time.sleep(0.2)
check("B floor appears after ACM fetch", got is not None)
if got is not None:
    check("B all 7 cage objects published",
          {o.id for o in got.world.collision_objects}
          == WALLS5 | {"wall_floor", "wall_column"})
    acm = got.allowed_collision_matrix
    def allowed(a, b):
        return bool(acm.entry_values[acm.entry_names.index(a)]
                    .enabled[acm.entry_names.index(b)])
    check("B ACM carries floor + column", "wall_floor" in acm.entry_names
          and "wall_column" in acm.entry_names, str(list(acm.entry_names)))
    check("B floor allowed vs base/shoulder",
          allowed("wall_floor", "base_link") and allowed("base_link", "wall_floor")
          and allowed("wall_floor", "base_link_inertia")
          and allowed("wall_floor", "shoulder_link"))
    check("B floor STILL checked vs wrist", not allowed("wall_floor", "wrist_1_link")
          and not allowed("wrist_1_link", "wall_floor"))
    check("B floor checked vs forearm too", not allowed("wall_floor", "forearm_link"))
    check("B column allowed vs base..forearm",
          allowed("wall_column", "base_link")
          and allowed("wall_column", "shoulder_link")
          and allowed("wall_column", "forearm_link")
          and allowed("forearm_link", "wall_column"))
    check("B column STILL checked vs wrist",
          not allowed("wall_column", "wrist_1_link")
          and not allowed("wrist_1_link", "wall_column"))
    check("B SRDF entries preserved",
          allowed("base_link", "base_link_inertia")
          and not allowed("base_link", "shoulder_link"))
    col = next(o for o in got.world.collision_objects if o.id == "wall_column")
    dims = col.primitives[0].dimensions
    p = col.primitive_poses[0].position
    check("B column geometry on the axis",
          abs(dims[0] - 0.36) < 1e-9 and abs(dims[1] - 0.36) < 1e-9
          and abs(dims[2] - 1.63) < 1e-9 and abs(p.x) < 1e-9 and abs(p.y) < 1e-9
          and abs(p.z - (1.6 - 0.03) / 2) < 1e-9,
          f"dims={list(dims)} center z={p.z:.3f}")

# C ── drag → MOUSE_UP persists every pose ────────────────────────────────────
from visualization_msgs.msg import InteractiveMarkerFeedback
fb = InteractiveMarkerFeedback(marker_name="wall_top",
                               event_type=InteractiveMarkerFeedback.POSE_UPDATE)
fb.pose.position.z = 1.4
fb.pose.orientation.w = 1.0
node._feedback(fb)
fb.event_type = InteractiveMarkerFeedback.MOUSE_UP
node._feedback(fb)
with open(POSE_FILE) as f:
    saved = yaml.safe_load(f)
check("C drag saved to pose file",
      abs(saved["wall_top"]["position"][2] - 1.4) < 1e-9
      and abs(saved["wall_front"]["position"][0] - 0.55) < 1e-9
      and len(saved) == 7,
      f"wall_top z={saved['wall_top']['position'][2]}, {len(saved)} objects")

print("RESULT:", "ALL PASS" if fails == 0 else f"{fails} FAILURES", flush=True)
sys.stdout.flush()
os._exit(fails)
