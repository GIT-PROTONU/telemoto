#!/usr/bin/env python3
"""Collision cage in the MoveIt planning scene — six movable walls.

Publishes box CollisionObjects enclosing the workspace (wall_front/back/left/
right/top/floor), each wrapped in an RViz interactive marker that slides along
its own normal: drag a wall and the planning scene follows. Everything that
reads the scene reacts:
  - move_group  → planned moves route around / refuse to leave the cage
  - MoveIt Servo (check_collisions: true) → jog decelerates near a wall and
    halts on contact; ur_servo_controller turns that status into the speedl gate
  - RViz MotionPlanning display → walls visible under Scene Geometry
  - the web UI's 3D twin renders every BOX collision object

⚠ The FLOOR needs special care: the robot's base and shoulder links are
PERMANENTLY near z=0, which would pin Servo's collision monitor at "contact"
and halt the jog forever. So the floor is collision-ALLOWED against
base_link / base_link_inertia / shoulder_link through the planning-scene ACM:
the current ACM is fetched from move_group (/get_planning_scene), extended
with the floor entries, and included in every keepalive publish (the planning
scene monitor replaces its ACM whenever a diff carries a non-empty one, so
the SRDF self-collision entries ride along unchanged — never publish a
partial ACM). Until that fetch succeeds the floor is NOT added: a cage
without a floor jogs fine, a floor without the ACM bricks the jog.

The objects are re-ADDed (replace semantics) on every drag and on a 1 Hz
keepalive, so a move_group restart re-acquires the cage automatically. On
clean shutdown the cage is REMOVEd from the scene.

Wall poses PERSIST across restarts/reboots: every drag (on mouse-up) writes
all poses to `pose_file` (default ~/.ros/telamoto_cage_poses.yaml), which is
loaded over the parameter defaults at startup. Delete the file (or a wall's
entry) to get the defaults back.
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Quaternion
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

FLOOR_ID = "wall_floor"
# Links that physically cannot reach the table the robot is bolted to — the
# floor is excluded from collision against exactly these, nothing else.
FLOOR_ALLOWED_LINKS = ("base_link", "base_link_inertia", "shoulder_link")
# MOVE_AXIS moves along the control's X axis: quaternions rotating X onto each
# world axis (s = 1/√2).
_S = 1.0 / math.sqrt(2.0)
_AXIS_QUAT = {
    "x": Quaternion(w=1.0),
    "y": Quaternion(w=_S, z=_S),
    "z": Quaternion(w=_S, y=-_S),
}


class SceneWall(Node):
    def __init__(self) -> None:
        super().__init__("scene_wall")

        self.declare_parameter("frame_id", "world")  # move_group planning frame
        # Inner faces of the cage [m]: vertical walls at ±half_x / ±half_y,
        # ceiling at top_z, floor surface at floor_z (slightly below the base
        # plane so the ACM exclusion is belt-and-braces, not load-bearing).
        self.declare_parameter("half_x",    0.8)
        self.declare_parameter("half_y",    0.8)
        self.declare_parameter("top_z",     1.6)
        self.declare_parameter("floor_z",  -0.03)
        self.declare_parameter("thickness", 0.05)
        self.declare_parameter("include_floor", True)
        # Dragged wall poses persist here over restarts/reboots ("" = disable).
        self.declare_parameter("pose_file", "~/.ros/telamoto_cage_poses.yaml")

        self._frame = self.get_parameter("frame_id").value
        hx = float(self.get_parameter("half_x").value)
        hy = float(self.get_parameter("half_y").value)
        top = float(self.get_parameter("top_z").value)
        flo = float(self.get_parameter("floor_z").value)
        t = float(self.get_parameter("thickness").value)
        self._include_floor = bool(self.get_parameter("include_floor").value)

        sx, sy = 2 * hx + 2 * t, 2 * hy + 2 * t       # spans (corners overlap)
        h, zc = top - flo + 2 * t, (top + flo) / 2
        # id → [size, position, drag axis]; inner faces at the configured offsets.
        self._walls = {
            "wall_front": [[t, sy, h], [+(hx + t / 2), 0.0, zc], "x"],
            "wall_back":  [[t, sy, h], [-(hx + t / 2), 0.0, zc], "x"],
            "wall_left":  [[sx, t, h], [0.0, +(hy + t / 2), zc], "y"],
            "wall_right": [[sx, t, h], [0.0, -(hy + t / 2), zc], "y"],
            "wall_top":   [[sx, sy, t], [0.0, 0.0, top + t / 2], "z"],
            FLOOR_ID:     [[sx, sy, t], [0.0, 0.0, flo - t / 2], "z"],
        }
        if not self._include_floor:
            del self._walls[FLOOR_ID]
        self._poses = {}
        for wid, (size, xyz, _axis) in self._walls.items():
            p = Pose()
            p.position.x, p.position.y, p.position.z = xyz
            p.orientation.w = 1.0
            self._poses[wid] = p
        self._pose_file = os.path.expanduser(
            str(self.get_parameter("pose_file").value or ""))
        self._load_poses()

        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Floor ACM: fetched from move_group, extended, republished verbatim
        # afterwards. None = not fetched yet = floor withheld from the scene.
        self._acm = None
        self._acm_pending = False
        self._acm_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        if self._include_floor:
            self.create_timer(2.0, self._fetch_acm)

        self._server = InteractiveMarkerServer(self, "scene_wall")
        for wid in self._walls:
            self._server.insert(self._make_marker(wid), feedback_callback=self._feedback)
        self._server.applyChanges()

        # Keepalive: replace-ADD once a second so a (re)started move_group or
        # Servo planning-scene monitor always converges on the current cage.
        self.create_timer(1.0, self._publish_cage)
        self._publish_cage()
        self.get_logger().info(
            f"cage in '{self._frame}': x ±{hx} m, y ±{hy} m, top {top} m, "
            f"floor {flo} m — drag walls in RViz (InteractiveMarkers, "
            "namespace /scene_wall); floor appears once move_group's ACM is fetched")

    # ── pose persistence ────────────────────────────────────────────────────

    def _load_poses(self) -> None:
        if not self._pose_file:
            return
        try:
            with open(self._pose_file) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return
        except Exception as e:                        # noqa: BLE001 — defaults are safe
            self.get_logger().warn(
                f"pose file {self._pose_file} unreadable ({e}); using defaults")
            return
        loaded = []
        for wid, d in data.items():
            if wid not in self._poses:
                continue
            try:
                p = Pose()
                (p.position.x, p.position.y,
                 p.position.z) = (float(v) for v in d["position"])
                (p.orientation.x, p.orientation.y, p.orientation.z,
                 p.orientation.w) = (float(v) for v in
                                     d.get("orientation", (0.0, 0.0, 0.0, 1.0)))
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warn(f"pose file entry '{wid}' malformed ({e}); skipped")
                continue
            self._poses[wid] = p
            loaded.append(wid)
        if loaded:
            self.get_logger().info(
                f"restored {len(loaded)} wall pose(s) from {self._pose_file}")

    def _save_poses(self) -> None:
        if not self._pose_file:
            return
        data = {wid: {"position": [p.position.x, p.position.y, p.position.z],
                      "orientation": [p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w]}
                for wid, p in self._poses.items()}
        try:
            os.makedirs(os.path.dirname(self._pose_file), exist_ok=True)
            tmp = self._pose_file + ".tmp"
            with open(tmp, "w") as f:
                yaml.safe_dump(data, f)
            os.replace(tmp, self._pose_file)          # atomic: never a torn file
        except OSError as e:
            self.get_logger().warn(f"could not save wall poses ({e})")

    # ── floor ACM ───────────────────────────────────────────────────────────

    def _fetch_acm(self) -> None:
        if self._acm is not None or self._acm_pending or not self._acm_cli.service_is_ready():
            return
        self._acm_pending = True
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self._acm_cli.call_async(req).add_done_callback(self._acm_response)

    def _acm_response(self, fut) -> None:
        self._acm_pending = False
        try:
            acm = fut.result().scene.allowed_collision_matrix
        except Exception as e:                        # noqa: BLE001 — retry on next tick
            self.get_logger().warn(f"ACM fetch failed ({e}); floor withheld, retrying")
            return
        if not acm.entry_names:
            # An empty ACM would WIPE the SRDF self-collision entries on every
            # monitor when republished → adjacent links read as permanent
            # contact. move_group is probably not ready yet; retry.
            self.get_logger().warn("ACM from move_group is empty; floor withheld, retrying")
            return
        self._acm = self._extend_acm(acm)
        self.get_logger().info(
            f"ACM fetched ({len(acm.entry_names)} entries) — floor added, "
            f"collision-allowed against {', '.join(FLOOR_ALLOWED_LINKS)}")
        self._publish_cage()

    @staticmethod
    def _extend_acm(acm: AllowedCollisionMatrix) -> AllowedCollisionMatrix:
        if FLOOR_ID in acm.entry_names:
            return acm
        floor_col = [n in FLOOR_ALLOWED_LINKS for n in acm.entry_names]
        for entry, allowed in zip(acm.entry_values, floor_col):
            entry.enabled.append(allowed)
        acm.entry_names.append(FLOOR_ID)
        acm.entry_values.append(AllowedCollisionEntry(enabled=floor_col + [False]))
        return acm

    # ── planning scene ──────────────────────────────────────────────────────

    def _active_walls(self):
        for wid in self._walls:
            if wid == FLOOR_ID and self._acm is None:
                continue                              # never a floor without its ACM
            yield wid

    def _collision_object(self, wid: str, operation: int) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = wid
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                         dimensions=self._walls[wid][0])]
        obj.primitive_poses = [self._poses[wid]]
        obj.operation = operation
        return obj

    def _publish_cage(self, operation: int = CollisionObject.ADD) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            self._collision_object(wid, operation) for wid in self._active_walls()]
        if self._acm is not None and operation == CollisionObject.ADD:
            scene.allowed_collision_matrix = self._acm
        self._scene_pub.publish(scene)

    # ── interactive markers ─────────────────────────────────────────────────

    def _make_marker(self, wid: str) -> InteractiveMarker:
        size, _xyz, axis = self._walls[wid]
        im = InteractiveMarker()
        im.header.frame_id = self._frame
        im.name = wid
        im.description = f"{wid} — drag along {axis}"
        im.pose = self._poses[wid]
        im.scale = 0.6

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x, box.scale.y, box.scale.z = size
        box.color.r, box.color.g, box.color.b = 1.0, 0.55, 0.1
        box.color.a = 0.3 if wid in ("wall_top", FLOOR_ID) else 0.45

        body = InteractiveMarkerControl()             # grab the wall itself…
        body.name = f"move_{axis}"
        body.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        body.orientation = _AXIS_QUAT[axis]           # …slides along its normal
        body.always_visible = True
        body.markers.append(box)
        im.controls.append(body)

        arrows = InteractiveMarkerControl()
        arrows.name = f"axis_{axis}"
        arrows.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        arrows.orientation = _AXIS_QUAT[axis]
        im.controls.append(arrows)
        return im

    def _feedback(self, fb: InteractiveMarkerFeedback) -> None:
        if fb.marker_name not in self._poses:
            return
        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._poses[fb.marker_name] = fb.pose
            self._publish_cage()
        elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._save_poses()                        # drag finished → persist

    def remove_cage(self) -> None:
        self._publish_cage(CollisionObject.REMOVE)


def main() -> None:
    rclpy.init()
    node = SceneWall()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Best effort — on external shutdown (launch SIGINT) the context is
        # already invalid and the cage stays in the scene; that's fine.
        if rclpy.ok():
            node.remove_cage()
        node.destroy_node()


if __name__ == "__main__":
    main()
