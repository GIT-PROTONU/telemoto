#!/usr/bin/env python3
"""Movable wall in the MoveIt planning scene — collision-detection test fixture.

Publishes a box CollisionObject ("test_wall") and wraps it in an RViz
interactive marker: drag the wall (or its axis arrows / yaw ring) and the
planning scene follows. Everything that reads the scene reacts:
  - move_group  → planned moves route around / refuse to enter the wall
  - MoveIt Servo (check_collisions: true) → jog decelerates near the wall and
    halts on contact; ur_servo_controller turns that status into the speedl gate
  - RViz MotionPlanning display → wall visible under Scene Geometry

The CollisionObject is re-ADDed (replace semantics) on every drag and on a 1 Hz
keepalive, so a move_group restart re-acquires the wall automatically. On clean
shutdown the wall is REMOVEd from the scene.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Quaternion
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

WALL_ID = "test_wall"


class SceneWall(Node):
    def __init__(self) -> None:
        super().__init__("scene_wall")

        self.declare_parameter("frame_id", "world")     # move_group planning frame
        self.declare_parameter("size",     [0.05, 1.2, 1.2])   # x thickness, y width, z height [m]
        self.declare_parameter("initial_xyz", [0.8, 0.0, 0.6])  # wall centre

        self._frame = self.get_parameter("frame_id").value
        self._size  = list(self.get_parameter("size").value)
        xyz         = list(self.get_parameter("initial_xyz").value)

        self._pose = Pose()
        self._pose.position.x, self._pose.position.y, self._pose.position.z = xyz
        self._pose.orientation.w = 1.0

        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self._server = InteractiveMarkerServer(self, "scene_wall")
        self._server.insert(self._make_marker(), feedback_callback=self._feedback)
        self._server.applyChanges()

        # Keepalive: replace-ADD once a second so a (re)started move_group or
        # Servo planning-scene monitor always converges on the current pose.
        self.create_timer(1.0, self._publish_wall)
        self._publish_wall()
        self.get_logger().info(
            f"wall '{WALL_ID}' {self._size} m in '{self._frame}' at {xyz} — "
            "drag it in RViz (InteractiveMarkers display, namespace /scene_wall)")

    # ── planning scene ──────────────────────────────────────────────────────

    def _collision_object(self, operation: int) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = WALL_ID
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self._size)
        obj.primitives = [box]
        obj.primitive_poses = [self._pose]
        obj.operation = operation
        return obj

    def _publish_wall(self, operation: int = CollisionObject.ADD) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [self._collision_object(operation)]
        self._scene_pub.publish(scene)

    # ── interactive marker ──────────────────────────────────────────────────

    def _make_marker(self) -> InteractiveMarker:
        im = InteractiveMarker()
        im.header.frame_id = self._frame
        im.name = WALL_ID
        im.description = "test wall — drag me"
        im.pose = self._pose
        im.scale = max(self._size) * 1.2

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x, box.scale.y, box.scale.z = self._size
        box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.55, 0.1, 0.55

        s = 1.0 / math.sqrt(2.0)

        # The wall body itself slides on the floor plane (control X axis = world Z).
        body = InteractiveMarkerControl()
        body.name = "slide_xy"
        body.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        body.orientation = Quaternion(w=s, y=s)
        body.always_visible = True
        body.markers.append(box)
        im.controls.append(body)

        for name, mode, quat in (
            ("move_x",   InteractiveMarkerControl.MOVE_AXIS,   Quaternion(w=s, x=s)),
            ("move_y",   InteractiveMarkerControl.MOVE_AXIS,   Quaternion(w=s, z=s)),
            ("move_z",   InteractiveMarkerControl.MOVE_AXIS,   Quaternion(w=s, y=s)),
            ("rotate_z", InteractiveMarkerControl.ROTATE_AXIS, Quaternion(w=s, y=s)),
        ):
            c = InteractiveMarkerControl()
            c.name = name
            c.interaction_mode = mode
            c.orientation = quat
            im.controls.append(c)
        return im

    def _feedback(self, fb: InteractiveMarkerFeedback) -> None:
        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._pose = fb.pose
            self._publish_wall()

    def remove_wall(self) -> None:
        self._publish_wall(CollisionObject.REMOVE)


def main() -> None:
    rclpy.init()
    node = SceneWall()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Best effort — on external shutdown (launch SIGINT) the context is
        # already invalid and the wall stays in the scene; that's fine.
        if rclpy.ok():
            node.remove_wall()
        node.destroy_node()


if __name__ == "__main__":
    main()
