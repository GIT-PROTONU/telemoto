#!/usr/bin/env python3
"""LeRobot teach node: records teleop demonstrations as a LeRobotDataset.

Runs best in the isolated `lerobot` pixi env (imports lerobot lazily at the
first record start — the node otherwise works in a plain ROS2 env as a
no-op status beacon):

    pixi run --environment lerobot python scripts/lerobot_record.py \
      --ros-args -p dataset_root:=~/.ros/teleocorpus -p task:=grab

Watches /joint_states (UR10 = shoulder_pan/lift/elbow, wrist_1/2/3). Each
ROS2 service call starts/stops an episode:

    ros2 service call /lerobot_record/start  std_srvs/srv/Trigger
    ros2 service call /lerobot_record/stop   std_srvs/srv/Trigger

While recording, every incoming /joint_states update becomes one LeRobot
frame: `observation.state` = measured joint positions and `action` = the same
positions (the robot jogs via speedl — there is no explicit joint-position
target on the wire; policies learn position targets from this and replay them
through servoj planned moves). `task` strings accumulate in the episode for
the dataset. `fps` defaults to 30; frames written faster than that are
decimated, jitter is tolerated by LeRobot's tolerance_s.

The dataset dir given by `--ros-args -p dataset_root:=<abs path>` is created
on the first start and the SAME root is resumed on later runs (append
episodes to an existing corpus). A `robot_type` default of `ur10` and the
`task` param seed the metadata. Episode metadata is only flushed on stop
(finalize), matching LeRobot's buffered parquet writer.
"""
import json
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import String as MsgString

# UR10 CB3.1 joint order (as published by ur_rtde_joint_pub.py).
UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class LeRobotRecord(Node):
    def __init__(self):
        super().__init__("lerobot_record")
        self.declare_parameter("dataset_root", str(Path.home() / ".lerobot/teleocorpus"))
        self.declare_parameter("repo_id", "telamoto/teleocorpus")
        self.declare_parameter("robot_type", "ur10")
        self.declare_parameter("fps", 30)
        self.declare_parameter("task", "teleop")
        self.declare_parameter("use_videos", False)
        self.declare_parameter("joints", UR_JOINTS)

        self._root = str(self.get_parameter("dataset_root").value)
        self._repo = self.get_parameter("repo_id").value
        self._robot_type = self.get_parameter("robot_type").value
        self._fps = int(self.get_parameter("fps").value)
        self._task = self.get_parameter("task").value
        self._use_videos = self.get_parameter("use_videos").value
        joints = self.get_parameter("joints").value
        self._joints = list(joints) if isinstance(joints, (list, tuple)) else UR_JOINTS

        self._lock = threading.RLock()
        self._ds = None          # lerobot LeRobotDataset (lazy import)
        self._recording = False
        self._frames = []
        self._last_frame_t = -1.0
        self._min_frame_interval = 1.0 / self._fps
        self._js_pos = {j: None for j in self._joints}   # fastest writer path
        self._total_frames = 0    # frames written across ALL episodes this run
        self._total_eps = 0       # episodes saved across ALL resumptions

        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10)
        self._srv_start = self.create_service(
            Trigger, "/lerobot_record/start", self._cb_start)
        self._srv_stop = self.create_service(
            Trigger, "/lerobot_record/stop", self._cb_stop)
        self._status_pub = self.create_publisher(
            MsgString, "/lerobot_record/status", 10)
        self.create_timer(0.5, self._publish_status)

    def _publish_status(self):
        with self._lock:
            msg = MsgString()
            msg.data = json.dumps({
                "recording": self._recording,
                "frames": len(self._frames),
                "episodes": self._total_eps,
            })
            self._status_pub.publish(msg)

    # ---- ROS callbacks ---------------------------------------------------
    def _js_cb(self, msg: JointState) -> None:
        with self._lock:
            if not self._recording:
                return
            pos = self._js_pos
            have = set(msg.name)
            for j in self._joints:
                if j in have:
                    pos[j] = msg.position[msg.name.index(j)]
            if any(pos[j] is None for j in self._joints):
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._last_frame_t >= 0 and (t - self._last_frame_t) < self._min_frame_interval:
                return
            self._last_frame_t = t
            vals = [pos[j] for j in self._joints]
            self._frames.append(
                {"observation.state": vals, "action": vals,
                 "task": self._task})

    def _cb_start(self, _req, resp):
        with self._lock:
            if self._recording:
                resp.success, resp.message = False, "already recording"
                return resp
            try:
                self._open_dataset()
            except Exception as exc:
                self.get_logger().error(f"dataset open failed: {exc}")
                resp.success, resp.message = False, str(exc)
                return resp
            self._frames = []
            self._last_frame_t = -1.0
            self._recording = True
            resp.success, resp.message = True, f"recording to {self._root}"
            return resp

    def _cb_stop(self, _req, resp):
        with self._lock:
            if not self._recording:
                resp.success, resp.message = False, "not recording"
                return resp
            self._recording = False
            try:
                self._save_episode()
                resp.success, resp.message = True, f"saved {len(self._frames)} frames"
            except Exception as exc:
                self.get_logger().error(f"save failed: {exc}")
                resp.success, resp.message = False, str(exc)
            return resp

    # ---- dataset ops (lerobot lives here, lazy) -------------------------
    def _open_dataset(self):
        """Create or resume the LeRobotDataset at `self._root`."""
        if self._ds is not None:
            return self._ds
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        import numpy as np  # noqa: F401
        root = Path(self._root)
        if root.exists():
            if not any(root.iterdir()):
                root.rmdir()   # LeRobotDataset.create needs a fresh path
            else:
                self._ds = LeRobotDataset.resume(self._repo, root=self._root)
                self.get_logger().info(f"appending to existing corpus {self._root}")
                return self._ds
        else:
            features = {
                "observation.state": {
                    "dtype": "float32", "shape": (len(self._joints),),
                    "names": list(self._joints)},
                "action": {"dtype": "float32", "shape": (len(self._joints),)},
            }
            self._ds = LeRobotDataset.create(
                repo_id=self._repo, root=self._root, fps=self._fps,
                features=features, robot_type=self._robot_type,
                use_videos=self._use_videos)
            self.get_logger().info(f"created corpus {self._root}")
        return self._ds

    def _save_episode(self):
        if not self._frames:
            self.get_logger().warn("no frames recorded this episode; discarding")
            return
        ds = self._open_dataset()
        if hasattr(ds, "add_frame"):
            import numpy as np
            for fr in self._frames:
                ds.add_frame({
                    "observation.state": np.array(fr["observation.state"], dtype=np.float32),
                    "action": np.array(fr["action"], dtype=np.float32),
                    "task": fr["task"]})
        ds.save_episode()
        ds.finalize()
        # A finalized writer is spent; the next episode must resume fresh.
        self._ds = None
        self._total_eps += 1
        self._total_frames += len(self._frames)
        self.get_logger().info(f"episode saved ({len(self._frames)} frames)")


def main():
    rclpy.init()
    node = LeRobotRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            if node._recording:
                node._recording = False
                try:
                    node._save_episode()
                except Exception:
                    pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()