#!/usr/bin/env python3
"""FollowJointTrajectory action server that executes trajectories via URScript
on the UR secondary interface (port 30002).

No RTDE inputs and no ur_robot_driver required — avoids the on_configure()
SIGABRT on CB3 3.15 where PolyScopeX holds all RTDE input variables.
Joint states are expected from ur_rtde_joint_pub.py on /joint_states.
"""
import asyncio
import socket
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory

UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

URSCRIPT_PORT = 30002


class URServoController(Node):
    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip", "192.168.10.2")
        self._robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        cb = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self.get_logger().info(
            f"URScript trajectory executor ready — robot: {self._robot_ip}:{URSCRIPT_PORT}"
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _reorder(msg_joints: list[str], positions: list[float]) -> list[float]:
        """Return positions in UR joint order."""
        idx = [msg_joints.index(j) for j in UR_JOINT_ORDER]
        return [positions[i] for i in idx]

    @staticmethod
    def _build_script(trajectory) -> str:
        jnames = list(trajectory.joint_names)
        lines = ["def _traj():"]
        prev_t = 0.0
        for pt in trajectory.points:
            q = URServoController._reorder(jnames, list(pt.positions))
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = max(t - prev_t, 0.008)
            prev_t = t
            q_str = ", ".join(f"{v:.6f}" for v in q)
            lines.append(
                f"  servoj([{q_str}], a=1.4, v=0.5,"
                f" t={dt:.4f}, lookahead_time=0.1, gain=300)"
            )
        lines += ["end", "_traj()", ""]
        return "\n".join(lines)

    def _run_script(self, script: str) -> None:
        """Send URScript and block until the robot closes the connection."""
        with socket.create_connection((self._robot_ip, URSCRIPT_PORT), timeout=10) as s:
            s.sendall(script.encode())
            s.settimeout(120.0)
            while s.recv(4096):
                pass  # drain until the robot closes

    # ── action callback ───────────────────────────────────────────────────────

    async def _execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            r = FollowJointTrajectory.Result()
            r.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return r

        script = self._build_script(traj)
        self.get_logger().info(
            f"Executing {len(traj.points)}-point trajectory via URScript ..."
        )

        try:
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._run_script, script)
        except Exception as exc:
            self.get_logger().error(f"URScript execution failed: {exc}")
            goal_handle.abort()
            r = FollowJointTrajectory.Result()
            r.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            return r

        goal_handle.succeed()
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory complete")
        return r


def main() -> None:
    rclpy.init()
    node = URServoController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
