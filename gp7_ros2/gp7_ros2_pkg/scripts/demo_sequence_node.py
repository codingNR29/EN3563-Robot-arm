#!/usr/bin/env python3
"""
GP7 Demo Sequence Node
=======================
Runs an automated demonstration sequence on startup:
  1. Home position
  2. Reach out
  3. Open gripper
  4. Close gripper  
  5. Return home

This is a self-contained script — no external topic publication needed.
Simply run it while Gazebo is active:

  ros2 run gp7_ros2_pkg demo_sequence_node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import time


ARM_JOINTS = [
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t',
]

GRIPPER_MIMIC = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]


def make_trajectory(waypoints: list) -> JointTrajectory:
    """Build a JointTrajectory message from a list of (positions, time_sec)."""
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINTS
    for positions, t_sec in waypoints:
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(t_sec), nanosec=int((t_sec % 1) * 1e9))
        traj.points.append(pt)
    return traj


class DemoSequenceNode(Node):

    def __init__(self):
        super().__init__('gp7_demo_sequence_node')

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10,
        )

        self.get_logger().info('Demo sequence node started. Running in 3 seconds…')
        self.create_timer(3.0, self._run_sequence)
        self._sequence_done = False

    # ── helpers ──────────────────────────────────────────────────────────────

    def _send_arm_goal_sync(self, waypoints, label):
        """Send a trajectory goal and block until it finishes."""
        self.get_logger().info(f'▶  Arm → {label}')
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm action server not reachable!')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = make_trajectory(waypoints)

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal "{label}" REJECTED.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f'   ✅ {label} complete.')
            return True
        else:
            self.get_logger().warn(f'   ⚠️  {label} ended with error: {result.error_string}')
            return False

    def _send_gripper(self, driver_pos: float, label: str):
        positions = [driver_pos * m for m in GRIPPER_MIMIC]
        msg = Float64MultiArray()
        msg.data = positions
        self._gripper_pub.publish(msg)
        self.get_logger().info(f'▶  Gripper → {label} ({driver_pos:.2f} rad)')
        time.sleep(2.0)   # Give the gripper controller time to execute

    # ── main sequence ─────────────────────────────────────────────────────────

    def _run_sequence(self):
        if self._sequence_done:
            return
        self._sequence_done = True

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  GP7 Demo Sequence Starting')
        self.get_logger().info('═══════════════════════════════════════')

        # Step 1 — home
        self._send_arm_goal_sync(
            [([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)],
            'home',
        )

        # Step 2 — reach out
        self._send_arm_goal_sync(
            [
                ([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], 2.0),
                ([0.0, 0.8, 0.8, 0.0, 0.0, 0.0], 4.0),
                ([0.0, 0.8, 1.2, -0.5, 0.0, 0.0], 6.0),
            ],
            'reach_out',
        )

        # Step 3 — open gripper
        self._send_gripper(0.15, 'open')

        # Step 4 — close gripper
        self._send_gripper(-0.5, 'close')

        # Step 5 — open gripper again
        self._send_gripper(0.15, 'open')

        # Step 6 — return home
        self._send_arm_goal_sync(
            [([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 4.0)],
            'home (return)',
        )

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('  ✅ Demo sequence complete!')
        self.get_logger().info('═══════════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    node = DemoSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
