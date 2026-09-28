#!/usr/bin/env python3
"""
GP7 Arm Controller Node
========================
Sends JointTrajectory goals to the arm_controller running inside Gazebo Sim.

Preset positions
----------------
0  home        — all joints at 0 rad
1  reach_out   — arm stretched forward
2  compact     — folded-up park position
3  wave        — sweeping joint_1_s back and forth

Usage
-----
After launching Gazebo:
  ros2 run gp7_ros2_pkg arm_controller_node

Then publish a goal index (0-3):
  ros2 topic pub /gp7/goal_index std_msgs/msg/Int32 "{data: 1}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration
import math


# ── Joint names as declared in gp7.urdf ros2_control section ────────────────
ARM_JOINTS = [
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_6_t',
]

# ── Preset waypoints [j1_s, j2_l, j3_u, j4_r, j5_b, j6_t] (radians) ───────
# Each preset is a list of (positions, time_from_start_sec) tuples.
PRESETS = {
    0: {
        'name': 'home',
        'waypoints': [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0),
        ],
    },
    1: {
        'name': 'reach_out',
        'waypoints': [
            ([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], 2.0),
            ([0.0, 0.8, 0.8, 0.0, 0.0, 0.0], 4.0),
            ([0.0, 0.8, 1.2, -0.5, 0.0, 0.0], 6.0),
        ],
    },
    2: {
        'name': 'compact',
        'waypoints': [
            ([0.0, -1.1, 3.0, 0.0, 1.5, 0.0], 5.0),
        ],
    },
    3: {
        'name': 'wave',
        'waypoints': [
            ([1.0, 0.5, 0.5, 0.0, 0.0, 0.0], 2.0),
            ([-1.0, 0.5, 0.5, 0.0, 0.0, 0.0], 5.0),
            ([0.0, 0.5, 0.5, 0.0, 0.0, 0.0], 7.0),
        ],
    },
}


class ArmControllerNode(Node):
    """ROS 2 node that sends JointTrajectory action goals to arm_controller."""

    def __init__(self):
        super().__init__('gp7_arm_controller_node')

        # ── Action client ────────────────────────────────────────────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        # ── Track whether we have received at least one valid joint state ────
        # The JointTrajectoryController rejects goals when its hardware state
        # interfaces contain NaN (i.e. before the first joint state message).
        self._joint_states_valid = False
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        # ── Subscription: listen for goal index commands ─────────────────────
        self._goal_sub = self.create_subscription(
            Int32,
            '/gp7/goal_index',
            self._goal_index_callback,
            10,
        )

        self.get_logger().info(
            'GP7 arm controller node started.\n'
            'Publish to /gp7/goal_index (std_msgs/Int32) to select a preset:\n'
            '  0 → home\n'
            '  1 → reach_out\n'
            '  2 → compact\n'
            '  3 → wave\n'
            'Waiting for first /joint_states message before accepting goals…'
        )

    # ── helpers ──────────────────────────────────────────────────────────────

    def _build_goal(self, preset_id: int) -> FollowJointTrajectory.Goal:
        preset = PRESETS[preset_id]
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINTS

        # Stamp the header with current time so the controller knows when
        # the trajectory reference epoch is.
        trajectory.header.stamp = self.get_clock().now().to_msg()

        for positions, t_sec in preset['waypoints']:
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            # Leave velocities/accelerations empty — the controller will
            # compute a smooth spline automatically.
            t_int = int(t_sec)
            t_nsec = int((t_sec - t_int) * 1e9)
            point.time_from_start = Duration(sec=t_int, nanosec=t_nsec)
            trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        return goal_msg

    def _send_goal(self, preset_id: int):
        if not self._joint_states_valid:
            self.get_logger().warn(
                'No valid /joint_states received yet. '
                'The controller is not ready — try again in a moment.'
            )
            return

        preset_name = PRESETS[preset_id]['name']
        self.get_logger().info('Waiting for arm_controller action server…')

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'arm_controller action server not available! '
                'Is Gazebo running with the arm_controller loaded?'
            )
            return

        goal_msg = self._build_goal(preset_id)
        self.get_logger().info(
            f'Sending goal → preset {preset_id} ("{preset_name}")'
        )
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        """Mark states as valid once we receive a non-empty JointState msg."""
        if not self._joint_states_valid and len(msg.position) > 0:
            self._joint_states_valid = True
            self.get_logger().info(
                '✅ /joint_states received — arm controller is ready for goals.'
            )

    def _goal_index_callback(self, msg: Int32):
        idx = msg.data
        if idx not in PRESETS:
            self.get_logger().warn(
                f'Unknown preset index {idx}. Valid range: 0–{len(PRESETS)-1}'
            )
            return
        self._send_goal(idx)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Goal was REJECTED by the controller.\n'
                'This usually means:\n'
                '  • arm_controller is not in "active" state\n'
                '    → Run: ros2 control list_controllers\n'
                '  • Joint names mismatch\n'
                '  • /joint_states not yet populated (NaN state)'
            )
            return
        self.get_logger().info('Goal ACCEPTED — executing trajectory…')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pass  # Uncomment for verbose feedback:
        # fb = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: {fb}')

    def _result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Trajectory completed successfully.')
        else:
            self.get_logger().warn(
                f'⚠️  Trajectory finished with error {result.error_code}: '
                f'{result.error_string}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down arm controller node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
