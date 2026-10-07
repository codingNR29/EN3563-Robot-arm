#!/usr/bin/env python3
"""
GP7 Arm Controller Node
========================
Sends JointTrajectory goals to the arm_controller running inside Gazebo Sim.

Topics
------
  /gp7/goal_index  (std_msgs/Int32)  — select a preset (0-3)
  /gp7/stop        (std_msgs/Empty)  — cancel motion and hold current position

Preset positions
----------------
  0  home        — all joints at 0 rad
  1  reach_out   — arm stretched forward
  2  compact     — folded-up park position
  3  wave        — sweeping joint_1_s back and forth

Usage
-----
  ros2 run gp7_ros2_pkg arm_controller_node

  # Move
  ros2 topic pub /gp7/goal_index std_msgs/msg/Int32 "{data: 1}" --once

  # Stop immediately
  ros2 topic pub /gp7/stop std_msgs/msg/Empty "{}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Empty
from builtin_interfaces.msg import Duration


# ── Joint names as declared in gp7.urdf ros2_control section ────────────────
ARM_JOINTS = [
    'joint_1_s',
    'joint_2_l',
    'joint_3_u',
    'joint_4_r',
    'joint_5_b',
    'joint_6_t',
]

# ── Preset waypoints ─────────────────────────────────────────────────────────
# Format: list of (positions, time_from_start_seconds) tuples.
# positions = [j1_s, j2_l, j3_u, j4_r, j5_b, j6_t] in RADIANS.
# The controller computes smooth splines between waypoints automatically.
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
            ([0.0, 0.5,  0.0,  0.0,  0.0, 0.0], 2.0),
            ([0.0, 0.8,  0.8,  0.0,  0.0, 0.0], 4.0),
            ([0.0, 0.8,  1.2, -0.5,  0.0, 0.0], 6.0),
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
            ([ 1.0, 0.5, 0.5, 0.0, 0.0, 0.0], 2.0),
            ([-1.0, 0.5, 0.5, 0.0, 0.0, 0.0], 5.0),
            ([ 0.0, 0.5, 0.5, 0.0, 0.0, 0.0], 7.0),
        ],
    },
}


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('gp7_arm_controller_node')

        # ── Action client ─────────────────────────────────────────────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        # Current active goal handle (so we can cancel it on /gp7/stop)
        self._current_goal_handle = None

        # Last known joint positions (used to build a hold-in-place trajectory)
        self._current_positions = [0.0] * 6

        # Volatile QoS — node will NOT receive messages that were published
        # before it started.  Prevents stale ros2 topic pub processes from
        # triggering motion on restart.
        _volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._joint_states_valid = False
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, _volatile_qos
        )
        self._goal_sub = self.create_subscription(
            Int32, '/gp7/goal_index', self._goal_index_callback, _volatile_qos
        )
        self._stop_sub = self.create_subscription(
            Empty, '/gp7/stop', self._stop_callback, _volatile_qos
        )

        self.get_logger().info(
            'GP7 arm controller node started.\n'
            'Publish to /gp7/goal_index (std_msgs/Int32) to move:\n'
            '  0 → home\n'
            '  1 → reach_out\n'
            '  2 → compact\n'
            '  3 → wave\n'
            'Publish to /gp7/stop (std_msgs/Empty) to stop immediately.\n'
            'Waiting for first /joint_states message…'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _make_trajectory(self, waypoints):
        """Build a JointTrajectory message from a list of (positions, t) tuples."""
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        traj.header.stamp = self.get_clock().now().to_msg()
        for positions, t_sec in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            t_int = int(t_sec)
            pt.time_from_start = Duration(sec=t_int,
                                          nanosec=int((t_sec - t_int) * 1e9))
            traj.points.append(pt)
        return traj

    def _send_trajectory(self, waypoints, label='trajectory'):
        if not self._joint_states_valid:
            self.get_logger().warn(
                'No /joint_states received yet — controller not ready. Try again.'
            )
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller action server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self._make_trajectory(waypoints)

        self.get_logger().info(f'Sending goal → {label}')
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg: JointState):
        # Track current positions so we can hold-in-place on stop
        if all(j in dict(zip(msg.name, msg.position)) for j in ARM_JOINTS):
            # msg.name may not match ARM_JOINTS order — map by name
            name_to_pos = dict(zip(msg.name, msg.position))
            self._current_positions = [
                name_to_pos.get(j, 0.0) for j in ARM_JOINTS
            ]
            if not self._joint_states_valid:
                self._joint_states_valid = True
                self.get_logger().info(
                    '✅ /joint_states received — arm controller ready for goals.'
                )

    def _goal_index_callback(self, msg: Int32):
        idx = msg.data
        if idx not in PRESETS:
            self.get_logger().warn(
                f'Unknown preset {idx}. Valid: 0–{len(PRESETS)-1}'
            )
            return
        preset = PRESETS[idx]
        # Cancel any running trajectory first
        self._cancel_current_goal()
        self._send_trajectory(preset['waypoints'],
                              label=f'preset {idx} ("{preset["name"]}")')

    def _stop_callback(self, _msg):
        """Cancel current motion and hold the robot at its current position."""
        self.get_logger().info('🛑 Stop received — cancelling motion.')
        self._cancel_current_goal()
        # Send a hold-in-place waypoint 0.5 s from now
        hold = [(list(self._current_positions), 0.5)]
        self._send_trajectory(hold, label='HOLD (stop)')

    def _cancel_current_goal(self):
        if self._current_goal_handle is not None:
            self.get_logger().info('Cancelling previous goal…')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Goal REJECTED.\n'
                'Check: ros2 control list_controllers — all 3 must be "active".'
            )
            return
        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal ACCEPTED — executing…')
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pass  # Uncomment to see live feedback:
        # self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def _result_callback(self, future):
        self._current_goal_handle = None
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Trajectory completed.')
        else:
            self.get_logger().warn(
                f'⚠️  Trajectory ended — code {result.error_code}: '
                f'{result.error_string}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
