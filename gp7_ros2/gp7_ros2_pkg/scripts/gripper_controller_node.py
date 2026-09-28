#!/usr/bin/env python3
"""
GP7 Gripper Controller Node
=============================
Sends position commands to the gripper_controller (JointGroupPositionController)
running inside Gazebo Sim via the /gripper_controller/commands topic.

The gripper has one primary "driver" joint (gripper_controller joint) that
mimic-links drive all other finger segments.

Usage
-----
After launching Gazebo:
  ros2 run gp7_ros2_pkg gripper_controller_node

Publish to move gripper:
  ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: -0.5}" --once  # close
  ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: 0.15}" --once  # open
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


# ── All gripper joints in the order declared in ros2_controllers.yaml ────────
GRIPPER_JOINTS = [
    'gripper_controller',
    'gripper_base_to_gripper_left2',
    'gripper_left3_to_gripper_left1',
    'gripper_base_to_gripper_right3',
    'gripper_base_to_gripper_right2',
    'gripper_right3_to_gripper_right1',
]

# Mimic multipliers from URDF:
#   gripper_controller        multiplier =  1.0
#   gripper_base_to_gripper_left2  multiplier =  1.0
#   gripper_left3_to_gripper_left1 multiplier = -1.0
#   gripper_base_to_gripper_right3 multiplier = -1.0
#   gripper_base_to_gripper_right2 multiplier = -1.0
#   gripper_right3_to_gripper_right1 multiplier =  1.0
MIMIC_MULTIPLIERS = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]


class GripperControllerNode(Node):
    """
    ROS 2 node that forwards a single gripper angle command to all
    gripper joints respecting their mimic multipliers.
    """

    def __init__(self):
        super().__init__('gp7_gripper_controller_node')

        # ── Publisher: /gripper_controller/commands (Float64MultiArray) ──────
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10,
        )

        # ── Subscription: simple single float command ────────────────────────
        self._cmd_sub = self.create_subscription(
            Float64,
            '/gp7/gripper_cmd',
            self._gripper_cmd_callback,
            10,
        )

        self.get_logger().info(
            'GP7 gripper controller node started.\n'
            'Publish to /gp7/gripper_cmd (std_msgs/Float64) to control gripper:\n'
            '  -0.5 → close fingers\n'
            '   0.0 → neutral\n'
            '   0.15 → open fingers (fully)\n'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _gripper_cmd_callback(self, msg: Float64):
        """Expand a single driver angle into all 6 gripper joint commands."""
        driver_pos = msg.data
        positions = [driver_pos * m for m in MIMIC_MULTIPLIERS]

        out = Float64MultiArray()
        out.data = positions
        self._cmd_pub.publish(out)

        self.get_logger().info(
            f'Gripper command sent: driver={driver_pos:.3f} rad  '
            f'→ {[f"{p:.3f}" for p in positions]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gripper controller node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
