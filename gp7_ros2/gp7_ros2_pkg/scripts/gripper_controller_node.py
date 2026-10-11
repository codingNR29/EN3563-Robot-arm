#!/usr/bin/env python3
"""
GP7 Gripper Controller Node
=============================
Sends position commands to the gripper_controller (JointGroupPositionController)
running inside Gazebo Sim via the /gripper_controller/commands topic.

The gripper has one primary "driver" joint (gripper_driver joint) that
mimic-links drive all other finger segments.

Usage
-----
After launching Gazebo:
  ros2 run gp7_ros2_pkg gripper_controller_node

Publish to move gripper:
  ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: -0.5}" --once  # close
  ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: 0.15}" --once  # open
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class GripperControllerNode(Node):
    """
    ROS 2 node that commands the single actuated gripper joint.
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
        """Command the driver; Gazebo applies the URDF mimic relationships."""
        driver_pos = msg.data
        if not math.isfinite(driver_pos) or not -0.5 <= driver_pos <= 0.15:
            self.get_logger().error('Gripper command must be between -0.5 and 0.15 rad.')
            return
        positions = [driver_pos]

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
