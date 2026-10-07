"""
gp7_sim_control.launch.py
==========================
One-shot launch file that starts:
  1. Gazebo Sim (via gp7_gazebo/launch/gazebo.launch.py)
  2. arm_controller_node      — listens on /gp7/goal_index
  3. gripper_controller_node  — listens on /gp7/gripper_cmd

Usage
-----
  ros2 launch gp7_ros2_pkg gp7_sim_control.launch.py

Then, in a separate terminal, send arm goals:
  ros2 topic pub /gp7/goal_index std_msgs/msg/Int32 "{data: 1}" --once

Or control the gripper:
  ros2 topic pub /gp7/gripper_cmd std_msgs/msg/Float64 "{data: -0.5}" --once
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ── Include the existing Gazebo launch ─────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gp7_gazebo'),
                'launch',
                'gazebo.launch.py',
            )
        )
    )

    # ── Arm controller node (delayed so controllers are ready first) ────────
    arm_node = Node(
        package='gp7_ros2_pkg',
        executable='arm_controller_node',
        name='gp7_arm_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Gripper controller node ─────────────────────────────────────────────
    gripper_node = Node(
        package='gp7_ros2_pkg',
        executable='gripper_controller_node',
        name='gp7_gripper_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Delay control nodes by 8 s to let Gazebo + controller_manager boot up
    delayed_arm     = TimerAction(period=8.0, actions=[arm_node])
    delayed_gripper = TimerAction(period=8.0, actions=[gripper_node])

    return LaunchDescription([
        gazebo_launch,
        delayed_arm,
        delayed_gripper,
    ])
