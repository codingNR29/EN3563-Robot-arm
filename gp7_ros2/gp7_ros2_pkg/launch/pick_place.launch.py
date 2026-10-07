"""Launch the assignment scene, adaptive gripper and verified motion sequence."""
import os
import random
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gp7_motion.model import task_description


def setup(context):
    share = get_package_share_directory('gp7_description')
    gazebo = get_package_share_directory('gp7_gazebo')
    urdf = task_description(Path(share, 'urdf/gp7.urdf').read_text(), share,
                            str(Path(gazebo, 'config/ros2_controllers.yaml')))
    # A seeded random point in a tested reachable patch, reproducible for recordings.
    rng = random.Random(int(LaunchConfiguration('seed').perform(context)))
    pickup = (rng.uniform(.50, .60), rng.uniform(-.23, -.15))
    world = ET.parse(Path(gazebo, 'worlds/pick_place.sdf'))
    world.find(".//model[@name='box_b']/pose").text = f'{pickup[0]} {pickup[1]} 0.025 0 0 0'
    fd, world_path = tempfile.mkstemp(prefix='gp7_pick_place_', suffix='.sdf')
    os.close(fd)
    world.write(world_path)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
        get_package_share_directory('ros_gz_sim'), 'launch/gz_sim.launch.py'))),
        launch_arguments={'gz_args': f'-r {"-s " if headless else ""}{world_path}'}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'gp7_pick_place', '-name', 'motoman_gp7',
                            '-string', urdf, '-allow_renaming', 'false'], output='screen')
    controllers = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                                  '--controller-manager-timeout', '90'], output='screen')
    motion = Node(package='gp7_ros2_pkg', executable='pick_place_node', output='screen',
                  parameters=[{'use_sim_time': True, 'pickup_x': pickup[0], 'pickup_y': pickup[1],
                               'place_x': .55, 'place_y': .35}],
                  condition=IfCondition(LaunchConfiguration('run_task')))
    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller startup failed'))]
        return [motion]
    return [sim, rsp, bridge, spawn, controllers,
            RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('run_task', default_value='true'),
        DeclareLaunchArgument('seed', default_value='7'),
        OpaqueFunction(function=setup),
    ])
