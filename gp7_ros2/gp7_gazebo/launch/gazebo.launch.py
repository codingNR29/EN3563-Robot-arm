import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim      = get_package_share_directory('ros_gz_sim')
    pkg_gp7_gazebo      = get_package_share_directory('gp7_gazebo')
    pkg_gp7_description = get_package_share_directory('gp7_description')

    # ── Resolve the controller YAML absolute path ─────────────────────────────
    controllers_yaml = os.path.join(
        pkg_gp7_gazebo, 'config', 'ros2_controllers.yaml'
    )

    # ── Read URDF and patch the $(find ...) ROS-1 placeholder ────────────────
    urdf_file = os.path.join(pkg_gp7_description, 'urdf', 'gp7.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_desc = robot_desc.replace(
        '$(find gp7_gazebo)/config/ros2_controllers.yaml',
        controllers_yaml,
    )

    # ── GZ_SIM_RESOURCE_PATH so Gazebo can locate mesh files ─────────────────
    workspace_install_share = os.path.dirname(pkg_gp7_description)
    set_env_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_install_share,
    )

    # ── Gazebo Sim ─────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}],
    )

    # ── Spawn the robot model inside Gazebo ───────────────────────────────────
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_motoman_gp7',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'motoman_gp7',
            '-allow_renaming', 'true',
        ],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    #
    # CRITICAL ORDER:
    #   1. joint_state_broadcaster MUST be active before arm/gripper controllers
    #      are activated — otherwise their on_configure() fails because there
    #      are no joint states to read.
    #
    # STRATEGY:
    #   • Fire JSB spawner 20 s after launch (gives Gazebo plenty of boot time).
    #   • Use OnProcessExit so arm + gripper spawners start ONLY after JSB
    #     spawner process exits (= JSB is active and healthy).
    #   • --controller-manager-timeout 60 makes each spawner keep retrying for
    #     up to 60 s instead of giving up immediately.

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_arm_controller',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_gripper_controller',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    # Delay JSB spawner 20 s so Gazebo + gz_ros2_control finish initialising
    delayed_jsb = TimerAction(
        period=20.0,
        actions=[load_joint_state_broadcaster],
    )

    # Arm + gripper spawners fire ONLY after JSB spawner process exits cleanly
    arm_and_gripper_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                load_arm_controller,
                load_gripper_controller,
            ],
        )
    )

    return LaunchDescription([
        set_env_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        delayed_jsb,
        arm_and_gripper_after_jsb,
    ])
