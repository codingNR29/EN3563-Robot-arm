import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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

    robot_desc = robot_desc.replace('package://gp7_description/', pkg_gp7_description + '/')

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
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    # ── Clock bridge ──────────────────────────────────────────────────────────
    # Gazebo Sim publishes clock internally. This bridge forwards it to the
    # ROS /clock topic so controller_manager (and any use_sim_time node) can
    # receive it and eliminate the "No clock received" warning.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
    )

    # ── Spawn the robot model inside Gazebo ───────────────────────────────────
    # -allow_renaming false: fail instead of silently spawning a 2nd robot.
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_motoman_gp7',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'motoman_gp7',
            '-allow_renaming', 'false',
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
    #   • Start JSB spawner and wait for controller_manager readiness.
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

    # Start the spawner immediately; it waits for controller_manager readiness
    delayed_jsb = TimerAction(
        period=0.0,
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
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf'),
        set_env_gz_resource_path,
        gz_sim,
        clock_bridge,          # ← bridges Gazebo clock → ROS /clock
        robot_state_publisher,
        gz_spawn_entity,
        delayed_jsb,
        arm_and_gripper_after_jsb,
    ])
