import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_gp7_gazebo = get_package_share_directory('gp7_gazebo')
    pkg_gp7_description = get_package_share_directory('gp7_description')

    urdf_file = os.path.join(pkg_gp7_description, 'urdf', 'gp7.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    workspace_install_share = os.path.dirname(pkg_gp7_description)
    
    set_env_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=workspace_install_share
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'motoman_gp7',
                   '-allow_renaming', 'true'],
    )
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        set_env_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])
