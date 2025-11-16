import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_description = "quadruped_description"
    urdf_file = 'robot.urdf'

    # URDF content
    robot_desc_path = PathJoinSubstitution([
        get_package_share_directory(package_description), "quadruped", urdf_file
    ])
    robot_desc_content = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )

    # Launch Gazebo with additional args for running and verbose
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gz_args': '-r -v 4',
            'paused': 'false'
        }.items()
    )

    # Spawn the robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'quadruped'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': True}],
        output='screen'
    )

    # Spawners for the controllers with controller-manager-timeout
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '-1'],
        output='screen'
    )

    load_quadruped_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'quadruped_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '-1'],
        output='screen'
    )

    # Event handlers to wait for spawn_entity (plugin loads after spawn)
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[load_joint_state_broadcaster]
        )
    )

    delayed_quadruped_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[load_quadruped_controller]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_quadruped_controller
    ])