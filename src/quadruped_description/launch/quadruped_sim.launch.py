import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_description = "quadruped_description"
    urdf_file = 'robot.urdf'

    # URDF
    robot_desc_path = PathJoinSubstitution([
        get_package_share_directory(package_description), "quadruped", urdf_file
    ])
    robot_desc_content = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn robot in Gazebo
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

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc_content},
                    PathJoinSubstitution([get_package_share_directory(package_description), 'config', 'quadruped_control.yaml'])],
        output='screen'
    )

    # Spawners avec dépendances (attends que controller_manager démarre)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', '--controller-manager', '/controller_manager', 'joint_state_broadcaster'],
        output='screen'
    )

    load_quadruped_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', '--controller-manager', '/controller_manager', 'quadruped_controller'],
        output='screen'
    )

    # Event handlers pour séquencer
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_joint_state_broadcaster]
        )
    )

    delayed_quadruped_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_quadruped_controller]
        )
    )

    # RViz
    rviz_config_path = os.path.join(get_package_share_directory(package_description), "rviz", 'quadruped.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager,
        delayed_joint_state_broadcaster,
        delayed_quadruped_controller,
        rviz_node
    ])