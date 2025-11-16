import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_description = "quadruped_description"
    
    # Arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Démarrer Gazebo avec GUI (false pour mode headless)'
    )
    
    # Chemins
    pkg_share = get_package_share_directory(package_description)
    urdf_file = os.path.join(pkg_share, "quadruped", "robot.urdf")
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'verbose': 'false',
            'gui': LaunchConfiguration('gui')
        }.items()
    )
    
    # Robot description
    robot_desc_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc_content
        }],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'quadruped_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )
    
    # Charger les contrôleurs
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    load_quadruped_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['quadruped_controller'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gui_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_quadruped_controller,
    ])
