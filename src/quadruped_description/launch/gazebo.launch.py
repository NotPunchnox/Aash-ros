import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
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
    config_file = os.path.join(pkg_share, "config", "quadruped_control.yaml")
    rviz_config_file = os.path.join(pkg_share, "rviz", "display.rviz")
    
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
    
    # Robot description avec passage du fichier de config
    robot_desc_content = ParameterValue(
        Command(['xacro ', urdf_file, ' controller_config_file:=', config_file]),
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
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'quadruped',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Charger joint_state_broadcaster après spawn
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ]
        )
    )
    
    # Charger quadruped_controller avec un délai
    load_quadruped_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['quadruped_controller'],
                output='screen'
            )
        ]
    )
    
    # RViz (commenté pour le moment à cause des problèmes d'affichage)
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )
    
    return LaunchDescription([
        gui_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_quadruped_controller,
        # rviz,
    ])
