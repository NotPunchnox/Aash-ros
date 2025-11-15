import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_file = 'robot.urdf'
    package_description = "quadruped_description"

    print("Fetching URDF ==>")
    robot_desc_path = PathJoinSubstitution([
        get_package_share_directory(package_description), "quadruped", urdf_file
    ])

    robot_desc_content = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc_content}],
        output='screen'
    )

    return LaunchDescription(
        [
            robot_state_publisher_node
        ]
    )