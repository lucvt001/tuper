import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'overall_params.yaml'
    )

    return LaunchDescription([
        Node(
            name='thrust_cmd_relay',
            executable='thrust_cmd_relay',
            package='formation_controller',
            output='screen',
            parameters=[config]
        ),
        Node(
            name='thrust_vector_relay',
            executable='thrust_vector_relay',
            package='formation_controller',
            output='screen',
            parameters=[config]
        ),
        Node(
            name='thrust_vector_horizontal_relay',
            executable='thrust_vector_horizontal_relay',
            package='formation_controller',
            output='screen',
            parameters=[config]
        ),
    ])