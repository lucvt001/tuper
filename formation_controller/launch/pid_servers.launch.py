import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pid_config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'pid_params.yaml'
    )

    return LaunchDescription([
        Node(
            name='x_pid',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_pid',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_delta_pid',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_pid_turn',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
        Node(
            name='y_delta_pid_turn',
            executable='pid_server',
            package='formation_controller',
            output='screen',
            parameters=[pid_config]
        ),
    ])