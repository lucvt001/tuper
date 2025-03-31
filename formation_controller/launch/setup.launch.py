import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'overall_params.yaml'
    )

    return LaunchDescription([
        Node(
            name='ground_truth_tf2',
            executable='ground_truth_tf2',
            package='formation_controller',
            parameters=[config]
        ),
        
        Node(
            name='ping_synchronizer',
            executable='ping_synchronizer',
            package='formation_controller',
            parameters=[config]
        ),
        
        Node(
            name='string_stamped_processing',
            executable='string_stamped_processing',
            package='formation_controller',
            parameters=[config]
        ),
        
        Node(
            name='fuse_distance_triangulation',
            executable='fuse_distance_triangulation',
            package='formation_controller',
            parameters=[config]
        ),
        
        Node(
            package='formation_controller',
            executable='relative_target_position_publisher',
            name='relative_target_position_publisher',
            output='screen',
            parameters=[config]
        ),

        Node(
            name='get_differential_value',
            executable='get_differential_value',
            package='formation_controller',
            parameters=[config]
        ),

        Node(
            name='formation_shape_broadcaster',
            executable='formation_shape_broadcaster',
            package='formation_controller',
            parameters=[config]
        ),
               
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('formation_controller') + '/launch/pid_servers.launch.py'
            )
        ),  

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('formation_controller') + '/launch/relay_nodes.launch.py'
            )
        ),  

    ])
