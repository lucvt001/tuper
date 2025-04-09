import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(get_package_share_directory('mqtt_msg_converter'), 'config', 'mqtt_msg_converter.yaml')

    gps_converter = Node(
        name='gps_converter',
        executable='string_to_navsatfix',
        package='mqtt_msg_converter',
        parameters=[config], output='screen'
    )

    heading_converter = Node(
        name='heading_converter',
        executable='int32_to_float32',
        package='mqtt_msg_converter',
        parameters=[config], output='screen'
    )

    return LaunchDescription([
        gps_converter,
        heading_converter,
    ])
