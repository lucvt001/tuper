import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('sam_thruster_relay'), 'config', 'relay_params.yaml')

    thrust_cmd_relay = Node(
        name='thrust_cmd_relay',
        executable='thrust_cmd_relay',
        package='sam_thruster_relay',
        parameters=[config]
    )

    thrust_vector_relay = Node(
        name='thrust_vector_relay',
        executable='thrust_vector_relay',
        package='sam_thruster_relay',
        parameters=[config]
    )

    return LaunchDescription([
        thrust_cmd_relay,
        thrust_vector_relay,
    ])