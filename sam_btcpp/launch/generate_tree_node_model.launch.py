from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sam_btcpp',
            executable='generate_tree_node_model',
            name='generate_tree_node_model',
            output='screen',
            parameters=[{
                'tree_node_model_xmlfile_path': '/home/smarc2user/colcon_ws/src/tuper/sam_btcpp/behavior_trees/new_node.xml'
            }]
        )
    ])
