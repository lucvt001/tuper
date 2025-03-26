import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sam_btcpp',
            executable='main',
            name='leaders_bt',
            output='screen',
            parameters=[{
                'xml_directory': '/home/smarc2user/colcon_ws/src/tuper/sam_btcpp/behavior_trees/',
                'tree_name': 'LeadersStraightAndTurn',  # Active tree_name
                'loop_rate': 20
            }]
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'topic_tools', 'relay', 
                 '/leader1/core/thruster1_cmd', '/leader1/core/thruster2_cmd', '--wait-for-start'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'topic_tools', 'relay', 
                 '/leader2/core/thruster1_cmd', '/leader2/core/thruster2_cmd', '--wait-for-start'],
            output='screen'
        )
    ])
