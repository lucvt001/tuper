from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    path_prefix = "/home/smarc2user/colcon_ws/src/tuper/path_planner/config/"
    leader1_csv = path_prefix + "leader1_path.csv"
    leader2_csv = path_prefix + "leader2_path.csv"
    follower_csv = path_prefix + "follower_path.csv"

    return LaunchDescription([

        Node(
            package='path_planner',
            executable='csv_tf_broadcaster',
            name='csv_tf_broadcaster',
            parameters=[{
                'csv_file': leader1_csv,
                'child_frame': 'leader1/target_pose',
                'parent_frame': 'map_gt',
                'path_topic': 'leader1/target_path',
            }]
        ),

        Node(
            package='path_planner',
            executable='csv_tf_broadcaster',
            name='csv_tf_broadcaster',
            parameters=[{
                'csv_file': leader2_csv,
                'child_frame': 'leader2/target_pose',
                'parent_frame': 'map_gt',
                'path_topic': 'leader2/target_path',
            }]
        ),

        Node(
            package='path_planner',
            executable='csv_tf_broadcaster',
            name='csv_tf_broadcaster',
            parameters=[{
                'csv_file': follower_csv,
                'child_frame': 'follower/target_pose',
                'parent_frame': 'map_gt',
                'path_topic': 'follower/target_path',
            }]
        ),

    ])
