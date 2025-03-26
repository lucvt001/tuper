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
            executable='visualize_paths',
            name='visualize_paths',
            parameters=[{
                "csv_files": [
                    leader1_csv,
                    leader2_csv,
                    follower_csv
                ],
            }]
        )
    ])
