from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path_prefix = "/home/smarc2user/colcon_ws/src/tuper/path_planner/config/"
    leader_csv = path_prefix + "center_path.csv"

    return LaunchDescription([
        Node(
            package="path_planner",
            executable='generate_offset_path',
            name='generate_leader1_path',
            parameters=[{
                "leader_csv": leader_csv,
                "offset_yaml": path_prefix + "leader1_offset.yaml",
                "output_csv": path_prefix + "leader1_path.csv",
                "do_plot": False,
            }]
        ),

        Node(
            package="path_planner",
            executable='generate_offset_path',
            name='generate_leader2_path',
            parameters=[{
                "leader_csv": leader_csv,
                "offset_yaml": path_prefix + "leader2_offset.yaml",
                "output_csv": path_prefix + "leader2_path.csv",
                "do_plot": False,
            }]
        ),

        Node(
            package="path_planner",
            executable='generate_offset_path',
            name='generate_follower_path',
            parameters=[{
                "leader_csv": leader_csv,
                "offset_yaml": path_prefix + "follower_offset.yaml",
                "output_csv": path_prefix + "follower_path.csv",
                "do_plot": False,
            }]
        ),
    ])
