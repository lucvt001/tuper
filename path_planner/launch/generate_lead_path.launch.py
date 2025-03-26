from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path_prefix = "/home/smarc2user/colcon_ws/src/tuper/path_planner/config/"
    yaml_path = path_prefix + "path.yaml"
    output_csv = path_prefix + "center_path.csv"

    return LaunchDescription([
        Node(
            package="path_planner",
            executable='generate_lead_path',
            name='generate_lead_path',
            output='screen',
            parameters=[{
                "input_yaml": yaml_path,
                "output_csv": output_csv
            }]
        )
    ])
