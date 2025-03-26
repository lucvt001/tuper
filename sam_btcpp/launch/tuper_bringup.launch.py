import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory('formationw_controller'),
    #     'config',
    #     'overall_params.yaml'
    # )

    return LaunchDescription([    

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('formation_controller') + '/launch/setup.launch.py'
            )
        ),  

        # TimerAction(
        #     period=0.5,  # Delay in seconds
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'bag', 'record', '-a'],  # Record all topics
        #             output='screen'
        #         )
        #     ]
        # ),

        TimerAction(
            period=1.0,  # Delay in seconds
            actions=[
                Node(
                    package='position_filter',
                    executable='position_filter',
                    name='filterpy',
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=8.0,  # Delay in seconds
            actions=[
                Node(
                    package='sam_btcpp',
                    executable='main',
                    name='follower_bt',
                    output='screen',
                    parameters=[{
                        'xml_directory': '/home/smarc2user/colcon_ws/src/tuper/sam_btcpp/behavior_trees/',
                        'tree_name': 'FollowerMainTree',
                        'do_connect_groot2': True,
                        # 'btlog_output_folder': '/home/smarc2user/colcon_ws/src/tuper/sam_btcpp/btlogs/',
                        'loop_rate': 20
                    }]
                ),
            ]
        ),

        TimerAction(
            period=7.0,  # Delay in seconds
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory('sam_btcpp') + '/launch/leaders_motion.launch.py'
                    )
                ),              
            ]
        ),

    ])
