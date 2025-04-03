import os
from ament_index_python.packages import get_package_share_directory
from launch import Action, LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_context import LaunchContext

launch_args = [
    DeclareLaunchArgument('use_gps', default_value='True', description='gps or filter. True for gps (leaders), False for followers (filter).'),
    DeclareLaunchArgument('is_unity_sam', default_value='True', description='True if running on Unity with SAM, False otherwise.'),
    DeclareLaunchArgument('ns', default_value='agent0', description='Namespace for the agent. Should be agent0, agent1, agent2, etc.'),
]

def launch_setup(context: LaunchContext) -> list[Action]:

    # Overall configuration file for most nodes
    config = os.path.join(
        get_package_share_directory('formation_controller'),
        'config',
        'overall_params.yaml'
    )

    use_gps = LaunchConfiguration('use_gps')
    is_unity_sam = LaunchConfiguration('is_unity_sam')
    ns = LaunchConfiguration('ns')

    # Subscribe to gps and heading topic of supreme leader. Broadcast world (utm) -> map, no translation, only rotation
    # Publish origin gps to /NS/origin_gps so that each agent can calculate its current local position, FLU frame
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='formation_controller', parameters=[config]
    )

    # Listen to target -> filtered_position transform and publish it to three topics: x, y, z
    # Used for PID control of each axis
    tf_repub = Node(
        name='tf_repub',
        executable='tf_repub',
        package='formation_controller', parameters=[config]
    )

    # Read the yaml file and broadcast all the static transforms: leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package='formation_controller',
        parameters=[{
            'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
            'leader_frame': 'agent0/base_link',
        }]
    )

    # Listen to gps and heading of this agent. Calculate the local position relative to origin_gps
    # And then broadcast the transform world (utm) -> agent
    # Only used for agents on the surface with access to gps (aka leaders)
    gps_heading_to_tf = Node(
        name='gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package='formation_controller', parameters=[config],
        condition=IfCondition(PythonExpression([use_gps]))
    )

    # Fuse ping_distance1 and ping_distance2 to track x, y, vx, vy of the agent relative to supreme_leader
    # And then broadcast supreme_leader -> agentX transform. Orientation is ignored.
    # Only used for agents relying on ping distance (aka followers)
    ukf_filter = Node(
        name='ukf_filter',
        executable='position_filter',
        package='position_filter',
        condition=UnlessCondition(PythonExpression([use_gps]))
    )
    ukf_filter = TimerAction(period=1.0, actions=[ukf_filter])

    # PID servers for followers control scheme
    pid_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/pid_servers.launch.py'
        )
    )

    # Required for the controller to work
    differential_value_node = Node(
        name='get_differential_value',
        executable='get_differential_value',
        package='formation_controller', parameters=[config]
    )

    # Relay nodes for Unity with SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('formation_controller') + '/launch/relay_nodes.launch.py'
        ), condition=IfCondition(PythonExpression([is_unity_sam]))
    )

    return [
        PushRosNamespace(ns.perform(context)),
        origin_pub,
        tf_repub,
        formation_shape_broadcaster,
        gps_heading_to_tf,
        ukf_filter,
        pid_servers,
        differential_value_node,
        relay_nodes,
    ]

def generate_launch_description():
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup)
    ])