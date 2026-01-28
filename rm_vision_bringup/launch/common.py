import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

def create_robot_state_publisher(cam_id):
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params[f'odom2camera_{cam_id}']['xyz'], 
        ' rpy:=', launch_params[f'odom2camera_{cam_id}']['rpy'],
        ' camera_name:=', f'camera_{cam_id}'])
    
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{cam_id}',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}],
    )

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

tracker_node = ComposableNodeContainer(
    name='armor_tracker_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',  # 多线程容器
    output='both',
    emulate_tty=True,
    composable_node_descriptions=[
        ComposableNode(
            package='armor_tracker',
            plugin='rm_auto_aim::ArmorTrackerNode',
            name='armor_tracker',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    ],
)

ballistic_node = Node(
    package='ballistic_calculation',
    executable='ballistic_calculation_node',
    output='screen',
    emulate_tty=True,
    parameters=[node_params],
    
)

video_reader_node = Node(
    package='video_reader',
    executable='video_reader_node',
    output='screen',
    emulate_tty=True,
    parameters=[
        node_params
    ],
)

rune_solver_node = Node(
    package='rune_solver',
    executable='rune_solver_node',
    output='screen',
    emulate_tty=True,
    parameters=[
        node_params
    ],
)
recorder_node = Node(
    package='topic_recorder',
    executable='topic_recorder_node',
    name='topic_recorder_node',
    output='screen',
    emulate_tty=True,
    parameters=[{'config_path': os.path.join(
                                get_package_share_directory('rm_vision_bringup'), 'config', 'topic_record_params.yaml')}],
)