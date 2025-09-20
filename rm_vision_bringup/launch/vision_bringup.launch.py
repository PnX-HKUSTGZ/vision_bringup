import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node ,ballistic_node, rune_solver_node, recorder_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    def get_video_reader_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='video_reader_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(*nodes):
        node_list = list(nodes)
        workspace_root =os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.join(get_package_share_directory('rm_vision_bringup'))))))
        third_party_lib_path = os.path.join(workspace_root, 'third_party_install', 'lib')
        
        container = ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=node_list,
            output='both',
            emulate_tty=True,
            additional_env={'LD_LIBRARY_PATH': third_party_lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
            ros_arguments=['--ros-args', ],
        )
        return TimerAction(
            period=2.0,
            actions=[container],
        )
    
    rune_detector_node = ComposableNode(    
        package='rune_detector',
        plugin='rm_auto_aim::RuneDetectorNode',
        name='rune_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
        )
    # armor_detector_node = ComposableNode(
    #                 package='armor_detector',
    #                 plugin='rm_auto_aim::ArmorDetectorNode',
    #                 name='armor_detector',
    #                 parameters=[node_params, {'use_ai_detector': True}],
    #                 extra_arguments=[{'use_intra_process_comms': True}]
    #             )
    #改为双镜头
    armor_detector_node_main = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector_main',
        parameters=[node_params, {
            'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info',
            'result_topic': '/detector/armors',
            'use_ai_detector': True
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    armor_detector_node_wide = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector_wide',
        parameters=[node_params, {
            'image_topic': '/wide_cam/image_raw',
            'camera_info_topic': '/wide_cam/camera_info',
            'result_topic': '/wide_detector/armors',
            'use_ai_detector': True
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 串口
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false'],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=['--ros-args', ],
        )
    
    if launch_params['video_play']:
        image_node = get_video_reader_node('video_reader', 'video_reader::VideoReaderNode')
        try:
            wide_camera_node = Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='wide_camera_node',
                parameters=[node_params],  
                remappings=[
                    ('/image_raw', '/wide_cam/image_raw'),
                    ('/camera_info', '/wide_cam/camera_info')
                ],
                output='screen'
            )
        except Exception as e:
            print(f"Failed to create wide_camera_node: {e}")
            wide_camera_node = None
            
    else:
        image_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        # 广角相机节点（用 v4l2_camera 驱动，需单独 Node 启动）
        wide_camera_node = Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='wide_camera_node',
                parameters=[node_params],  
                remappings=[
                    ('/image_raw', '/wide_cam/image_raw'),
                    ('/camera_info', '/wide_cam/camera_info')
                ],
                output='screen'
                
            )
        
    if launch_params['rune']:
        cam_detector = get_camera_detector_container(
            image_node, 
            armor_detector_node_main, 
            armor_detector_node_wide, 
            rune_detector_node
        )
    else:
        cam_detector = get_camera_detector_container(
            image_node,
            armor_detector_node_main, 
            armor_detector_node_wide
        )


    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    delay_ballistic_node = TimerAction(
        period=2.5,
        actions=[ballistic_node],
    )

    delay_recorder_node = TimerAction(
        period=2.4,
        actions=[recorder_node],
    )

    delay_rune_solver_node = TimerAction(
        period=2.0,
        actions=[rune_solver_node],
    )

    launch_description_list = [
        robot_state_publisher,
        wide_camera_node,
        cam_detector,
        delay_serial_node,
        delay_tracker_node,
        delay_ballistic_node,
    ]
    if launch_params['rune']:
        launch_description_list.append(delay_rune_solver_node)
    if launch_params['enable_recorder']:
        launch_description_list.append(delay_recorder_node)

    return LaunchDescription(launch_description_list)
