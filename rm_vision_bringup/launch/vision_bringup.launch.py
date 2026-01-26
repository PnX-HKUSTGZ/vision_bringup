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
            parameters=[{
                'camera_info_url': 'package://rm_vision_bringup/config/camera_info.yaml',
                'exposure_time': 2500,
                'camera_frame_id': 'camera_link',
                'gain': 5.0,
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    def get_video_reader_node(package, plugin, name='video_reader_node', remappings=None, extra_params=None):
        # 合并参数：默认 node_params + 额外参数
        params = [node_params]
        if extra_params:
            params.append(extra_params)

        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=params,
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(container_name='camera_detector_container', namespace='', *nodes):
        node_list = list(nodes)
        workspace_root =os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.join(get_package_share_directory('rm_vision_bringup'))))))
        third_party_lib_path = os.path.join(workspace_root, 'third_party_install', 'lib')
        
        container = ComposableNodeContainer(
            name= container_name,
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=node_list,
            output='both',
            emulate_tty=True,
            additional_env={'LD_LIBRARY_PATH': third_party_lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
            ros_arguments=['--ros-args', ],
            #prefix=['xterm -e gdb --args'], # 用 gdb 调试
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
            'use_ai_detector': False
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    armor_detector_node_wide = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector_wide',
        parameters=[node_params, {
            'use_ai_detector': True
        }],
        remappings=[
                ('/image_raw', '/wide_cam/image_raw'),
                ('/image_raw/compressed', '/wide_cam/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/wide_cam/image_raw/compressedDepth'),
                ('/image_raw/theora', '/wide_cam/image_raw/theora'),
                ('/camera_info', '/wide_cam/camera_info'),
                ('/detector/binary_img', '/detector_wide/binary_img'),
                ('/detector/binary_img/compressed', '/detector_wide/binary_img/compressed'),
                ('/detector/binary_img/compressedDepth', '/detector_wide/binary_img/compressedDepth'),
                ('/detector/binary_img/theora', '/detector_wide/binary_img/theora'),
                ('/detector/number_img', '/detector_wide/number_img'),
                ('/detector/number_img/compressed', '/detector_wide/number_img/compressed'),
                ('/detector/number_img/compressedDepth', '/detector_wide/number_img/compressedDepth'),
                ('/detector/number_img/theora', '/detector_wide/number_img/theora'),
                ('/detector/result_img', '/detector_wide/result_img'),
                ('/detector/result_img/compressed', '/detector_wide/result_img/compressed'),
                ('/detector/result_img/compressedDepth', '/detector_wide/result_img/compressedDepth'),
                ('/detector/result_img/theora', '/detector_wide/result_img/theora'),
                ('/detector/armors', '/detector_wide/armors'),
                ('/detector/marker', '/detector_wide/marker')
            ],
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
            ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false', '-p', 'wide_cam:=true' if launch_params['wide_cam'] else 'wide_cam:=false'],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=['--ros-args', '-p', 'wide_cam:=true' if launch_params['wide_cam'] else 'wide_cam:=false'],
        )
    
    if launch_params['video_play']:
        # [修改] 主相机视频节点
        image_node = get_video_reader_node(
            'video_reader', 
            'video_reader::VideoReaderNode',
            name='video_reader_main'
        )
        
        # [修改] 广角相机视频节点
        # 注意：如果你想播放不同的视频，可以在这里传入 extra_params={'video_path': '/path/to/wide.mp4'}
        wide_camera_node = get_video_reader_node(
            'video_reader', 
            'video_reader::VideoReaderNode',
            name='video_reader_wide',
            remappings=[
                ('/image_raw', '/wide_cam/image_raw'),
                ('/image_raw/compressed', '/wide_cam/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/wide_cam/image_raw/compressedDepth'),
                ('/image_raw/theora', '/wide_cam/image_raw/theora'),
                ('/camera_info', '/wide_cam/camera_info')
            ]
        )
            
        
    else:
        image_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        wide_camera_node = ComposableNode(
            package='v4l2_camera',
            plugin='v4l2_camera::V4L2Camera',  # 切换到 v4l2_camera 的插件
            name='wide_camera_node',
            parameters=[node_params],
            remappings=[
                ('/image_raw', '/wide_cam/image_raw'),
                ('/camera_info', '/wide_cam/camera_info')
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
        
        
    if launch_params['rune']:
        cam_detector = get_camera_detector_container(
            'main_camera_container', 'main',
            image_node, 
            armor_detector_node_main,  
            rune_detector_node
        )
        if launch_params['wide_cam']:
            cam_detector_wide = get_camera_detector_container(
                'wide_camera_container', 'wide',
                wide_camera_node,
                armor_detector_node_wide,
                #rune_detector_node
            )
    else:
        cam_detector = get_camera_detector_container(
            'main_camera_container', 'main',
            image_node,
            armor_detector_node_main, 
        )
        if launch_params['wide_cam']:
            cam_detector_wide = get_camera_detector_container(
                'wide_camera_container', 'wide',
                wide_camera_node,
                armor_detector_node_wide,
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
        period=3.0,
        actions=[rune_solver_node],
    )
    if launch_params['wide_cam']:
        # 给 wide_camera_container 加一个 1秒 的延时，避开主相机的启动高峰
        delay_cam_detector_wide = TimerAction(
            period=1.0, 
            actions=[cam_detector_wide]
        )
    if launch_params['wide_cam']:
        launch_description_list = [
            robot_state_publisher,
            cam_detector,
            delay_cam_detector_wide,
            delay_serial_node,
            delay_tracker_node,
            delay_ballistic_node,
        ]
    else:
        launch_description_list = [
            robot_state_publisher,
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
