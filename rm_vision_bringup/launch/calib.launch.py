#!/usr/bin/env python3
"""
merge_launch.py - 合并相机标定启动文件
支持V4L2摄像头和海康摄像头,通过camera_type参数选择
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    camera_type = 'hik' # v4l2或hik
    
    # 1. V4L2摄像头节点(当camera_type为v4l2时启动)
    if camera_type == 'v4l2':
        camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='wide_camera_node',
            namespace='',
            output='screen',
            parameters=[
                # 基础配置
                {'video_device': '/dev/video0'},
                {'output_encoding': 'rgb8'},
                {'image_size': [1280, 720]},
                {'framerate': 30.0},
                {'camera_name': 'wide_camera'},
                
                # 图像质量参数（确保在合理范围内）
                {'brightness': 50},      # 0-100范围
                {'contrast': 50},        # 0-100范围
                {'saturation': 60},      # 0-100范围
                {'sharpness': 50},       # 0-100范围
                
                # 自动控制
                {'exposure_auto': 1},    # 自动曝光
                {'focus_auto': 1},       # 自动对焦
            ]
        )
    else : 
    # 2. 海康摄像头节点（当camera_type为hik时启动）
        camera_node = Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=[
                {'width': 1280},
                {'height': 720},
                {'fps': 30.0},
                {'exposure_time': 5000},
                {'gain': 16.0},
                {'pixel_format': 'rgb8'},
            ]
        )
    
    # 3. 相机标定节点 - 动态选择话题
    # 使用条件表达式的正确方法
    calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='camera_calibrator',
        output='screen',
        arguments=[
            '--size', '7x7',
            '--square', '0.03',
            '--pattern', 'circles'
        ],
        remappings=[
            ('image', '/image_raw'),
        ]
    )
    
    return LaunchDescription([
        # 根据条件启动对应的相机节点
        camera_node,
        # 启动标定节点
        calibration_node,
    ])