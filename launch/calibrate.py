#!/usr/bin/env python3

"""
RPi camera -> calibration script

Camera modes (from imx219_mode_tbls.h, same as `v4l2-ctl --list-formats-ext -d0`)
	0 3280 x 2464 @ 21FPS
	1 3280 x 1848 @ 28FPS
	2 1920 x 1080 @ 30FPS
	3 1640 x 1232 @ 30FPS
	4 1280 x  720 @ 60FPS
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction

def launch_setup(context):
    camera_node = ComposableNode(
        name='isaac_camera',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'mode': 4,  # See list above
            'optical_frame_name': 'rpi_camera',
        }],
        remappings=[
        # Outputs:
            ('left/image_raw', 'image_raw'),
            ('left/camera_info', 'camera_info'),
        ],
    )

    nitros_container = ComposableNodeContainer(
        name='nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_node],
        namespace='',
        output='screen',
    )

    calibration_node = Node(
        name='calibrator',
        package='camera_calibration',
        executable='cameracalibrator',
        namespace='',
        output='screen',
        arguments=[
            '--size', '7x10',
            '--square', '0.15',
        ],
        remappings=[
        # Inputs:
            ('image', 'image_raw'),
        ],
    )

    return [nitros_container, calibration_node]

def generate_launch_description():
    launch_description = LaunchDescription()
    launch_description.add_action(OpaqueFunction(function=launch_setup))
    return launch_description
