#!/usr/bin/env python3

"""
RPi camera -> rectify -> apriltag detector -> pose

Camera modes (from imx219_mode_tbls.h, same as `v4l2-ctl --list-formats-ext -d0`)
	0 3280 x 2464 @ 21FPS
	1 3280 x 1848 @ 28FPS
	2 1920 x 1080 @ 30FPS
	3 1640 x 1232 @ 30FPS
	4 1280 x  720 @ 60FPS
"""

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import OpaqueFunction

def launch_setup(context):
    camera_info = PathJoinSubstitution([FindPackageShare('isaac_rpi_bringup'), 'config', 'rpi_1280x720.yaml'])

    camera_node = ComposableNode(
        name='isaac_camera',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'mode': 4,  # See list above
            'camera_info_url': f"file://{camera_info.perform(context)}",
            'optical_frame_name': 'rpi_camera',
        }],
        remappings=[
            ('left/image_raw', 'image_raw'),
            ('left/camera_info', 'camera_info'),
        ],
    )

    # RectifyNode is acting flaky, disable for now
    rectify_node = ComposableNode(
        name='isaac_rectify',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1280,
            'output_height': 720,
        }],
    )

    apriltag_node = ComposableNode(
        name='isaac_apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.1,
            'max_tags': 64,
            'tile_size': 4,
        }],
        remappings=[
            # ('image', 'image_rect'),
            # ('camera_info', 'camera_info_rect'),
            ('image', 'image_raw'),
        ],
    )

    nitros_container = ComposableNodeContainer(
        name='nitros_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_node, apriltag_node],
        namespace='',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'isaac_rectify:=debug']
    )

    return [nitros_container]

def generate_launch_description():
    launch_description = LaunchDescription()
    launch_description.add_action(OpaqueFunction(function=launch_setup))
    return launch_description
