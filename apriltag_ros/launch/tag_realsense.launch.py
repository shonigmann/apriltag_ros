import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

# detect all 16h5 tags
param = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.086,
    "max_hamming": 0,
    "threads": 4,
    "z_up": True
}
image_topic_ = "/image_raw"
camera_name = "/camera/color"
image_topic = camera_name + image_topic_
info_topic = camera_name + "/camera_info"

def generate_launch_description():

    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", image_topic ), ("/apriltag/camera_info",info_topic)],
        parameters=[param])

    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
