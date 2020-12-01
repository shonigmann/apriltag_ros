import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

# detect all 16h5 tags
param = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.08,
    "max_hamming": 0,
    "threads": 4,
    "z_up": True
}

image_topic_ = "/image_raw"
camera_name = "/camera_color_frame"
image_topic = camera_name + image_topic_
info_topic = camera_name + "/camera_info"
config = os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11_all.yaml') 

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", image_topic), ("/apriltag/camera_info", info_topic)],
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
