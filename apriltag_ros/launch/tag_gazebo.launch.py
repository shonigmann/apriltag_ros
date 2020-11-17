import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 16h5 tags
cfg_16h5 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.08,
    "max_hamming": 0,
 
}

image_topic_ = "/image_raw"
camera_name = "/rgb_camera"
image_topic = camera_name + image_topic_
info_topic = camera_name + "/camera_info"

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/apriltag/image", image_topic), ("/apriltag/camera_info", info_topic)],
        parameters=[cfg_16h5])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
