# apriltag_ros ROS2 Node

Inspired from https://github.com/christianrauch/apriltag_ros.git and fuse features from ROS1 version.

Apriltag_ros is ROS2 wrapper for apriltag detection. It take sensor_msgs/Image as input, and return pose and position in tf2 format.

You can specify number in tag family to filter detection of output, and set frame name for each of them. Please set the size of each tag correctly to make sure publish tf is accurate.

## Installation

```
mkdir -p ~/apriltag_ros2_ws/src                # Make a new workspace 
cd ~/apriltag_ros2_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git                  # Clone Apriltag library
git clone https://github.com/H-HChen/apriltag_ros.git -b foxy-devel   # Clone Apriltag ROS wrapper
cd ..               
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
colcon build --symlink-install                      # Build all packages in the workspace
```

## Usage

Launch tag detection node, and specify the camera namespace and topic name. Image topic will remap to "camera_name/image_topic".

```
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/color image_topic:=image_raw
```

### Custom parameter

Modify configuration file in /apriltag_ros/apriltag_ros/cfg/

Default loaded configuration file is for detecting all apriltags in 36h11 family.

You can also change config file to load in launch file.

This is an example of configuration for detecting specified tag ids.

```
image_transport: 'raw'    # image format
family: '36h11'           # tag family name
size: 0.08                # default tag size
threads: 4
max_hamming: 0          # maximum allowed hamming distance (corrected bits)
z_up: true              # rotate about x-axis to have Z pointing upwards

# see "apriltag.h" for more documentation on these optional parameters
decimate: 0.0           # decimate resolution for quad detection
blur: 1.0               # sigma of Gaussian blur for quad detection
refine-edges: 1         # snap to strong gradients
debug: 0                # write additional debugging images to current working directory
tag_ids: [0]            # tag ID
tag_frames: [tag_frame]  # optional frame name
tag_sizes: [0.08]   # optional tag-specific edge size
```
