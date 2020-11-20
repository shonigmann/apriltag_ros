# apriltag_ros ROS2 Node

inspired from https://github.com/christianrauch/apriltag_ros.git and fuse a little bit from ros1 version.

More stable and ROS2 support!!!

Plz enjoy it

## Quickstart

Starting with a working ROS installation (Kinetic and Melodic are supported):
```
mkdir -p ~/apriltag_ros2_ws/src                # Make a new workspace 
cd ~/apriltag_ros2_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/H-HChen/apriltag_ros.git -b foxy-devel   # Clone Apriltag ROS wrapper
cd ..               
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
colcon build --symlink-install    # Build all packages in the workspace \
```
See the [ROS wiki](http://wiki.ros.org/apriltag_ros) for details and tutorials.

trouble shooting:Could not load library LoadLibrary error: libopencv_imgcodecs.so.4.5

https://github.com/cggos/dip_cvqt/issues/1
