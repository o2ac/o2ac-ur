#!/bin/bash

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Run rosdep install to avoid missing binary packages (the binaries are not synced between stages)
echo "==== Running rosdep install for underlay_ws"
rosdep install --from-paths /root/o2ac-ur/catkin_ws/src --ignore-src -r -y 2>&1 || FAILED=true
echo "==== Running rosdep install for catkin_ws"
rosdep install --from-paths /root/o2ac-ur/underlay_ws/src --ignore-src -r -y 2>&1 || FAILED=true
if [ $FAILED ]; then
    echo "rosdep install failed, but we continue anyway (it might be due to python-pymodbus missing on Ubuntu 20.04)"
fi

# Source the updated ROS environment.
source /opt/ros/noetic/setup.bash

# Source the Catkin workspaces.
source /root/o2ac-ur/underlay_ws/devel/setup.bash
source /root/o2ac-ur/catkin_ws/devel/setup.bash

################################################################################

# Run tests
rostest o2ac_routines test_robot_motions.test

# Get 'roslaunch' process ID and kill it.
# kill $!
