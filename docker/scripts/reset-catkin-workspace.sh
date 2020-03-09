#!/bin/bash

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# TODO: Does this have any effect on the current shell?
# Source the ROS environment.
source /opt/ros/melodic/setup.bash

################################################################################

# Remove the Catkin workspace:
# Delete expected 'catkin_make' artefacts.
cd /root/catkin_ws/ && rm -r .catkin_workspace build/ devel/ install/
cd /root/catkin_ws/src/ && rm CMakeLists.txt
# Delete unexpected 'catkin build' artefacts.
cd /root/catkin_ws/ && catkin clean -y
cd /root/catkin_ws/ && rm -r CMakeLists.txt .catkin_tools/

################################################################################

# Remove the underlay workspace:
# Delete expected 'catkin_make' artefacts.
cd /root/underlay_ws/ && rm -r .catkin_workspace build/ devel/ install/
cd /root/underlay_ws/src/ && rm CMakeLists.txt
# Delete unexpected 'catkin build' artefacts.
cd /root/underlay_ws/ && catkin clean -y
cd /root/underlay_ws/ && rm -r CMakeLists.txt .catkin_tools/

################################################################################

bash /root/scripts/initialize-catkin-workspace.sh
