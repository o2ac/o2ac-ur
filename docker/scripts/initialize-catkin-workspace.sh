#!/bin/bash

################################################################################

# Make additional preparations when inside the GitLab CI environement.
if [ "${CI}" = "true" ]; then
  # Terminate the script after the first failure and return non-zero exit code.
  set -e

  # Download and initialize all the Git submodules recursively.
  # https://git-scm.com/book/en/v2/Git-Tools-Submodules
  cd /root/ && git submodule update --init --recursive
fi

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Source the updated ROS environment.
source /opt/ros/melodic/setup.bash

################################################################################

# Initialize the underlay workspace with wstool and install dependencies
cd /root/underlay_ws/      && \
    wstool update -t src   && \
    rosdep install --from-paths src --ignore-src -r -y

# Initialize, build and source the underlay workspace.
cd /root/underlay_ws/ && catkin config -init && catkin build
source /root/underlay_ws/devel/setup.bash


# Install the dependencies of the main workspace
cd /root/catkin_ws/ && rosdep install --from-paths src --ignore-src -r -y

# Source the main Catkin workspace.
## The underlay workspace *has to be already sourced* for this second workspace
## to be an extension of underlay_ws. catkin config should display this line:
## "Extending: [env] /root/underlay_ws/devel:/opt/ros/melodic"
cd /root/catkin_ws/ && catkin config -init && catkin build
source /root/catkin_ws/devel/setup.bash

################################################################################

# Generate the URDF files in o2ac_parts_description

rosrun o2ac_parts_description generate_urdf_from_meshes.py
