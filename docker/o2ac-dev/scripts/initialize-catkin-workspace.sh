#!/bin/bash

################################################################################

# Display information in CI for version traceability.
if [[ "${CI}" == "true" ]]; then
  echo -e "\n"
  echo -e "######################## DEBUG DATA ########################"
  echo -e "Current commit: $(cd /root/HSR/ && git rev-parse --short HEAD)"
  echo -e "DOCKER_RUNTIME: $DOCKER_RUNTIME"
  echo -e "DOCKER_IMAGE_VERSION: $DOCKER_IMAGE_VERSION"
  echo -e "NVIDIA_CUDAGL_VERSION: $NVIDIA_CUDAGL_VERSION"
  echo -e "NVIDIA_CUDNN_VERSION: $NVIDIA_CUDNN_VERSION"
  echo -e "ROS_DESKTOP_VERSION: $ROS_DESKTOP_VERSION"
  echo -e "ROS_TMC_VERSION: $ROS_TMC_VERSION"
  echo -e "ROS_GAZEBO_VERSION: $ROS_GAZEBO_VERSION"
  echo -e "PYTHON_PIP_VERSION: $PYTHON_PIP_VERSION"
  echo -e "############################################################"
  echo -e "\n"
fi

################################################################################

# Make additional preparations when inside the GitLab CI environement.
if [ "${CI}" = "true" ]; then
  # Terminate the script after the first failure and return non-zero exit code.
  set -e

  # Download and initialize all the Git submodules recursively.
  # https://git-scm.com/book/en/v2/Git-Tools-Submodules
  cd /root/o2ac-ur/ && git submodule update --init --recursive
fi

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Source the updated ROS environment.
source /opt/ros/melodic/setup.bash

# FIXME: Setting PhoLocalization env variable manually seems to be required for ci.
#        Apparently it's not loaded correctly although the installation seems be complete.
export PHO_LOCALIZATION_PATH=/opt/PhotoneoSDK/Localization

################################################################################

# Initialize the underlay workspace with wstool and install dependencies
cd /root/o2ac-ur/underlay_ws/      && \
    wstool update -t src --delete-changed-uris

rosdep install --from-paths src --ignore-src -r -y > /dev/null 2>&1 || FAILED=true

if [ $FAILED ]; then
    echo "rosdep install failed, but we continue anyway (it might be due to python-pymodbus missing on Ubuntu 20.04)"
fi

# Initialize, build and source the underlay workspace.
# Blacklist packages that we do not use but that are part of metapackages we need
cd /root/o2ac-ur/underlay_ws/ && catkin config -init --blacklist robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_rviz \
                          robotiq_3f_gripper_control robotiq_3f_gripper_rviz robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization \
                          ur3_e_moveit_config ur10_e_moveit_config
catkin build
source /root/o2ac-ur/underlay_ws/devel/setup.bash


# Install the dependencies of the main workspace
cd /root/o2ac-ur/catkin_ws/ && rosdep install --from-paths src --ignore-src -r -y

# Source the main Catkin workspace.
## The underlay workspace *has to be already sourced* for this second workspace
## to be an extension of underlay_ws. catkin config should display this line:
## "Extending: [env] /root/o2ac-ur/underlay_ws/devel:/opt/ros/melodic"
cd /root/o2ac-ur/catkin_ws/ && catkin config -init && catkin build
source /root/o2ac-ur/catkin_ws/devel/setup.bash

################################################################################

# Generate the URDF and PLY files in o2ac_parts_description

rosrun o2ac_parts_description generate_urdf_from_meshes.py
rosrun o2ac_parts_description generate_assembled_piece.py
rosrun o2ac_parts_description convert_stl_to_ply.py
