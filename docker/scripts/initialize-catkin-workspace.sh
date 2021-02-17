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
# Blacklist packages that we do not use but that are part of metapackages we need
cd /root/underlay_ws/ && catkin config -init
catkin config --blacklist robotiq_3f_gripper_articulated_gazebo robotiq_3f_gripper_articulated_gazebo_plugins robotiq_3f_rviz \ 
                          robotiq_3f_gripper_control robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_visualization \
                          chomp_motion_planner moveit_planners_chomp moveit_chomp_optimizer_adapter \
                          moveit_ros_warehouse moveit_ros_benchmarks moveit_servo moveit_resources_prbt_ikfast_manipulator_plugin \
                          pilz_industrial_motion_planner pilz_industrial_motion_planner_testutils \
                          ur3_e_moveit_config ur10_e_moveit_config \
                          robotiq_gazebo
catkin build
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
