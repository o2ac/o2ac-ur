#!/bin/bash

################################################################################

# Build and source the workspaces
source /root/o2ac-ur/docker/o2ac-dev/scripts/initialize-catkin-workspace.sh

################################################################################

# Run tests
rostest o2ac_routines test_robot_motions.test

# Get 'roslaunch' process ID and kill it.
# kill $!
