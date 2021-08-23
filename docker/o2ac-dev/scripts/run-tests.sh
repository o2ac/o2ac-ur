#!/bin/bash

################################################################################

# Manually runs all the tests. Could be replaced with `catkin run_tests -iv`.

source /root/o2ac-ur/catkin_ws/devel/setup.bash

rostest o2ac_routines test_robot_motions.test
rostest o2ac_routines test_assembly.test
# rostest o2ac_routines test_taskboard.test
