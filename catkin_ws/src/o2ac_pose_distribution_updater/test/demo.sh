#!/bin/sh

rosrun o2ac_pose_distribution_updater print_scene > tmp0 && rosrun o2ac_pose_distribution_updater planner_test tmp0 > tmp1 && rosrun o2ac_pose_distribution_updater simulation tmp1
rm -f tmp0 tmp1
