#!/bin/sh

rosrun o2ac_pose_distribution_updater print_scene panel_bearing 03-PANEL2.stl any > tmp0 && echo Planning starts && rosrun o2ac_pose_distribution_updater planner_test tmp0 > tmp1 && echo Visualization of the plan starts && rosrun o2ac_pose_distribution_updater simulation tmp1
rm -f tmp0 tmp1
