/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <o2ac_examples/o2ac_pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr char LOGNAME[] = "o2ac_demo";

int main(int argc, char **argv) {
  ROS_INFO_NAMED(LOGNAME, "Init o2ac_demo");
  ros::init(argc, argv, "o2ac_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep(); // Wait for ApplyPlanningScene service
  moveit::planning_interface::PlanningSceneInterface psi;
  ros::NodeHandle pnh("~");

  // Construct and run pick/place task
  o2ac_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
  pick_place_task.loadParameters();
  pick_place_task.init();
  if (pick_place_task.plan()) {
    ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
    if (pnh.param("execute", false)) {
      pick_place_task.execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    } else {
      ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    }
  } else {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
