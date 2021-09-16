#include "o2ac_helper_functions.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h> // Includes the TF conversions

// This example moves to different positions defined by frames in the scene,
// and shows how to transform between frames using a simple helper function.

int main(int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener tflistener;

  std::string movegroup_name, ee_link;
  geometry_msgs::PoseStamped command_cartesian_position;

  // Dynamic parameters. Last arg is the default value. You can assign these
  // from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "a_bot");
  nh.param<std::string>("ee_link", ee_link, "a_bot_gripper_tip_link");

  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate *loop_rate_ = new ros::Rate(ros_rate);

  int bin = 1;
  std::string bin_header;

  ros::Duration(1.0).sleep(); // 1 second

  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;

  // Configure planner
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link);
  moveit::planning_interface::MoveItErrorCode
      success_plan = moveit_msgs::MoveItErrorCodes::FAILURE,
      motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds
  while (ros::ok()) {
    if (true) {

      if (bin == 1) {
        bin_header = "/set2_bin1";
      }
      if (bin == 2) {
        bin_header = "/set2_bin2";
      }
      if (bin == 3) {
        bin_header = "/set2_bin3";
      }
      if (bin == 4) {
        bin_header = "/set2_bin4";
      }

      geometry_msgs::PoseStamped ps;
      ps.pose = makePose(0, 0, .02);
      ps.header.frame_id = bin_header;
      ps = transform_pose_now(ps, "/world", tflistener);
      // The above line is an example of an easy way to transform between
      // frames, (here: from the bin to the world frame) it is not necessary
      // here, since MoveIt already accepts stamped poses.

      // The lines below rotate the pose so the robot looks downwards.
      rotatePoseByRPY(0.0, 0.0, M_PI / 2.0, ps.pose);
      rotatePoseByRPY(0.0, M_PI / 2.0, 0.0, ps.pose);

      ROS_INFO_STREAM("Moving to pose: "
                      << ps.pose.position.x << ", " << ps.pose.position.y
                      << ", " << ps.pose.position.z << "; "
                      << ps.pose.orientation.x << ", " << ps.pose.orientation.y
                      << ", " << ps.pose.orientation.z << ", "
                      << ps.pose.orientation.w);

      group.setStartStateToCurrentState();
      group.setPoseTarget(ps, ee_link);
      success_plan = group.plan(myplan);
      if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        loop_rate_->sleep(); // Sleep for some milliseconds. The while loop will
                             // run every 5 seconds in this example.
      }
      bin++;
      if (bin > 4) {
        bin = 1;
      }
    }
  }
};
