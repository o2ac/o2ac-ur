#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

// This example makes the robot end effector move up and down every 10 seconds.

int main(int argc, char **argv) {

  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::PoseStamped command_cartesian_position;
  std::string movegroup_name, ee_link;

  // Dynamic parameters. Last arg is the default value. You can assign these
  // from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "a_bot");
  nh.param<std::string>("ee_link", ee_link, "a_bot_gripper_tip_link");

  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate *loop_rate_ = new ros::Rate(ros_rate);

  int direction = 1;

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

      command_cartesian_position = group.getCurrentPose(ee_link);
      ROS_INFO_STREAM("EE at: "
                      << command_cartesian_position.pose.position.x << ", "
                      << command_cartesian_position.pose.position.y << ", "
                      << command_cartesian_position.pose.position.z);

      command_cartesian_position.pose.position.z -= direction * 0.10;

      ROS_INFO_STREAM("Move to: "
                      << command_cartesian_position.pose.position.x << ", "
                      << command_cartesian_position.pose.position.y << ", "
                      << command_cartesian_position.pose.position.z);

      group.setStartStateToCurrentState();
      group.setPoseTarget(command_cartesian_position, ee_link);
      success_plan = group.plan(myplan);
      if (success_plan) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        direction *= -1;     // In the next iteration the motion will be on the
                             // opposite direction
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will
                             // run every 10 seconds in this example.
      }
    }
  }
};
