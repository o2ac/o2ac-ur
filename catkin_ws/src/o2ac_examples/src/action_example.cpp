#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robotiq_msgs/CModelCommandAction.h>
#include <ros/ros.h>

// This example shows how to open and close the gripper via an action
// Tested with a real UR and a gripper connected to it. Consult the launch file.
// Based on http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_gripper");

  // Create the action client
  // True causes the client to spin its own thread
  actionlib::SimpleActionClient<robotiq_msgs::CModelCommandAction>
      b_bot_gripper_client("/b_bot/gripper_action_controller", true);

  ROS_INFO("Waiting for action server to start.");
  b_bot_gripper_client.waitForServer(); // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // Send a goal to the action
  robotiq_msgs::CModelCommandGoal goal;
  goal.position = 0.0; // Opening width. 0 to close, 0.085 to open the gripper.
  b_bot_gripper_client.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout =
      b_bot_gripper_client.waitForResult(ros::Duration(5.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = b_bot_gripper_client.getState();
    ROS_INFO("Gripper-closing action finished: %s", state.toString().c_str());
  } else
    ROS_INFO("Gripper-closing action did not finish before the time out.");

  // ###############

  goal.position = 0.5;
  b_bot_gripper_client.sendGoal(goal);
  finished_before_timeout =
      b_bot_gripper_client.waitForResult(ros::Duration(5.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = b_bot_gripper_client.getState();
    ROS_INFO_STREAM("Gripper-opening action finished: " << state.toString());
  } else
    ROS_INFO("Gripper-opening action did not finish before the time out.");

  return 0;
}
