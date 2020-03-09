#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

# This file is based on the kinetic MoveIt tutorial for the Python movegroup interface.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()
    
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    group_name = rospy.get_param("move_group_name", "a_bot")
    rospy.loginfo(group_name)
    ee_link = rospy.get_param("ee_link", "a_bot_robotiq_85_tip_link")
    rospy.loginfo(ee_link)
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.group = group
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, pose_goal_stamped):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.set_pose_target(pose_goal_stamped)  # How to set multiple goals? Hm.

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01)

  def cycle_through_bins(self):
    # Define the pose in each bin for the end effector
    pose_goal = geometry_msgs.msg.PoseStamped()
    # This pose points downward.
    pose_goal.pose.orientation.x = -0.5
    pose_goal.pose.orientation.y = 0.5
    pose_goal.pose.orientation.z = 0.5
    pose_goal.pose.orientation.w = 0.5
    pose_goal.pose.position.z = 0.02

    bin_header_ids = ['/set2_bin1', '/set2_bin2', '/set2_bin3', '/set2_bin4']
    while not rospy.is_shutdown():
      for bin_id in bin_header_ids:
        pose_goal.header.frame_id = bin_id
        rospy.loginfo("Trying to move to pose:")
        rospy.loginfo(pose_goal)
        if self.go_to_pose_goal(pose_goal):
          rospy.sleep(4)
        if rospy.is_shutdown():
          break

  def check_all_bins(self):
    # Moves a_bot and b_bot to different bins and records success
    
    # Define the pose in the bin for the end effector
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.pose.orientation.x = -0.5
    pose_goal.pose.orientation.y = 0.5
    pose_goal.pose.orientation.z = 0.5
    pose_goal.pose.orientation.w = 0.5
    pose_goal.pose.position.z = 0.02

    # First use a_bot
    self.group = moveit_commander.MoveGroupCommander("a_bot")
    success = True

    # bin_header_ids = ['/set2_bin1', '/set2_bin2', '/set2_bin3', '/set2_bin4', '/set1_bin1', '/set1_bin2', '/set1_bin3']
    bin_header_ids = ['/set2_bin1', '/set2_bin2']
    for bin_id in bin_header_ids:
      pose_goal.header.frame_id = bin_id
      rospy.loginfo("Trying to move a_bot to bin:" + bin_id)
      if self.go_to_pose_goal(pose_goal):
        rospy.sleep(2)
      else:
        success = False
    
    if not success:
      # Add a line to the log here
      rospy.loginfo("Something went wrong")
    
    # Move the robot back (there will be a more convenient function for this later)
    home_pose = geometry_msgs.msg.PoseStamped()
    home_pose.pose = pose_goal.pose.orientation
    home_pose.pose.position.x = -0.4
    home_pose.pose.position.y = 0.4
    home_pose.pose.position.z = 0.4
    home_pose.header.frame_id = "a_bot_base_link"
    self.go_to_pose_goal(home_pose)

    # Now check with b_bot
    self.group = moveit_commander.MoveGroupCommander("b_bot")
    success = True

    bin_header_ids = ['/set2_bin1', '/set2_bin2', '/set2_bin3']
    for bin_id in bin_header_ids:
      pose_goal.header.frame_id = bin_id
      rospy.loginfo("Trying to move b_bot to bin:" + bin_id)
      if self.go_to_pose_goal(pose_goal):
        rospy.sleep(2)
      else:
        success = False
  
    if not success:
      # Add a line to the log here
      rospy.loginfo("Something went wrong")


def main():
  try:
    tutorial = MoveGroupPythonInterfaceTutorial()

    print "============ Press `Enter` to go to different bin positions ..."
    raw_input()
    # tutorial.cycle_through_bins()
    tutorial.check_all_bins()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
