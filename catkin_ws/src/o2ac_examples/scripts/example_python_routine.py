#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X Corp.
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
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
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

import rospy
import geometry_msgs.msg
import tf_conversions
from math import pi

from o2ac_routines.base import O2ACCommonBase

class ExampleClass(O2ACCommonBase):
  # Use a class like this to extend the base class and create your own routines.

  def __init__(self):
    super(ExampleClass, self).__init__()
    rospy.sleep(.5)

  def my_move_function(self, robot="b_bot"):
    # Create a target Pose in the world
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "workspace_center"  # The frame in which the pose is defined
    target_pose.pose.position.x = 0.1 
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.1 
    
    target_pose.pose.orientation.w = 1.0  # This sets the orientation to neutral (= the same as the frame_id)
    
    # This command sets an orientation from roll, pitch, yaw
    target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*3/4, pi/2))
    
    # This uses the go_to_pose_goal function defined in base.py to go to the pose
    self.go_to_pose_goal(robot, target_pose, 
            speed=0.03, acceleration=.1, end_effector_link=robot+"_robotiq_85_tip_link", move_lin=False)
    
    # The end_effector_link defines which part of the robot is moved to the target pose (it can be another part, or a tool!)
    # move_lin defines if the robot will attempt a linear motion or "free motion" planning


if __name__ == '__main__':
  try:
    c = ExampleClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to move a_bot to the target pose.")
      rospy.loginfo("Enter 2 to move b_bot to the target pose.")
      rospy.loginfo("Enter 3 to move robhots to home position.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        c.my_move_function("a_bot")
      if i == '2':
        c.my_move_function("b_bot")
      if i == '3':
        c.go_to_named_pose("home","a_bot")
        c.go_to_named_pose("home","b_bot")
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
