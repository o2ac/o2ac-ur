#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, OMRON SINIC X
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
#  * Neither the name of Team o2ac nor the names of its
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
import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi, degrees, radians, cos, sin
import math

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg

from o2ac_routines.base import o2acBaseRoutines
from o2ac_routines.panda_control import Panda


class ExperimentClass(o2acBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(ExperimentClass, self).__init__()
    
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    self.magic_a_bot_pick_offset_tray_2_x = -.004  # negative moves towards a_bot
    self.panda = Panda()
    
  ################ ----- Routines  
  ################ 
  ################ 
  def insert(self, robot_name):
    # Insert the peg into the horizontal jig. Assumes that the peg is already grasped by the robot
    if robot_name == "b_bot":
        tip_link = "b_bot_robotiq_85_tip_link"
        axial_offset = 0.05
        y_offset = 0.0
        z_offset = -0.002
    elif robot_name == "a_bot":
        tip_link = "panda_tip"
        axial_offset = 0.072
        y_offset = -0.003
        z_offset = -0.0035
        
        self.panda.grasp()
        self.panda.grasp() # Sometimes the command somehow opens the gripper, so we send it twice.
        self.panda.cs.switch_controllers(["moveit"])
    
    self.set_softness(robot_name, softness_on=False)

    above_pose = geometry_msgs.msg.PoseStamped()
    above_pose.header.frame_id = "target_bearing_frame"
    above_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    above_pose.pose.position.x = -0.05 - axial_offset
    above_pose.pose.position.z = 0.2
    
    before_pose1 = copy.deepcopy(above_pose)
    before_pose1.pose.position.x = -0.05 - axial_offset
    before_pose1.pose.position.y = y_offset
    before_pose1.pose.position.z = z_offset
    before_pose2 = copy.deepcopy(before_pose1)
    before_pose2.pose.position.x = -0.0 - axial_offset

    self.log_to_debug_monitor("Moving to pose above the jig", "operation")
    self.go_to_pose_goal(robot_name, above_pose, speed=.1,end_effector_link=tip_link, move_lin = True)
    self.log_to_debug_monitor("Moving to before_pose1", "operation")
    self.go_to_pose_goal(robot_name, before_pose1, speed=.1,end_effector_link=tip_link, move_lin = True)
    self.log_to_debug_monitor("Moving to before_pose2", "operation")
    self.go_to_pose_goal(robot_name, before_pose2, speed=.05,end_effector_link=tip_link, move_lin = True)

    if robot_name == "b_bot":
        self.movelin_around_shifted_tcp(robot_name, desired_twist = [0,0,0,radians(8),0,0], tcp_position = [0,0, 0.308, 0,0,0], velocity = 0.5)
        self.do_linear_push(robot_name, force=30, direction_vector=[0, sin(radians(8)), cos(radians(8))], forward_speed=0.01)
        self.set_softness(robot_name, softness_on=True)
        d = 0.004  # Distance to move backwards after soft wrist. This is hand-tuned.
        self.move_lin_rel("b_bot", [0, -d*sin(radians(8)), -d*cos(radians(8))])
        self.movelin_around_shifted_tcp(robot_name, desired_twist = [0,0,0,-radians(8),0,0], tcp_position = [0,0, 0.308, 0,0,0], velocity = 0.5)
        rospy.sleep(1)
        # self.confirm_to_proceed("Check if stuck")
        
        ### self.move_lin_rel("b_bot", [0, -.001, 0], acceleration = 0.05, velocity = .002)  # Move upwards after positioning
        ### rospy.sleep(1)
        self.do_helix_motion("b_bot", max_force = 70, max_radius=.001, helix_forward_limit=0.01, helix_forward_increment=0.001)
        

        ### HACKY SOLUTION ONLY FOR THE VIDEO
        self.do_linear_push(robot_name, force=50)
        self.move_lin_rel("b_bot", [0, 0, 0], relative_rotation=[0,0,-radians(179)], velocity = 2.0, acceleration = 2.0)
        self.move_lin_rel("b_bot", [0, 0, 0.002])
        self.move_lin_rel("b_bot", [0, 0, 0], relative_rotation=[0,0,radians(179)], velocity = 2.0, acceleration = 2.0)
        self.move_lin_rel("b_bot", [0, 0, 0.002])
        self.set_softness(robot_name, softness_on=False)
        rospy.sleep(2)
        self.send_gripper_command("b_bot", 0.075)
        rospy.sleep(1)
        self.confirm_to_proceed("=== DID THE GRIPPER OPEN?")
        self.move_lin_rel("b_bot", [0, 0, -0.07])
    elif robot_name == "a_bot":
        # self.panda.move_till_contact(0.02)
        self.panda.set_collision_behaviour_low()
        self.set_softness(robot_name, softness_on=True)
        self.panda.helix_motion(0.01)
    
    ### TODO: Open, move back, push
    # self.send_gripper_command("b_bot", "open")
    # rospy.sleep(1)
    # self.set_softness(robot_name, softness_on=False)
    # self.move_lin_rel("b_bot", [0, 0, -0.03])
    # self.send_gripper_command("b_bot", "close")
    # rospy.sleep(1)
    # self.do_linear_push(robot_name, force=50)
    # rospy.sleep(1)
    # self.do_linear_push(robot_name, force=50)
    # rospy.sleep(1)
    # self.do_linear_push(robot_name, force=50)
    
    rospy.loginfo("Done!")


  def set_softness(self, robot_name, softness_on=False):
    if robot_name == "b_bot":
        self.set_flex_wrist(rigid_mode=not softness_on)
    elif robot_name == "a_bot":
        softness = 800.0 if softness_on else 5000.0
        self.panda.set_joint_impedance(softness)
        self.panda.cs.switch_controllers(["moveit"])


if __name__ == '__main__':
  try:
    c = ExperimentClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to move the robots home to starting positions.")
      rospy.loginfo("Enter 10 to move the robots to position 'back'.")
      rospy.loginfo("Enter 2 to do the insertion with b_bot.")
      rospy.loginfo("Enter 3 to do the insertion with the panda.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        c.go_to_named_pose("home", "a_bot", speed=c.speed_fastest, acceleration=c.acc_fastest)
        c.go_to_named_pose("home", "b_bot", speed=c.speed_fastest, acceleration=c.acc_fastest)
      if i == '10':
        c.go_to_named_pose("back", "a_bot", speed=c.speed_fastest, acceleration=c.acc_fastest)
        c.go_to_named_pose("back", "b_bot", speed=c.speed_fastest, acceleration=c.acc_fastest)
      if i == '2':
        c.insert("b_bot")
        pass
      if i == '3':
        c.insert("a_bot")
      if i == '4':
        # This resets the TCP. Has to be called if movelin_around_shifted_tcp was interrupted.
        c.set_tcp_in_ur("b_bot")
      if i == 'x':
        break
      if i == "":
        i == True
        continue
  except rospy.ROSInterruptException:
    pass
