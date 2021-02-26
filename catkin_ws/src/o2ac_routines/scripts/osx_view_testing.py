#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Team O2AC
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
#  * Neither the name of Team O2AC nor the names of its
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
import tf
import tf_conversions
from math import pi
tau = 2.0*pi  # Part of math from Python 3.6

from o2ac_routines.common import O2ACCommon

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import o2ac_msgs.msg

import o2ac_vision

class TestClass(O2ACCommon):

  def __init__(self):
    super(TestClass, self).__init__()
    rospy.sleep(.5)

  def views(self):
    self.go_to_pose_goal("b_bot", self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    self.go_to_pose_goal("b_bot", self.tray_view_low, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    self.close_view(1)
    rospy.sleep(2)

    self.close_view(2)
    rospy.sleep(2)

    self.close_view(3)
    rospy.sleep(2)

    self.close_view(4)
    return True
  
  def close_view(self, number):
    if number == 1:
      pose = self.tray_view_close_front_b
    elif number == 2:
      pose = self.tray_view_close_back_b
    elif number == 3:
      pose = self.tray_view_close_front_a
    elif number == 4:
      pose = self.tray_view_close_back_a
    self.go_to_pose_goal("b_bot", pose, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    return
  
  def call_belt_action_and_show(self):
    client = actionlib.SimpleActionClient('beltDetection', o2ac_msgs.msg.beltDetectionAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_result()

    print '\nbelt detection result'
    print(res)
    print "grasp points: ", grasp_points
    
    # self.publish_marker(pose, "place_pose")

if __name__ == '__main__':
  try:
    c = TestClass()

    while not rospy.is_shutdown():
      rospy.loginfo("Enter a number to run tests: ")
      rospy.loginfo("1: Go home with all robots")
      rospy.loginfo("2: Move b_bot above tray at 37 cm")
      rospy.loginfo("3: Move b_bot close (22 cm)")
      rospy.loginfo("31, 32, 33, 34: Close views")
      rospy.loginfo("4: Do distant view and 4 close views")
      rospy.loginfo("5: Call belt grasp point detection and show result")
      rospy.loginfo("61 (62): Look at taskboard bearing with inside cam (with outside camera, for CAD)")
      rospy.loginfo("9: Find bearing from center view")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
      elif r == '2':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        ps.pose.position.z = .37
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '3':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        ps.pose.position.z = .22
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '31':
        c.close_view(1)
      elif r == '32':
        c.close_view(2)
      elif r == '33':
        c.close_view(3)
      elif r == '34':
        c.close_view(4)
      elif r == '4':
        c.views()
      if r == '5':
        r = c.get_3d_poses_from_ssd()
        r2 = c.get_feasible_grasp_points(object_id=6)
        print(len(r2))

        # c.detect_object_in_camera_view("06_MBT4-400")  # Belt
        # c.call_belt_action_and_show()
      # if r == '5':
        # c.detect_object_in_camera_view("06_MBT4-400")  # Belt
        # c.call_belt_action_and_show()
      elif r == '61':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "taskboard_bearing_target_link"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*(0.5, 0.5, 0.5, 0.5))
        ps.pose.position = geometry_msgs.msg.Point(-0.153, -0.0025, -0.014)
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
      elif r == '62':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "taskboard_bearing_target_link"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*(0.62871, 0.545, 0.36517, 0.41756))
        ps.pose.position = geometry_msgs.msg.Point(-0.14509, -0.021323, 0.063084)
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_optical_frame", speed=.1, acceleration=.04)
      elif r == '9':
        c.look_for_item_in_tray("07_SBARB6200ZZ_30", "b_bot")
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."

