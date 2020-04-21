#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Team o2ac
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
import tf
import tf_conversions
from math import pi

from o2ac_routines.common import O2ACCommon

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import o2ac_msgs.msg

class TestClass(O2ACCommon):

  def __init__(self):
    super(TestClass, self).__init__()
    
    self.a_bot_downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    
    self.bridge = CvBridge()
    self._img = Image()
    
    rospy.sleep(.5)

  def views(self):
    high_height = .37
    low_height = .22
    x_offset = .04  # At low_height
    y_offset = .06  # At low_height

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "tray_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    ps.pose.position.z = .37
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    ps.pose.position.z = .22
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    ps.pose.position.x = x_offset
    ps.pose.position.y = y_offset
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    ps.pose.position.x = -x_offset
    ps.pose.position.y = y_offset
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    ps.pose.position.x = -x_offset
    ps.pose.position.y = -y_offset
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    ps.pose.position.x = x_offset
    ps.pose.position.y = -y_offset
    c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
    return True

if __name__ == '__main__':
  try:
    c = TestClass()

    while not rospy.is_shutdown():
      rospy.loginfo("Enter a number to run tests: ")
      rospy.loginfo("1: Go home with all robots")
      rospy.loginfo("2: Move b_bot above tray at 37 cm")
      rospy.loginfo("3: Move b_bot above tray at 22 cm")
      rospy.loginfo("4: Do distant view and 4 close views")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
      elif r == '2':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        ps.pose.position.z = .37
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '3':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        ps.pose.position.z = .22
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_cam_camera_color_frame", speed=.1, acceleration=.04)
      if r == '4':
        c.views()
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
