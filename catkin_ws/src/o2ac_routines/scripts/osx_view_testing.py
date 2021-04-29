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
from math import pi, degrees, radians
tau = 2.0*pi  # Part of math from Python 3.6

from o2ac_routines.common import O2ACCommon

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import o2ac_msgs.msg

import o2ac_vision
from o2ac_routines.helpers import wait_for_UR_program
import o2ac_routines.helpers as helpers

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

    goal = o2ac_msgs.msg.get2DPosesFromSSDGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()
    res = client.get_result()

    print '\nbelt detection result'
    print(res)
    print "grasp points: ", grasp_points
    
    # self.skill_server.publish_marker(pose, "place_pose")

if __name__ == '__main__':
  try:
    rospy.init_node('o2ac_routines', anonymous=False)
    c = TestClass()

    while not rospy.is_shutdown():
      rospy.loginfo("Enter a number to run tests: ")
      rospy.loginfo("1: Go home with all robots")
      rospy.loginfo("11: Activate b_bot inside, outside (12) camera")
      rospy.loginfo("2: Move b_bot above tray at 37 cm")
      rospy.loginfo("3: Move b_bot close (22 cm)")
      rospy.loginfo("31, 32, 33, 34: Close views")
      rospy.loginfo("4: Do distant view and 4 close views")
      rospy.loginfo("5: Call tray detection and show result")
      rospy.loginfo("8: Look for shaft")
      rospy.loginfo("CAD matching: 61: bearing, 62: base plate, 63: motor plate, 64: bearing plate")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
      elif r == '11':
        c.camera.activate("b_bot_inside_camera")
      elif r == '12':
        c.camera.activate("b_bot_outside_camera")
      elif r == '2':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        ps.pose.position.z = .37
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
      elif r == '3':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        ps.pose.position.z = .22
        # c.go_to_named_pose("home", "a_bot")
        c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.04)
      elif r == '31':
        c.close_view(1)
      elif r == '32':
        c.close_view(2)
      elif r == '33':
        c.close_view(3)
      elif r == '34':
        c.close_view(4)
      elif r == '331':
        for ps in c.close_tray_views:
          c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '332':
        for ps in c.close_tray_views_rot_left:
          c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '333':
        for ps in c.close_tray_views_rot_right:
          c.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '5':
        c.camera.activate("b_bot_outside_camera")
        rospy.sleep(1)
        res = c.get_3d_poses_from_ssd()
        obj_id = 7 #bearing
        print("=====")
        print(res)
        print("=====")
        r2 = c.get_feasible_grasp_points(obj_id)
        # print("=====")
        # print(c.objects_in_tray[obj_id])
        # print("=====")
        try:
          print(len(r2))
          print(str(r2[0]))
        except:
          pass
      elif r == '51':
        c.go_to_named_pose("home", "b_bot")
        p = r2[0]
        p.pose.position.z = 0.0
        c.simple_pick("a_bot", p, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == '52':
        c.go_to_named_pose("home", "a_bot")
        p = r2[0]
        p.pose.position.z = 0.0
        c.simple_pick("b_bot", p, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == '61':
        if not c.assembly_database.db_name == "wrs_assembly_1":
          c.assembly_database.load_db("wrs_assembly_1")
        c.look_for_item_in_tray("bearing", "b_bot")
      elif r == '62':
        if not c.assembly_database.db_name == "wrs_assembly_1":
          c.assembly_database.load_db("wrs_assembly_1")
        c.look_for_item_in_tray("base", "b_bot")
      elif r == '63':
        if not c.assembly_database.db_name == "wrs_assembly_1":
          c.assembly_database.load_db("wrs_assembly_1")
        c.look_for_item_in_tray("panel_motor", "b_bot")
      elif r == '64':
        if not c.assembly_database.db_name == "wrs_assembly_1":
          c.assembly_database.load_db("wrs_assembly_1")
        c.look_for_item_in_tray("panel_bearing", "b_bot")
      elif r == '7':
        c.b_bot.linear_push(force=10, direction="+Z", relative_to_ee=False, timeout=15.0)
      elif r == "8":
        goal = c.look_and_get_grasp_point(8)  # shaft
        if not goal:
          rospy.logerr("Could not find shaft in tray. Skipping procedure.")
        else:
          goal.pose.position.z = 0.001
          # goal.pose.position.x -= 0.01 # MAGIC NUMBER
          c.camera.activate("b_bot_inside_camera")
          c.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == "81":
        goal = c.look_and_get_grasp_point(5)  # motor pulley
        if not goal:
          rospy.logerr("Could not find motor pulley in tray. Skipping procedure.")
        else:
          goal.pose.position.z = 0.001
          # goal.pose.position.x -= 0.01 # MAGIC NUMBER
          c.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == "88":
        c.check_if_shaft_in_v_groove()
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
