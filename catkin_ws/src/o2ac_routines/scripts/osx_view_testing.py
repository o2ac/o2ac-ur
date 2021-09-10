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
from moveit_commander import robot
import rospy
import numpy as np

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

from ur_control import conversions

import o2ac_vision
from o2ac_routines.helpers import wait_for_UR_program
import o2ac_routines.helpers as helpers

class TestClass(O2ACCommon):

  def __init__(self):
    super(TestClass, self).__init__()
    rospy.sleep(.5)

  def views(self):
    self.b_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    self.b_bot.go_to_pose_goal(self.tray_view_low, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    rospy.sleep(2)

    self.close_view(1)
    rospy.sleep(2)

    self.close_view(2)
    rospy.sleep(2)

    self.close_view(3)
    rospy.sleep(2)

    self.close_view(4)
    return True
  
  def close_view(self, number, robot_name="b_bot"):
    if number == 1:
      pose = self.tray_view_close_front_b
    elif number == 2:
      pose = self.tray_view_close_back_b
    elif number == 3:
      pose = self.tray_view_close_front_a
    elif number == 4:
      pose = self.tray_view_close_back_a
    self.active_robots[robot_name].go_to_pose_goal(pose, end_effector_link=robot_name+"_outside_camera_color_frame", speed=.1, acceleration=.04)
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
      rospy.loginfo("11-14: Activate camera (11:a_in, 12:a_out, 13:b_in, 14:b_out)")
      rospy.loginfo("2: Move b_bot above tray at 37 cm (2a: a_bot)")
      rospy.loginfo("3: Move b_bot close (22 cm)")
      rospy.loginfo("31, 32, 33, 34: Close views")
      rospy.loginfo("37: b_bot above centering area view")
      rospy.loginfo("4: Call shaft notch detection")
      rospy.loginfo("5: Call SSD detection and show result")
      rospy.loginfo("(CAD matching) 61: base plate, 62: motor plate, 63: bearing plate, 64: motor, 65: bearing ")
      rospy.loginfo("8: Look for shaft")
      rospy.loginfo("89: Look at output pulley, start detecting screws")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.a_bot.go_to_named_pose("home")
        c.b_bot.go_to_named_pose("home")
      elif r == '11':
        c.vision.activate_camera("a_bot_inside_camera")
      elif r == '12':
        c.vision.activate_camera("a_bot_outside_camera")
      elif r == '13':
        c.vision.activate_camera("b_bot_inside_camera")
      elif r == '14':
        c.vision.activate_camera("b_bot_outside_camera")
      elif r == '2':
        tray_views = c.define_local_tray_views(robot_name="b_bot")
        c.b_bot.go_to_pose_goal(tray_views[0], end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
      elif r == '2a':
        tray_views = c.define_local_tray_views(robot_name="a_bot")
        c.a_bot.go_to_pose_goal(tray_views[0], end_effector_link="a_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
      elif r == '3':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
        ps.pose.position.z = .22
        # c.a_bot.go_to_named_pose("home")
        c.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '31':
        c.close_view(1)
      elif r == '32':
        c.close_view(2)
      elif r == '33':
        c.close_view(3)
      elif r == '34':
        c.close_view(4)
      elif r == '31a':
        c.close_view(1, robot_name="a_bot")
      elif r == '32a':
        c.close_view(2, robot_name="a_bot")
      elif r == '33a':
        c.close_view(3, robot_name="a_bot")
      elif r == '34a':
        c.close_view(4, robot_name="a_bot")
      elif r == "37":
        p_view = conversions.to_pose_stamped("right_centering_link", [-c.tray_view_high.pose.position.z, 0, 0, 0, 0, 0])
        p_view_tray = c.listener.transformPose("tray_center", p_view)
        p_view_tray.pose.orientation = c.tray_view_high.pose.orientation
        c.b_bot.go_to_pose_goal(p_view_tray, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, wait=True)
      elif r == '331':
        for ps in c.close_tray_views:
          c.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '332':
        for ps in c.close_tray_views_rot_left:
          c.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '333':
        for ps in c.close_tray_views_rot_right:
          c.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
      elif r == '4':
        c.vision.call_shaft_hole_detection()
      elif r == '5':
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
      elif r == '500':
        c.get_bearing_angle()
      elif r == '51':
        c.b_bot.go_to_named_pose("home")
        p = r2[0]
        p.pose.position.z = 0.0
        c.simple_pick("a_bot", p, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == '52':
        c.a_bot.go_to_named_pose("home")
        p = r2[0]
        p.pose.position.z = 0.0
        c.simple_pick("b_bot", p, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == '61':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("base", "b_bot")
      elif r == '62':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("panel_motor", "b_bot")
      elif r == '63':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("panel_bearing", "b_bot")
      elif r == '64':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("motor", "b_bot")
      elif r == '641':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("motor", "b_bot", skip_moving=True)
      elif r == '642':
        p_view = conversions.to_pose_stamped("move_group/motor/center", [0, 0, 0, 0, 0, 0])
        c.get_large_item_position_from_top("motor", "b_bot", skip_moving=True)
      elif r == "644":
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.b_bot.go_to_pose_goal(c.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
        res = c.get_3d_poses_from_ssd()
        obj_id = c.assembly_database.name_to_id("motor")
        r2 = c.get_feasible_grasp_points(obj_id)
        p = r2[0]
        p.pose.position.z = 0.015
        c.simple_pick("b_bot", p, gripper_force=100.0, grasp_width=.085, axis="z")
        c.b_bot.gripper.open()
      elif r == '645':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        while not rospy.is_shutdown():
          c.b_bot.go_to_pose_goal(c.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
          res = c.get_3d_poses_from_ssd()
          obj_id = c.assembly_database.name_to_id("motor")
          try:
            r2 = c.get_feasible_grasp_points(obj_id)
            p = r2[0]
            p.pose.position.z = 0.015
            if np.random.uniform() > 0.5:
              p = helpers.rotatePoseByRPY(tau/4, 0, 0, p)
            c.simple_pick("b_bot", p, gripper_force=100.0, grasp_width=.085, axis="z")
            c.b_bot.gripper.open()
          except:
            rospy.logerr("motor not found")
            rospy.sleep(2)
      elif r == '65':
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
        c.get_large_item_position_from_top("bearing", "b_bot")
      elif r == "8":
        goal = c.look_and_get_grasp_point(8)  # shaft
        if not goal:
          rospy.logerr("Could not find shaft in tray. Skipping procedure.")
        else:
          goal.pose.position.z = 0.001
          # goal.pose.position.x -= 0.01 # MAGIC NUMBER
          c.vision.activate_camera("b_bot_inside_camera")
          c.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == "81":
        goal = c.look_and_get_grasp_point(5)  # motor pulley
        if not goal:
          rospy.logerr("Could not find motor pulley in tray. Skipping procedure.")
        else:
          goal.pose.position.z = 0.001
          # goal.pose.position.x -= 0.01 # MAGIC NUMBER
          c.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")
      elif r == "86":
        c.look_at_motor_from_top()
      elif r == "87":
        c.look_at_motor()
      elif r == "871":
        print(c.get_motor_angle())
      elif r == "872":
        c.set_assembly("wrs_assembly_2020")
        c.confirm_motor_and_place_in_aid(calibration=True)
      elif r == "88":
        c.check_screw_hole_visible_on_shaft_in_v_groove()
      elif r == "881":
        res = c.vision.call_shaft_hole_detection()
        print("=== shaft screw_hole detection returned:")
        print(res)
      elif r == "89":
        if not c.assembly_database.db_name == "wrs_assembly_2020":
          c.set_assembly("wrs_assembly_2020")
          rospy.sleep(1.0)
        c.check_output_pulley_angle()
      elif r == "899":
        c.vision.activate_pulley_screw_detection()
      elif r == "890":
        c.vision.activate_pulley_screw_detection(False)
      elif r == "ledaon":
        c.activate_led("a_bot")
      elif r == "ledaoff":
        c.activate_led("a_bot", on=False)
      elif r == "ledbon":
        c.activate_led("b_bot")
      elif r == "ledboff":
        c.activate_led("b_bot", on=False)
      elif r == 'push':
        c.b_bot.linear_push(force=10, direction="-Z", relative_to_ee=False, timeout=15.0)
      elif r == "reset":
        c.reset_scene_and_robots()
        c.reset_assembly_visualization()
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
