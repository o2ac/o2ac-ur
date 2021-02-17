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
tau = 2.0*pi  # Part of math from Python 3.6

from o2ac_routines.common import O2ACCommon

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import o2ac_msgs.msg

class CalibrationClass(O2ACCommon):
  """
  These routines check the robots' calibration by moving them to
  objects defined in the scene.
  """

  def __init__(self):
    super(CalibrationClass, self).__init__()
    
    self.a_bot_downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/2))
    self.bin_names = ["bin3_1", "bin2_4", "bin2_3", "bin2_2", "bin2_1", "bin1_2", "bin1_1", "bin1_4", "bin1_5", "bin1_3" ]

    self.bridge = CvBridge()
    self._img = Image()
    
    # Neutral downward in the taskboard frames
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, with_approach=False, move_lin=False, go_home=True, end_effector_link=""):
    home_pose = "home"
    if "screw" in end_effector_link:
      home_pose = "screw_ready"
      
    if with_approach:
      rospy.logwarn("with_approach only moves in the X direction of the header frame. Be careful.")

    for pose in poses:  
      ps_approach = copy.deepcopy(pose)
      ps_approach.pose.position.x -= .05
      rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
      self.publish_marker(pose, "place_pose")
      raw_input()
      if go_home:
        self.go_to_named_pose(home_pose, robot_name)
      if with_approach:
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      if rospy.is_shutdown():
        break
      if with_approach:
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
        self.go_to_pose_goal(robot_name, pose,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      else:
        self.go_to_pose_goal(robot_name, pose,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      
      rospy.loginfo("============ Press `Enter` to proceed ")
      raw_input()
      if with_approach:
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      if go_home:
        self.go_to_named_pose(home_pose, robot_name, force_ur_script=move_lin)
    
    if go_home:
      rospy.loginfo("Moving all robots home again.")
      self.go_to_named_pose("home", "a_bot")
      self.go_to_named_pose("home", "b_bot")
      self.go_to_named_pose("home", "c_bot")
    return

  def assembly_calibration_base_plate(self, robot_name="b_bot", end_effector_link = "", context = ""):
    rospy.loginfo("============ Calibrating base plate for the assembly task. ============")
    rospy.loginfo("eef link " + end_effector_link + " should be 5 mm above each corner of the plate.")
    if robot_name=="a_bot":
      self.go_to_named_pose("back", "b_bot")
    elif robot_name=="b_bot":
      self.go_to_named_pose("back", "a_bot")

    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link or "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    
    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.01
    if context == "b_bot_m4_assembly_plates":
      self.go_to_named_pose("screw_ready", robot_name)
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/8, 0, 0) )
    if context == "motor_plate" and "screw" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/6, 0, 0) )
    if robot_name == "a_bot":
      # pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/2, 0, 0) )
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, 0, 0) ) # To turn the gripper / tool
      # pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau*7/12, 0, 0) ) # For checking the screw tool is aligned


    if context == "motor_plate":
      for i in range(2):
        poses.append(copy.deepcopy(pose0))
      poses[0].header.frame_id = "assembled_assy_part_02_bottom_screw_hole_aligner_2"
      poses[1].header.frame_id = "assembled_assy_part_02_bottom_screw_hole_aligner_1"
    else:
      for i in range(4):
        poses.append(copy.deepcopy(pose0))
      poses[0].header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_2"
      poses[1].header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_1"
      poses[2].header.frame_id = "assembled_assy_part_01_fixation_hole_1"
      poses[3].header.frame_id = "assembled_assy_part_01_fixation_hole_2"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, end_effector_link=end_effector_link, move_lin=True)
    return 

  def assembly_calibration_assembled_parts(self):
    rospy.loginfo("============ Calibrating full assembled parts for the assembly task. ============")
    rospy.loginfo("b_bot gripper tip should go close to some important spots.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.02

    for i in range(5):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "assembled_assy_part_03"   # Top of plate 2
    poses[1].pose.position.x = .058
    poses[1].pose.position.y = -.0025
    poses[1].pose.position.z = .095 + .01
    poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0) )

    poses[2].header.frame_id = "assembled_assy_part_08_front_tip"  # Front of rotary shaft
    poses[2].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -tau/2) )
    poses[2].pose.position.x = .03

    poses[3].header.frame_id = "assembled_assy_part_14_screw_head"
    poses[3].pose.position.x = -.03

    poses[4].header.frame_id = "assembled_assy_part_04_tip"
    poses[4].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/2, 0, -tau/2) )
    poses[4].pose.position.x = .03

    self.cycle_through_calibration_poses(poses, "b_bot", speed=0.3, go_home=True)
    return 
  
  def taskboard_calibration_with_tools(self, robot_name="b_bot", end_effector_link = ""):
    rospy.loginfo("============ Calibrating taskboard screw holes. ============")
    rospy.loginfo("eef link " + end_effector_link + " should be 5 mm above each corner of the plate.")
    if robot_name=="a_bot":
      self.go_to_named_pose("back", "b_bot")
    elif robot_name=="b_bot":
      self.go_to_named_pose("back", "a_bot")

    self.go_to_named_pose("horizontal_screw_ready", robot_name)
    
    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.01
    if robot_name == "a_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0) )
    elif robot_name == "b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0) )


    for i in range(3):
      poses.append(copy.deepcopy(pose0))
    poses[0].header.frame_id = "taskboard_set_screw_link"
    poses[1].header.frame_id = "taskboard_m3_screw_link"
    poses[2].header.frame_id = "taskboard_m4_screw_link"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, with_approach=True, end_effector_link=end_effector_link, move_lin=True)
    return 
      
  def touch_workspace_center(self):
    rospy.loginfo("============ Touching workspace center. ============")
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    poses = []

    pose_a = geometry_msgs.msg.PoseStamped()
    pose_a.header.frame_id = "workspace_center"
    pose_a.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    pose_a.pose.position.x = .0
    pose_a.pose.position.y = -.2
    pose_a.pose.position.z = .03

    pose_b = copy.deepcopy(pose_a)
    pose_b.pose.position.x = .0
    pose_b.pose.position.y = .3
    pose_b.pose.position.z = .03
    pose_c = copy.deepcopy(pose_a)
    pose_c.pose.position.x = -.3
    pose_c.pose.position.y = .2
    pose_c.pose.position.z = .03
    
    rospy.loginfo("============ Going to 2 cm above the table. ============")
    self.go_to_pose_goal("b_bot", pose_b, speed=1.0)
    self.go_to_pose_goal("a_bot", pose_a, speed=1.0)

    rospy.loginfo("============ Press enter to go to .1 cm above the table. ============")
    i = raw_input()
    if not rospy.is_shutdown():
      pose_a.pose.position.z = .001
      pose_b.pose.position.z = .001
      pose_c.pose.position.z = .001
      self.go_to_pose_goal("b_bot", pose_b, speed=0.01)
      self.go_to_pose_goal("a_bot", pose_a, speed=0.01)

    rospy.loginfo("============ Press enter to go home. ============")
    raw_input()
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    return

  def make_space_for_robot(self, robot_name):
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "a_bot")
    elif robot_name=="a_bot":
      self.go_to_named_pose("back", "b_bot")

  def screw_tool_test_assembly(self, robot_name = "b_bot", tool_name="_screw_tool_m4_tip_link"):
    rospy.loginfo("============ Moving the screw tool m4 to the screw holes on the base plate ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    self.make_space_for_robot(robot_name)

    self.go_to_named_pose("screw_ready", robot_name)
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "assembled_assy_part_01_screw_hole_panel1_1"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/8, 0, 0))
      pose0.pose.position.x -= .01
    
    for i in range(4):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "assembled_assy_part_01_screw_hole_panel1_2"
    poses[2].header.frame_id = "assembled_assy_part_01_screw_hole_panel2_1"
    poses[3].header.frame_id = "assembled_assy_part_01_screw_hole_panel2_2"
    end_effector_link=robot_name+ tool_name
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    return
    
  def screw_action_test(self, robot_name = "b_bot"):
    rospy.loginfo("============ Screwing in one of the plate screws with the tool using the action ============")
    rospy.loginfo("============ The screw tool m4 and a screw have to be carried by the robot! ============")
    self.go_to_named_pose("screw_ready", robot_name)

    if robot_name=="b_bot":
      self.go_to_named_pose("screw_plate_ready", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    if robot_name=="b_bot":
      pose0.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_aligner_1"
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/8, 0, 0))
      pose0.pose.position.x = -.01
    
    self.do_screw_action(robot_name, pose0, screw_size = 4, screw_height = .02)
    self.go_to_named_pose("screw_plate_ready", robot_name)
    return

  def screw_feeder_calibration(self, robot_name = "b_bot"):
    rospy.loginfo("============ Moving the screw tool m3 or m4 to its screw feeder ============")
    rospy.loginfo("============ The screw tool has to be carried by the robot! ============")
    
    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    pose0 = geometry_msgs.msg.PoseStamped()
    if robot_name == "a_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
      pose0.header.frame_id = "m3_feeder_outlet_link"
      ee_link = robot_name + "_screw_tool_m3_tip_link"
    else:
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      pose0.header.frame_id = "m4_feeder_outlet_link"
      ee_link = robot_name + "_screw_tool_m4_tip_link"
    
    self.toggle_collisions(collisions_on=False)
    
    poses = []
    for i in range(3):
      poses.append(copy.deepcopy(pose0))
    poses[0].pose.position.x = -.02
    poses[1].pose.position.x = 0
    poses[2].pose.position.x = -.02
    
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=ee_link)
    self.toggle_collisions(collisions_on=True)
    return
  
  def screw_feeder_pick_test(self, robot_name = "b_bot", screw_size = 4):
    rospy.loginfo("============ Picking a screw from a feeder ============")
    rospy.loginfo("============ The screw tool has to be carried by the robot! ============")
    
    self.pick_screw_from_feeder(robot_name, screw_size=screw_size)
    return

if __name__ == '__main__':
  try:
    c = CalibrationClass()

    while not rospy.is_shutdown():
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to check calibrations for the following things: ")
      rospy.loginfo("1000: Move with different jerk values")
      rospy.loginfo("100 (1001): Go home with all robots (using UR script joint move)")
      rospy.loginfo("101 (1011): Go back with all robots (using UR script joint move)")
      rospy.loginfo("111: The robots (Using the assembly base plate)")
      rospy.loginfo("===== TASKBOARD TASK")
      rospy.loginfo("21, 22: Go to screw holes with a_bot m3, b_bot m4")
      rospy.loginfo("===== ASSEMBLY TASK (no parts may be mounted!)")
      rospy.loginfo("501-502: Assembly base plate (a_bot, b_bot)")
      rospy.loginfo("503-505: Assembly base plate (b_bot m4, b_bot m3, a_bot m3)")
      rospy.loginfo("511-512: Motor plate holes (b_bot, b_bot m4)")
      rospy.loginfo("6: ===== TOOLS   Go to screw_ready with b (a goes to back)")
      rospy.loginfo("621, 622: Equip/unequip m4 screw tool with b_bot")
      rospy.loginfo("623, 624: Equip/unequip m3 screw tool with b_bot")
      rospy.loginfo("625, 626: Equip/unequip m6 nut tool with a_bot")
      rospy.loginfo("627, 628: Equip/unequip set screw tool with b_bot")
      rospy.loginfo("63: Go to assembly base plate with m4 screw tool (b_bot)")
      rospy.loginfo("65 (651/652): Go to belt tool pickup position (and equip/unequip it)")
      rospy.loginfo("66 (661/662): Go to plunger tool pickup position (and equip/unequip it)")
      rospy.loginfo("671, 672: Calibrate screw feeders (a_bot, b_bot)")
      rospy.loginfo("681, 682: Pick m4 screw from feeder (a_bot, b_bot)")
      rospy.loginfo("691, 692: Pick m3 screw from feeder (a_bot, b_bot)")
      rospy.loginfo("70: Do screw action with b_bot on rightmost hole")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
      elif r == '1000':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "workspace_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/2))
        ps.pose.position.z = .2
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.04)
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.08)
        c.go_to_named_pose("home", "b_bot")
        c.move_lin("b_bot", ps, speed=.2, acceleration=.15)
        c.go_to_named_pose("home", "b_bot")
      elif r == '100':
        c.go_to_named_pose("home", "a_bot")
        c.go_to_named_pose("home", "b_bot")
      elif r == '1001':
        c.go_to_named_pose("home", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
      elif r == '101':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("back", "b_bot")
      elif r == '1011':
        c.go_to_named_pose("back", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
        c.go_to_named_pose("back", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=True)
      elif r == '111':
        c.check_robot_calibration(position="assembly_corner_4")
      elif r == '21':
        c.taskboard_calibration_with_tools(robot_name="a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link")
      elif r == '22':
        c.taskboard_calibration_with_tools(robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link")
      elif r == '501':
        c.assembly_calibration_base_plate("a_bot")
      elif r == '502':
        c.assembly_calibration_base_plate("b_bot")
      elif r == '503':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link")
      elif r == '504':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m3_tip_link")
      elif r == '505':
        c.assembly_calibration_base_plate("a_bot", end_effector_link="a_bot_screw_tool_m3_tip_link")
      elif r == '5041':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", context="b_bot_m4_assembly_plates")
      elif r == '511':
        c.assembly_calibration_base_plate("b_bot", context="motor_plate")
      elif r == '512':
        c.assembly_calibration_base_plate("b_bot", end_effectodr_link="b_bot_screw_tool_m4_tip_link", context="motor_plate")
      elif r == '6':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("screw_ready", "b_bot")
      elif r == '621':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      elif r == '622':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      elif r == '623':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 3)
      elif r == '624':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 3)
      elif r == '625':
        c.make_space_for_robot("a_bot")
        c.go_to_named_pose("tool_pick_ready", "a_bot")
        c.do_change_tool_action("a_bot", equip=True, screw_size = 66)
      elif r == '626':
        c.make_space_for_robot("a_bot")
        c.go_to_named_pose("tool_pick_ready", "a_bot")
        c.do_change_tool_action("a_bot", equip=False, screw_size = 66)
      elif r == '627':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 1)
      elif r == '628':
        c.make_space_for_robot("b_bot")
        c.go_to_named_pose("tool_pick_ready", "b_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 1)
      elif r == '65':
        c.go_to_named_pose("home", "b_bot")
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "belt_tool_pickup_link"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))
        ps.pose.position.x = -.05
        c.go_to_pose_goal("b_bot", ps, speed=.2, acceleration=.04)
      elif r == '651':
        c.do_change_tool_action("b_bot", equip=True, screw_size = 100)
      elif r == '66':
        c.go_to_named_pose("home", "b_bot")
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "plunger_tool_pickup_link"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))
        ps.pose.position.x = -.05
        c.go_to_pose_goal("b_bot", ps, speed=.2, acceleration=.04)
      elif r == '661':
        c.do_change_tool_action("b_bot", equip=True, screw_size = 200)
      elif r == '63':
        c.screw_tool_test_assembly(robot_name="b_bot")
      elif r == '671':
        c.screw_feeder_calibration(robot_name="a_bot")
      elif r == '672':
        c.screw_feeder_calibration(robot_name="b_bot")
      elif r == '681':
        c.screw_feeder_pick_test(robot_name="a_bot", screw_size=4)
      elif r == '682':
        c.screw_feeder_pick_test(robot_name="b_bot", screw_size=4)
      elif r == '691':
        c.screw_feeder_pick_test(robot_name="a_bot", screw_size=3)
      elif r == '692':
        c.screw_feeder_pick_test(robot_name="b_bot", screw_size=3)
      elif r == '70':
        c.screw_action_test(robot_name="b_bot")
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
