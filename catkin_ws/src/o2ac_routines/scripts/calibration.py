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

from o2ac_routines.base import o2acBaseRoutines

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import actionlib
import o2ac_msgs.msg

class CalibrationClass(o2acBaseRoutines):
  """
  These routines check the robots' calibration by moving them to
  objects defined in the scene.
  """

  def __init__(self):
    super(CalibrationClass, self).__init__()
    
    self.a_bot_downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    self.bin_names = ["bin3_1", "bin2_4", "bin2_3", "bin2_2", "bin2_1", "bin1_2", "bin1_1", "bin1_4", "bin1_5", "bin1_3" ]

    self.bridge = CvBridge()
    self._img = Image()
    
    # Neutral downward in the taskboard frames
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, with_approach=False, move_lin=False, go_home=True, end_effector_link=""):
    home_pose = "home"
    if "screw" in end_effector_link:
      home_pose = "screw_ready"
      
    # rospy.loginfo("============ Moving " + robot_name + " to " + poses[0].header.frame_id)
    if with_approach:                 # To calculate the approach, we publish the target pose to TF
      rospy.logwarn("with_approach does not work yet. Do not use it.")
      # ps_approach = geometry_msgs.msg.PoseStamped()
      # ps_approach.header.frame_id = "calibration_target_pose"
      # ps_approach.pose.position.x -= .05

    for pose in poses:  
      rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
      self.publish_marker(pose, "place_pose")
      raw_input()
      if go_home:
        self.go_to_named_pose(home_pose, robot_name)
      if with_approach:
        ps_approach = copy.deepcopy(pose) # Dirty fix for the TF frame below not being found
        # br = tf.TransformBroadcaster()
        # br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        #                   (pose.pose.orientation.x, pose.pose.orientation.y,
        #                    pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(),
        #                    "calibration_target_pose", pose.header.frame_id)
        # rospy.sleep(.5)
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
        # br = tf.TransformBroadcaster()
        # br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        #                   (pose.pose.orientation.x, pose.pose.orientation.y,
        #                    pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(),
        #                    "calibration_target_pose", pose.header.frame_id)
        # rospy.sleep(.2)
        self.go_to_pose_goal(robot_name, ps_approach,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)
      if go_home:
        self.go_to_named_pose(home_pose, robot_name, force_ur_script=move_lin)
    
    # if go_home:
    #   rospy.loginfo("Moving all robots home again.")
    #   self.go_to_named_pose("home", "a_bot")
    #   self.go_to_named_pose("home", "b_bot")
    #   self.go_to_named_pose("home", "c_bot")
    return
  
  def check_robot_calibration(self, position=""):
    calib_pose = geometry_msgs.msg.PoseStamped()
    if position == "":
      calib_pose.header.frame_id = "workspace_center"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      calib_pose.pose.position.x = -0.2
      calib_pose.pose.position.z = 0.07
    if position == "assembly_corner_4":
      calib_pose.header.frame_id = "assembled_assy_part_01_corner_4"
      calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      calib_pose.pose.position.x = -0.01
    

    rospy.loginfo("============ Testing robot calibration. ============")
    rospy.loginfo("Each robot will move to this position in front of c_bot:")
    rospy.loginfo(calib_pose)

    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("back", "b_bot")

    rospy.loginfo("============ Press `Enter` to move b_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.send_gripper_command("b_bot", "close")
      self.go_to_pose_goal("b_bot", calib_pose, speed=1.0)

    rospy.loginfo("============ Press `Enter` to move a_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.go_to_named_pose("back", "b_bot")
      self.send_gripper_command("precision_gripper_inner", "close")
      self.go_to_pose_goal("a_bot", calib_pose, speed=1.0)

    rospy.loginfo("============ Press `Enter` to move robots back home.")
    raw_input()
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    return

  def check_level_calibration(self, robot_name="b_bot"):
    rospy.loginfo("============ Going to 5 mm above workspace center points with " + robot_name + " ============")
    if not robot_name == "b_bot":
      self.go_to_named_pose("back", "b_bot")
    if not robot_name == "a_bot":
      self.go_to_named_pose("back", "a_bot")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "workspace_center"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    if robot_name == "a_bot":
      rospy.logerr("This is not implemented for a_bot yet")
    if robot_name == "b_bot":
      pose0.pose.position.z = .005
      
      for i in range(5):
        poses.append(copy.deepcopy(pose0))

      poses[1].pose.position.x = -.3
      poses[1].pose.position.y = .2
      poses[3].pose.position.x = -.3
      poses[2].pose.position.y = -.2
      
      poses[3].pose.position.x = .3
      poses[3].pose.position.y = .2
      poses[4].pose.position.x = .3
      poses[4].pose.position.y = -.2

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, move_lin=True, go_home=True)
    return
  
  def assembly_calibration_base_plate(self, robot_name="b_bot", end_effector_link = "", context = ""):
    rospy.loginfo("============ Calibrating base plate for the assembly task. ============")
    rospy.loginfo("eef link " + end_effector_link + " should be 5 mm above each corner of the plate.")
    if robot_name=="a_bot":
      self.send_gripper_command("a_bot", "close")
      self.go_to_named_pose("back", "b_bot")
    elif robot_name=="b_bot":
      self.send_gripper_command("b_bot", "close")
      self.go_to_named_pose("back", "a_bot")

    if end_effector_link=="":
      self.go_to_named_pose("home", robot_name)
    elif "screw" in end_effector_link or "suction" in end_effector_link:
      self.go_to_named_pose("screw_ready", robot_name)
    
    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation.w = 1.0
    pose0.pose.position.x = -.005
    if context == "b_bot_m4_assembly_plates":
      self.go_to_named_pose("screw_plate_ready", robot_name)
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/4, 0, 0) )
    if context == "motor_plate" and "screw" in end_effector_link:
      self.go_to_named_pose("screw_plate_ready", robot_name)
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/3, 0, 0) )

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
      poses[2].header.frame_id = "assembled_assy_part_01_corner_3"
      poses[3].header.frame_id = "assembled_assy_part_01_corner_4"

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
    poses[1].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0) )

    poses[2].header.frame_id = "assembled_assy_part_08_front_tip"  # Front of rotary shaft
    poses[2].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, -pi) )
    poses[2].pose.position.x = .03

    poses[3].header.frame_id = "assembled_assy_part_14_screw_head"
    poses[3].pose.position.x = -.03

    poses[4].header.frame_id = "assembled_assy_part_04_tip"
    poses[4].pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi, 0, -pi) )
    poses[4].pose.position.x = .03

    self.cycle_through_calibration_poses(poses, "b_bot", speed=0.3, go_home=True)
    return 
      
  def touch_workspace_center(self):
    rospy.loginfo("============ Touching workspace center. ============")
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    poses = []

    pose_a = geometry_msgs.msg.PoseStamped()
    pose_a.header.frame_id = "workspace_center"
    pose_a.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
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

  def place_screw_test(self, set_name = "set_1_", screw_size = 4, screw_number = 1):
    # TODO: Update this function for the 2020 scene
    rospy.loginfo("============ Placing a screw in a tray using c_bot ============")
    rospy.loginfo("============ Screw tool m4 and a screw have to be carried by the robot! ============")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("back", "b_bot")

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = set_name + "tray_2_screw_m" + str(screw_size) + "_" + str(screw_number)
    if set_name == "set_1_":
      self.go_to_named_pose("feeder_pick_ready", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    elif set_name == "set_2_":
      self.go_to_named_pose("feeder_pick_ready", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
    elif set_name == "set_3_":
      self.go_to_named_pose("screw_place_ready_near_b_bot", "c_bot")
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.do_place_action("c_bot", ps, tool_name="screw_tool", screw_size = 4)
    return
  
  def screw_tool_tests(self):
    # TODO: Update this function for the 2020 scene
    rospy.loginfo("============ Calibrating screw_tool M4 with b_bot. ============")
    self.go_to_named_pose("screw_ready", "b_bot")
    poses = []

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "workspace_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    ps.pose.position.x = .0
    ps.pose.position.y = .0
    ps.pose.position.z = .05

    rospy.loginfo("============ Press enter to hold tool vertically. ============")
    i = raw_input()

    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    ps.pose.position.x = -.01
    ps.pose.position.y = .0
    ps.pose.position.z = .05
    self.publish_marker(ps, "pose")
    self.groups["b_bot"].set_pose_target(ps, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.groups["b_bot"].set_max_velocity_scaling_factor(.05)
    self.groups["b_bot"].go()
    self.groups["b_bot"].stop()
    self.groups["b_bot"].clear_pose_targets()

    rospy.loginfo("============ Press enter to go home. ============")
    raw_input()
    self.go_to_named_pose("screw_ready", "b_bot")
    return

  def screw_holder_tests(self, robot_name="b_bot"):
    # TODO: Update this function for the 2020 scene
    rospy.loginfo("============ Going to screw tool holder with " + robot_name + ". ============")

    if robot_name == "b_bot":
      self.groups["b_bot"].set_joint_value_target([-30.0 * pi/180.0, -48 * pi/180.0, 96 * pi/180.0, 
                                      -50 * pi/180.0, -27 * pi/180.0, -180 * pi/180.0])
      self.groups["b_bot"].set_max_velocity_scaling_factor(.2)
      self.groups["b_bot"].go(wait=True)
      self.groups["b_bot"].stop()

    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "screw_tool_m4_helper_link"
    if robot_name == "b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      pose0.pose.position.x -= .03
      pose0.pose.position.z = .017
    
    for i in range(3):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "screw_tool_m3_helper_link"
    poses[2].header.frame_id = "screw_tool_m6_helper_link"

    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False)
    return

  def screw_tool_test_assembly(self, robot_name = "b_bot", tool_name="_screw_tool_m4_tip_link"):
    rospy.loginfo("============ Moving the screw tool m4 to the four corners of the base plate ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    if robot_name=="b_bot":
      self.go_to_named_pose("back", "c_bot")
    elif robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")

    self.go_to_named_pose("screw_ready", robot_name)
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "assembled_assy_part_01_corner_2"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
      pose0.pose.position.x -= .02
    
    for i in range(4):
      poses.append(copy.deepcopy(pose0))

    poses[1].header.frame_id = "assembled_assy_part_01_corner_3"
    poses[2].header.frame_id = "assembled_assy_part_01_corner_4"
    poses[3].header.frame_id = "assembled_assy_part_01_corner_1"
    end_effector_link=robot_name+ tool_name
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=end_effector_link)
    return
  
  def screw_pickup_test(self, robot_name = "b_bot"):
    rospy.loginfo("============ Picking up an m4 screw with the tool ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")

    self.go_to_named_pose("screw_ready", robot_name)
    if robot_name=="b_bot":
      self.go_to_named_pose("screw_pick_ready", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_screw_m4_1"
    if robot_name=="b_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi*11/12, 0, 0))
      pose0.pose.position.x = -.01

    self.publish_marker(pose0, "pose")
    print("published marker")
    return
    
    self.do_pick_action(robot_name, pose0, screw_size = 4, z_axis_rotation = 0.0, use_complex_planning = True, tool_name = "screw_tool")
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
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/4, 0, 0))
      pose0.pose.position.x = -.01
    
    self.do_screw_action(robot_name, pose0, screw_size = 4, screw_height = .02)
    self.go_to_named_pose("screw_plate_ready", robot_name)
    return

  def screw_feeder_calibration(self, robot_name = "c_bot"):
    rospy.loginfo("============ Moving the screw tool m4 to the screw feeder ============")
    rospy.loginfo("============ The screw tool m4 has to be carried by the robot! ============")
    if robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
      self.go_to_named_pose("back", "a_bot")

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    # Turn to the right
    self.groups["c_bot"].set_joint_value_target([0, -2.0980, 1.3992, -1.6153, -1.5712, -3.1401])
    self.groups["c_bot"].set_max_velocity_scaling_factor(1.0)
    self.groups["c_bot"].go(wait=True)
    self.groups["c_bot"].stop()

    poses = []
    pose0 = geometry_msgs.msg.PoseStamped()
    self.toggle_collisions(collisions_on=False)
    pose0.header.frame_id = "m3_feeder_outlet_link"
    if robot_name=="c_bot":
      pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      pose0.pose.position.x = -.03
    
    for i in range(6):
      poses.append(copy.deepcopy(pose0))

    poses[1].pose.position.x = 0

    poses[3].header.frame_id = "m4_feeder_outlet_link"
    poses[4].header.frame_id = "m4_feeder_outlet_link"
    poses[4].pose.position.x = 0
    poses[5].header.frame_id = "m4_feeder_outlet_link"
    
    self.cycle_through_calibration_poses(poses, robot_name, speed=0.3, go_home=False, move_lin=True, end_effector_link=robot_name + "_screw_tool_m4_tip_link")
    self.toggle_collisions(collisions_on=True)
    return
  
  def screw_feeder_pick_test(self, robot_name = "c_bot", screw_size = 4):
    rospy.loginfo("============ Picking a screw from a feeder ============")
    rospy.loginfo("============ The screw tool has to be carried by c_bot! ============")
    if robot_name=="c_bot":
      self.go_to_named_pose("back", "b_bot")
      self.go_to_named_pose("back", "a_bot")

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    # Turn to the right
    self.groups["c_bot"].set_joint_value_target([0, -2.0980, 1.3992, -1.6153, -1.5712, -3.1401])
    self.groups["c_bot"].set_max_velocity_scaling_factor(1.0)
    self.groups["c_bot"].go(wait=True)
    self.groups["c_bot"].stop()

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    if robot_name=="c_bot":
      ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.do_pick_action("c_bot", ps, screw_size=screw_size, tool_name="screw_tool")
    return

if __name__ == '__main__':
  try:
    c = CalibrationClass()

    while not rospy.is_shutdown():
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to check calibrations for the following things: ")
      rospy.loginfo("1: The robots (central position with nothing on the table)")
      rospy.loginfo("01: run ROS external control on b_bot")
      rospy.loginfo("1000: Move with different jerk values")
      rospy.loginfo("100 (1001): Go home with all robots (using UR script joint move)")
      rospy.loginfo("101 (1011): Go back with all robots (using UR script joint move)")
      rospy.loginfo("111: The robots (Using the assembly base plate)")
      rospy.loginfo("12: Touch the table (all bots)")
      rospy.loginfo("122-123: Level workspace = Go to different spots on table with b, a_bot")
      rospy.loginfo("318, 3181, 3182: Screws in set 1 with b_bot_suction_tool, b_bot, a_bot")
      rospy.loginfo("===== ASSEMBLY TASK (no parts may be mounted!)")
      rospy.loginfo("501-502: Assembly base plate (a_bot, b_bot)")
      rospy.loginfo("503-504: Assembly base plate (b_bot m4, b_bot m3)")
      rospy.loginfo("511-512: Motor plate holes (b_bot, b_bot m4)")
      rospy.loginfo("56: Go to tray screw positions with m4 tool for b_bot")
      rospy.loginfo("561, 562: Go to tray screw positions with b_bot, a_bot")
      rospy.loginfo("6: ===== TOOLS   Go to screw_ready with b (a goes to back)")
      rospy.loginfo("61: Go to screw holder with b_bot")
      rospy.loginfo("621, 622: Equip/unequip m4 screw tool with b_bot")
      rospy.loginfo("623, 624: Equip/unequip m3 screw tool with b_bot")
      rospy.loginfo("625, 626: Equip/unequip m6 screw tool with b_bot")
      rospy.loginfo("627, 628: Equip/unequip suction tool with b_bot")
      rospy.loginfo("63: Go to assembly base plate with m4 screw tool (b_bot)")
      rospy.loginfo("691: Do screw action with b_bot on rightmost hole")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.check_robot_calibration()
      elif r == '01':
        c.activate_ros_control_on_ur() 
      elif r == '1000':
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "workspace_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
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
      elif r == '12':
        c.touch_workspace_center()
      elif r == '122':
        c.check_level_calibration("b_bot")
      elif r == '123':
        c.check_level_calibration("a_bot")
      elif r == '318':
        c.tray_screw_calibration(robot_name="b_bot", end_effector_link="b_bot_suction_tool_tip_link", task="kitting", set_number=1)
      elif r == '3181':
        c.tray_screw_calibration(robot_name="b_bot", task="kitting", set_number=1)
      elif r == '3182':
        c.tray_screw_calibration(robot_name="a_bot", task="kitting", set_number=1)
      elif r == '501':
        c.assembly_calibration_base_plate("a_bot")
      elif r == '502':
        c.assembly_calibration_base_plate("b_bot")
      elif r == '503':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link")
      elif r == '504':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m3_tip_link")
      elif r == '5041':
        c.assembly_calibration_base_plate("b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", context="b_bot_m4_assembly_plates")
      elif r == '511':
        c.assembly_calibration_base_plate("b_bot", context="motor_plate")
      elif r == '512':
        c.assembly_calibration_base_plate("b_bot", end_effectodr_link="b_bot_screw_tool_m4_tip_link", context="motor_plate")
      elif r == '56':
        c.tray_screw_calibration(robot_name="b_bot", end_effector_link="b_bot_screw_tool_m4_tip_link", task="assembly")
      elif r == '561':
        c.tray_screw_calibration(robot_name="b_bot", task="assembly")
      elif r == '562':
        c.tray_screw_calibration(robot_name="a_bot", task="assembly")
      elif r == '6':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("screw_ready", "b_bot")
      elif r == '61':
        c.screw_holder_tests(robot_name="b_bot")
      elif r == '621':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("home", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      elif r == '622':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      elif r == '623':
        c.go_to_named_pose("back", "a_bot")
        c.go_to_named_pose("home", "b_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 3)
      elif r == '624':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 3)
      elif r == '625':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 6)
      elif r == '626':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 6)
      elif r == '627':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=True, screw_size = 50)
      elif r == '628':
        c.go_to_named_pose("back", "a_bot")
        c.do_change_tool_action("b_bot", equip=False, screw_size = 50)
      elif r == '63':
        c.screw_tool_test_assembly(robot_name="b_bot")
      elif r == '67':
        c.screw_pickup_test(robot_name="b_bot")
      elif r == '691':
        c.screw_action_test(robot_name="b_bot")
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
