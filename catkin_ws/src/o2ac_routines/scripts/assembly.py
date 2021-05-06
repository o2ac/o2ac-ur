#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Team o2ac
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
import moveit_msgs
import tf_conversions
import tf
from math import pi, radians, sin, cos
tau = 2.0*pi  # Part of math from Python 3.6
import math
import numpy as np

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.common import O2ACCommon
from o2ac_routines.helpers import wait_for_UR_program
import o2ac_routines.helpers as helpers

from ur_control.constants import TERMINATION_CRITERIA
class AssemblyClass(O2ACCommon):
  """
  This contains the routine used to run the assembly task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    
    # Initialize debug monitor
    if not self.assembly_database.db_name == "wrs_assembly_1":
      self.set_assembly("wrs_assembly_1")

    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

    # Spawn tools and objects
    self.define_tool_collision_objects()
    screw_ids = ['m3', 'm4']
    for screw_id in screw_ids:
      self.spawn_tool('screw_tool_' + screw_id)
      self.upload_tool_grasps_to_param_server(screw_id)

  def set_assembly(self, assembly_name="wrs_assembly_1"):
    self.assembly_database.change_assembly(assembly_name)
    return True


  ################ ----- Subroutines  

    # toolalign =
    # if not toolalign:
    #   rospy.logerr("Failed to load tool align program on b_bot")
    #   return False
    # print("Running belt pick on b_bot.")
    # if not self.b_bot.execute_loaded_program():
    #   rospy.logerr("Failed to execute tool align program on b_bot")
    #   return False
    # wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20)) # (using UR script)

  def subtask_zero(self):
    # ============= SUBTASK BASE (picking and orienting and placing the baseplate) =======================
    rospy.loginfo("======== SUBTASK BASE ========")
    self.unlock_base_plate()

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "workspace_center"
    pick_pose.pose.position = geometry_msgs.msg.Point(0.183, 0.13, 0.03)
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(tau/4, tau/4, 0))
    
    approach_pose = copy.deepcopy(pick_pose)
    approach_pose.pose.position.z += 0.15

    self.b_bot.gripper.open(wait=False)
    self.go_to_pose_goal("b_bot", approach_pose, speed=0.2, move_lin = True)
    self.go_to_pose_goal("b_bot", pick_pose, speed=0.2, move_lin = True)
    
    self.b_bot.gripper.close(force = 100)
    if self.b_bot.gripper.opening_width < 0.003:
      rospy.logerr("Gripper did not grasp the base plate. Aborting.")
      return False
    
    self.go_to_pose_goal("b_bot", approach_pose, speed=0.1, acceleration=0.1, move_lin = True)   

    approach_orient_pose = copy.deepcopy(approach_pose)
    approach_orient_pose.pose.position.x += -0.050
    approach_orient_pose.pose.position.y += +0.220 

    orient_pose = copy.deepcopy(approach_orient_pose)
    orient_pose.pose.position.z += -0.173

    self.go_to_pose_goal("b_bot", approach_orient_pose, speed=0.1, acceleration=0.1, move_lin = True)   
    self.go_to_pose_goal("b_bot", orient_pose, speed=0.1, acceleration=0.1, move_lin = True)   
    self.b_bot.gripper.open(wait=True)

    gripper_rotated = self.b_bot.get_current_pose_stamped()
    gripper_rotated.pose.orientation = helpers.rotateQuaternionByRPY(tau/4, 0, 0, orient_pose.pose.orientation)

    self.go_to_pose_goal("b_bot", gripper_rotated, speed=0.5, move_lin = True)
    self.b_bot.gripper.close(force = 100, wait=True)
    self.b_bot.gripper.open(wait=True)
    self.go_to_pose_goal("b_bot", orient_pose, speed=0.5, move_lin = True)
    self.b_bot.gripper.close(force = 100, wait=True)

    self.go_to_pose_goal("b_bot", approach_orient_pose, speed=0.2, move_lin = True)
    
    # Move plate
    place_onboard_pose = geometry_msgs.msg.PoseStamped()
    place_onboard_pose.header.frame_id = "workspace_center"
    place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.174, 0.013, 0.13)
    # place_onboard_pose.header.frame_id = "assy_01_terminal_top"
    # place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.02, 0.0, 0.0)
    place_onboard_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, 0))

    approach_onboard_pose = copy.deepcopy(place_onboard_pose)
    approach_onboard_pose.pose.position.z += 0.1

    self.go_to_pose_goal("b_bot", approach_onboard_pose, speed=0.2, move_lin = True)
    self.go_to_pose_goal("b_bot", place_onboard_pose, speed=0.2, move_lin = True)

    # FIXME: Direction should be -Z
    self.b_bot.linear_push(force=8, direction="+Z", relative_to_ee=False, timeout=15.0)
    
    self.b_bot.gripper.open()
    self.move_lin_rel("b_bot", relative_translation=[0, 0, 0.05], use_robot_base_csys=True)
    self.lock_base_plate()
    return True

  def subtask_a(self):
    # ============= SUBTASK A (picking and inserting and fastening the motor) =======================
    rospy.loginfo("======== SUBTASK A (motor) ========")
    rospy.logerr("Subtask A not implemented yet")
    return False

  def subtask_b(self):
    rospy.loginfo("======== SUBTASK B (motor pulley) ========")
    rospy.logerr("Subtask B not implemented yet")
    return

  def subtask_c1(self):
    rospy.loginfo("======== SUBTASK C (bearing) ========")
    rospy.logerr("Subtask C not implemented yet")
    if not self.pick_up_and_insert_bearing():
      return False
    return self.fasten_bearing()
  
  def pick_up_and_insert_bearing(self):
    self.go_to_named_pose("home","a_bot")
    self.go_to_named_pose("home","b_bot")

    goal = self.look_and_get_grasp_point("bearing")
    if not goal:
      rospy.logerr("Could not find bearing in tray. Skipping procedure.")
      return False
    self.vision.activate_camera("b_bot_inside_camera")
    goal.pose.position.x -= 0.01 # MAGIC NUMBER
    goal.pose.position.z = 0.0115
    self.simple_pick("b_bot", goal, gripper_force=100.0, approach_height=0.05, axis="z")

    if self.b_bot.gripper.opening_width < 0.01:
      rospy.logerr("Fail to grasp bearing")
      return
    elif self.b_bot.gripper.opening_width < 0.045:
      rospy.loginfo("bearing found to be upwards")
      self.playback_sequence("bearing_orient")
      # success_b = self.b_bot.load_program(program_name="wrs2020/bearing_orient_totb.urp")
    else:
      rospy.loginfo("bearing found to be upside down")
      self.playback_sequence("bearing_orient_down")
      # success_b = self.b_bot.load_program(program_name="wrs2020/bearing_orient_down_totb.urp")
      #'down' means the small area contacts with tray.

    if self.b_bot.gripper.opening_width < 0.01:
      rospy.logerr("Bearing not found in gripper. Must have been lost. Aborting.")
      #TODO(felixvd): Look at the regrasping/aligning area next to the tray
      return False

    self.playback_sequence("bearing_move_to_assembly")

    # Insert bearing
    if not self.insert_bearing():
      return

    self.bearing_holes_aligned = self.align_bearing_holes(task="assembly")
    return self.bearing_holes_aligned

  def insert_bearing(self):
    
    plane = "YZ"
    radius = 0.002
    radius_direction = "+Z"
    revolutions = 5

    steps = 100
    duration = 30.0
    
    target_force = helpers.get_target_force('-X', 5.0)
    selection_matrix = [0., 0.8, 0.8, 0.8, 0.8, 0.8]

    termination_criteria = lambda cpose: cpose[0] > 0.085

    rospy.logwarn("** STARTING FORCE CONTROL **")
    result = self.b_bot.execute_spiral_trajectory(plane, radius, radius_direction, steps, revolutions, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(4.0), wiggle_revolutions=10.0,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.logwarn("** FORCE CONTROL COMPLETE **")


    self.b_bot.gripper.open(wait=True)

    self.move_lin_rel("b_bot", relative_translation = [0.014,0,0], acceleration = 0.015, velocity = .03, use_robot_base_csys=True)

    self.close_gripper('b_bot', velocity=0.01, wait=True)

    termination_criteria = lambda cpose: cpose[0] > 0.085
    radius = 0.001

    rospy.logwarn("** STARTING FORCE CONTROL 2**")
    result = self.b_bot.execute_spiral_trajectory(plane, radius, radius_direction, steps, revolutions, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(4.0), wiggle_revolutions=10.0,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.logwarn("** FORCE CONTROL COMPLETE 2**")
    
    self.b_bot.gripper.open(wait=True)

    if result != TERMINATION_CRITERIA:
      rospy.logerr("** Insertion Failed!! **")
      return False

    # rospy.logwarn("** CHANGE POSITIONS USING MOVEIT **")
    # post_insertion_pose = [1.6088, -1.1894, 1.7653, -2.0387, -2.7843, -1.4562]
    # self.move_joints('b_bot', post_insertion_pose)
    return True

  def fasten_bearing(self, task="assembly"):
    self.go_to_named_pose("home", "a_bot")
    self.equip_tool('b_bot', 'screw_tool_m4')
    self.vision.activate_camera("b_bot_outside_camera")
    intermediate_screw_bearing_pose = [31.0 /180.0*3.14, -137.0 /180.0*3.14, 121.0 /180.0*3.14, -114.0 /180.0*3.14, -45.0 /180.0*3.14, -222.0 /180.0*3.14]
    
    def pick_and_fasten_bearing_screw(screw_number, pick_screw=True):
      """Returns tuple (screw_success, break_out_of_loop)"""
      # Pick screw
      if pick_screw:
        self.move_joints("b_bot", intermediate_screw_bearing_pose)
        self.go_to_named_pose("feeder_pick_ready","b_bot")
        pick_success = self.skill_server.pick_screw_from_feeder("b_bot", screw_size=4)
        if not pick_success:
          rospy.logerr("Could not pick screw. Why?? Breaking out.")
          self.allow_collisions_with_robot_hand
          self.unequip_tool('b_bot', 'screw_tool_m4')
          return (False, True)
      
      # Fasten screw
      self.move_joints("b_bot", intermediate_screw_bearing_pose)
      self.go_to_named_pose("horizontal_screw_ready","b_bot")
      screw_pose = geometry_msgs.msg.PoseStamped()
      screw_pose.header.frame_id = "/assembled_assy_part_07_screw_hole_" + str(n)
      screw_pose.pose.position.x = 0.01 ## WHY? NO IDEA
      screw_pose.pose.position.z = 0.00  ## MAGIC NUMBER
      screw_pose.pose.orientation.w = 1.0
      if task == "assembly":
        screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(tau/2, 0, 0))
      screw_pose_approach = copy.deepcopy(screw_pose)
      screw_pose_approach.pose.position.x -= 0.05
      
      self.go_to_pose_goal("b_bot", screw_pose_approach, end_effector_link = "b_bot_screw_tool_m4_tip_link", move_lin=False)
      screw_success = self.skill_server.do_screw_action("b_bot", screw_pose, screw_size=4)
      self.go_to_pose_goal("b_bot", screw_pose_approach, end_effector_link = "b_bot_screw_tool_m4_tip_link", move_lin=False)
      self.move_joints("b_bot", intermediate_screw_bearing_pose)
      # self.go_to_named_pose("home","b_bot")
      return (screw_success, False)

    screw_status = dict()
    for n in [1,3,2,4]:  # Cross pattern
      screw_status[n] = "empty"
      if rospy.is_shutdown():
        break
      (screw_success, breakout) = pick_and_fasten_bearing_screw(n)
      if breakout:
        break
      if screw_success:
        screw_status[n] = "done"
      if not screw_success and self.screw_is_suctioned["m4"]:
        screw_status[n] = "empty"
      if not screw_success and not self.screw_is_suctioned["m4"]:
        screw_status[n] = "maybe_stuck_in_hole"
      rospy.loginfo("Screw " + str(n) + " detected as " + screw_status[n])
    
    # Reattempt 
    all_screws_done = all(value == "done" for value in screw_status.values())
    while not all_screws_done and not rospy.is_shutdown():
      for n in [1,2,3,4]:
        if rospy.is_shutdown():
          break
        if screw_status[n] == "empty":
          (screw_success, breakout) = pick_and_fasten_bearing_screw(n)
        elif screw_status[n] == "maybe_stuck_in_hole":
          (screw_success, breakout) = pick_and_fasten_bearing_screw(n, pick_screw=False)
        if screw_success:
          screw_status[n] = "done"
        if not screw_success and self.screw_is_suctioned["m4"]:
          screw_status[n] = "empty"
        if not screw_success and not self.screw_is_suctioned["m4"]:
          screw_status[n] = "maybe_stuck_in_hole"
        rospy.loginfo("Screw " + str(n) + " detected as " + screw_status[n])
      all_screws_done = all(value == "done" for value in screw_status.values())

    self.move_joints("b_bot", intermediate_screw_bearing_pose)
    self.go_to_named_pose("tool_pick_ready","b_bot")
    self.unequip_tool('b_bot', 'screw_tool_m4')
  
  def subtask_c2(self):
    rospy.loginfo("======== SUBTASK C (output shaft) ========")
    rospy.logerr("Subtask C not implemented yet")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest)
    return False

  def subtask_d(self):
    # Fasten large pulley to output shaft
    rospy.loginfo("======== SUBTASK D (output pulley) ========")
    rospy.logerr("Subtask D not implemented yet")
    return False

  def subtask_e(self):
    rospy.loginfo("======== SUBTASK E (idler pulley) ========")
    rospy.logerr("Subtask E not implemented yet")

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return True

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (motor panel (small L-plate)) ========")
    # goal = self.look_and_get_grasp_point(2)  # bearing panel
    # if not goal:
    #     rospy.logerr("Could not find bearing plate in tray. Breaking out.")
    #     return False
    self.panel_subtask(panel="motor_panel")
    return False

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (bearing panel (large L-plate)) ========")
    self.panel_subtask(panel="bearing_panel")

  def panel_subtask(self, panel):
    """
    input parameter panel needs to be "motor_panel" or "bearing_panel"
    """

    if not self.go_to_named_pose("feeder_pick_ready", "b_bot"):
      rospy.logerr("b_bot did not move out of the way. Aborting.")
      return False

    # TODO: Use vision here instead
    if panel == "bearing_panel":
      a_bot_start_hardcoded_bearing_plate = [1.014740824, -1.597331663, 1.923653427, -1.910340925, -1.59755593, 1.014281272]
      self.move_joints("a_bot", a_bot_start_hardcoded_bearing_plate)
      success_a = self.a_bot.load_program(program_name="wrs2020/bearing_plate_full.urp", recursion_depth=3)
    elif panel == "motor_panel":
      a_bot_start_hardcoded_motor_plate = [1.093529105, -1.416419939, 1.730439011, -1.895824571, -1.598531071, 1.09309482]
      self.move_joints("a_bot", a_bot_start_hardcoded_motor_plate)
      success_a = self.a_bot.load_program(program_name="wrs2020/motor_plate_full.urp", recursion_depth=3)
    
    if not success_a:
      rospy.logerr("Failed to load plate placing program on a_bot")
      return False
    
    if not self.a_bot.execute_loaded_program():
      rospy.logerr("Failed to execute plate placing program on a_bot")
      return False
    rospy.loginfo("Running bearing plate rearrangement on a_bot.")
    wait_for_UR_program("/a_bot", rospy.Duration.from_sec(40))

    self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
    if not self.skill_server.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.go_to_named_pose("home", "a_bot")
      return False

    if panel == "bearing_panel":
      part_name = "assembled_assy_part_03_"
    elif panel == "motor_panel":
      part_name = "assembled_assy_part_02_"

    screw_target_pose = geometry_msgs.msg.PoseStamped()
    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_1"
    screw_target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(radians(-20), 0, 0))
    if not self.fasten_screw_vertical('b_bot', screw_target_pose):
      # Fallback for screw 1
      rospy.logerr("Failed to fasten panel screw 1, trying to realign tool and retry.")
      self.realign_tool("b_bot", "screw_tool_m4")
      self.go_to_named_pose("feeder_pick_ready", "b_bot")
      self.pick_screw_from_feeder("b_bot", screw_size = 4)

      # Realign plate
      self.a_bot.gripper.close(force = 100)
      self.move_lin_rel("a_bot", relative_translation=[0, -0.015, 0])
      self.a_bot.gripper.open(opening_width=0.08, wait=True)
      if panel == "bearing_panel":
        success_a = self.a_bot.load_program(program_name="wrs2020/bearing_plate_positioning.urp", recursion_depth=3)
      else:
        success_a = self.a_bot.load_program(program_name="wrs2020/motor_plate_positioning.urp", recursion_depth=3)
      if not success_a:
        rospy.logerr("Failed to load plate positioning program on a_bot")
        return False
      if not self.a_bot.execute_loaded_program():
        rospy.logerr("Failed to execute plate positioning program on a_bot")
        return False
      wait_for_UR_program("/a_bot", rospy.Duration.from_sec(20))
      
      # Retry fastening
      if not self.fasten_screw_vertical('b_bot', screw_target_pose):
        rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
        return False

    self.a_bot.gripper.close()
    self.a_bot.gripper.open()
    if not self.go_to_named_pose("home", "a_bot"):
      rospy.logerr("Failed to move a_bot home!")
      return False

    if not self.skill_server.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick second screw, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.go_to_named_pose("home", "a_bot")
      return False

    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_2"
    if not self.fasten_screw_vertical('b_bot', screw_target_pose):
      # Fallback for screw 2: Realign tool, recenter plate, try again
      rospy.logerr("Failed to fasten panel screw 2, trying to realign tool and retrying.")
      self.realign_tool("b_bot", "screw_tool_m4")
      self.go_to_named_pose("feeder_pick_ready", "b_bot")
      self.pick_screw_from_feeder("b_bot", screw_size = 4)

      # Recenter plate
      center_plate_pose = geometry_msgs.msg.PoseStamped()
      if panel == "bearing_panel":
        center_plate_pose.header.frame_id = part_name + "pulley_ridge_middle"
      else:  # motor panel
        center_plate_pose.header.frame_id = part_name + "motor_screw_hole_5"
      center_plate_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(60), -tau/4))
      center_plate_pose.pose.position.x = 0.0025
      self.a_bot.gripper.open(opening_width=0.08, wait=False)
      self.go_to_pose_goal("a_bot", center_plate_pose, move_lin=False)
      self.a_bot.gripper.close(force = 100)
      self.a_bot.gripper.open()
      if not self.go_to_named_pose("home", "a_bot"):
        rospy.logerr("Failed to move a_bot home!")
        return False
      if not self.fasten_screw_vertical('b_bot', screw_target_pose):
        rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
        return False
    return True

  def subtask_h(self):
    # Attach belt
    rospy.loginfo("======== SUBTASK H (belt) ========")
    rospy.logerr("Subtask H not implemented yet")
    return False

  def subtask_i(self):
    # Insert motor cables
    rospy.loginfo("======== SUBTASK I (cables) ========")
    rospy.logerr("Subtask I not implemented yet")
    return False

  def spawn_objects_for_closed_loop_test(self):
    objects = ['panel_bearing', 'panel_motor']
    poses = [[0.4, -0.35, 0.8, tau/4, 0, tau/4], [0.5, 0.3, 1, tau/4, 0, 0]]
    self.spawn_multiple_objects('wrs_assembly_1', ['base'], [[0.12, 0.2, 0.0, tau/4, 0.0, -tau/4]], 'attached_base_origin_link')
    self.spawn_multiple_objects('wrs_assembly_1', objects, poses, 'world')

  def spawn_objects_for_demo(self):
    objects = ['panel_motor', 'panel_bearing', 'motor', 'motor_pulley', 'bearing_housing',
      'shaft', 'end_cap', 'bearing_spacer', 'output_pulley', 'idler_spacer', 'idler_pulley', 'idler_pin']  # , 'base']
    poses = [[0.12, 0.02, 0.001, 0.0, 0.0, tau/2],
      [0.02, -0.06, 0.001, 0.0, 0.0, -tau/4],
      [-0.09, -0.12, 0.001, tau/4, -tau/4, 0.0],
      [-0.02, -0.16, 0.005, 0.0, -tau/4, 0.0],
      [0.0, 0.0, 0.001, 0.0, tau/4, 0.0],
      [-0.04, 0.0, 0.005, 0.0, 0.0, -tau/2],
      [-0.1, -0.06, 0.001, 0.0, -tau/4, 0.0],
      [-0.07, -0.06, 0.001, 0.0, -tau/4, 0.0],
      [-0.02, -0.08, 0.005, 0.0, -tau/4, 0.0],
      [-0.04, -0.03, 0.001, 0.0, -tau/4, 0.0],
      [-0.05, -0.13, 0.001, 0.0, -tau/4, 0.0],
      [-0.1, -0.03, 0.005, 0.0, 0.0, 0.0]]  # , [-0.1, 0.16, 0.001, tau/4, 0.0, 0.0]]
    self.spawn_multiple_objects('wrs_assembly_1', ['base'], [[0.12, 0.2, 0.0, tau/4, 0.0, -tau/4]], 'attached_base_origin_link')
    self.spawn_multiple_objects('wrs_assembly_1', objects, poses, 'tray_center')

  def mtc_pick_screw_tool(self, screw_type):
    rospy.loginfo("======== PICK TASK ========")
    success = False
    if screw_type in ['m3', 'm4']:
      return self.do_plan_pick_action('screw_tool_' + screw_type, 'tools', 'screw_tool_m3_pickup_link', [-1.0, 0.0, 0.0], save_solution_to_file = 'pick_screw_tool')

  def mtc_suck_screw(self, screw_type):
    rospy.loginfo("======== FASTEN TASK ========")
    success = False
    tool = 'screw_tool_' + screw_type
    screw_tool_tip_frame = tool + '/' + tool + '_tip'
    screw_pickup_pose = geometry_msgs.msg.PoseStamped()
    screw_pickup_pose.header.frame_id = screw_type + '_feeder_outlet_link'
    screw_pickup_pose.pose.position.x = -0.01
    screw_pickup_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(2*pi/3, 0, 0))
    if screw_type in ['m3', 'm4']:
      return self.do_plan_fastening_action('screw_tool_' + screw_type, screw_pickup_pose, object_subframe_to_place = screw_tool_tip_frame, save_solution_to_file = 'pick_screw')

  def mtc_place_object_in_tray_center(self, object_name):
    rospy.loginfo("======== PLACE TASK ========")
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = 'tray_center'
    target_pose.pose.position.x = -0.04
    target_pose.pose.position.y = 0.08
    target_pose.pose.orientation.w = 1
    self.do_plan_place_action(object_name, target_pose, save_solution_to_file = 'place_' + object_name)

  def mtc_pickplace_l_panel(self):
    rospy.loginfo("======== PICKPLACE TASK ========")

    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = 'base/screw_hole_panel2_1'
    target_pose.pose.orientation.w = 1

    self.do_plan_pickplace_action('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', robot_names = ['b_bot','a_bot'], force_robot_order = True, save_solution_to_file = 'pickplace')

  def mtc_pick_place_task(self):
    rospy.loginfo("======== PICK-PLACE TASK ========")
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
    pose.pose.orientation.w = 1
    return self.do_plan_pickplace_action('b_bot', 'panel_bearing', pose, save_solution_to_file = 'panel_bearing/bottom_screw_hole_aligner_1')

  def real_assembly_task(self):
    self.start_task_timer()

    # Look into the tray
    self.look_and_get_grasp_point(2)
    self.confirm_to_proceed("press enter to proceed to pick and set base plate")
    self.subtask_zero()  # Base plate
    
    ## Equip screw tool for subtasks G, F
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    self.go_to_named_pose("feeder_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.vision.activate_camera("b_bot_outside_camera")

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    self.subtask_g()  # Large plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    self.subtask_f() # motor plate

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=False, screw_size=4)

    rospy.loginfo("==== Finished.")

    self.subtask_c1() # bearing 
    # # self.subtask_c2() # shaft

    # # To prepare subtask E
    # self.pick_retainer_pin_from_tray_and_place_in_holder(do_centering=False)
    # self.confirm_to_proceed("press enter to proceed")
    # self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # assy.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=assy.use_real_robot)
    # assy.do_change_tool_action("b_bot", equip=False, screw_size=4)

    # self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # self.subtask_e(pick_from_holder=True) #idle pulley

    # self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # self.subtask_a() # motor
    
    # self.subtask_c() # bearing, clamping pulley set
    return

if __name__ == '__main__':
  try:
    rospy.init_node('o2ac_routines', anonymous=False)
    rospy.loginfo("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    assy = AssemblyClass()
    while True:
      rospy.loginfo("Enter 1 to move the robots home.")
      rospy.loginfo("Enter 11 (12) to equip (unequip) m4 tool (b_bot).")
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (b_bot).")
      rospy.loginfo("Enter 25,26(27,28) to open,close gripper for b_bot (a_bot)")
      rospy.loginfo("Enter 30 to pick screw m3 from feeder with a_bot (31 for b_bot).")
      rospy.loginfo("Enter 40 to pick screw m4 from feeder with a_bot (41 for b_bot).")
      rospy.loginfo("Enter 68 to spawn objects for testing mtc_modules tasks")
      rospy.loginfo("Enter 69-75 to test mtc_modules tasks, (pick, place, pik-place, pick tool, pick screw, release, fix L plate on base)")
      rospy.loginfo("Enter 80 to execute the planned subassembly (fix L plate on base)")
      rospy.loginfo("Enter 90 for base plate (b_bot).")
      rospy.loginfo("Enter 90-94 for subtasks (90: Base plate, 91: large plate, 92: motor plate, 93: bearing, 94: motor).")
      rospy.loginfo("Enter 95-99 for subtasks (95: motor pulley, 96: idler pin, 97: shaft, 98: clamp pulley, 99: belt).")
      rospy.loginfo("Enter reset to reset the scene")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        assy.go_to_named_pose("home", "a_bot")
        assy.go_to_named_pose("home", "b_bot")
      if i == '11':
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if i == '12':
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if i == '13':
        assy.do_change_tool_action("b_bot", equip=True, screw_size=3)
      if i == '14':
        assy.do_change_tool_action("b_bot", equip=False, screw_size=3)
      if i == '25':
        assy.b_bot.gripper.open()
      if i == '26':
        assy.b_bot.gripper.close()
      if i == '27':
        assy.a_bot.gripper.open()
      if i == '28':
        assy.a_bot.gripper.close()
      if i == '30':
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
        assy.skill_server.pick_screw_from_feeder("a_bot", screw_size=3)
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
      if i == '31':
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
        assy.skill_server.pick_screw_from_feeder("b_bot", screw_size=3)
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
      if i == '40':
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
        assy.skill_server.pick_screw_from_feeder("a_bot", screw_size=4)
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
      if i == '41':
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
        assy.skill_server.pick_screw_from_feeder("b_bot", screw_size=4)
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
      if i == '67':
        assy.spawn_objects_for_closed_loop_test()
      if i == '68':
        assy.spawn_objects_for_demo()
      if i == '69':
        result = assy.do_plan_pick_action('panel_bearing', robot_name = '', save_solution_to_file = 'pick_panel_bearing')
        for solution in result.solution.sub_trajectory:
          scene_diff = solution.scene_diff
          planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
          planning_scene_diff_req.scene = scene_diff
          # self.apply_planning_scene_diff.call(planning_scene_diff_req)   # DEBUG: Update the scene pretending the action has been completed
      if i == '691':
        rospy.loginfo("Loading 'pick_panel_bearing'")
        mp_res = assy.load_MTC_solution('pick_panel_bearing')
        rospy.loginfo("Running")
        assy.execute_MTC_solution(mp_res.solution, speed = 0.2)
      if i == '70':
        assy.mtc_place_object_in_tray_center('panel_bearing')
      if i == '71':
        assy.mtc_pickplace_l_panel()
      if i == '72':
        assy.mtc_pick_screw_tool('m4')
      if i == '73':
        assy.mtc_suck_screw('m4')
      if i == '74':
        assy.do_plan_release_action('panel_bearing', 'home', save_solution_to_file = 'release_panel_bearing')
      if i == '75':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        assy.do_plan_wrs_subtask_b_action('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', save_solution_to_file = 'subassembly')
      if i == '80':
        rospy.loginfo("Loading")
        mp_res = assy.load_MTC_solution('subassembly')
        rospy.loginfo("Running")
        assy.execute_MTC_solution(mp_res.solution, speed = 0.2)
      if i == '800':
        assy.b_bot.load_and_execute_program(program_name="wrs2020_push_motor_plate.urp", wait=True)
      if i == '801':
        assy.skill_server.move_lin_rel("a_bot", relative_translation=[0, -0.01, 0], use_robot_base_csys=True, max_wait=5.0)
      if i == '802':
        assy.skill_server.move_lin_rel("a_bot", relative_translation=[0,  0.01, 0], use_robot_base_csys=True, max_wait=5.0)
      if i == '81':
        assy.do_change_tool_action('b_bot', equip=True, screw_size=4)
      if i == '82':
        assy.do_change_tool_action('b_bot', equip=False, screw_size=4)
      if i == '83':
        assy.skill_server.pick_screw_from_feeder('b_bot', 4)
      if i == '84':
        assy.skill_server.do_linear_push('a_bot', force=15, direction="Y-", max_approach_distance=0.05, forward_speed=0.003)
      if i == '85':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        assy.fasten_screw('b_bot', target_pose)
      elif i == '90':
        assy.subtask_zero() # Base plate
      elif i == '91':
        assy.subtask_g()  # Large plate
      elif i == '92':
        assy.subtask_f()  # Motor plate
      elif i == '93':
        assy.subtask_c1() # bearing
      elif i == '931':
        assy.pick_up_and_insert_bearing()
      elif i == '9311':
        assy.align_bearing_holes(task="assembly")
      elif i == '932':
        assy.fasten_bearing()
      elif i == '933':
        assy.insert_bearing()
      elif i == '94':
        assy.subtask_a() # Motor
      elif i == '95':
        assy.subtask_b() # motor pulley
      elif i == '96':
        assy.subtask_e() # idler pin
        # 97: shaft, 98: clamp pulley, 99: belt).")
      elif i == "899":
        center_plate_pose = geometry_msgs.msg.PoseStamped()
        center_plate_pose.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
        center_plate_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(60), -tau/4))
        assy.a_bot.gripper.open(opening_width=0.07, wait=False)
        assy.go_to_pose_goal("a_bot", center_plate_pose, move_lin=False, speed=1.0)
        assy.a_bot.gripper.close(force = 100)
        assy.a_bot.gripper.open()
      elif i == "898":
        assy.go_to_named_pose("home", "a_bot", force_ur_script=False)
      elif i == '100': # Test Force control be careful!!
        b_bot_starting_position = [1.7078, -1.5267, 2.0624, -2.1325, -1.6114, 1.7185]
        assy.move_joints("b_bot", b_bot_starting_position)
        
        timeout = 20.0

        selection_matrix = [1.,1.,0.,1.,1.,1.]
        target_force =     [0.,0.,0.,0.,0.,0.]

        assy.activate_ros_control_on_ur(robot="b_bot", recursion_depth=1)
        rospy.logwarn("STARTING Force Control with target_force: %s %s %s" % (str(target_force), "timeout", str(timeout)))
        assy.b_bot.force_control(target_force=target_force, selection_matrix=selection_matrix, timeout=timeout, stop_on_target_force=True)
        rospy.logwarn("FINISHED Force Control")
      elif i == '101': # Test Force control be careful!!
        force = 10.0 #N
        direction = '-Z'

        assy.b_bot.linear_push(force=force, direction=direction, timeout=20.0)
        
      elif i == '102':
        b_bot_script_start_pose = [1.7094888, -1.76184906, 2.20651847, -2.03368343, -1.54728252, 0.96213197]
        assy.move_joints("b_bot", b_bot_script_start_pose)
        
        # force = 1.0 #N
        # direction = '-X'

        # assy.b_bot.linear_push(initial_pose=b_bot_starting_position, force=force, direction=direction, timeout=20.0)
      elif i == 'START' or i == 'start' or i == "9999":
        for i in [1,2]:
          rospy.loginfo("Starting set number " + str(i))
          assy.competition_mode = True
          assy.real_assembly_task()
          assy.competition_mode = False
          rospy.loginfo("SET NUMBER " + str(i) + " COMPLETED. PUT THE ROBOT IN PAUSE MODE AND REPLACE THE PARTS")
          raw_input()
          if rospy.is_shutdown():
            rospy.loginfo("ABORTING")
            break
      elif i == "new":
        rospy.loginfo("SET NUMBER " + str(i) + " COMPLETED. PUT THE ROBOT IN PAUSE MODE AND REPLACE THE PARTS")
        raw_input()
        if rospy.is_shutdown():
          rospy.loginfo("ABORTING")
          break
        rospy.loginfo("Starting new set")
        assy.real_assembly_task()
      if i == "reset":
        assy.reset_scene_and_robots()
      if i == "activate":
        assy.activate_ros_control_on_ur()
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
