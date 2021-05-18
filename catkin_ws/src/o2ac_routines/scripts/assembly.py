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
import moveit_msgs.msg
import moveit_task_constructor_msgs.msg

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
    if not self.assembly_database.db_name == "wrs_assembly_2020":
      self.set_assembly("wrs_assembly_2020")

    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

    # Spawn tools and objects
    self.define_tool_collision_objects()
    screw_ids = ['m3', 'm4']
    for screw_id in screw_ids:
      self.spawn_tool('screw_tool_' + screw_id)
      self.upload_tool_grasps_to_param_server(screw_id)

  def set_assembly(self, assembly_name="wrs_assembly_2020"):
    self.assembly_database.change_assembly(assembly_name)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'attached_base_origin_link'
    # quaternion = tf.transformations.quaternion_about_axis(pi/2, (1,0,0))
    # pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
    pose.pose.orientation.w = 1.0
    # pose.pose.position = 
    self.assembly_database.publish_assembly_frames(pose, prefix="assembled_")
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

    success = self.pick("b_bot", "base")
    if not success or self.b_bot.gripper.opening_width < 0.003:
      rospy.logerr("Gripper did not grasp the base plate. Aborting.")
      return False

    self.confirm_to_proceed("Has the base plate been picked?")
    self.planning_scene_interface.allow_collisions("base", "")  # Allow collisions with all other objects

    # Center and reorient plate outside of tray

    approach_orient_pose = geometry_msgs.msg.PoseStamped()
    approach_orient_pose.header.frame_id = "workspace_center"
    approach_orient_pose.pose.position = geometry_msgs.msg.Point(0.133, 0.35, 0.18)
    approach_orient_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(tau/4, tau/4, 0))

    orient_pose = copy.deepcopy(approach_orient_pose)
    orient_pose.pose.position.z = 0.01

    self.b_bot.go_to_pose_goal(approach_orient_pose, speed=0.1, acceleration=0.2, move_lin = True)
    self.b_bot.go_to_pose_goal(orient_pose, speed=0.1, acceleration=0.1, move_lin = True)
    self.b_bot.gripper.open(wait=True)

    self.b_bot.robot_group.detach_object("base")
    rospy.loginfo("tic")
    self.allow_collisions_with_robot_hand("base", "b_bot")
    rospy.loginfo("toc")

    self.b_bot.move_lin_rel(relative_rotation=[0,0,tau/4], speed=1.0, acceleration=0.5)

    self.b_bot.gripper.close(force = 100, wait=True)
    self.b_bot.gripper.open(wait=True)
    self.b_bot.go_to_pose_goal(orient_pose, speed=1.0, acceleration=0.5)
    self.b_bot.gripper.close(force = 100, wait=True)

    # TODO: Center the object
    self.b_bot.robot_group.attach_object("base")

    self.b_bot.go_to_pose_goal(approach_orient_pose, speed=0.2)
    
    # Move plate
    place_onboard_pose = geometry_msgs.msg.PoseStamped()
    place_onboard_pose.header.frame_id = "workspace_center"
    place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.175, 0.009, 0.13)
    # TODO: Define the terminal subframes
    # place_onboard_pose.header.frame_id = "assy_01_terminal_top"
    # place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.01, -0.005, 0.0)
    place_onboard_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, 0))

    approach_onboard_pose = copy.deepcopy(place_onboard_pose)
    approach_onboard_pose.pose.position.z += 0.08

    self.b_bot.go_to_pose_goal(approach_onboard_pose, speed=0.3, acceleration=0.1)
    self.b_bot.go_to_pose_goal(place_onboard_pose, speed=0.2)

    # FIXME: Direction should be -Z
    self.b_bot.linear_push(force=8, direction="+Z", relative_to_ee=False, timeout=15.0)
    
    self.b_bot.gripper.open(wait=False)
    
    self.publish_part_in_assembled_position("base")

    self.b_bot.move_lin_rel(relative_translation=[0, 0, 0.05], relative_to_robot_base=True)
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
    if self.pick_up_and_insert_bearing(task="assembly"):
      if self.fasten_bearing(task="assembly"):
        self.unequip_tool('b_bot', 'screw_tool_m4')
        return True
    return False
  
  def subtask_c2(self):
    rospy.loginfo("======== SUBTASK C (output shaft) ========")
    rospy.logerr("Subtask C not implemented yet")
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    return False

  def subtask_d(self):
    # Fasten large pulley to output shaft
    rospy.loginfo("======== SUBTASK D (output pulley) ========")
    rospy.logerr("Subtask D not implemented yet")
    return False

  def subtask_e(self):
    rospy.loginfo("======== SUBTASK E (idler pulley) ========")
    rospy.logerr("Subtask E not implemented yet")

    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return True

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (motor panel (small L-plate)) ========")
    # goal = self.look_and_get_grasp_point(2)  # bearing panel
    # if not goal:
    #     rospy.logerr("Could not find bearing plate in tray. Breaking out.")
    #     return False
    self.panel_subtask(panel="panel_motor")
    return False

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (bearing panel (large L-plate)) ========")
    self.panel_subtask(panel="panel_bearing")

  def panel_subtask(self, panel):
    """
    input parameter panel needs to be "panel_motor" or "panel_bearing"
    """

    if not self.b_bot.go_to_named_pose("feeder_pick_ready"):
      rospy.logerr("b_bot did not move out of the way. Aborting.")
      return False

    if not self.pick("a_bot", panel):
      rospy.logerr("Did not succeed picking " + panel + "!")
      return False

    if panel == "panel_bearing":
      success_a = self.a_bot.load_program(program_name="wrs2020/bearing_plate_full.urp", recursion_depth=3)
    elif panel == "panel_motor":
      success_a = self.a_bot.load_program(program_name="wrs2020/motor_plate_full.urp", recursion_depth=3)
    
    if not success_a:
      rospy.logerr("Failed to load plate placing program on a_bot")
      return False
    
    if not self.a_bot.execute_loaded_program():
      rospy.logerr("Failed to execute plate placing program on a_bot")
      return False
    rospy.loginfo("Running bearing plate rearrangement on a_bot.")
    wait_for_UR_program("/a_bot", rospy.Duration.from_sec(40))

    self.publish_part_in_assembled_position(panel)

    self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
    if not self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.a_bot.go_to_named_pose("home")
      return False

    if panel == "panel_bearing":
      part_name = "assembled_part_03_"
    elif panel == "panel_motor":
      part_name = "assembled_part_02_"

    screw_target_pose = geometry_msgs.msg.PoseStamped()
    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_1"
    screw_target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(radians(-20), 0, 0))
    rospy.loginfo("tic (screw_tool_m4 collisions setting)")
    self.planning_scene_interface.allow_collisions("screw_tool_m4", panel)  # Allow collisions with all other objects
    rospy.loginfo("toc (screw_tool_m4 collisions setting)")
    if not self.fasten_screw_vertical('b_bot', screw_target_pose):
      # Fallback for screw 1
      rospy.logerr("Failed to fasten panel screw 1, trying to realign tool and retry.")
      self.realign_tool("b_bot", "screw_tool_m4")
      self.b_bot.go_to_named_pose("feeder_pick_ready")
      self.pick_screw_from_feeder("b_bot", screw_size = 4)

      # Realign plate
      self.a_bot.gripper.close(force = 100)
      self.a_bot.move_lin_rel(relative_translation=[0, -0.015, 0])
      self.a_bot.gripper.open(opening_width=0.08, wait=True)
      if panel == "panel_bearing":
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
    rospy.loginfo("Successfully fastened screw 1")

    self.a_bot.gripper.close()
    self.a_bot.gripper.open()
    if not self.a_bot.go_to_named_pose("home"):
      rospy.logerr("Failed to move a_bot home!")
      return False

    if not self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick second screw, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.a_bot.go_to_named_pose("home")
      return False

    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_2"
    if not self.fasten_screw_vertical('b_bot', screw_target_pose):
      # Fallback for screw 2: Realign tool, recenter plate, try again
      rospy.logerr("Failed to fasten panel screw 2, trying to realign tool and retrying.")
      self.realign_tool("b_bot", "screw_tool_m4")
      self.b_bot.go_to_named_pose("feeder_pick_ready")
      self.pick_screw_from_feeder("b_bot", screw_size = 4)

      # Recenter plate
      center_plate_pose = geometry_msgs.msg.PoseStamped()
      if panel == "panel_bearing":
        center_plate_pose.header.frame_id = part_name + "pulley_ridge_middle"
      else:  # motor panel
        center_plate_pose.header.frame_id = part_name + "motor_screw_hole_5"
      center_plate_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(60), -tau/4))
      center_plate_pose.pose.position.x = 0.0025
      self.a_bot.gripper.open(opening_width=0.08, wait=False)
      self.a_bot.go_to_pose_goal(center_plate_pose, move_lin=False)
      self.a_bot.gripper.close(force = 100)
      self.a_bot.gripper.open()
      if not self.a_bot.go_to_named_pose("home"):
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
    self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[0.12, 0.2, 0.0, tau/4, 0.0, -tau/4]], 'attached_base_origin_link')
    self.spawn_multiple_objects('wrs_assembly_2020', objects, poses, 'world')

  def spawn_objects_for_demo(self, base_plate_in_tray=False, layout_number=1):
    objects = ['panel_motor', 'panel_bearing', 'motor', 'motor_pulley', 'bearing',
      'shaft', 'end_cap', 'bearing_spacer', 'output_pulley', 'idler_spacer', 'idler_pulley', 'idler_pin']
    if layout_number == 1:
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
              [-0.1, -0.03, 0.005, 0.0, 0.0, 0.0]]
    elif layout_number == 2:
      poses = [[-0.04, 0.01, 0.001, 0.0, 0.0, tau/2],
              [0.01, -0.08, 0.001, 0.0, 0.0, tau/2],
              [0.1, -0.13, 0.001, tau/4, -tau/4, 0.0],
              [0.05, -0.07, 0.011, 0.0, -tau/4, 0.0],
              [0.06, 0, 0.001, 0.0, tau/4, 0.0],
              [0.04, 0.03, 0.011, 0.0, 0.0, -tau/2],
              [0.11, -0.06, 0.001, 0.0, -tau/4, 0.0],
              [0.08, -0.06, 0.001, 0.0, -tau/4, 0.0],
              [0, -0.03, 0.011, 0.0, -tau/4, 0.0],
              [0.1, -0.03, 0.001, 0.0, -tau/4, 0.0],
              [0.05, -0.13, 0.001, 0.0, -tau/4, 0.0],
              [0.04, -0.17, 0.011, 0.0, 0.0, 0.0]]
    elif layout_number == 3:
      poses = [[0.000, 0.020, 0.001, 0.000, -0.000, 3.140000], 
              [0.100, -0.040, 0.001, 0.000, -0.000, 1.580000], 
              [-0.060, 0.060, 0.001, 2.218, -1.568, 0.932410], 
              [-0.040, 0.160, 0.011, -3.141, -1.570, -3.141592], 
              [0.080, 0.130, 0.001, 2.756, 1.570, -2.756991], 
              [-0.090, -0.040, 0.011, 0.000, -0.000, 1.570000], 
              [-0.060, 0.120, 0.001, -3.141, -1.570, -3.141592], 
              [-0.040, 0.100, 0.001, -3.141, -1.570, -3.141592], 
              [0.000, 0.120, 0.011, 2.366, -1.569, 2.366991], 
              [-0.090, 0.100, 0.001, -3.141, -1.570, -3.141592], 
              [-0.080, 0.150, 0.001, -3.141, -1.570, -3.141592], 
              [-0.010, 0.060, 0.011, 0.001, -0.001, -1.571593]]

    self.spawn_multiple_objects('wrs_assembly_2020', objects, poses, 'tray_center')
    if not base_plate_in_tray:  # Spawn the base plate on the fixation, for MTC demos
      self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[0.12, 0.2, 0.0, tau/4, 0.0, -tau/4]], 'attached_base_origin_link')
    else:
      if layout_number == 3:
        self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[-0.1, -0.05, 0.001, tau/4, 0.0, 0.0]], 'tray_center')
      else: # layout 2
        self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[0.1, 0.05, 0.001, tau/4, 0.0, tau/2]], 'tray_center')
    return

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
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    self.b_bot.go_to_named_pose("feeder_pick_ready", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.vision.activate_camera("b_bot_outside_camera")

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    success = False
    while not success and not rospy.is_shutdown():
      success = self.subtask_g()  # Large plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    success = False
    while not success and not rospy.is_shutdown():
      success = self.subtask_f() # motor plate

    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=False, screw_size=4)

    success = False
    while not success and not rospy.is_shutdown():
      success = self.subtask_c1() # bearing 
    # # self.subtask_c2() # shaft

    rospy.loginfo("==== Finished.")

    # # To prepare subtask E
    # self.pick_retainer_pin_from_tray_and_place_in_holder(do_centering=False)
    # self.confirm_to_proceed("press enter to proceed")
    # self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # assy.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=assy.use_real_robot)
    # assy.do_change_tool_action("b_bot", equip=False, screw_size=4)

    # self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # self.subtask_e(pick_from_holder=True) #idle pulley

    # self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

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
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (a_bot).")
      rospy.loginfo("Enter 21,22 to close,open grippers")
      rospy.loginfo("Enter 30, 31 to pick screw m3, m4 from feeder.")
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
        assy.a_bot.go_to_named_pose("home")
        assy.b_bot.go_to_named_pose("home")
      elif i == '1a':
        assy.a_bot.go_to_named_pose("home")
      elif i == '1b':
        assy.b_bot.go_to_named_pose("home")
      elif i == '11':
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      elif i == '12':
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      elif i == '13':
        assy.do_change_tool_action("a_bot", equip=True, screw_size=3)
      elif i == '14':
        assy.do_change_tool_action("a_bot", equip=False, screw_size=3)
      elif i == '21':
        assy.a_bot.gripper.close(wait=False)
        assy.b_bot.gripper.close()
      elif i == '22':
        assy.a_bot.gripper.open(wait=False)
        assy.b_bot.gripper.open()
      elif i == '30':
        assy.a_bot.go_to_named_pose("feeder_pick_ready")
        assy.skill_server.pick_screw_from_feeder("a_bot", screw_size=3)
        assy.a_bot.go_to_named_pose("feeder_pick_ready")
      elif i == '31':
        assy.a_bot.go_to_named_pose("feeder_pick_ready")
        assy.skill_server.pick_screw_from_feeder("a_bot", screw_size=4)
        assy.a_bot.go_to_named_pose("feeder_pick_ready")
      elif i == "41":
        assy.publish_part_in_assembled_position("base")
      elif i == "421":
        assy.publish_part_in_assembled_position("panel_bearing")
      elif i == "422":
        assy.publish_part_in_assembled_position("panel_motor")
      elif i == "43":
        assy.publish_part_in_assembled_position("bearing")
      elif i == "44":
        assy.publish_part_in_assembled_position("motor")
      elif i == "45":
        assy.publish_part_in_assembled_position("shaft")
        assy.publish_part_in_assembled_position("endcap")
      elif i == "46":
        assy.publish_part_in_assembled_position("pulley")
      elif i == "51":
        grasp_pose = assy.assembly_database.get_grasp_pose("panel_bearing", "default_grasp")
        res = assy.plan_pick_place("panel_bearing", "a_bot", grasp_pose)
      elif i == "52":
        assy.execute_MTC_solution(res.solution, speed=0.1)
      elif i == "53":
        assy.pick(robot_name="a_bot", object_name="panel_bearing")
      elif i == "54":
        assy.pick(robot_name="b_bot", object_name="base")
      elif i == '55':
        assy.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=2)
      elif i == '552':
        assy.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=1)
      elif i == '553':
        assy.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=3)
      elif i == '67':
        assy.spawn_objects_for_closed_loop_test()
      elif i == '68':
        assy.spawn_objects_for_demo()
      elif i == '69':
        result = assy.do_plan_pick_action('panel_bearing', robot_name = '', save_solution_to_file = 'pick_panel_bearing')
        for solution in result.solution.sub_trajectory:
          scene_diff = solution.scene_diff
          planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
          planning_scene_diff_req.scene = scene_diff
          # self.apply_planning_scene_diff.call(planning_scene_diff_req)   # DEBUG: Update the scene pretending the action has been completed
      elif i == '691':
        rospy.loginfo("Loading 'pick_panel_bearing'")
        mp_res = assy.load_MTC_solution('pick_panel_bearing')
        rospy.loginfo("Running")
        assy.execute_MTC_solution(mp_res.solution, speed = 0.2)
      elif i == '70':
        assy.mtc_place_object_in_tray_center('panel_bearing')
      elif i == '71':
        assy.mtc_pickplace_l_panel()
      elif i == '72':
        assy.mtc_pick_screw_tool('m4')
      elif i == '73':
        assy.mtc_suck_screw('m4')
      elif i == '74':
        assy.do_plan_release_action('panel_bearing', 'home', save_solution_to_file = 'release_panel_bearing')
      elif i == '75':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        assy.do_plan_wrs_subtask_b_action('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', save_solution_to_file = 'subassembly')
      elif i == '80':
        rospy.loginfo("Loading")
        mp_res = assy.load_MTC_solution('subassembly')
        rospy.loginfo("Running")
        assy.execute_MTC_solution(mp_res.solution, speed = 0.2)
      elif i == '800':
        assy.b_bot.load_and_execute_program(program_name="wrs2020_push_motor_plate.urp", wait=True)
      elif i == '801':
        assy.skill_server.move_lin_rel("a_bot", relative_translation=[0, -0.01, 0], relative_to_robot_base=True, max_wait=5.0)
      elif i == '802':
        assy.skill_server.move_lin_rel("a_bot", relative_translation=[0,  0.01, 0], relative_to_robot_base=True, max_wait=5.0)
      elif i == '81':
        assy.do_change_tool_action('b_bot', equip=True, screw_size=4)
      elif i == '82':
        assy.do_change_tool_action('b_bot', equip=False, screw_size=4)
      elif i == '83':
        assy.skill_server.pick_screw_from_feeder('b_bot', 4)
      elif i == '84':
        assy.skill_server.do_linear_push('a_bot', force=15, direction="Y-", max_approach_distance=0.05, forward_speed=0.003)
      elif i == '85':
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
        assy.pick_up_and_insert_bearing(task="assembly")
      elif i == '9311':
        assy.align_bearing_holes(task="assembly")
      elif i == "93move":
        assy.playback_sequence("bearing_orient")
        assy.playback_sequence("bearing_move_to_assembly")
      elif i == '932':
        assy.fasten_bearing(task="assembly")
      elif i == '933':
        assy.insert_bearing(task="assembly")
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
        assy.a_bot.go_to_named_pose("home", force_ur_script=False)
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
      elif i == "print":  # Print collision objects
        assy.print_objects_in_tray()
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
        assy.a_bot.activate_ros_control_on_ur()
        assy.b_bot.activate_ros_control_on_ur()
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
