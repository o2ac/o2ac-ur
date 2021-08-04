#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
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
#  * Neither the name of OMRON SINIC X nor the names of its
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

from o2ac_routines.base import AssemblyStatus
from ur_control import conversions, transformations
import rospy
import geometry_msgs.msg
import moveit_msgs
import tf_conversions
import tf
from math import pi, radians, sin, cos, pi
tau = 2.0*pi  # Part of math from Python 3.6
import math
import time
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
import o2ac_routines.helpers as helpers

from ur_control.constants import DONE, TERMINATION_CRITERIA

class O2ACAssembly(O2ACCommon):
  """
  This class contains the assembly routines.
  """
  def __init__(self):
    super(O2ACAssembly, self).__init__()
    
    # Load the initial database
    if not self.assembly_database.db_name == "wrs_assembly_2021":
      self.set_assembly("wrs_assembly_2021")

    # Spawn tools and objects
    self.define_tool_collision_objects()

    ### Only used for MTC planning
    # screw_ids = ['m3', 'm4']
    # for screw_id in screw_ids:
    #   self.spawn_tool('screw_tool_' + screw_id)
    #   self.upload_tool_grasps_to_param_server(screw_id)
    
    self.belt_storage_location = geometry_msgs.msg.PoseStamped()
    self.belt_storage_location.header.frame_id = "left_centering_link"
    self.belt_storage_location.pose.position.x = -0.005  # Height
    self.belt_storage_location.pose.position.y = 0.05
    self.belt_storage_location.pose.position.z = 0.05

  ################ ----- Subtasks

  def pick_and_store_belt(self):
    self.b_bot.go_to_named_pose("home")
    self.a_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="a_bot_outside_camera_color_frame", speed=.8)

    self.vision.activate_camera("a_bot_outside_camera")
    self.activate_led("a_bot")
    res = self.get_3d_poses_from_ssd()
    r2 = self.get_feasible_grasp_points("belt")
    if r2:
      pick_goal = r2[0]
      pick_goal.pose.position.z = -0.001
      pick_goal.pose.position.x = -0.02  # MAGIC NUMBER
    else:
      rospy.logerr("Could not find belt grasp pose! Aborting.")
      return False
    
    # TODO(felixvd): Adjust this check so that the gripper does not open before the vision confirmed the belt pick

    self.vision.activate_camera("a_bot_inside_camera")
    self.simple_pick("a_bot", pick_goal, gripper_force=100.0, approach_height=0.15, grasp_width=.04, axis="z")
    
    self.b_bot.go_to_named_pose("home")
    self.simple_place("a_bot", self.belt_storage_location)
    self.a_bot.move_lin_rel(relative_translation=[0,-0.05,.1])
    
    success = self.vision.check_pick_success("belt")
    if success:
      rospy.loginfo("Belt storage success!")
    else:
      rospy.loginfo("Belt storage failed!")
      # TODO(felixvd): Open gripper over tray in case an object was picked accidentally
      
    self.a_bot.go_to_named_pose("home")
    return success

  ################ ----- Subtasks

  def subtask_zero(self, skip_initial_perception=False, use_b_bot_camera=False):
    # ============= SUBTASK BASE (picking and orienting and placing the baseplate) =======================
    rospy.loginfo("======== SUBTASK BASE ========")
    
    self.unlock_base_plate()
    self.publish_status_text("Target: base plate")

    self.pick_base_panel(skip_initial_perception=skip_initial_perception, use_b_bot_camera=use_b_bot_camera)

    self.confirm_to_proceed("Did the base plate move up?")

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)

    if not self.simple_gripper_check("a_bot", min_opening_width=0.05):
      rospy.logerr("Gripper did not grasp the base plate. Aborting.")
      self.a_bot.gripper.open()
      return False
    
    if not self.use_real_robot:
      self.allow_collisions_with_robot_hand("base_fixture_top", "a_bot", allow=True)
    ## Move to fixation
    base_drop = conversions.to_pose_stamped("assembled_part_01", [0.111, 0.007, 0.07, tau/4., 0, -tau/4.])
    
    # There is a risk of overextending the wrist joint if we don't use the joint pose
    above_base_drop = [1.57783019, -1.430060581, 1.67834741, -1.82884373, -1.56911117, 0.00590014457]
    base_inserted = conversions.to_pose_stamped("assembled_part_01", [0.108, -0.006, 0.067, 1.568, 0.103, -1.582])  # Taught
    seq = []
    seq.append(helpers.to_sequence_item(above_base_drop, 0.5, linear=False))
    seq.append(helpers.to_sequence_item(base_drop, 0.3))
    seq.append(helpers.to_sequence_item(base_inserted, 0.05))
    if not self.execute_sequence("a_bot", seq, "place base plate"):
      return False
    # self.a_bot.move_joints(above_base_drop, speed=0.5)
    # self.a_bot.go_to_pose_goal(base_drop, speed=0.3, move_lin = True)
    # self.a_bot.go_to_pose_goal(base_inserted, speed=0.05, move_lin = True)
    self.a_bot.gripper.open()
    self.a_bot.gripper.forget_attached_item()

    def set_base_plate():
      self.lock_base_plate()
      rospy.sleep(0.3)
      self.unlock_base_plate()
      rospy.sleep(0.3)
      self.lock_base_plate()
    def a_bot_return():
      self.a_bot.move_lin_rel(relative_translation=[-0.03, 0, 0.03], relative_to_robot_base=True)
      self.a_bot.go_to_named_pose("home")
      self.allow_collisions_with_robot_hand("base", "a_bot", allow=False)
      self.publish_part_in_assembled_position("base")
    self.do_tasks_simultaneous(a_bot_return, set_base_plate)
    if not self.use_real_robot:
      self.allow_collisions_with_robot_hand("base_fixture_top", "a_bot", allow=False)
    return True

  def subtask_a(self):
    # ============= SUBTASK A (picking and inserting and fastening the motor) =======================
    rospy.loginfo("======== SUBTASK A (motor) ========")
    self.publish_status_text("Target: Motor")
    self.pick_and_center_motor()
    self.orient_motor()

    self.do_change_tool_action("a_bot", equip=True, screw_size=3)
    self.a_bot.go_to_named_pose("screw_ready")

    self.align_motor_pre_insertion()
    self.insert_motor()
    self.fasten_motor()
    # TODO: Fasten two motor screws with support from b_bot, open b_bot gripper, finish fastening with a_bot
    return False

  def subtask_b(self):
    rospy.loginfo("======== SUBTASK B (motor pulley) ========")
    rospy.logerr("Subtask B not implemented yet")
    return

  def subtask_c1(self):
    rospy.loginfo("======== SUBTASK C (bearing) ========")
    self.publish_status_text("Target: Bearing" )
    self.unequip_tool("b_bot")
    success = False
    if self.pick_up_and_insert_bearing(task="assembly"):
      self.b_bot.gripper.forget_attached_item()
      if self.align_bearing_holes(task="assembly"):
        self.b_bot.go_to_named_pose("home")
        if self.fasten_bearing(task="assembly"):
          self.fasten_bearing(task="assembly", only_retighten=True)
          self.unequip_tool('b_bot', 'screw_tool_m4')
          success = True
    self.despawn_object("bearing")
    return success
  
  def subtask_c2(self):
    rospy.loginfo("======== SUBTASK C (output shaft) ========")
    self.ab_bot.go_to_named_pose("home")
    
    self.allow_collisions_with_robot_hand("shaft", "b_bot", True)
    self.allow_collisions_with_robot_hand("end_cap", "a_bot", True)
    
    self.publish_status_text("Target: end cap" )
    
    if not self.pick_end_cap():
      return False
    if not self.orient_shaft_end_cap():
      return False

    self.publish_status_text("Target: shaft" )
    if not self.pick_shaft():
      return False
    if not self.orient_shaft():
      return False

    pre_insertion_shaft = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.2, 0, 0, -tau/4.])
    if not self.b_bot.go_to_pose_goal(pre_insertion_shaft, speed=0.2):
      rospy.logerr("Fail to go to pre_insertion_shaft")
      return False

    pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.002, -0.001, 0.25]+np.deg2rad([-180, 90, -90]).tolist())
    if not self.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.2, move_lin=False):
      rospy.logerr("Fail to go to pre_insertion_end_cap")
      return False

    self.confirm_to_proceed("insertion of end cap")
    if not self.insert_end_cap():
      rospy.logerr("failed to insert end cap")
      return False

    self.confirm_to_proceed("Did insertion succeed? Press Enter to open gripper")

    self.a_bot.gripper.send_command(0.06, velocity=0.01)
    self.a_bot.gripper.detach_object("end_cap")
    self.despawn_object("end_cap")
    
    self.confirm_to_proceed("prepare screw")

    if not self.fasten_end_cap():
      return False
    
    if not self.a_bot.go_to_named_pose("home"):
      return False
    
    self.confirm_to_proceed("insert to bearing")

    if not self.align_shaft("assembled_part_07_inserted", pre_insert_offset=0.065):
      return False

    if not self.insert_shaft("assembled_part_07_inserted", target=0.043):
      return False

    if not self.b_bot.go_to_named_pose("home"):
       return False

    self.allow_collisions_with_robot_hand("end_cap", "a_bot", False)
    self.allow_collisions_with_robot_hand("shaft", "b_bot", False)

  def subtask_d(self):
    # Fasten large pulley to output shaft
    rospy.loginfo("======== SUBTASK D (output pulley) ========")
    rospy.logerr("Subtask D not implemented yet")
    return False

  def subtask_e(self):
    rospy.loginfo("======== SUBTASK E (output pulley) ========") 

    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    self.allow_collisions_with_robot_hand("bearing_spacer", "a_bot", True)
    self.allow_collisions_with_robot_hand("output_pulley", "a_bot", True)

    self.publish_status_text("Target: bearing_spacer" )

    if not self.pick_bearing_spacer():
      return False

    if not self.playback_sequence("bearing_spacer_orient"):
      rospy.logerr("Fail to complete the playback sequence bearing_spacer_orient")
      return False

    # # Move a_bot to hold shaft from end cap side
    self.a_bot.gripper.close()

    self.confirm_to_proceed("prepare a_bot")

    approach_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, -0.15] + np.deg2rad([-90,-90,-90]).tolist())
    # self.a_bot.go_to_pose_goal(approach_hold_pose)
    pre_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
    # self.a_bot.go_to_pose_goal(pre_hold_pose)
    at_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.043, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
    # self.a_bot.go_to_pose_goal(at_hold_pose)

    trajectory = [[approach_hold_pose, 0.005, 0.5], [pre_hold_pose, 0.005, 0.5], [at_hold_pose, 0.0, 0.2]]
    if not self.a_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete the hold pose")
      return False

    self.confirm_to_proceed("Insertion")

    if not self.insert_bearing_spacer("assembled_part_07_inserted"):
      rospy.logerr("Fail to complete insertion of bearing_spacer")
      return False

    standby_pose = [1.77763, -1.13511, 0.81185, -1.24681, -1.56753, -1.36329]
    if not self.a_bot.move_joints(standby_pose):
      return False

    self.publish_status_text("Target: output_pulley" )

    if not self.pick_output_pulley():
      return False

    if not self.playback_sequence("output_pulley_orient"):
      rospy.logerr("Fail to complete the playback sequence bearing_spacer_orient")
      return False

    self.confirm_to_proceed("insertion")

    if not self.a_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete the hold pose")
      return False

    if not self.insert_output_pulley("assembled_part_07_inserted"):
      rospy.logerr("Fail to complete insertion of bearing_spacer")
      return False
    
    self.b_bot.gripper.open()
    self.b_bot.move_lin_rel(relative_translation=[0.1, 0.0, 0.1])
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    trajectory = [[pre_hold_pose, 0.005], [approach_hold_pose, 0.005]]
    if not self.a_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete retreat (a_bot)")
      return False

    return self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (motor panel (small L-plate)) ========")
    attempts = 0
    success = False
    while not success and attempts < 3 and not rospy.is_shutdown():
      rospy.loginfo("======== SUBTASK F, attempt " + str(attempts) + " ========")
      success = self.panel_subtask(panel="panel_motor", attempt_nr=attempts)
      attempts += 1
    return success

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (bearing panel (large L-plate)) ========")
    attempts = 0
    success = False
    while not success and attempts < 3 and not rospy.is_shutdown():
      success = self.panel_subtask(panel="panel_bearing", attempt_nr=attempts)
      attempts += 1
    return success

  def panel_subtask(self, panel, attempt_nr=0, allow_fallbacks=True, simultaneous_execution=True):
    """
    input parameter panel needs to be "panel_motor" or "panel_bearing"
    """
    if simultaneous_execution:
      return self.panel_subtask_simultaneous(panel, attempt_nr=attempt_nr, allow_fallbacks=allow_fallbacks)

    self.publish_status_text("Target: " + panel)
    if not self.b_bot.go_to_named_pose("feeder_pick_ready"):
      rospy.logerr("b_bot did not move out of the way. Aborting.")
      return False

    self.activate_led("a_bot")
    plate_pose = self.get_large_item_position_from_top(panel, "a_bot")
    if not plate_pose:
      rospy.logerr("Cannot find " + panel + " in tray. Return False.")
      return False

    # Pick using grasp pose only, ignoring scene object
    grasp_pose = self.assembly_database.get_grasp_pose(panel, "default_grasp")
    if not grasp_pose:
      rospy.logerr("Could not load grasp pose " + "default_grasp" + " for object " + panel + ". Aborting pick.")
      return False
    grasp_pose.header.frame_id = "move_group/" + panel
    try:
      self.listener.waitForTransform("move_group/" + panel, "tray_center", grasp_pose.header.stamp, rospy.Duration(1))
      grasp_pose_tray = self.listener.transformPose("tray_center", grasp_pose)
    except:
      rospy.logerr("Could not transform from object. Is the object " + panel + " in the scene?")
      return False
    
    self.planning_scene_interface.allow_collisions(panel, "")
    self.planning_scene_interface.allow_collisions(panel, "tray")
    self.planning_scene_interface.allow_collisions(panel, "tray_center")
    self.allow_collisions_with_robot_hand(panel, "a_bot")
    rospy.sleep(1.0)  # TODO(felixvd): Necessary after enabling collisions? Likely.
    if not self.too_close_to_border(grasp_pose_tray, border_dist=0.025):
      picked = self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.06, minimum_grasp_width=0.0001)
    else:
      picked = False
    
    if allow_fallbacks:
      # Fallback: Try moving the plate
      if not picked:
        self.a_bot.go_to_named_pose("home")
        self.unequip_tool("b_bot")
        if panel == "panel_motor":
          tool_pull_pose = conversions.to_pose_stamped("move_group/panel_motor", [0.03, 0.038, 0.0, 0, 0, 0])
        elif panel == "panel_bearing":
          tool_pull_pose = conversions.to_pose_stamped("move_group/panel_bearing/front_hole", [0.0, 0.0, 0.0, 0, 0, 0])
        
        print("tool_pull_pose", tool_pull_pose.pose.position)
        tool_pull_pose = self.listener.transformPose("tray_center", tool_pull_pose)
        print("tool_pull_pose tfed", tool_pull_pose.pose.position)

        # If close to border, pull towards the middle
        if self.too_close_to_border(grasp_pose_tray, border_dist=0.04):  
          # Add 1 cm distance to pull pose
          # print("tool_pull_pose before", tool_pull_pose.pose.position)
          # tool_pull_pose.pose.position.x += 0.01 * np.sign(tool_pull_pose.pose.position.x)
          # tool_pull_pose.pose.position.y += 0.01 * np.sign(tool_pull_pose.pose.position.y)
          print("tool_pull_pose after", tool_pull_pose.pose.position)
          self.move_towards_center_with_tool("b_bot", target_pose=tool_pull_pose, distance=0.05, start_with_spiral=True)
          self.planning_scene_interface.allow_collisions(panel, "")  # Collisions are reactivated in move_towards_center_with_tool
          self.planning_scene_interface.allow_collisions(panel, "tray")
          self.planning_scene_interface.allow_collisions(panel, "tray_center")
          self.allow_collisions_with_robot_hand(panel, "a_bot")
        else:  # If not close to border, try to hit a hole and make space around the plate
          self.declutter_with_tool("b_bot", tool_pull_pose) 
          
        self.b_bot.go_to_named_pose("feeder_pick_ready")
        return self.panel_subtask(panel, attempt_nr=attempt_nr, allow_fallbacks=False)

      # Fallback: Try to pick all 4 possible orientations
      if attempt_nr > 0:
        for i in range(4):
          rospy.logwarn("Fallback: Rotating plate (" + str() + " out of 3 times)")
          self.rotate_plate_collision_object_in_tray(panel)
          rospy.sleep(.5)
          grasp_pose_tray = self.listener.transformPose("tray_center", grasp_pose)
          if self.is_grasp_pose_feasible(grasp_pose_tray, border_dist=0.025):
            picked = self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.06, minimum_grasp_width=0.0001)
          if picked:
            break

    if not picked:
      rospy.logerr("Did not pick panel. Abort.")
      return False

    self.confirm_to_proceed("Go on to placing program?")

    # TODO: Check that the plate is seen by SSD when placed outside the tray

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
    helpers.wait_for_UR_program("/a_bot", rospy.Duration.from_sec(40))

    self.publish_part_in_assembled_position(panel)
    self.allow_collisions_with_robot_hand(panel, "a_bot")

    self.fasten_panel(panel)
    
    self.unlock_base_plate()
    rospy.sleep(0.5)
    self.lock_base_plate()
    self.allow_collisions_with_robot_hand(panel, "a_bot", allow=False)
    return True

  def panel_subtask_simultaneous(self, panel, attempt_nr=0, allow_fallbacks=True):
    """
    input parameter panel needs to be "panel_motor" or "panel_bearing"
    """
    self.publish_status_text("Target: " + panel)

    def b_bot_task(): # Pick tool & screw, then wait
      self.equip_tool(robot_name="b_bot", tool_name="screw_tool_m4")
      if not self.b_bot.go_to_named_pose("feeder_pick_ready"):
        rospy.logerr("b_bot did not move out of the way. Aborting.")
        return False

    grasp_pose = self.assembly_database.get_grasp_pose(panel, "default_grasp")
    if not grasp_pose:
      rospy.logerr("Could not load grasp pose " + "default_grasp" + " for object " + panel + ". Aborting pick.")
      return False
    grasp_pose.header.frame_id = "move_group/" + panel
    self.picked = False
    def a_bot_task(): # Pick and orient panel
      self.activate_led("a_bot")
      plate_pose = self.get_large_item_position_from_top(panel, "a_bot")
      if not plate_pose:
        rospy.logerr("Cannot find " + panel + " in tray. Return False.")
        return False

      # Pick using the grasp pose only, ignoring scene object
      try:
        self.listener.waitForTransform("move_group/" + panel, "tray_center", grasp_pose.header.stamp, rospy.Duration(1))
        grasp_pose_tray = self.listener.transformPose("tray_center", grasp_pose)
      except:
        rospy.logerr("Could not transform from object. Is the object " + panel + " in the scene?")
        return False
      
      self.planning_scene_interface.allow_collisions(panel, "")
      self.planning_scene_interface.allow_collisions(panel, "tray")
      self.planning_scene_interface.allow_collisions(panel, "tray_center")
      self.allow_collisions_with_robot_hand(panel, "a_bot")
      rospy.sleep(1.0)  # TODO(felixvd): Necessary after enabling collisions? Likely.
      if not self.too_close_to_border(grasp_pose_tray, border_dist=0.025):
        self.picked = self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.06, minimum_grasp_width=0.0001)
      else:
        self.picked = False
    
    self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180.0)
    
    try:
      self.listener.waitForTransform("move_group/" + panel, "tray_center", grasp_pose.header.stamp, rospy.Duration(1))
      grasp_pose_tray = self.listener.transformPose("tray_center", grasp_pose)
    except:
      rospy.logerr("Could not transform from object. Is the object " + panel + " in the scene?")
      return False
    if allow_fallbacks:
      # Fallback: Try moving the plate
      if not self.picked:
        self.a_bot.go_to_named_pose("home", wait=False)
        self.unequip_tool("b_bot")
        if panel == "panel_motor":
          tool_pull_pose = conversions.to_pose_stamped("move_group/panel_motor", [0.03, 0.038, 0.0, 0, 0, 0])
        elif panel == "panel_bearing":
          tool_pull_pose = conversions.to_pose_stamped("move_group/panel_bearing/front_hole", [0.0, 0.0, 0.0, 0, 0, 0])
        
        # print("tool_pull_pose", tool_pull_pose.pose.position)
        tool_pull_pose = self.listener.transformPose("tray_center", tool_pull_pose)
        # print("tool_pull_pose tfed", tool_pull_pose.pose.position)

        # If close to border, pull towards the middle
        if self.too_close_to_border(grasp_pose_tray, border_dist=0.04):
          # Add 1 cm distance to pull pose
          self.move_towards_center_with_tool("b_bot", target_pose=tool_pull_pose, distance=0.05, start_with_spiral=True)
          self.planning_scene_interface.allow_collisions(panel, "")  # Collisions are reactivated in move_towards_center_with_tool
          self.planning_scene_interface.allow_collisions(panel, "tray")
          self.planning_scene_interface.allow_collisions(panel, "tray_center")
          self.allow_collisions_with_robot_hand(panel, "a_bot")
        else:  # If not close to border, try to hit a hole and make space around the plate
          self.declutter_with_tool("b_bot", tool_pull_pose) 
          
        self.b_bot.go_to_named_pose("feeder_pick_ready", wait=False)
        return self.panel_subtask(panel, attempt_nr=attempt_nr, allow_fallbacks=False)

      # Fallback 2: Try to pick all 4 possible orientations
      if attempt_nr > 0:
        for i in range(4):
          rospy.logwarn("Fallback: Rotating plate (" + str() + " out of 3 times)")
          self.rotate_plate_collision_object_in_tray(panel)
          rospy.sleep(.5)
          grasp_pose_tray = self.listener.transformPose("tray_center", grasp_pose)
          if self.is_grasp_pose_feasible(grasp_pose_tray, border_dist=0.025):
            self.picked = self.simple_pick("a_bot", grasp_pose_tray, axis="z", grasp_width=0.06, minimum_grasp_width=0.0001)
          if self.picked:
            break
    
    if not self.picked:
      rospy.logerr("Did not pick panel. Abort.")
      return False

    self.confirm_to_proceed("Go on to placing program?")

    # TODO: Check that the plate is seen by SSD when placed outside the tray
    
    def a_bot_task2():
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
      helpers.wait_for_UR_program("/a_bot", rospy.Duration.from_sec(40))

      self.publish_part_in_assembled_position(panel)
      self.allow_collisions_with_robot_hand(panel, "a_bot")
    
    def b_bot_task2():
      self.equip_tool(robot_name="b_bot", tool_name="screw_tool_m4")
      self.vision.activate_camera(camera_name="b_bot_outside_camera")
      self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True)
    
    self.do_tasks_simultaneous(a_bot_task2, b_bot_task2, timeout=90.0)

    if not self.tools.screw_is_suctioned.get("m4", False): 
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.a_bot.go_to_named_pose("home")

    if panel == "panel_bearing":
      part_name = "assembled_part_03_"
    elif panel == "panel_motor":
      part_name = "assembled_part_02_"

    screw_target_pose = geometry_msgs.msg.PoseStamped()
    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_1"
    screw_target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(radians(-20), 0, 0))
    if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel):
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
      helpers.wait_for_UR_program("/a_bot", rospy.Duration.from_sec(20))
      
      # Retry fastening
      if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel):
        rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
        return False
    rospy.loginfo("Successfully fastened screw 1")

    def a_bot_task3():
      self.a_bot.gripper.close()
      self.a_bot.gripper.open()
      if not self.a_bot.go_to_named_pose("home", wait=False):
        rospy.logerr("Failed to move a_bot home!")
        return False
    def b_bot_task3():
      self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True)
    self.do_tasks_simultaneous(a_bot_task3, b_bot_task3, timeout=180.0)
    if not self.tools.screw_is_suctioned.get("m4", False): 
      rospy.logerr("Failed to pick second screw from feeder, could not fix the issue. Abort.")

    screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_2"
    if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel):
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
      if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel):
        rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
        return False
    self.unlock_base_plate()
    rospy.sleep(0.5)
    self.lock_base_plate()
    self.allow_collisions_with_robot_hand(panel, "a_bot", allow=False)
    return True

  def panels_assembly(self, simultaneous=True):
    self.publish_status_text("Target: panel bearing")
    if not self.pick_panel("panel_bearing"):
      return False
    self.panel_bearing_pose = None
    self.b_bot_success = False
    def a_bot_task():
      self.panel_bearing_pose = self.center_panel("panel_bearing", store=True)
    def b_bot_task():
      rospy.sleep(3.0)
      self.publish_status_text("Target: panel motor")
      self.b_bot_success = self.pick_panel("panel_motor", simultaneous=False)
    
    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60)
    else:
      a_bot_task()
      b_bot_task()
    if not self.b_bot_success:
      rospy.logerr("Fail to do panels_assembly1: simultaneous=%s" % simultaneous)
      return False
    self.panel_motor_pose = None
    self.base_pose = False
    def a_bot_task():
      self.panel_motor_pose = self.center_panel("panel_motor", store=True)
    def b_bot_task():
      rospy.sleep(3.0)
      self.activate_led("b_bot")
      rospy.loginfo("Looking for base plate with b_bot")
      self.base_pose = self.get_large_item_position_from_top("base", "b_bot")
      self.b_bot.go_to_named_pose("home")
    
    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60)
    else:
      a_bot_task()
      b_bot_task()

    if not self.panel_motor_pose or not self.base_pose:
      rospy.logerr("Fail to do panels_assembly2: simultaneous=%s" % simultaneous)
      return False

    self.skip_perception = isinstance(self.base_pose, geometry_msgs.msg.PoseStamped)
    self.a_bot_success = False
    self.b_bot_success = False
    def a_bot_task():
      self.publish_status_text("Target: base plate")
      self.a_bot_success = self.subtask_zero(skip_initial_perception=self.skip_perception)
    def b_bot_task():
      start_time = rospy.get_time()
      while not self.b_bot_success and rospy.get_time()-start_time < 15:
        self.b_bot_success = self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      if self.b_bot_success:
        self.b_bot_success = False
        self.b_bot_success = self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=False)
        self.b_bot.go_to_named_pose("feeder_pick_ready")

    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60)
    else:
      a_bot_task()
      b_bot_task()
    if not self.b_bot_success or not self.a_bot_success:
      rospy.logerr("Fail to do panels_assembly3: simultaneous=%s" % simultaneous)
      return False

    if not self.place_panel("a_bot", "panel_bearing", pick_again=True, grasp_pose=self.panel_bearing_pose):
      return False
    self.publish_status_text("Target: fasten panel bearing")
    if not self.fasten_panel("panel_bearing"):
      return False
    if not self.place_panel("a_bot", "panel_motor", pick_again=True, grasp_pose=self.panel_motor_pose):
      return False
    self.publish_status_text("Target: fasten panel motor")
    if not self.fasten_panel("panel_motor"):
      return False

    del self.panel_bearing_pose
    del self.panel_motor_pose
    del self.skip_perception
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

  ##############

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
    elif layout_number == 4:
      objects = ['base', 'panel_motor', 'panel_bearing']
      poses = [[0.1, 0.04, 0.001, tau/4, 0.0, tau/2],
               [-0.04, 0.01, 0.001, 0.0, 0.0, tau/2],
               [0.01, -0.08, 0.001, 0.0, 0.0, tau/2]] 
      self.spawn_multiple_objects('wrs_assembly_2020', objects, poses, 'tray_center')
      return

    self.spawn_multiple_objects('wrs_assembly_2020', objects, poses, 'tray_center')
    if not base_plate_in_tray:  # Spawn the base plate on the fixation, for MTC demos
      self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[0.12, 0.2, 0.0, tau/4, 0.0, -tau/4]], 'attached_base_origin_link')
    else:
      if layout_number == 3:
        self.spawn_multiple_objects('wrs_assembly_2020', ['base'], [[-0.1, -0.04, 0.001, tau/4, 0.0, 0.0]], 'tray_center')
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

  def single_assembly_task(self, tray_name=None, simultaneous_execution=False):
    if simultaneous_execution:
      return self.full_assembly_task_simultaneous()
    if tray_name:
      if not self.pick_tray_from_agv_stack_calibration_long_side(tray_name=tray_name):
        rospy.logerr("Fail to pick and place tray. Abort!")
        return False

    # Look into the tray
    success = self.panels_assembly()

    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=False, screw_size=4)

    if success:
      self.assembly_status.completed_subtask_c1 = self.subtask_c1() # bearing 
      # if self.assembly_status.completed_subtask_c1:
      #   self.assembly_status.completed_subtask_c2 = self.subtask_c2() # shaft
      #   if self.assembly_status.completed_subtask_c2:
      #     self.assembly_status.completed_subtask_e = self.subtask_e() # bearing spacer / output pulley
    
    self.do_change_tool_action("a_bot", equip=False, screw_size=3)
    self.ab_bot.go_to_named_pose("home")
    if not self.unload_drive_unit():
      rospy.logerr("Fail to unload drive unit. Abort!")
      return
    if tray_name:
      if not self.return_tray_to_agv_stack_calibration_long_side(tray_name):
        rospy.logerr("Fail to return tray. Abort!")
        return
    rospy.loginfo("==== Finished.")

    # self.subtask_a() # motor

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
    
    # self.subtask_c() # bearing, clamping pulley set

  def full_assembly_task(self, simultaneous_execution=False):
    self.ab_bot.go_to_named_pose("home")
    self.reset_scene_and_robots()
    trays = ["tray1", "tray2"]
    self.orient_tray_stack()
    for tray in trays:
      self.assembly_status = AssemblyStatus()
      self.single_assembly_task(tray, simultaneous_execution)
      self.set_assembly("wrs_assembly_2020")
    rospy.loginfo("==== Finished both tasks ====")
    return

  def full_assembly_task_simultaneous(self):
    if not self.assembly_status.tray_placed_on_table:
      self.orient_tray_stack()
      self.pick_tray_from_agv_stack_calibration_long_side("tray1")
      # TODO(cambel): add a loop for the second tray

    self.a_bot_success = False
    self.b_bot_success = False
    def b_bot_task():
      self.pick_and_store_motor()
      self.b_bot.go_to_named_pose("home")
    def a_bot_task():      
      # Look into the tray
      self.publish_status_text("Target: base plate")
      while not self.assembly_status.completed_subtask_zero and not rospy.is_shutdown():
          self.assembly_status.completed_subtask_zero = self.subtask_zero()  # Base plate
    
    self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60)
  
    self.vision.activate_camera("b_bot_outside_camera")

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    if not self.assembly_status.completed_subtask_g:
      self.assembly_status.completed_subtask_g = self.subtask_g()  # Bearing plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    if not self.assembly_status.completed_subtask_f:
      self.assembly_status.completed_subtask_f = self.subtask_f() # Motor plate

    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=False, screw_size=4)

    if self.assembly_status.completed_subtask_g:  # Bearing plate
      self.assembly_status.completed_subtask_c1 = self.subtask_c1() # bearing 
      if self.assembly_status.completed_subtask_c1:
        self.assembly_status.completed_subtask_c2 = self.subtask_c2() # shaft
      #   if self.assembly_status.completed_subtask_c2:
      #     self.assembly_status.completed_subtask_e = self.subtask_e() # bearing spacer / output pulley
    
    self.ab_bot.go_to_named_pose("home")
    self.unload_drive_unit()
    self.return_tray_to_agv_stack_calibration_long_side("tray1")
    self.assembly_status = AssemblyStatus()
    rospy.loginfo("==== Finished.")
