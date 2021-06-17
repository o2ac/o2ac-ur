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
from ur_control import conversions
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
    if not self.assembly_database.db_name == "wrs_assembly_2020":
      self.set_assembly("wrs_assembly_2020")

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
    pose.pose.orientation.w = 1.0
    self.assembly_database.publish_assembly_frames(pose, prefix="assembled_")
    return True


  ################ ----- Subroutines  

  def subtask_zero(self):
    # ============= SUBTASK BASE (picking and orienting and placing the baseplate) =======================
    rospy.loginfo("======== SUBTASK BASE ========")
    
    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")

    self.unlock_base_plate()
    self.publish_status_text("Target: base plate")

    centering_pose = self.assembly_database.get_grasp_pose("base", "terminal_grasp")
    grasp_pose = self.assembly_database.get_grasp_pose("base", "default_grasp")
    if not centering_pose or not grasp_pose:
      rospy.logerr("Could not load grasp poses for object " + "base" + ". Aborting pick.")
      return False

    centering_pose.header.frame_id = "move_group/base"
    centering_pose = self.listener.transformPose("tray_center", centering_pose)
    centering_pose.pose.position.z += .005
    
    above_centering_pose = copy.deepcopy(centering_pose)
    above_centering_pose.pose.position.z += .08
    
    self.a_bot.gripper.open(opening_width=0.06, wait=False)
    if not self.a_bot.go_to_pose_goal(above_centering_pose, speed=0.5, move_lin=False):
      return False
    if not self.a_bot.go_to_pose_goal(centering_pose, speed=0.5, move_lin=True):
      return False

    # TODO(felixvd): Check if too close to the border, do the alternative method
    self.allow_collisions_with_robot_hand("tray", "a_bot")
    if not self.center_with_gripper("a_bot", opening_width=0.07, gripper_force=80, move_back_to_initial_position=False):
      return False
    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    above_centering_pose.pose = helpers.rotatePoseByRPY(tau/4, 0, 0, above_centering_pose.pose)
    if not self.a_bot.go_to_pose_goal(above_centering_pose, speed=0.5):
      return False

    grasp_pose.header.frame_id = "move_group/base"
    grasp_pose = self.listener.transformPose("tray_center", grasp_pose)

    if not self.simple_pick("a_bot", grasp_pose, axis="z", approach_height=0.12):
      rospy.logerr("Fail to grasp base plate")
      return False

    if not self.simple_gripper_check("a_bot", min_opening_width=0.05):
      rospy.logerr("Gripper did not grasp the base plate. Aborting.")
      return False

    ##
    base_drop = conversions.to_pose_stamped("assembled_part_01", [0.111, 0.007, 0.07, tau/4., 0, -tau/4.])
    
    # There is a risk of overextending the wrist joint if we don't use the joint pose
    above_base_drop = [1.57783019, -1.430060581, 1.67834741, -1.82884373, -1.56911117, 0.00590014457]
    self.a_bot.move_joints(above_base_drop, speed=0.5)
    self.a_bot.go_to_pose_goal(base_drop, speed=0.3, move_lin = True)

    self.confirm_to_proceed("Go to recorded insertion pose?")
    base_inserted = conversions.to_pose_stamped("assembled_part_01", [0.108, -0.006, 0.067, 1.568, 0.103, -1.582])  # Taught
    self.a_bot.go_to_pose_goal(base_inserted, speed=0.05, move_lin = True)
    self.a_bot.gripper.open()

    self.a_bot.move_lin_rel(relative_translation=[-0.03, 0, 0.03], relative_to_robot_base=True)
    self.a_bot.go_to_named_pose("home")
    

    # success = self.pick("b_bot", "base")  # Uses MTC + attached objects

    # # # Pick using grasp pose only, ignoring scene object
    # # grasp_pose = self.assembly_database.get_grasp_pose("base", "default_grasp")
    # # if not grasp_pose:
    # #   rospy.logerr("Could not load grasp pose " + "default_grasp" + " for object " + "base" + ". Aborting pick.")
    # #   return False
    # # grasp_pose.header.frame_id = "move_group/base"
    # # try:
    # #   grasp_pose = self.listener.transformPose("tray_center", grasp_pose)
    # # except:
    # #   rospy.logerr("Could not transform from object. Is the object " + "base" + " in the scene?")
    # #   return False
    
    # # self.planning_scene_interface.allow_collisions("base", "")  # Allow collisions with all other objects
    # # success = self.simple_pick("b_bot", grasp_pose, axis="z", approach_height=0.1)
    
    # if not success or self.b_bot.gripper.opening_width < 0.003 and self.use_real_robot:
    #   rospy.logerr("Gripper did not grasp the base plate. Aborting.")
    #   return False

    # # Center and reorient plate outside of tray

    # approach_orient_pose = geometry_msgs.msg.PoseStamped()
    # approach_orient_pose.header.frame_id = "workspace_center"
    # approach_orient_pose.pose.position = geometry_msgs.msg.Point(0.133, 0.35, 0.15)
    # approach_orient_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(tau/4, tau/4, 0))

    # orient_pose = copy.deepcopy(approach_orient_pose)
    # orient_pose.pose.position.z = 0.01

    # self.b_bot.go_to_pose_goal(approach_orient_pose, speed=0.2, acceleration=0.1, move_lin = True)
    # success = self.b_bot.go_to_pose_goal(orient_pose, speed=0.1, acceleration=0.1, move_lin = True)
    # if not success:
    #   rospy.logerr("b_bot did not go to orient_pose. Critical. Try again.")
    #   success = self.b_bot.go_to_pose_goal(orient_pose, speed=0.1, acceleration=0.1, move_lin = True)
    #   if not success:
    #     self.confirm_to_proceed("Failed again. Drop plate?")
    #     self.b_bot.gripper.open(wait=True)
    #     return False
    # self.b_bot.gripper.open(wait=True)

    # self.b_bot.robot_group.detach_object("base")
    # rospy.loginfo("tic")
    # self.allow_collisions_with_robot_hand("base", "b_bot")
    # rospy.loginfo("toc")

    # self.b_bot.move_lin_rel(relative_rotation=[0,0,tau/4], speed=1.0, acceleration=0.5)

    # self.b_bot.gripper.close(force = 100, wait=True)
    # self.b_bot.gripper.open(wait=True)
    # self.b_bot.go_to_pose_goal(orient_pose, speed=1.0, acceleration=0.5)
    # self.b_bot.gripper.close(force = 100, wait=True)

    # # TODO: Center the object
    # self.b_bot.robot_group.attach_object("base")

    # self.b_bot.go_to_pose_goal(approach_orient_pose, speed=0.2)
    
    # # Move plate
    # place_onboard_pose = geometry_msgs.msg.PoseStamped()
    # place_onboard_pose.header.frame_id = "workspace_center"
    # place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.177, 0.008, 0.13)
    # # TODO: Define the terminal subframes
    # # place_onboard_pose.header.frame_id = "assy_01_terminal_top"
    # # place_onboard_pose.pose.position = geometry_msgs.msg.Point(-0.01, -0.005, 0.0)
    # place_onboard_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, 0))

    # approach_onboard_pose = copy.deepcopy(place_onboard_pose)
    # approach_onboard_pose.pose.position.z += 0.08

    # self.b_bot.go_to_pose_goal(approach_onboard_pose, speed=0.3, acceleration=0.1)
    # self.b_bot.go_to_pose_goal(place_onboard_pose, speed=0.2)

    # # FIXME: Direction should be -Z
    # self.b_bot.linear_push(force=8, direction="+Z", relative_to_ee=False, timeout=15.0)
    
    # self.b_bot.gripper.open(wait=False)
    
    self.publish_part_in_assembled_position("base")

    # self.b_bot.move_lin_rel(relative_translation=[0, 0, 0.05], relative_to_robot_base=True)
    self.lock_base_plate()
    rospy.sleep(0.3)
    self.unlock_base_plate()
    rospy.sleep(0.3)
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
    self.publish_status_text("Target: Bearing" )
    if self.pick_up_and_insert_bearing(task="assembly"):
      if self.fasten_bearing(task="assembly"):
        self.fasten_bearing(task="assembly", only_retighten=True)
        self.unequip_tool('b_bot', 'screw_tool_m4')
        return True
    return False
  
  def subtask_c2(self):
    rospy.loginfo("======== SUBTASK C (output shaft) ========")
    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    
    self.allow_collisions_with_robot_hand("shaft", "b_bot", True)
    self.allow_collisions_with_robot_hand("end_cap", "a_bot", True)

    self.publish_status_text("Target: shaft" )
    if not self.orient_shaft():
      return False
    
    self.publish_status_text("Target: end cap" )
    if not self.orient_shaft_end_cap():
      return False

    pre_insertion_shaft = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.2, 0, 0, -tau/4.])
    if not self.b_bot.go_to_pose_goal(pre_insertion_shaft, speed=0.2):
      rospy.logerr("Fail to go to pre_insertion_shaft")
      return False

    pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.003, 0, 0.25]+np.deg2rad([-180, 90, -90]).tolist())
    if not self.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.2, move_lin=False):
      rospy.logerr("Fail to go to pre_insertion_end_cap")
      return False

    self.confirm_to_proceed("insertion of end cap")

    self.a_bot.linear_push(force=3, direction="+Z", max_translation=0.1, timeout=10.0)
    self.a_bot.move_lin_rel(relative_translation=[0,0,0.002]) # release pressure before insertion

    selection_matrix = [0.3, 0.3, 0., 0.95, 1, 1]
    # TODO(cambel): define the termination criteria w.r.t the result of the linear push + offset  or with respect to shaft/screw_hole frame
    target_pose_target_frame = conversions.to_pose_stamped("tray_center", [-0.003, -0.000, 0.233]+np.deg2rad([-180, 90, -90]).tolist())
    result = self.a_bot.do_insertion(target_pose_target_frame, insertion_direction="+Z", force=2.0, timeout=20.0, 
                                                      radius=0.004, relaxed_target_by=0.003, selection_matrix=selection_matrix,
                                                      check_displacement_time=3.)
    success = result in (TERMINATION_CRITERIA, DONE)
    if not success:
      return False

    self.a_bot.gripper.send_command(0.06, velocity=0.01)
    self.a_bot.gripper.detach_object("end_cap")
    self.despawn_object("end_cap")
    
    self.confirm_to_proceed("prepare screw")

    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    if not self.do_change_tool_action("a_bot", equip=True, screw_size = 4):
      rospy.logerr("Failed equip m4")
      return False

    self.confirm_to_proceed("pick screw")
    if not self.pick_screw_from_feeder("a_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      return False
    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    
    self.confirm_to_proceed("go to above_hole_screw_pose")
    above_hole_screw_pose = conversions.to_pose_stamped("tray_center", [-0.001, 0.019, 0.388]+np.deg2rad([180, 30, 90]).tolist())
    if not self.a_bot.go_to_pose_goal(above_hole_screw_pose, speed=0.2, move_lin=False):
      rospy.logerr("Fail to go to above_hole_screw_pose")
      return False

    obj = self.assembly_database.get_collision_object("shaft")
    obj.header.frame_id = "b_bot_gripper_tip_link"
    obj.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, -tau/4, -tau/4))
    obj.pose = helpers.rotatePoseByRPY(0, 0, tau/2, obj.pose)
    obj.pose.position = conversions.to_point([-.006, 0, .0375])
    self.planning_scene_interface.add_object(obj)
    self.b_bot.gripper.attach_object(obj.id)

    self.confirm_to_proceed("try to screw")
    hole_screw_pose = conversions.to_pose_stamped("move_group/shaft/screw_hole", [0.0, 0.002, -0.010, 0, 0, 0])
    self.skill_server.do_screw_action("a_bot", hole_screw_pose, screw_height=0.02, screw_size=4, loosen_and_retighten_when_done=True)
    
    self.confirm_to_proceed("unequip tool")
    self.tools.set_suction("screw_tool_m4", suction_on=False, wait=False)

    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    
    if not self.do_change_tool_action("a_bot", equip=False, screw_size = 4):
      rospy.logerr("Failed unequip m4")
      return False
    
    if not self.a_bot.go_to_named_pose("home"):
      return False
    
    self.confirm_to_proceed("insert to bearing")

    if not self.align_shaft("assembled_part_07_inserted"):
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

    bearing_spacer_pose = self.look_and_get_grasp_point("bearing_spacer")
    bearing_spacer_pose.pose.position.x -= 0.005 # Magic numbers
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    if not self.simple_pick("a_bot", bearing_spacer_pose, grasp_height=-0.001, gripper_force=50.0, grasp_width=.05, axis="z", gripper_command=0.05):
      rospy.logerr("Fail to simple_pick")
      return False

    print("a_bot gripper opening??", self.a_bot.gripper.opening_width)
    if self.a_bot.gripper.opening_width < 0.02:
      rospy.logerr("Gripper did not grasp the bearing_spacer --> Stop")
      return False

    if not self.playback_sequence("bearing_spacer_orient"):
      rospy.logerr("Fail to complete the playback sequence bearing_spacer_orient")
      return False

    # # Move b_bot to hold shaft from end cap side
    self.b_bot.gripper.close()

    approach_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, -0.15] + np.deg2rad([-22.5, -88.5, -157.5]).tolist())
    pre_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, 0.02] + np.deg2rad([-22.5, -88.5, -157.5]).tolist())
    at_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.043, 0.000, 0.02] + np.deg2rad([-22.5, -88.5, -157.5]).tolist())

    trajectory = [[approach_hold_pose, 0.005], [pre_hold_pose, 0.005], [at_hold_pose, 0.0]]
    if not self.b_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete the hold pose")
      return False

    self.confirm_to_proceed("Insertion")

    if not self.insert_bearing_spacer("assembled_part_07_inserted"):
      rospy.logerr("Fail to complete insertion of bearing_spacer")
      return False
    
    trajectory = [[pre_hold_pose, 0.005], [approach_hold_pose, 0.005]]
    if not self.b_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete retreat (b_bot)")
      return False

    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    self.publish_status_text("Target: output_pulley" )

    output_pulley_pose = self.look_and_get_grasp_point("output_pulley", grasp_width=0.06, check_for_close_items=False)
    output_pulley_pose.pose.position.x -= 0.005 # Magic numbers
    output_pulley_pose.pose.position.z = 0.0 # Magic numbers
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    if not self.simple_pick("a_bot", output_pulley_pose, grasp_height=0.01, gripper_force=50.0, grasp_width=.06, axis="z", gripper_command=0.05):
      rospy.logerr("Fail to simple_pick")
      return False

    if self.a_bot.gripper.opening_width < 0.02:
      rospy.logerr("Gripper did not grasp the output_pulley --> Stop")
      return False

    if not self.playback_sequence("output_pulley_orient"):
      rospy.logerr("Fail to complete the playback sequence bearing_spacer_orient")
      return False

    # Move b_bot to hold shaft from end cap side
    self.b_bot.gripper.close()

    trajectory = [[approach_hold_pose, 0.005], [pre_hold_pose, 0.005], [at_hold_pose, 0.0]]
    if not self.b_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete the hold pose")
      return False

    self.confirm_to_proceed("insertion")

    if not self.insert_output_pulley("assembled_part_07_inserted"):
      rospy.logerr("Fail to complete insertion of bearing_spacer")
      return False
    
    self.a_bot.gripper.open()
    self.a_bot.move_lin_rel(relative_translation=[0,-0.2,0.1])
    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

    trajectory = [[pre_hold_pose, 0.005], [approach_hold_pose, 0.005]]
    if not self.b_bot.move_lin_trajectory(trajectory, speed=0.5):
      rospy.logerr("Fail to complete retreat (b_bot)")
      return False

    return self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (motor panel (small L-plate)) ========")
    # goal = self.look_and_get_grasp_point(2)  # bearing panel
    # if not goal:
    #     rospy.logerr("Could not find bearing plate in tray. Breaking out.")
    #     return False
    return self.panel_subtask(panel="panel_motor")

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (bearing panel (large L-plate)) ========")
    return self.panel_subtask(panel="panel_bearing")

  def panel_subtask(self, panel):
    """
    input parameter panel needs to be "panel_motor" or "panel_bearing"
    """

    self.publish_status_text("Target: " + panel)
    if not self.b_bot.go_to_named_pose("feeder_pick_ready"):
      rospy.logerr("b_bot did not move out of the way. Aborting.")
      return False

    # if not self.pick("a_bot", panel):
    #   rospy.logerr("Did not succeed picking " + panel + "!")
    #   return False

    # Pick using grasp pose only, ignoring scene object
    grasp_pose = self.assembly_database.get_grasp_pose(panel, "default_grasp")
    if not grasp_pose:
      rospy.logerr("Could not load grasp pose " + "default_grasp" + " for object " + panel + ". Aborting pick.")
      return False
    grasp_pose.header.frame_id = "move_group/" + panel
    try:
      grasp_pose = self.listener.transformPose("tray_center", grasp_pose)
    except:
      rospy.logerr("Could not transform from object. Is the object " + panel + " in the scene?")
      return False
    
    self.planning_scene_interface.allow_collisions(panel, "")  # Allow collisions with all other objects
    success = self.simple_pick("a_bot", grasp_pose, axis="z")


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

    self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
    self.vision.activate_camera(camera_name="b_bot_outside_camera")
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
      helpers.wait_for_UR_program("/a_bot", rospy.Duration.from_sec(20))
      
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
    self.unlock_base_plate()
    rospy.sleep(0.5)
    self.lock_base_plate()
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

  def full_assembly_task(self):
    # self.start_task_timer()

    # Look into the tray
    self.look_and_get_grasp_point(object_id=2)  # Base place
    self.confirm_to_proceed("press enter to proceed to pick and set base plate")
    success = False
    while not success and not rospy.is_shutdown():
        success = self.subtask_zero()  # Base plate
    
    ## Equip screw tool for subtasks G, F
    self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    self.b_bot.go_to_named_pose("feeder_pick_ready", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.vision.activate_camera("b_bot_outside_camera")

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    bearing_plate_success = False
    bearing_plate_success = self.subtask_g()  # Large plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    motor_plate_success = False
    motor_plate_success = self.subtask_f() # motor plate

    self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
    self.do_change_tool_action("b_bot", equip=False, screw_size=4)

    if bearing_plate_success:
      success = False
      while not success and not rospy.is_shutdown():
        success = self.subtask_c1() # bearing 
      success = False
      while not success and not rospy.is_shutdown():
        success = self.subtask_c2() # shaft
    
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
    return

  def disable_scene_object_collisions(self):
    """ Disables collisions between all world objects and everything else.
        Used because our meshes are so heavy that they impact performance too much.
    """
    object_names = self.planning_scene_interface.get_known_object_names()
    rospy.loginfo("Disabling collisions for all scene objects (except tools).")
    objects_without_tools = []
    for n in object_names:
      if not "tool" in n:
        objects_without_tools.append(n)
    print(objects_without_tools)
    self.planning_scene_interface.allow_collisions(objects_without_tools, "")

