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
import tf_conversions
import tf
import numpy as np
import math
from math import pi, degrees, radians, sin, cos

import ur_control
tau = 2.0*pi  # Part of math from Python 3.6
import math
import traceback
import time

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg

import o2ac_msgs
import o2ac_msgs.srv

# import o2ac_assembly_database
from o2ac_assembly_database.parts_reader import PartsReader

from o2ac_routines.common import O2ACCommon
from o2ac_routines.helpers import wait_for_UR_program, get_target_force
from ur_control import transformations as ur_transformations
from ur_control import conversions

class TaskboardClass(O2ACCommon):
  """
  This contains the routines used to run the taskboard task.
  """
  def __init__(self):
    super(TaskboardClass, self).__init__()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # Initialize debug monitor
    self.start_task_timer()
    
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/4))

    self.at_set_screw_hole = geometry_msgs.msg.PoseStamped()
    self.at_set_screw_hole.header.frame_id = "taskboard_set_screw_link"
    self.at_set_screw_hole.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    self.at_set_screw_hole.pose.position.x = 0.001   # MAGIC NUMBER
    self.at_set_screw_hole.pose.position.y = -0.0005   # MAGIC NUMBER
    self.at_set_screw_hole.pose.position.z = 0.001   # MAGIC NUMBER (points downward)
    if not self.assembly_database.db_name == "taskboard":
      self.assembly_database.change_assembly("taskboard")

  def spawn_example_objects(self):
    # This function spawns the objects into the tray as if they had been recognized by the vision node
    names = ["taskboard_idler_pulley_small", "bearing", "shaft", "motor_pulley"]
    offsets = {"bearing": [-.04, -.02, .001],
    "taskboard_idler_pulley_small": [.07, .06, .03],
    "shaft": [.03, -.06, .005], 
    "motor_pulley": [-.01, .12, .005]}

    # We publish each object to its own frame.
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(.5)
    counter = 0
    for name in names:
      collision_object = self.assembly_database.get_collision_object(name)
      if collision_object:
        counter += 1
        collision_object.header.frame_id = "collision_object_spawn_helper_frame" + str(counter)
        if name == "bearing":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        if name == "taskboard_idler_pulley_small" or name == "motor_pulley":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        elif name == "shaft":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
          
        offset = offsets[name]
        broadcaster.sendTransform((offset[0], offset[1], offset[2]), q_rotate, rospy.Time.now(), 
            "collision_object_spawn_helper_frame" + str(counter), "tray_center")
        rospy.sleep(1.0) # Wait for the transform to have propagated through the system

        self.planning_scene_interface.add_object(collision_object)
        # print("======== collision object: " + name)
        # print(collision_object.mesh_poses)
        # print(collision_object.subframe_names)
        # print(collision_object.subframe_poses)
      else:
        rospy.logerr("Could not retrieve collision object:" + name)

  #####
  
  def allow_collision_with_hand(self, robot_name, object_name):
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_gripper_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_left_finger_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_left_inner_knuckle_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_right_finger_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_right_inner_knuckle_link")

  def disallow_collision_with_hand(self, robot_name, object_name):
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_gripper_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_left_finger_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_left_inner_knuckle_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_right_finger_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_right_inner_knuckle_link")

  def look_in_tray(self):
    #TODO (felixvd)
    # Look at tray
    # loop through all items
    # check if they were recognized and are graspable
    # otherwise check position again
    # if still not graspable either skip to next item or try to reposition
    pass
  
  def prep_taskboard_task(self):
    """
    Equip the set screw tool and M3 tool, and move to the position before task start.
    """

    self.a_bot.go_to_named_pose("home")

    self.equip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    self.b_bot.go_to_named_pose("home")
    
    self.equip_tool("b_bot", "set_screw_tool")
    self.b_bot.go_to_named_pose("horizontal_screw_ready")

    self.move_b_bot_to_setscrew_initial_pos()
  
  def move_b_bot_to_setscrew_initial_pos(self):
    screw_approach = copy.deepcopy(self.at_set_screw_hole)
    screw_approach.pose.position.x = -0.005
    self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.5)

  def do_screw_tasks_from_prep_position(self):
    ### - Set screw
    
    # Move into the screw hole with motor on
    self.vision.activate_camera("b_bot_inside_camera")
    self.do_task("M2 set screw")
    
    # TODO: check set screw success with a_bot, do spiral motion with b_bot otherwise
    
    ### SCREW M3 WITH A_BOT
    # self.vision.activate_camera("a_bot_outside_camera")
    screw_picked = self.pick_screw_from_feeder("a_bot", screw_size = 3)
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    # Move b_bot back, a_bot to screw
    self.unequip_tool("b_bot", "set_screw_tool")
    self.equip_tool("b_bot", "screw_tool_m4")
    self.b_bot.go_to_named_pose("feeder_pick_ready")

    if screw_picked:
      self.subtask_completed["M3 screw"] = self.do_task("M3 screw")
    
    #### SCREW M4 WITH B_BOT
    self.subtask_completed["M4 screw"] = self.do_task("M4 screw")

    self.unequip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")
    self.unequip_tool("b_bot", "screw_tool_m4")

  def full_taskboard_task(self, do_screws=True):
    """
    Start the taskboard task from the fully prepped position (set screw tool and M3 tool equipped)
    """
    #####
    self.subtask_completed = {
      "M2 set screw": True,  # FIXME
      "M3 screw": False,
      "M4 screw": False,
      "belt": False,
      "bearing": False,
      "motor pulley": False,
      "shaft": False,  
      "idler pulley": True,  # FIXME
    }
    if do_screws:
      self.do_screw_tasks_from_prep_position()


    self.subtask_completed["belt"] = self.do_task("belt")
    
    # self.subtask_completed["idler pulley"] = self.do_task("idler pulley")
    # self.unequip_tool("b_bot", "screw_tool_m4")

    self.subtask_completed["shaft"] = self.do_task("shaft")
    
    self.subtask_completed["motor pulley"] = self.do_task("motor pulley")

    self.subtask_completed["bearing"] = self.do_task("bearing")
    if self.subtask_completed["bearing"]:
      self.subtask_completed["screw_bearing"] = self.do_task("screw_bearing")

    self.confirm_to_proceed("Continue into loop to retry parts?")
    order = ["shaft", "motor pulley", "belt", "bearing"]
    task_complete = False
    while not task_complete and not rospy.is_shutdown():
      for item in order:
        if not self.subtask_completed[item]:
          self.confirm_to_proceed("Reattempt " + str(item) + "?")
          self.subtask_completed[item] = self.do_task(item)
          # if item == "bearing" and self.subtask_completed[item]: 
          #   if not self.subtask_completed["screw_bearing"]:
          #     self.do_task("screw_bearing")
      

  def do_task(self, task_name, fake_execution_for_calibration=False):
    self.publish_status_text("Target: " + task_name)

    if task_name == "belt":
      # - Equip the belt tool with b_bot
      # self.equip_tool("belt_tool")
      
      self.a_bot.go_to_named_pose("home")
      
      self.b_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.1)

      self.vision.activate_camera("b_bot_outside_camera")
      self.activate_led("b_bot")
      res = self.get_3d_poses_from_ssd()
      r2 = self.get_feasible_grasp_points("belt")
      if r2:
        goal = r2[0]
        goal.pose.position.z = 0.0
      else:
        rospy.logerr("Could not find belt grasp pose! Aborting.")
        return False
      
      self.confirm_to_proceed("Pick tool with b_bot?")
      # Start the program with b_bot to pick the tool
      if not self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_pick_hook.urp", recursion_depth=3):
        return False
      rospy.sleep(2)

      self.simple_pick("a_bot", goal, gripper_force=100.0, grasp_width=.08, axis="z")
      self.a_bot.move_lin_rel(relative_translation=[0,0,.1])
      a_bot_wait_with_belt_pose = [0.646294116973877, -1.602117200891012, 2.0059760252581995, -1.3332312864116211, -0.8101084868060511, -2.4642069975482386]
      self.a_bot.move_joints(a_bot_wait_with_belt_pose)
      
      # TODO: Check for pick success with camera
      # b_bot_look_at_belt = [1.95739448, 1.92903739, -1.40047674, -1.98750128, -2.1883457, 1.7778782]

      self.confirm_to_proceed("Load and execute the belt threading programs?")
      success_a = self.a_bot.load_program(program_name="wrs2020/taskboard_belt_v5.urp", recursion_depth=3)      
      success_b = self.b_bot.load_program(program_name="wrs2020/taskboard_belt_v4.urp", recursion_depth=3)      
      if success_a and success_b:
        print("Loaded belt program on a_bot.")
        rospy.sleep(1)
        success = self.a_bot.execute_loaded_program()
        success = self.b_bot.execute_loaded_program()
        if success:
          print("Starting belt threading execution.")
          rospy.sleep(2)
          self.a_bot.close_ur_popup()
          self.b_bot.close_ur_popup()
      else:
        print("Problem loading. Not executing belt procedure.")
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
      return True
      

    # ==========================================================

    if task_name == "M2 set screw":
      # Equip and move to the screw hole
      # self.equip_tool("b_bot", "set_screw_tool")
      # self.b_bot.go_to_named_pose("horizontal_screw_ready")
      screw_approach = copy.deepcopy(self.at_set_screw_hole)
      screw_approach.pose.position.x = -0.005
      self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.b_bot.go_to_pose_goal(self.at_set_screw_hole, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.02)

      # This expects to be exactly above the set screw hole
      self.confirm_to_proceed("Turn on motor and move into screw hole?")
      dist = .002
      self.skill_server.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*dist, sin(radians(30))*dist], velocity=0.03, wait=False)
      # self.skill_server.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .002)
      self.tools.set_motor("set_screw_tool", "tighten", duration = 12.0)
      rospy.sleep(4.0) # Wait for the screw to be screwed in a little bit
      d = .004
      rospy.loginfo("Moving in further by " + str(d) + " m.")
      self.skill_server.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*d, sin(radians(30))*d], velocity=0.002, wait=False)
      
      # self.skill_server.do_linear_push("b_bot", force=40, direction_vector=[0, -cos(radians(30)), sin(radians(30))], forward_speed=0.001)
      rospy.sleep(8.0)
      self.confirm_to_proceed("Go back?")
      if self.b_bot.is_protective_stopped():
        rospy.logwarn("Robot was protective stopped after set screw insertion!")
        #TODO: Recovery? Try to loosen the shaft?
        self.b_bot.unlock_protective_stop()
        rospy.sleep(1)
        if self.b_bot.is_protective_stopped():
          return False

      # Go back
      self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)

      self.b_bot.go_to_named_pose("horizontal_screw_ready", speed=0.5, acceleration=0.5)
      # self.confirm_to_proceed("Unequip tool?")
      # self.b_bot.go_to_named_pose("home", speed=0.5, acceleration=0.5)
      # self.unequip_tool("b_bot", "set_screw_tool")
    
    # ==========================================================

    if task_name == "M3 screw":
      if not self.a_bot.robot_status.carrying_tool and not self.a_bot.robot_status.held_tool_id == "screw_tool_m3":
        self.equip_tool("a_bot", "screw_tool_m3")
      if not fake_execution_for_calibration:
        self.pick_screw_from_feeder("a_bot", screw_size = 3)
      self.a_bot.go_to_named_pose("horizontal_screw_ready")
      approach_pose = geometry_msgs.msg.PoseStamped()
      approach_pose.header.frame_id = "taskboard_m3_screw_link"
      approach_pose.pose.position.x = -.04
      approach_pose.pose.position.y = -.12
      approach_pose.pose.position.z = -.05
      self.a_bot.go_to_pose_goal(approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

      approach_pose.pose.position.y = -.0
      approach_pose.pose.position.z = -.0
      self.a_bot.go_to_pose_goal(approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m3_screw_link"
      hole_pose.pose.position.y = -.000  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.004  # MAGIC NUMBER (z-axis of the frame points down)
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      if not fake_execution_for_calibration:
        self.skill_server.do_screw_action("a_bot", hole_pose, screw_size = 3)
      else:
        hole_pose.pose.position.x -= 0.005
        self.a_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        self.a_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
      self.a_bot.go_to_named_pose("horizontal_screw_ready")
      self.a_bot.go_to_named_pose("home")
      if not fake_execution_for_calibration:
        self.unequip_tool("a_bot", "screw_tool_m3")

    # ==========================================================

    if task_name == "M4 screw":
      if not self.b_bot.robot_status.carrying_tool and not self.b_bot.robot_status.held_tool_id == "screw_tool_m4":
        self.equip_tool("b_bot", "screw_tool_m4")
      self.vision.activate_camera("b_bot_outside_camera")
      if not fake_execution_for_calibration:
        self.pick_screw_from_feeder("b_bot", screw_size = 4)
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m4_screw_link"
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
      hole_pose.pose.position.y = -.001  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.001  # MAGIC NUMBER (z-axis of the frame points down)
      if not fake_execution_for_calibration:
        self.skill_server.do_screw_action("b_bot", hole_pose, screw_size = 4)
      else:
        hole_pose.pose.position.x = -.01
        approach_pose = copy.deepcopy(hole_pose)
        approach_pose.pose.position.x -= .05
        self.b_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.b_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        self.b_bot.go_to_pose_goal(approach_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
        self.confirm_to_proceed("Did it go back?")
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      self.b_bot.go_to_named_pose("home")
      if not fake_execution_for_calibration:
        self.unequip_tool("b_bot", "screw_tool_m4")

    # ==========================================================

    if task_name == "motor pulley":
      self.a_bot.go_to_named_pose("home")
      self.b_bot.go_to_named_pose("home")
      
      goal = self.look_and_get_grasp_point(self.assembly_database.name_to_id("motor_pulley"))
      if not goal:
        rospy.logerr("Could not find motor_pulley in tray. Skipping procedure.")
        return False
      goal.pose.position.x -= 0.01 # MAGIC NUMBER
      goal.pose.position.z = 0.0
      self.vision.activate_camera("b_bot_inside_camera")
      self.activate_led("b_bot")
      self.simple_pick("b_bot", goal, gripper_force=50.0, grasp_width=.06, axis="z")
      if self.b_bot.gripper.opening_width < 0.01:
        rospy.logerr("Gripper did not grasp the pulley --> Stop")

      b_bot_script_start_pose = [1.7094888, -1.76184906, 2.20651847, -2.03368343, -1.54728252, 0.96213197]
      self.b_bot.move_joints(b_bot_script_start_pose)

      use_ros = True

      if use_ros:
        result = self.playback_sequence(routine_filename="orient_pulley")
        if result:
          self.insert_motor_pulley(task = "taskboard")
        else:
          rospy.logerr("Fail to complete the playback sequence")
          return False
      else:
        success_b = self.b_bot.load_program(program_name="wrs2020/pulley_v3.urp", recursion_depth=3)
        if success_b:
          print("Loaded pulley program.")
          rospy.sleep(1)
          self.b_bot.execute_loaded_program()
          print("Started execution. Waiting for b_bot to finish.")
        else:
          print("Problem loading. Not executing pulley procedure.")
          return False
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(40))
      return True
    
    # ==========================================================

    if task_name == "bearing":
      self.pick_up_and_insert_bearing(task="taskboard")

    if task_name == "screw_bearing":
      success = self.fasten_bearing(task="taskboard")
      self.unequip_tool('b_bot', 'screw_tool_m4')
      return success
    
    # ==========================================================

    if task_name == "shaft":
      use_ros = True
      if use_ros:
        self.pick_and_insert_shaft("taskboard")
      else:
        self.a_bot.go_to_named_pose("home")
        self.b_bot.go_to_named_pose("home")

        goal = self.look_and_get_grasp_point(self.assembly_database.name_to_id("shaft"))
        if not goal:
          rospy.logerr("Could not find shaft in tray. Skipping procedure.")
          print(self.objects_in_tray)
          return False
        goal.pose.position.z = 0.001
        self.vision.activate_camera("b_bot_inside_camera")
        self.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")
        
        success_b = self.b_bot.load_program(program_name="wrs2020/shaft_v3.urp", recursion_depth=3)
        
        if success_b:
          print("Loaded shaft program.")
          rospy.sleep(1)
          self.b_bot.execute_loaded_program()
          print("Started execution. Waiting for b_bot to finish.")
        else:
          print("Problem loading. Not executing shaft procedure.")
          return False
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(50))
        if self.b_bot.is_protective_stopped():
          rospy.logwarn("Robot was protective stopped after shaft insertion - shaft may be stuck!")
          #TODO: Recovery? Try to loosen the shaft?
          self.b_bot.unlock_protective_stop()
          rospy.sleep(1)
          if self.b_bot.is_protective_stopped():
            return False
      return True
    
    # ==========================================================
    
    if task_name == "idler pulley":

      use_ros = True
      if use_ros:
        self.pick_and_insert_idler_pulley("taskboard")
      
      else:
        self.a_bot.go_to_named_pose("home")
        self.b_bot.go_to_named_pose("home")
        
        goal = self.look_and_get_grasp_point("taskboard_idler_pulley_small")
        if not goal:
          rospy.logerr("Could not find idler pulley in tray. Skipping procedure.")
          return False
        self.vision.activate_camera("b_bot_inside_camera")
        goal.pose.position.x -= 0.01 # MAGIC NUMBER
        goal.pose.position.z = 0.014
        rospy.loginfo("Picking idler pulley at: ")
        self.b_bot.go_to_named_pose("home")
        pick_pose = copy.deepcopy(goal)
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))

        approach_pose = copy.deepcopy(pick_pose)
        approach_pose.pose.position.z += 0.1

        self.a_bot.gripper.open(wait=False)
        self.a_bot.gripper.open(wait=False, opening_width=0.07)

        ## wait for screw tool to hold

        ##### Centering using urp
        centeringgrasp = self.a_bot.load_program(program_name="wrs2020/taskboard_retainer_and_nut_v4_hu.urp", recursion_depth=3)
        if not centeringgrasp:
          rospy.logerr("Failed to load centeringgrasp program on a_bot")
          return False
        print("Running belt pick on a_bot.")
        if not self.a_bot.execute_loaded_program():
          rospy.logerr("Failed to execute centeringgrasp program on a_bot")
          return False
        wait_for_UR_program("/a_bot", rospy.Duration.from_sec(20))

        self.equip_tool("b_bot", "screw_tool_m4")
        # success_a = self.a_bot.load_program(program_name="wrs2020/tb_retainer_and_nut_v2_hu.urp", recursion_depth=3)
        success_b = self.b_bot.load_program(program_name="wrs2020/taskboard_retainer_and_nut_v4.urp", recursion_depth=3)

        if success_b:
          print("Loaded idler pulley program.")
          rospy.sleep(1)
          self.a_bot.execute_loaded_program()
          rospy.sleep(20) # abot picks
          self.b_bot.execute_loaded_program()
          rospy.sleep(10) # bbot holds
          self.confirm_to_proceed("Can popup be closed? 1")
          self.b_bot.close_ur_popup()
          self.tools.set_motor("screw_tool_m4", "tighten", duration=20)
          rospy.sleep(22) # bbot fiddles
          self.confirm_to_proceed("Can popup be closed? 2")
          self.a_bot.close_ur_popup()
          rospy.sleep(15) #a bot picks nut
          self.confirm_to_proceed("Can popup be closed? 3")
          self.a_bot.close_ur_popup()
          self.tools.set_motor("screw_tool_m4", "tighten", duration=20)
          rospy.sleep(30) # a bot spirals nut
          self.confirm_to_proceed("Can popups be closed? 4")
          self.a_bot.close_ur_popup()
          self.b_bot.close_ur_popup()
        else:
          print("Problem loading. Not executing idler pulley procedure.")
          return False
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(10))
        wait_for_UR_program("/a_bot", rospy.Duration.from_sec(10))
        if self.b_bot.is_protective_stopped():
          # rospy.logwarn("Robot was protective stopped after idler pulley insertion - idler pulley may be stuck!")
          # self.b_bot.unlock_protective_stop()
          self.b_bot.go_to_named_pose("home")
        self.unequip_tool("b_bot", "screw_tool_m4")
        self.a_bot.go_to_named_pose("home")
        self.b_bot.go_to_named_pose("home")
      return True
    
if __name__ == '__main__':
  try:
    rospy.init_node('o2ac_routines', anonymous=False)
    taskboard = TaskboardClass()
    taskboard.define_tool_collision_objects()

    i = 1
    while i and not rospy.is_shutdown():
      rospy.loginfo("Enter 1 to move robots to home")
      rospy.loginfo("Enter 11, 12 to close/open grippers")
      rospy.loginfo("Enter 13, 14 to equip/unequip m3 screw tool")
      rospy.loginfo("Enter 15, 16 to equip/unequip m4 screw tool")
      # rospy.loginfo("Enter 17, 18 to equip/unequip belt placement tool")
      rospy.loginfo("Enter 3 to calibrate screw tasks (31: M3, 32: M4)")
      rospy.loginfo("Subtasks: 51 (set screw), 52 (M3), 53 (M4), 54 (belt), 55 (motor pulley), 56 (shaft), 57 (bearing), 58 (idler pulley)")
      rospy.loginfo("Enter 8 to spawn example parts")
      rospy.loginfo("Enter reset to reset the scene")
      rospy.loginfo("Enter prep to prepare the task (do this before running)")
      rospy.loginfo("Enter ssup/ssdown to fine-tune the set screw tool position")
      rospy.loginfo("Enter start to run the task (competition mode, no confirmations)")
      rospy.loginfo("Enter test for a test run of the task (WITH confirmations)")
      rospy.loginfo("Enter x to exit")
      i = raw_input()
      
      if i == "prep":
        taskboard.prep_taskboard_task()
      if i == "ssup":
        taskboard.at_set_screw_hole.pose.position.z -= 0.001
        taskboard.move_b_bot_to_setscrew_initial_pos()
      if i == "ssdown":
        taskboard.at_set_screw_hole.pose.position.z += 0.001
        taskboard.move_b_bot_to_setscrew_initial_pos()
      if i == "start":
        taskboard.competition_mode = True
        taskboard.full_taskboard_task()
        taskboard.competition_mode = False
      if i == "test":
        taskboard.competition_mode = False
        taskboard.full_taskboard_task()
      if i == "1":
        taskboard.a_bot.go_to_named_pose("home")
        taskboard.b_bot.go_to_named_pose("home")
      if i == "11":
        taskboard.a_bot.gripper.close(wait=False)
        taskboard.b_bot.gripper.close(wait=False)
      if i == "12":
        taskboard.a_bot.gripper.open(wait=False)
        taskboard.b_bot.gripper.open(wait=False)
      if i == "13":
        taskboard.equip_tool("a_bot", "screw_tool_m3") 
      if i == "14":
        taskboard.unequip_tool("a_bot", "screw_tool_m3")
      if i == "15":
        taskboard.equip_tool("b_bot", "screw_tool_m4")
      if i == "16":
        taskboard.unequip_tool("b_bot", "screw_tool_m4")
      if i == "17":
        taskboard.equip_tool("b_bot", "set_screw_tool")
      if i == "18":
        taskboard.unequip_tool("b_bot", "set_screw_tool")
      if i == "3":
        taskboard.b_bot.go_to_named_pose("home")
        taskboard.do_task("M3 screw", fake_execution_for_calibration=True)
        taskboard.a_bot.go_to_named_pose("home")
        taskboard.do_task("M4 screw", fake_execution_for_calibration=True)
        taskboard.b_bot.go_to_named_pose("home")
      if i == "31":
        taskboard.do_task("M3 screw", fake_execution_for_calibration=True)
      if i == "32":
        taskboard.do_task("M4 screw", fake_execution_for_calibration=True)
      if i == "51":
        taskboard.do_task("M2 set screw")
      if i == "52":
        taskboard.do_task("M3 screw")
      if i == "53":
        taskboard.do_task("M4 screw")
      if i == "54":
        taskboard.do_task("belt")
      if i == "55":
        taskboard.do_task("motor pulley")
      if i == "56":
        taskboard.do_task("shaft")
      if i == "57":
        taskboard.do_task("bearing")
      if i == "577":
        taskboard.do_task("screw_bearing")
      if i == "575":
        taskboard.align_bearing_holes(task="taskboard")
      if i == "58":
        taskboard.do_task("idler pulley")
      if i == "8":
        taskboard.spawn_example_objects()      
      if i == "9":
        taskboard.activate_led("b_bot", on=True)
      if i == "99":
        taskboard.activate_led("b_bot", on=False)
      if i == "d1":
        taskboard.b_bot.check_for_dead_controller_and_force_start()
      if i == "f1":
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "tray_center"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        pick_pose.pose.position.x = 0.03
        taskboard.simple_pick("a_bot", pick_pose, approach_height = 0.05, item_id_to_attach = "bearing", lift_up_after_pick=True)
      if i == "f2":
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "tray_center"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        ps.pose.position.z = -0.15
        taskboard.go_to_pose_goal("a_bot", ps, speed=0.1, end_effector_link="bearing/back_hole", move_lin = False)
      if i == "reset":
        taskboard.reset_scene_and_robots()
      if i == "x":
        break
      i = True

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass

