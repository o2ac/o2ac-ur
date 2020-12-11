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
from math import pi, degrees, radians, sin, cos
tau = 2.0*pi  # Part of math from Python 3.6
import traceback
import time

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg

import o2ac_msgs
import o2ac_msgs.srv

# import o2ac_assembly_handler
from o2ac_assembly_handler.assy_reader import AssyReader

from o2ac_routines.common import O2ACCommon

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

    self.item_names = ["Bearing", "Belt", "Idler pulley", 
                      "M2 set screw", "M3 screw", 
                      "M4 screw", "Pulley", "Shaft"]
    
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/4))

    # self.assy_reader = AssyReader("taskboard")


  def multiply_quaternion_msgs(self, q1_msg, q2_msg):
    q1 = [q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w]
    q2 = [q2_msg.x, q2_msg.y, q2_msg.z, q2_msg.w]
    return geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply(q1, q2))
    
  def spawn_example_objects(self):
    # This function spawns the objects into the tray as if they had been recognized by the vision node
    names = ["taskboard_idler_pulley_small", "bearing", "drive_shaft", "motor_pulley"]
    offsets = {"bearing": [-.04, -.02, .001],
    "taskboard_idler_pulley_small": [.07, .06, .03], 
    "drive_shaft": [.03, -.06, .005], 
    "motor_pulley": [-.01, .12, .005], 
    "endcap": [-.05, -.1, .005]}

    # We publish each object to its own frame.
    broadcaster = tf.TransformBroadcaster()
    counter = 0
    for name in names:
      collision_object = self.assy_reader.get_collision_object(name)
      if collision_object:
        counter += 1
        collision_object.header.frame_id = "collision_object_spawn_helper_frame" + str(counter)
        if name == "bearing":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        if name == "taskboard_idler_pulley_small" or name == "motor_pulley":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        elif name == "drive_shaft":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
          
        offset = offsets[name]
        broadcaster.sendTransform((offset[0], offset[1], offset[2]), q_rotate, rospy.Time.now(), 
            "collision_object_spawn_helper_frame" + str(counter), "tray_center")
        rospy.sleep(2.0) # Wait for the transform to have propagated through the system

        self.planning_scene_interface.add_object(collision_object)
        # print("======== collision object: " + name)
        # print(collision_object.mesh_poses)
        # print(collision_object.subframe_names)
        # print(collision_object.subframe_poses)
      else:
        rospy.logerr("Could not retrieve collision object:" + name)

  #####
  
  def allow_collision_with_hand(self, robot_name, object_name):
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_left_finger_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_left_inner_knuckle_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_right_finger_tip_link")
    self.planning_scene_interface.allow_collisions(object_name, robot_name + "_robotiq_85_right_inner_knuckle_link")

  def disallow_collision_with_hand(self, robot_name, object_name):
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_left_finger_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_left_inner_knuckle_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_right_finger_tip_link")
    self.planning_scene_interface.disallow_collisions(object_name, robot_name + "_robotiq_85_right_inner_knuckle_link")

  def full_taskboard_task(self):
    self.start_task_timer()
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # Proposed order:
    # - Set screw
    # - Belt 
    # - Small screws (ideally at the same time)
    # - Bearing
    # - Shaft
    # - Motor pulley
    # - Retainer pin + nut last. This is the hardest.
    
    # Set screw has to be first, because b_bot is right on top of it
    self.confirm_to_proceed("Current item: M2 set screw")
    self.do_task("M2 set screw")
    self.confirm_to_proceed("Current item: M3 screw")
    self.do_task("M3 screw")
    self.confirm_to_proceed("Current item: M4 screw")
    self.do_task("M4 screw")
    self.confirm_to_proceed("Current item: Belt")
    self.do_task("belt")
    self.confirm_to_proceed("Current item: Bearing")
    self.do_task("bearing")
    self.confirm_to_proceed("Current item: Pulley")
    self.do_task("pulley")
    self.confirm_to_proceed("Current item: Shaft")
    self.do_task("shaft")
    self.confirm_to_proceed("Current item: Idler pulley")
    self.do_task("idler pulley")

  def prep_taskboard_video(self):
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    self.do_change_tool_action("b_bot", equip=True, screw_size = 2)  # Set screw tool
    self.go_to_named_pose("horizontal_screw_ready", "b_bot")

    screw_approach = geometry_msgs.msg.PoseStamped()
    screw_approach.header.frame_id = "taskboard_set_screw_link"
    screw_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, 0, 0))
    screw_approach.pose.position.x -= 0.005    
    self.go_to_pose_goal("b_bot", screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)

  def full_taskboard_task_for_video(self):
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ### - Set screw
    
    # Move into the screw hole with motor on
    self.set_motor("set_screw_tool", "tighten", duration = 10.0)
    dist = .005
    self.move_lin_rel("b_bot", relative_translation=[0, cos(radians(30))*dist, sin(radians(30))*dist], velocity=0.03, wait=False)
    

    # TODO: check set screw success with a_bot, do spiral motion with b_bot otherwise

    # Move b_bot back, a_bot to screw
    self.go_to_named_pose("home", "a_bot")
    self.do_change_tool_action("b_bot", equip=False, screw_size = 2)
    
    #### SCREW WITH A_BOT
    self.go_to_named_pose("horizontal_screw_ready", "a_bot")
    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose.header.frame_id = "taskboard_m3_screw_link"
    approach_pose.pose.position.x = -.04
    approach_pose.pose.position.y = -.12
    approach_pose.pose.position.z = -.05
    taskboard.go_to_pose_goal("a_bot", approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

    approach_pose.pose.position.y = -.0
    approach_pose.pose.position.z = -.0
    taskboard.go_to_pose_goal("a_bot", approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
    raw_input()

    hole_pose = geometry_msgs.msg.PoseStamped()
    hole_pose.header.frame_id = "taskboard_m3_screw_link"
    hole_pose.pose.position.z = -.005  # MAGIC NUMBER
    hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
    taskboard.do_screw_action("a_bot", hole_pose, screw_size = 3)
    self.go_to_named_pose("horizontal_screw_ready", "a_bot")
    self.go_to_named_pose("home", "a_bot")
    
    ###
    
    #### SCREW WITH B_BOT
    self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
    self.pick_screw_from_feeder("b_bot", screw_size = 4)
    self.go_to_named_pose("horizontal_screw_ready", "b_bot")
    hole_pose = geometry_msgs.msg.PoseStamped()
    hole_pose.header.frame_id = "taskboard_m4_screw_link"
    hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, 0, 0))
    self.do_screw_action("b_bot", hole_pose, screw_size = 4)

    self.do_change_tool_action("a_bot", equip=False, screw_size = 3)
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.do_change_tool_action("b_bot", equip=False, screw_size = 4)

    # - Belt 
    # Pick belt tool with b_bot
    # Pick belt with a_bot
    # Move to start positions with a_bot, b_bot
    # Run the taught program

    # - Bearing
    # Run regrasp

    # - Shaft
    
    # - Motor pulley


    # - Retainer pin + nut last. This is the hardest.
    
  
  def do_task(self, task_name):
    
    if task_name == "belt":
      #
      # - Equip the belt tool with b_bot
      # self.equip_tool("belt_tool")
      
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
    # ==========================================================

    if task_name == "M2 set screw":
      # Equip and move to the screw hole
      # self.do_change_tool_action("b_bot", equip=True, screw_size = 2)  # Set screw tool
      # self.go_to_named_pose("horizontal_screw_ready", "b_bot")
      at_hole = geometry_msgs.msg.PoseStamped()
      at_hole.header.frame_id = "taskboard_set_screw_link"
      at_hole.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      at_hole.pose.position.x = 0.001   # MAGIC NUMBER
      at_hole.pose.position.y = 0.001   # MAGIC NUMBER
      at_hole.pose.position.z = -0.002   # MAGIC NUMBER
      screw_approach = copy.deepcopy(at_hole)
      screw_approach.pose.position.x = -0.005
      self.go_to_pose_goal("b_bot", screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.go_to_pose_goal("b_bot", at_hole, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.02)

      # This expects to be exactly above the set screw hole
      self.confirm_to_proceed("Turn on motor and move into screw hole?")
      dist = .002
      self.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*dist, sin(radians(30))*dist], velocity=0.03, wait=False)
      # self.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .002)
      self.set_motor("set_screw_tool", "tighten", duration = 12.0)
      rospy.sleep(4.0) # Wait for the screw to be screwed in a little bit
      d = .004
      rospy.loginfo("Moving in further by " + str(d) + " m.")
      self.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*d, sin(radians(30))*d], velocity=0.002, wait=False)
      
      # self.do_linear_push("b_bot", force=40, direction_vector=[0, -cos(radians(30)), sin(radians(30))], forward_speed=0.001)
      rospy.sleep(8.0)
      self.confirm_to_proceed("Go back?")

      # Go back
      self.go_to_pose_goal("b_bot", screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)

      self.go_to_named_pose("horizontal_screw_ready", "b_bot", speed=0.5, acceleration=0.5)
      self.confirm_to_proceed("Unequip tool?")
      self.go_to_named_pose("home", "b_bot", speed=0.5, acceleration=0.5)
      self.do_change_tool_action("b_bot", equip=False, screw_size = 2)  # Set screw tool
    
    # ==========================================================

    if task_name == "M3 screw":
      # self.go_to_named_pose("home", "a_bot")
      # self.go_to_named_pose("home", "b_bot")
      # self.do_change_tool_action("a_bot", equip=True, screw_size = 3)
      # self.pick_screw_from_feeder("a_bot", screw_size = 3)
      # self.go_to_named_pose("home", "a_bot")
      self.go_to_named_pose("horizontal_screw_ready", "a_bot")

      # Do an extra motion to avoid the cable
      approach_pose = geometry_msgs.msg.PoseStamped()
      approach_pose.header.frame_id = "taskboard_m3_screw_link"
      approach_pose.pose.position.x = -.04
      approach_pose.pose.position.y = -.12
      approach_pose.pose.position.z = -.05
      self.go_to_pose_goal("a_bot", approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)

      approach_pose.pose.position.y = -.0  # MAGIC NUMBER
      approach_pose.pose.position.z = -.005  # MAGIC NUMBER
      approach_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      self.go_to_pose_goal("a_bot", approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
      self.confirm_to_proceed("Screw in?")

      hole_pose = copy.deepcopy(approach_pose)
      hole_pose.pose.position.x = 0
      self.do_screw_action("a_bot", hole_pose, screw_size = 3)
      self.go_to_pose_goal("a_bot", approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True)
      self.confirm_to_proceed("Unequip tool?")
      self.go_to_named_pose("horizontal_screw_ready", "a_bot")
      self.go_to_named_pose("home", "a_bot")
      self.do_change_tool_action("a_bot", equip=False, screw_size = 3)
      self.go_to_named_pose("home", "a_bot")

    # ==========================================================

    if task_name == "M4 screw":
      
      self.go_to_named_pose("home", "b_bot")
      self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      self.pick_screw_from_feeder("b_bot", screw_size = 4)
      self.go_to_named_pose("horizontal_screw_ready","b_bot")
      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m4_screw_link"
      hole_pose.pose.position.x += .00
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
      approach_pose = copy.deepcopy(hole_pose)
      approach_pose.pose.position.x -= .03
      
      self.go_to_pose_goal("b_bot", approach_pose, speed=0.5, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)

      self.confirm_to_proceed("Proceed to screw?")
      self.do_screw_action("b_bot", hole_pose, screw_size = 4)
      self.go_to_pose_goal("b_bot", approach_pose, speed=0.5, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True)
      self.go_to_named_pose("horizontal_screw_ready", "b_bot")
      self.confirm_to_proceed("Unequip tool?")
      
      self.go_to_named_pose("home", "b_bot")
      self.do_change_tool_action("b_bot", equip=False, screw_size = 4)

    # ==========================================================

    if task_name == "pulley":
      # pick up pulley
      self.allow_collision_with_hand('b_bot', 'motor_pulley')
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.header.frame_id = "move_group/motor_pulley"
      pick_pose.pose.position = geometry_msgs.msg.Point(-0.02, 0, 0)
      pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, tau/2))
      tool_name = "motor_pulley"
      robot_name = "b_bot"
      taskboard.simple_pick("b_bot", pick_pose, item_id_to_attach="motor_pulley")
      # insert pulley
      self.allow_collision_with_hand('b_bot', 'taskboard_base')
      insert_pose = geometry_msgs.msg.PoseStamped()
      insert_pose.header.frame_id = "taskboard_small_shaft"
      insert_pose.pose.position = geometry_msgs.msg.Point(0.025, 0, 0)
      insert_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, tau/2, tau/2))
      taskboard.simple_place("b_bot", insert_pose, item_id_to_detach="motor_pulley")
      self.disallow_collision_with_hand('b_bot', 'taskboard_base')
      self.disallow_collision_with_hand('b_bot', 'motor_pulley')
      self.planning_scene_interface.allow_collisions('motor_pulley', 'taskboard_small_shaft')
      self.planning_scene_interface.allow_collisions('motor_pulley', 'taskboard_base')
      self.planning_scene_interface.allow_collisions('taskboard_base', 'taskboard_plate')
      # Go back to home position
      taskboard.go_to_named_pose("home","b_bot")
    
    # ==========================================================

    if task_name == "bearing":
      # TODO: Rewrite either with MTC or manually, so that B ends up with the bearing
      # Then rewrite like this:
      # 1. Grasp bearing with b_bot
      # 2. Place and regrasp to center it 
      # 3. Push on the taskboard with a_bot at this position:
      # 0.00063529; 0.0099509; 0.048085
      # -0.0079834; 0.71742; 0.0079052; 0.69655   (90 deg rotation around y)
      # 4. Insert with b_bot

      # Check if bearing is facing upright
      bearing_hole_pose = geometry_msgs.msg.PoseStamped()
      bearing_hole_pose.header.frame_id = "move_group/bearing/front_hole"
      bearing_hole_pose.pose.orientation.w = 1.0
      bearing_hole_pose_in_world = taskboard.listener.transformPose("workspace_center", bearing_hole_pose)
      self.allow_collision_with_hand('b_bot', 'bearing')
      if (bearing_hole_pose_in_world.pose.position.z < 0.03): # Bearing faces upward
        rospy.loginfo("Regrasp and Insert")
        # Pick up the bearing by b_bot
        rospy.loginfo("Pick up the bearing")
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "move_group/bearing"
        pick_pose.pose.position = geometry_msgs.msg.Point(-0.065, 0, 0)
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, 0, 0))
        taskboard.simple_pick("b_bot", pick_pose, item_id_to_attach="bearing", approach_height=-0.05, grasp_height=-0.05, sign=-1)
        # Hand over the bearing from b_bot to a_bot
        rospy.loginfo("Handover pose (B)")
        taskboard.go_to_named_pose("bearing_handover", "b_bot")
        rospy.loginfo("Handover pose (A)")
        handover_pose = geometry_msgs.msg.PoseStamped()
        handover_pose.header.frame_id = "b_bot_robotiq_85_tip_link"
        handover_pose.pose.position = geometry_msgs.msg.Point(0.05, 0, 0)
        handover_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, 0, tau/2))
        taskboard.go_to_pose_goal("a_bot", handover_pose, move_lin=False)
        rospy.loginfo("Handover")
        handover_pose2 = geometry_msgs.msg.PoseStamped()
        handover_pose2.header.frame_id = "b_bot_robotiq_85_tip_link"
        handover_pose2.pose.position = geometry_msgs.msg.Point(-0.01, 0, 0)
        handover_pose2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, 0, tau/2))
        taskboard.go_to_pose_goal("a_bot", handover_pose2, move_lin=False)
        taskboard.groups["b_bot"].detach_object("bearing")
        taskboard.groups["a_bot"].attach_object("bearing")
        # Move hands to avoid collision
        rospy.loginfo("Escape")
        taskboard.go_to_pose_goal("a_bot", handover_pose, move_lin=False)
        taskboard.go_to_named_pose("home","b_bot")
      else:
        # pick up bearing
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "move_group/bearing"
        pick_pose.pose.position = geometry_msgs.msg.Point(-0.03, 0.0, 0.0)
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, 0, tau/2))
        taskboard.simple_pick("a_bot", pick_pose, item_id_to_attach="bearing")
      # insert bearing
      rospy.loginfo("Insert bearing by a_bot")
      self.allow_collision_with_hand('b_bot', 'taskboard_plate')
      self.allow_collision_with_hand('b_bot', 'taskboard_bearing_target_link')
      insert_pose = geometry_msgs.msg.PoseStamped()
      insert_pose.header.frame_id = "taskboard_bearing_target_link"
      insert_pose.pose.position = geometry_msgs.msg.Point(0.02, 0.0, 0.0)
      insert_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, 0, 0))
      taskboard.simple_place("a_bot", insert_pose, item_id_to_detach="bearing")
      self.disallow_collision_with_hand('b_bot', 'bearing')
      self.disallow_collision_with_hand('b_bot', 'taskboard_bearing_target_link')
      # self.planning_scene_interface.allow_collisions('taskboard_base', 'taskboard_plate')
      self.planning_scene_interface.allow_collisions('bearing', 'taskboard_plate')
      taskboard.go_to_named_pose("home","a_bot")

    if task_name == "screw_bearing":  # Just an intermediate for debugging.
      # taskboard.equip_tool('b_bot', 'screw_tool_m4')
      # taskboard.do_change_tool_action('b_bot', equip=True, screw_size=4)
      intermediate_pose = [31.0 /180.0*3.14, -137.0 /180.0*3.14, 121.0 /180.0*3.14, -114.0 /180.0*3.14, -45.0 /180.0*3.14, -222.0 /180.0*3.14]
      for n in range(4):
        if rospy.is_shutdown():
          break
        # taskboard.go_to_named_pose("home","b_bot")
        self.move_joints("b_bot", intermediate_pose)
        taskboard.go_to_named_pose("feeder_pick_ready","b_bot")
        self.pick_screw_from_feeder("b_bot", screw_size=4)
        # taskboard.go_to_named_pose("home","b_bot")
        self.move_joints("b_bot", intermediate_pose)
        taskboard.go_to_named_pose("horizontal_screw_ready","b_bot")
        screw_pose = geometry_msgs.msg.PoseStamped()
        screw_pose.header.frame_id = "/taskboard_bearing_target_screw_" + str(n+1) + "_link"
        screw_pose.pose.position.z = -0.003  ## MAGIC NUMBER
        screw_pose.pose.orientation.w = 1.0
        screw_pose_approach = copy.deepcopy(screw_pose)
        screw_pose_approach.pose.position.x -= 0.05
        taskboard.go_to_pose_goal("b_bot", screw_pose_approach, end_effector_link = "b_bot_screw_tool_m3_tip_link", move_lin=False)
        if self.use_real_robot:
          self.do_screw_action("b_bot", screw_pose, screw_size=4)
          taskboard.go_to_pose_goal("b_bot", screw_pose_approach, end_effector_link = "b_bot_screw_tool_m3_tip_link", move_lin=False)
          taskboard.go_to_named_pose("home","b_bot")
        else:
          time.sleep(1.0)
      # taskboard.unequip_tool('b_bot', 'screw_tool_m4')
      self.move_joints("b_bot", intermediate_pose)
      # taskboard.go_to_named_pose("tool_pick_ready","b_bot")
      # taskboard.do_change_tool_action('b_bot', equip=False, screw_size=4)

      
    
    # ==========================================================

    if task_name == "shaft":
      # pick up shaft
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.header.frame_id = "move_group/drive_shaft"
      pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/2, tau/4, tau/2))
      pick_pose.pose.position = geometry_msgs.msg.Point(0.08, 0.0, -0.05)
      self.allow_collision_with_hand('b_bot', 'drive_shaft')
      taskboard.simple_pick("a_bot", pick_pose, item_id_to_attach="drive_shaft", axis="z")
      # insert shaft
      insert_pose = geometry_msgs.msg.PoseStamped()
      insert_pose.header.frame_id = "taskboard_assy_part_07_front_hole"
      insert_pose.pose.position = geometry_msgs.msg.Point(-0.04, 0.0, -0.01)
      insert_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/2))
      self.allow_collision_with_hand('b_bot', 'taskboard_assy_part_07')
      self.allow_collision_with_hand('b_bot', 'taskboard_assy_part_07_front_hole')
      taskboard.simple_place("a_bot", insert_pose, approach_height=0.1, place_height=0.1, item_id_to_detach="drive_shaft")
      self.disallow_collision_with_hand('b_bot', 'taskboard_assy_part_07')
      self.disallow_collision_with_hand('b_bot', 'taskboard_assy_part_07_front_hole')
      self.disallow_collision_with_hand('b_bot', 'drive_shaft')
      self.planning_scene_interface.allow_collisions('drive_shaft', 'taskboard_assy_part_07')
      self.planning_scene_interface.allow_collisions('taskboard_assy_part_07', 'taskboard_plate')
      # Go back to home position
      taskboard.go_to_named_pose("home","a_bot")
    
    # ==========================================================
    
    if task_name == "Idler pulley":
      rospy.logerr("Idler pulley is not implemented yet!")
    

if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    # taskboard.spawn_example_objects()
    taskboard.define_tool_collision_objects()

    i = 1
    while i and not rospy.is_shutdown():
      rospy.loginfo("Enter 1 to move robots to home")
      rospy.loginfo("Enter 11, 12 to equip/unequip nut_tool_m6")
      rospy.loginfo("Enter 13, 14, 141 to equip/unequip/discard nut_tool_m10")
      rospy.loginfo("Enter 15, 16 to equip/unequip belt placement tool")
      rospy.loginfo("Enter 191, 192 to equip/unequip m4 screw tool")
      rospy.loginfo("Enter 3, 4 to screw m3, m4 (starting from horizontal_screw_ready)")
      rospy.loginfo("Enter 51, 52... for subtasks: set screw, M3, M4, belt, motor pulley, shaft, bearing, idler pulley")
      rospy.loginfo("Enter start to start the task")
      rospy.loginfo("Enter x to exit")
      i = raw_input()

      if i == "start":
        taskboard.full_taskboard_task()
      if i == "1":
        taskboard.go_to_named_pose("home","a_bot")
        taskboard.go_to_named_pose("home","b_bot")
      if i == "11":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 66)
      if i == "12":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 66)
      if i == "15":
        taskboard.equip_unequip_belt_tool(equip=True)
      if i == "16":
        taskboard.equip_unequip_belt_tool(equip=False)
      if i == "191":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      if i == "192":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      if i == "193":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 3)
      if i == "194":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 3)
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
      if i == "58":
        taskboard.do_task("idler pulley")
      if i == "999":
        taskboard.activate_ros_control_on_ur("a_bot")
        taskboard.activate_ros_control_on_ur("b_bot")
      
      if i == "9":
        dist = .005
        taskboard.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*dist, sin(radians(30))*dist], velocity=0.03, wait=False)
      if i == "91":
        dist = -.005
        taskboard.move_lin_rel("b_bot", relative_translation=[0, -cos(radians(30))*dist, sin(radians(30))*dist], velocity=0.03, wait=False)
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
      if i == "x":
        break
      i = True

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass

