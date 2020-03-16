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
import tf_conversions
from math import pi
from math import *

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg

import o2ac_msgs
import o2ac_msgs.srv


from o2ac_routines.base import O2ACCommonBase
from o2ac.routines.helpers import is_program_running
from o2ac.routines.helpers import wait_for_UR_program

class TaskboardClass(O2ACCommonBase):
  """
  This contains the routines used to run the taskboard task.
  """
  def __init__(self):
    super(TaskboardClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")

  def set_up_item_parameters(self):
    self.item_names = ["Bearing with housing", "6 mm bearing retainer pin", "17 mm spacer for bearings", 
                      "9 mm spacer for bearings", "Rotary shaft", "4 mm round belt", 
                      "M6 Nut & Bolt", "M12 nut", "6 mm washer", 
                      "10 mm washer", "M3 set screw", "M3 bolt", 
                      "M4 bolt", "Pulley", "10 mm end cap"]
    self.item_pick_heights = [0.02, 0.02, 0.04,
                              0.047, 0.072, 0.0, 
                              0.02, 0.02, -0.002, 
                              -0.002, 0.001, 0.0,
                              0.005, 0.007, 0.001]

    self.item_place_heights = [0.02, 0.0, 0.047,
                               0.046, 0.074, 0.04, 
                               0.04, 0.04, 0.0, 
                               0.0, 0.0, 0.04, 
                               0.04, 0.006, 0.0]
    self.gripper_operation_to_use = ["outer", "inner_from_inside", "inner_from_outside", "complex_pick_from_inside", "complex_pick_from_outside"]
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    # 
    self.pick_poses = []
    self.place_poses = []
    for i in range(1,16):
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.pose.orientation = self.downward_orientation
      if i == 11:
        pick_pose.pose.orientation = self.downward_orientation_cylinder_axis_along_workspace_x
      pick_pose.header.frame_id = "mat_part" + str(i)
      # pick_pose.pose.position.z = self.item_pick_heights[i]
      self.pick_poses.append(pick_pose)

      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.pose.orientation = self.downward_orientation
      place_pose.header.frame_id = "taskboard_part" + str(i)
      
      # place_pose.pose.position.z = self.item_place_heights[i]
      self.place_poses.append(place_pose)
   

  ################ ----- Routines  
  ################ 
  ################ 
      
  def belt_circle_motion(self, robot_name, speed = 0.02, go_fast = False, rotations = 1):
    self.toggle_collisions(collisions_on=False)
    if go_fast:
      self.send_gripper_command("a_bot", "close")
      speed_fast = 1.5
      speed_slow = .1
    else:
      self.send_gripper_command("a_bot", "close")
      speed_fast = .2
      speed_slow = .02
    
    turning_around_large_pulley = True
    if turning_around_large_pulley:
      r_pulley=0.036
    else:
      r_pulley=0.019

    theta_offset = 90  # To adjust the starting angle
    theta_belt= 0 + theta_offset
    theta_increase=40

    start_pose = geometry_msgs.msg.PoseStamped()
    start_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    start_pose.header.frame_id = "workspace_center"
    start_pose = self.listener.transformPose("taskboard_part6_large_pulley", start_pose)    
    start_pose.pose.position.x = cos(radians(theta_belt))*r_pulley
    start_pose.pose.position.y = sin(radians(theta_belt))*r_pulley
    start_pose.pose.position.z = 0

    approach_pose = copy.deepcopy(start_pose)
    approach_pose.pose.position.z = 0.03
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)
    self.go_to_pose_goal(robot_name, start_pose, speed=speed_slow, move_lin=True)

    rotation_count = 0
    while rotation_count < rotations:
      rotation_count += 1
      theta_belt= 0 + theta_offset
      next_pose = geometry_msgs.msg.PoseStamped()
      next_pose.pose.orientation = start_pose.pose.orientation
      next_pose.header.frame_id = "taskboard_part6_large_pulley"
      while theta_belt <= 340+theta_offset and not rospy.is_shutdown():
          #By default, the Spiral_Search function will maintain contact between both mating parts at all times
          theta_belt=theta_belt+theta_increase
          x=cos(radians(theta_belt))*r_pulley
          y=sin(radians(theta_belt))*r_pulley
          next_pose.pose.position.x = x
          next_pose.pose.position.y = y
          print(theta_belt)
          #  print(radians(theta_belt))
          print(cos(radians(theta_belt)))
          print(cos(radians(theta_belt))*r_pulley)
          print(next_pose.pose.position)
          self.go_to_pose_goal(robot_name, next_pose, move_lin=True)
      
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)
    
    self.toggle_collisions(collisions_on=True)
    # -------------
    return True
  
  
  ####

  def full_taskboard_task(self):
    self.start_task_timer()
    self.log_to_debug_monitor("Taskboard task", "task")
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # Proposed order:
    # - Set screw 11 first, because it is affected by any movement of the mat
    # - Washers 9 and 10 early, because they are easy
    # - Spacers 3, 4 early, because they are easy and tall
    # - Pulley 14 early, because it is easy
    # 
    # - End cap 15 after washer 10
    # - Small screws 12, 13 not too late, because they may fall over, but not too early because things might be in the way
    # - Retainer pin 2 late, because we need space to pick it up and it is risky + expensive (few points, but takes time)
    # - Big nut 8 late, because it may move the mat
    # 
    # - Belt 6 after bearing 1. nut/bolt 7 anytime.
    
    # Set screw has to be first, because b_bot is right on top of it
    self.log_to_debug_monitor("No.  1 / 14: Set screw (id=11)", "subtask")
    self.confirm_to_proceed("No.  1 / 14: Set screw (id=11)")
    self.do_task_number(11) # set screw
    self.log_to_debug_monitor("No.  2 / 14: Retainer pin (id=2)", "subtask")
    self.confirm_to_proceed("No.  2 / 14: Retainer pin (id=2)")
    self.do_task_number(2)  # Retainer pin

    self.log_to_debug_monitor("No.  3 / 14: Spacer small (id=3)", "subtask")
    self.confirm_to_proceed("No.  3 / 14: Spacer small (id=3)")
    self.do_task_number(3)  # Spacer small
    self.log_to_debug_monitor("No.  4 / 14: Spacer large (id=4)", "subtask")
    self.confirm_to_proceed("No.  4 / 14: Spacer large (id=4)")
    self.do_task_number(4)  # Spacer large

    self.log_to_debug_monitor("No.  5 / 14: Washer small (id=9)", "subtask")
    self.confirm_to_proceed("No.  5 / 14: Washer small (id=9)")
    self.do_task_number(9)  # Washer small
    self.log_to_debug_monitor("No.  6 / 14: Washer large (id=10)", "subtask")
    self.confirm_to_proceed("No.  6 / 14: Washer large (id=10)")
    self.do_task_number(10) # Washer large
    self.log_to_debug_monitor("No.  7 / 14: Pulley (id=14)", "subtask")
    self.confirm_to_proceed("No.  7 / 14: Pulley (id=14)")
    self.do_task_number(14) # Pulley

    self.log_to_debug_monitor("No. 11 / 14: M4 screw (id=13)", "subtask")
    self.confirm_to_proceed("No. 11 / 14: M4 screw (id=13) pick-up only")
    self.do_task_number(131) # M4 screw 
    self.log_to_debug_monitor("No. 12 / 14: M3 screw (id=12)", "subtask")
    self.confirm_to_proceed("No. 12 / 14: M3 screw (id=12) pick-up only")
    self.do_task_number(121) # M3 screw 

    self.log_to_debug_monitor("No.  8 / 14: Bearing (id=1)", "subtask")
    self.confirm_to_proceed("No.  8 / 14: Bearing (id=1)")
    self.do_task_number(1)  # Bearing
    self.log_to_debug_monitor("No.  9 / 14: Belt (id=6)", "subtask")
    self.confirm_to_proceed("No.  9 / 14: Belt (id=6)")
    self.do_task_number(6)  # Belt

    self.log_to_debug_monitor("No. 13 / 14: M10 nut (id=8)", "subtask")
    self.confirm_to_proceed("No. 13 / 14: M10 nut (id=8)")
    self.do_task_number(8)  # M10 nut

    ### End cap has to be done before M4 screw
    self.log_to_debug_monitor("No. 10 / 14: End cap (id=15)", "subtask")
    self.confirm_to_proceed("No. 10 / 14: End cap (id=15)")
    self.do_task_number(15) # end cap
    self.log_to_debug_monitor("No. 11 / 14: M4 screw (id=13)", "subtask")
    self.confirm_to_proceed("No. 11 / 14: M4 screw (id=13)")
    self.do_task_number(132) # M4 screw?
    self.log_to_debug_monitor("No. 12 / 14: M3 screw (id=12)", "subtask")
    self.confirm_to_proceed("No. 12 / 14: M3 screw (id=12)")
    self.do_task_number(122) # M3 screw?
    self.set_feeder_power(False)

    self.log_to_debug_monitor("No. 14 / 14: M6 nut/bolt (id=7)", "subtask")
    self.confirm_to_proceed("No. 14 / 14: M6 nut/bolt (id=7)")
    self.do_task_number(7)  # M6 nut/bolt
    
  def equip_unequip_set_screw_tool(self, equip=True):
    pick_up_set_screw_tool_pose = geometry_msgs.msg.PoseStamped()
    pick_up_set_screw_tool_pose.header.frame_id = "taskboard_set_screw_tool_link"
    pick_up_set_screw_tool_pose.pose.position.x = -.005
    pick_up_set_screw_tool_pose.pose.orientation.w = 1.0
    if equip: # Pick up tool
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      taskboard.send_gripper_command(gripper="b_bot", command="close")
      self.confirm_to_proceed("Press enter to proceed.")
      pick_up_set_screw_tool_pose.pose.position.x -= .03
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .03
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")
    elif not equip: # Place tool
      pick_up_set_screw_tool_pose.pose.position.x -= .01
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .01
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      pick_up_set_screw_tool_pose.pose.position.x -= .06
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .06
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")

  def equip_unequip_belt_tool(self, equip=True):
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    belt_tool_pick_pose = geometry_msgs.msg.PoseStamped()
    belt_tool_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    belt_tool_pick_pose.header.frame_id = "belt_placement_tool"
    belt_tool_pick_pose.pose.position.z = .011
    
    belt_tool_approach = copy.deepcopy(belt_tool_pick_pose)
    belt_tool_approach.pose.position.z += .1
    
    if equip: # Pick up tool
      self.send_gripper_command("b_bot", "open")
    
    taskboard.go_to_pose_goal("b_bot", belt_tool_approach, speed=0.1, move_lin=True)
    taskboard.go_to_pose_goal("b_bot", belt_tool_pick_pose, speed=0.1, move_lin=True)

    if equip:
      self.send_gripper_command("b_bot", "close")
    else:
      self.send_gripper_command("b_bot", "open")
    
    self.confirm_to_proceed("Press enter to move back up.")
    
    taskboard.go_to_pose_goal("b_bot", belt_tool_approach, speed=0.1, move_lin=True)

  def do_task_number(self, i):
    self.log_to_debug_monitor("=== Subtask id {} start ===".format(i), "operation")

    if i == 1:
      b_bot_dx_pick = 0.0 ## MAGIC NUMBER
      b_bot_dy_pick = 0.0 ## MAGIC NUMBER
      bearing_pick_pose_b = copy.deepcopy(self.pick_poses[i-1])
      bearing_pick_pose_b.pose.position.x += b_bot_dx_pick
      bearing_pick_pose_b.pose.position.y += b_bot_dy_pick

      self.go_to_named_pose("home","a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back","c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.log_to_debug_monitor("Pick", "operation")
      self.send_gripper_command("b_bot", "open")
      self.pick("b_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                      speed_fast = 1.0, speed_slow = 0.5, gripper_command="close",
                      approach_height = 0.07)

      self.pick_poses[i-1].pose.position.z += 0.2
      self.go_to_pose_goal("b_bot", self.pick_poses[i-1], speed=0.5, move_lin=True)

      bearing_b_place_pose = copy.deepcopy(self.place_poses[i-1])  # This is part 4 ???
      bearing_b_place_pose.pose.position.x = .03
      bearing_b_place_pose.pose.position.y = .03
      bearing_b_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      bearing_b_place_pose.pose.position.z = self.item_place_heights[i-1] + .001

      bearing_b_place_pose_approach = copy.deepcopy(bearing_b_place_pose)
      bearing_b_place_pose_approach.pose.position.z = .15
      self.go_to_pose_goal("b_bot", bearing_b_place_pose_approach, speed=1.0, move_lin=True)
      self.go_to_pose_goal("b_bot", bearing_b_place_pose, speed=0.5, move_lin=True)
      self.log_to_debug_monitor("Push", "operation")
      self.do_linear_push("b_bot", 5, wait = True)
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(1.0)
      self.send_gripper_command(gripper="b_bot", command="close", force=1.0, velocity = .013)
      rospy.sleep(1.0)
      self.send_gripper_command(gripper="b_bot", command="open", velocity = .013)
      rospy.sleep(2.0)

      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      self.send_gripper_command(gripper="precision_gripper_inner", command="close")
      
      bearing_b_place_pose.pose.position.z = 0.0
      
      z_a_bot = 0.015
      bearing_a_place_pose = copy.deepcopy(bearing_b_place_pose)
      bearing_a_place_pose.pose.orientation = self.place_poses[i-1].pose.orientation
      self.log_to_debug_monitor("Place", "operation")
      self.place("a_bot", bearing_a_place_pose, z_a_bot,
                              speed_fast = 0.5, speed_slow = 0.02, gripper_command="none",
                              approach_height = 0.05, lift_up_after_place = False)
      bearing_a_place_pose_final = copy.deepcopy(self.place_poses[i-1])
      bearing_a_place_pose_final.pose.position.z = z_a_bot
      rospy.loginfo("Moving bearing to final pose")
      self.go_to_pose_goal("a_bot", bearing_a_place_pose_final, speed=.5, move_lin=True)
      self.horizontal_spiral_motion("a_bot", max_radius=.0065, radius_increment = .004)
      bearing_a_place_pose_retreat = copy.deepcopy(bearing_a_place_pose_final)
      bearing_a_place_pose_retreat.pose.position.z += .05
      self.go_to_pose_goal("a_bot", bearing_a_place_pose_retreat, speed=.5, move_lin=True)
      self.go_to_named_pose("home","a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      # ### Push with b_bot
      ### Pushing with b_bot only does not work. The tool needs to be grasped beforehand.
      self.log_to_debug_monitor("Push", "operation")
      self.go_to_pose_goal("b_bot", bearing_a_place_pose_retreat, speed=1.0, move_lin=True)
      self.send_gripper_command(gripper="b_bot", command=0.035)
      self.log_to_debug_monitor("Push", "operation")
      self.do_linear_push("b_bot", 20, wait = True)
      self.go_to_pose_goal("b_bot", bearing_a_place_pose_retreat, speed=1.0, move_lin=True)
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      rospy.loginfo("Done")
      # TODO: Make sure the task succeeded by pushing with b_bot and plate 3

    if i == 2: #unadjusted
      self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      ###set the tool
      tool_pickup_pose = geometry_msgs.msg.PoseStamped()
      tool_pickup_pose.header.frame_id = "retainer_pin_insertion_tool"
      tool_pickup_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      tool_grasped_height = 0.03
      self.log_to_debug_monitor("Pick", "operation")
      self.pick("b_bot",tool_pickup_pose, tool_grasped_height,
                          speed_fast = 1.0, speed_slow = 1.0, gripper_command="close",
                          approach_height = 0.1, lift_up_after_pick=False)
      self.confirm_to_proceed("Confirm that pin tool was picked")
      tool_pickup_pose_high = copy.deepcopy(tool_pickup_pose)
      tool_pickup_pose_high.pose.position.z += .2
      self.go_to_pose_goal("b_bot", tool_pickup_pose_high, speed=1.0, move_lin=True)

      tool_place_approach = copy.deepcopy(self.place_poses[i-1])
      tool_place_approach.pose.position.z += .15
      self.go_to_pose_goal("b_bot", tool_place_approach, speed=1.0, move_lin=True)
      self.place("b_bot",self.place_poses[i-1], tool_grasped_height + .001,
                      speed_fast = 1.0, speed_slow = 0.5, gripper_command="none",
                      approach_height = 0.05, lift_up_after_place = False)

      self.send_gripper_command(gripper="b_bot", command=.02, velocity = .013)
      rospy.sleep(2.0)
      self.send_gripper_command(gripper="b_bot", command="open")
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      ### a_bot screw pickup/place routine
      pickup_pin_with_a_bot = False
      if pickup_pin_with_a_bot:
        ###pick up screw
        inclined_pick_pose = copy.deepcopy(self.pick_poses[i-1])
        inclined_pick_pose.pose.position.x += -.025
        inclined_pick_pose.pose.position.z += .01
        inclined_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/5, pi))

        self.pick("a_bot",inclined_pick_pose, 0.00,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                                approach_height = 0.05, special_pick = False)
        inclined_pick_pose_high = copy.deepcopy(inclined_pick_pose)
        inclined_pick_pose_high.pose.position.z += 0.2
        self.go_to_pose_goal("a_bot", inclined_pick_pose_high, speed=0.1, move_lin=True)
        
        ###place the pin
        pin_place_approach = copy.deepcopy(self.place_poses[i-1])
        pin_place_approach.pose.position.x += -.025
        pin_place_approach.pose.position.z = 0.2
        pin_place_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/5, pi))
        self.go_to_pose_goal("a_bot", pin_place_approach, speed=0.1, move_lin=True)
        
        pin_place_pose = copy.deepcopy(pin_place_approach)
        pin_place_pose.pose.position.z = 0.07
        self.go_to_pose_goal("a_bot", pin_place_pose, speed=0.1, move_lin=True)
        pin_place_pose.pose.position.x = 0

        self.send_gripper_command(gripper="precision_gripper_inner", command="open")

        pin_place_pose.pose.position.y = 0.0
        pin_place_pose.pose.position.z += 0.1
        pin_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        self.go_to_pose_goal("a_bot", pin_place_pose, speed=0.1)
        self.go_to_named_pose("home","a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      else:
        pin_pick_approach = copy.deepcopy(self.pick_poses[i-1])
        pin_pick_approach.pose.position.z += .15

        self.go_to_pose_goal("b_bot", pin_pick_approach, speed=1.0, move_lin=True)
        self.pick("b_bot", self.pick_poses[i-1], grasp_height=0.02,
                                speed_fast = 1.0, speed_slow = 0.2, gripper_command="",
                                approach_height = 0.1, special_pick = False)
        self.go_to_pose_goal("b_bot", pin_pick_approach, speed=1.0, move_lin=True)

        self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.adjust_centering(go_fast=True)

        pin_place_approach = copy.deepcopy(self.place_poses[i-1])
        pin_place_approach.pose.position.z += 0.2

        self.go_to_pose_goal("b_bot", pin_place_approach, speed=0.1, move_lin=True)
        self.place("b_bot",self.place_poses[i-1], place_height=.01+.06,
                      speed_fast = 1.0, speed_slow = 0.1, gripper_command="open",
                      approach_height = 0.1, lift_up_after_place = False)
        self.send_gripper_command(gripper="b_bot", command=.05)
        rospy.sleep(.5)
        # self.go_to_pose_goal("b_bot", pin_place_approach, speed=0.1, move_lin=True)

      ### Lift tool with b_bot and do spiral motion
      b_pose = copy.deepcopy(self.place_poses[i-1])
      b_pose.pose.position.z = 0.15
      
      # self.go_to_pose_goal("b_bot", b_pose, speed=1.0)
      b_pose.pose.position.z -= 0.12
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0)
      self.send_gripper_command(gripper="b_bot", command="close")
      rospy.sleep(1.0)
      
      b_pose.pose.position.z += 0.01
      self.go_to_pose_goal("b_bot", b_pose, speed=0.2)
      self.horizontal_spiral_motion("b_bot", max_radius =.005, radius_increment = .005)

      b_pose.pose.position.z = 0.1
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
        
      ### Push in the pin
      b_pose.pose.position.x += 0.004
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
      self.do_linear_push("b_bot", 5, wait = True)
      b_pose.pose.position.z += 0.1
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
      
      ### Discard the tool
      self.go_to_named_pose("discard_taskboard_tool","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(3.0)
      
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if i == 3:   # Spacer large
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                              speed_fast = 0.1, speed_slow = 0.015, acc_fast = .05, acc_slow=.1, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07)
      # self.go_to_named_pose("taskboard_intermediate_pose", "a_bot", speed=0.07, acceleration=.05)
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.15, speed_slow = 0.02, acc_fast = .1, acc_slow=.1, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07, lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    if i == 4:    # Spacer small
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                              speed_fast = 0.3, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07)
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.3, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07, lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", max_radius=.004, radius_increment=.002)

    if i == 5:
      rospy.loginfo("Part 5 was deleted and is skipped.")
      pass

    if i == 6: # Belt
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      # Set the placement aid
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      belt_tool_pick_pose = geometry_msgs.msg.PoseStamped()
      belt_tool_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      belt_tool_pick_pose.header.frame_id = "belt_placement_tool"
      belt_tool_grasp_height = .042
      self.send_gripper_command("b_bot", .013)
      self.send_gripper_command("b_bot", "open")
      self.pick("b_bot", belt_tool_pick_pose, grasp_height=belt_tool_grasp_height, speed_fast = 1.0, speed_slow = 1.0, gripper_command="close",
                              approach_height = 0.15)
      
      # self.go_to_named_pose("home", "b_bot")

      ### OPTIONAL, UNTESTED: Push on the bearing
      # bearing_push_pose = copy.deepcopy(self.place_poses[i-1])
      # bearing_push_pose.pose.position.z = .06
      # bearing_push_pose_high = copy.deepcopy(bearing_push_pose)
      # bearing_push_pose.pose.position.z = .15
      # self.log_to_debug_monitor("Push", "operation")
      # self.go_to_pose_goal("b_bot", bearing_push_pose_high, speed=0.15, move_lin=True)
      # self.go_to_pose_goal("b_bot", bearing_push_pose, speed=0.15, move_lin=True)
      # self.send_gripper_command(gripper="b_bot", command=0.035)
      # self.log_to_debug_monitor("Push", "operation")
      # self.do_linear_push("b_bot", 20, wait = True)
      # self.go_to_pose_goal("b_bot", bearing_push_pose_high, speed=0.15, move_lin=True)
      # self.go_to_named_pose("home","b_bot")

      ### Place the tool
      belt_tool_place_pose = geometry_msgs.msg.PoseStamped()
      belt_tool_place_pose.header.frame_id = "taskboard_part6_small_pulley"
      belt_tool_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      belt_tool_place_pose.pose.position.y = .055
      belt_tool_place_pose.pose.position.z = belt_tool_grasp_height -.005

      belt_tool_place_pose_approach = copy.deepcopy(belt_tool_place_pose)
      belt_tool_place_pose_approach.pose.position.z += .15

      self.move_lin("b_bot", belt_tool_place_pose_approach, speed=1.0)
      self.move_lin("b_bot", belt_tool_place_pose, speed=.3)

      self.do_linear_push("b_bot", direction="X+", force=4, wait=True)
      self.send_gripper_command("b_bot", .01)
      rospy.sleep(1.0)
      self.send_gripper_command("b_bot", "open")
      self.move_lin("b_bot", belt_tool_place_pose_approach, speed=1.0)

      self.go_to_named_pose("taskboard_center_pose", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Pick up the belt
      belt_pick_pose = copy.deepcopy(self.pick_poses[5])
      belt_pick_pose.pose.position.y += .055
      self.pick("b_bot", belt_pick_pose, grasp_height=0.0,
                      speed_fast = 1.0, speed_slow = 0.1, gripper_command="close")
      self.go_to_named_pose("taskboard_center_pose", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Place the belt
      belt_place_intermediate = geometry_msgs.msg.PoseStamped()
      belt_place_intermediate.header.frame_id = "taskboard_part6_large_pulley"
      belt_place_intermediate.pose.position.y = .05
      belt_place_intermediate.pose.position.z = .03
      belt_place_intermediate.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

      belt_place_approach = copy.deepcopy(belt_place_intermediate)
      belt_place_approach.pose.position.z = .09
      belt_place_approach_start = copy.deepcopy(belt_place_approach)
      belt_place_approach_start.pose.position.y = .01
      belt_place_approach_high = copy.deepcopy(belt_place_approach_start)
      belt_place_approach_high.pose.position.z = .18

      belt_place_pose_final = geometry_msgs.msg.PoseStamped()
      belt_place_pose_final.header.frame_id = "taskboard_part6_large_pulley"
      belt_place_pose_final.pose.position.y = .01
      belt_place_pose_final.pose.position.z = .0085
      belt_place_pose_final.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

      self.go_to_pose_goal("b_bot", belt_place_approach_high, speed=1.0)
      self.go_to_pose_goal("b_bot", belt_place_approach_start, speed=0.5)
      self.go_to_pose_goal("b_bot", belt_place_approach, speed=0.3)
      self.go_to_pose_goal("b_bot", belt_place_intermediate, speed=0.3)
      self.go_to_pose_goal("b_bot", belt_place_pose_final, speed=0.1)
      
      self.send_gripper_command(gripper="b_bot", command=.01)
      rospy.sleep(1)
      self.send_gripper_command(gripper="b_bot", command="open")
      belt_place_retreat = copy.deepcopy(belt_place_pose_final)
      belt_place_retreat.pose.position.z += .03
      self.go_to_pose_goal("b_bot", belt_place_retreat, speed=1.0)
      self.go_to_named_pose("back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Fiddle in the belt
      rospy.logwarn("Doing belt spiral motion")
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.belt_circle_motion("a_bot", rotations=2, go_fast=True)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Remove the placement aid
      self.go_to_named_pose("taskboard_center_pose", "b_bot")
      belt_tool_place_pose.pose.position.z -= .03
      self.pick("b_bot", belt_tool_place_pose, grasp_height=belt_tool_grasp_height, speed_fast = 1.0, speed_slow = 1.0, gripper_command="close",
                              approach_height = 0.15)
      
      # Drop the tool
      self.go_to_named_pose("discard_taskboard_tool","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(3.0)
      
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      # self.place("b_bot", belt_tool_place_pose, place_height=belt_tool_grasp_height+.002, speed_fast = 0.2, speed_slow = 0.03, gripper_command="open",
      #                         approach_height = 0.03, lift_up_after_place = True)

    if i == 7:
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      # Pick up M6 screw, arrange it in the gripper, and pick it with b_bot
      partScrew = geometry_msgs.msg.PoseStamped()
      partScrew.header.frame_id = "mat_part7_1"
      partScrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi*30/180, +pi/2))
      partScrew.pose.position.y += .013  # KIND OF MAGIC NUMBER (increasing it moves a_bot forward)
      self.pick("a_bot",partScrew, grasp_height=.01,
                      speed_fast = 0.2, speed_slow = 0.02, gripper_command="close",
                      approach_height = 0.05)
      # self.tilt_up_gripper(speed_fast=0.1, speed_slow=0.02, screw_size = 6)
      self.confirm_to_proceed("did the tilt work?")

      self.do_change_tool_action("b_bot", equip=True, screw_size = 6)        
      self.confirm_to_proceed("Pick screw from gripper?")
      self.pick_screw_from_precision_gripper(screw_size=6, robot_name="b_bot")
      self.confirm_to_proceed("Screw picked?")
      self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Pick the nut with a_bot, place it near c_bot
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      nut_pick_pose = geometry_msgs.msg.PoseStamped()
      nut_pick_pose.header.frame_id = "mat_part7_2"
      nut_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      self.confirm_to_proceed("Pick the nut with a_bot now?")
      self.pick("a_bot",nut_pick_pose, 0.0,
                                  speed_fast = 0.31, speed_slow = 0.05, gripper_command="inner_gripper_from_inside",
                                  approach_height = 0.05)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      nut_place_a_bot = geometry_msgs.msg.PoseStamped()
      nut_place_a_bot.header.frame_id = "workspace_center"
      nut_place_a_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      nut_place_a_bot.pose.position.x = -.25
      nut_place_a_bot.pose.position.y = -.32

      nut_pick_c_bot = copy.deepcopy(nut_place_a_bot)
      nut_pick_c_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi*3/4))

      self.place("a_bot",nut_place_a_bot,0.0,
                                  speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                  approach_height = 0.05,lift_up_after_place = True)
      self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      self.confirm_to_proceed("Pick the nut with c_bot now?")
      # Pick up the nut with c_bot
      self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.do_change_tool_action("c_bot", equip=True, screw_size=66)  #66 = nut tool m6
      self.go_to_named_pose("feeder_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.pick_nut_from_table("c_bot", object_pose=nut_pick_c_bot,end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Position b_bot screw tool (via intermediate pose?)
      # self.go_to_named_pose("taskboard_screw_tool_horizontal_approach", "b_bot")  #TODO
      self.go_to_named_pose("screw_ready", "b_bot", speed=self.speed_fast, acceleration=self.acc_fast, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("horizontal_screw_ready", "b_bot", speed=self.speed_fast, acceleration=self.acc_fast, force_ur_script=self.use_real_robot)
      screw_tool_hold = geometry_msgs.msg.PoseStamped()
      screw_tool_hold.header.frame_id = "taskboard_part7_1"
      screw_tool_hold.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, 0, 0))
      screw_tool_hold.pose.position.x = .01
      screw_tool_hold_approach = copy.deepcopy(screw_tool_hold)
      screw_tool_hold_approach.pose.position.x = -.01
      screw_tool_hold_approach_high = copy.deepcopy(screw_tool_hold_approach)
      screw_tool_hold_approach_high.pose.position.z = +.05
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach_high, speed=.1, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.horizontal_spiral_motion("b_bot", .004, radius_increment = .002, spiral_axis="Y")
      self.go_to_pose_goal("b_bot", screw_tool_hold, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")

      self.confirm_to_proceed("Is b_bot screw positioned well?")

      # Position c_bot for the nut fastening
      self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      nut_tool_hold = geometry_msgs.msg.PoseStamped()
      nut_tool_hold.header.frame_id = "taskboard_part7_2"
      nut_tool_hold.pose.orientation.w = 1.0
      nut_tool_hold_approach = copy.deepcopy(nut_tool_hold)
      nut_tool_hold_approach.pose.position.x = -.05
      nut_tool_hold_approach_high = copy.deepcopy(nut_tool_hold_approach)
      nut_tool_hold_approach_high.pose.position.z = +.05

      # self.go_to_named_pose("taskboard_nut_tool_approach", "c_bot")  #TODO
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach_high, speed=.3, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.confirm_to_proceed("Went to approach high with c_bot nut tool. Descend?")
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach, speed=.3, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      
      # Fix the nut with the nut tool
      self.set_motor("nut_tool_m6", direction = "tighten", wait=False, speed = 800, duration = 15)
      # self.set_motor("screw_tool_m6", direction = "tighten", wait=False, speed = 500, duration = 15)
      self.go_to_pose_goal("c_bot", nut_tool_hold, speed=.01, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      # self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, spiral_axis="YZ")
      # self.go_to_pose_goal("c_bot", nut_tool_hold, speed=.01, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")

      rospy.sleep(2.0)

      # Move back with b_bot first
      self.set_suction("screw_tool_m6", False, False)
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach, speed=.05, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach_high, speed=.05, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_named_pose("screw_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Move back with c_bot, unequip
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach, speed=.05, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach_high, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.do_change_tool_action("c_bot", equip=False, screw_size=66)  #66 = nut tool m6

      # Unequip the tool with b_bot
      self.do_change_tool_action("b_bot", equip=False, screw_size=6)

    if i == 8:
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      #pick up the tool
      tool_pose = geometry_msgs.msg.PoseStamped()
      tool_pose.header.frame_id = "M10nut_tool"
      tool_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      tool_grasped_height = 0.042

      tool_pose_high = copy.deepcopy(tool_pose)
      tool_pose_high.pose.position.z += .2

      ## MAGIC NUMBERS!!
      b_bot_dx_pick = 0.0
      b_bot_dy_pick = 0.0
      # b_bot_dx_place = 0.0
      b_bot_dy_place = 0.0

      pick_pose_low = copy.deepcopy(self.pick_poses[i-1])
      pick_pose_low.pose.position.x += b_bot_dx_pick
      pick_pose_low.pose.position.y += b_bot_dy_pick
      pick_pose_low.pose.position.z = self.item_pick_heights[i-1] + tool_grasped_height + .01
      pick_pose_high = copy.deepcopy(pick_pose_low)
      pick_pose_high.pose.position.z += .15

      place_pose_low = copy.deepcopy(self.place_poses[i-1])
      place_pose_low.pose.position.y += b_bot_dy_place
      place_pose_low.pose.position.z = .02 + tool_grasped_height + .01
      place_pose_high = copy.deepcopy(place_pose_low)
      place_pose_high.pose.position.z += .15
      
      self.send_gripper_command("b_bot", "open")
      self.pick("b_bot",tool_pose, tool_grasped_height,
                              speed_fast = 0.5, speed_slow = 0.2, gripper_command="close",
                              approach_height = 0.05, lift_up_after_pick=False)
      self.confirm_to_proceed("Confirm that the m10 nut tool is grasped!")
      self.go_to_pose_goal("b_bot", tool_pose_high, speed=1.5, move_lin=True)

      # Push into the nut to pick it up
      self.go_to_pose_goal("b_bot", pick_pose_high, speed=1.5, move_lin=True)
      self.confirm_to_proceed("High above the nut. go low?")
      self.go_to_pose_goal("b_bot", pick_pose_low, speed=1.5, move_lin=True)
      self.do_linear_push("b_bot", 10, wait = True)
      self.horizontal_spiral_motion("b_bot", max_radius = .006, radius_increment = .005)
      self.do_linear_push("b_bot", 40, wait = True)
      rospy.sleep(1.0)

      self.go_to_pose_goal("b_bot", pick_pose_high, speed=1.0, move_lin=True)
      #place and fasten
      self.go_to_pose_goal("b_bot", place_pose_high, speed=1.0, move_lin=True)
      self.confirm_to_proceed("High above the place. go low?")
      self.go_to_pose_goal("b_bot", place_pose_low, speed=0.1, move_lin=True)        

      self.do_nut_fasten_action("m10_nut", wait = False)
      self.do_linear_push("b_bot", 10, wait = True)
      self.horizontal_spiral_motion("b_bot", max_radius = .003, radius_increment = .004)
      rospy.sleep(4.0)
      self.do_nut_fasten_action("m10_nut", wait = False)
      self.do_linear_push("b_bot", 10, wait = True)
      rospy.sleep(8.0)
      self.do_nut_fasten_action("none", wait = False)

      # Go back up
      self.go_to_pose_goal("b_bot", place_pose_high, speed=0.1, move_lin=True)
      # Drop the tool
      self.go_to_named_pose("discard_taskboard_tool","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(3.0)
      
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if i in [9, 10]:
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1], approach_height = 0.03,
                              speed_fast = 0.5, speed_slow = 0.05, gripper_command="easy_pick_only_inner")
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1]-.001, approach_height = 0.03,
                              speed_fast = 0.5, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                              lift_up_after_place = False)
      if i == 9:
        self.horizontal_spiral_motion("a_bot", .004, radius_increment=0.003)
      if i == 10:
        self.horizontal_spiral_motion("a_bot", .007, radius_increment=0.004)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    
    if i == 11:      #set screw
      use_cushioned_tool = False
      if use_cushioned_tool:
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move_rel"
        req.robot_name = "b_bot"
        req.relative_translation.y = .002
        req.velocity = .005
        res = self.urscript_client.call(req)
        wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(4.0))
        # self.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .002)

        self.confirm_to_proceed("Turn on motor?")
        
        self.set_motor("set_screw_tool", "tighten", duration = 5.0)
        self.do_nut_fasten_action("set_screw", wait = False)
        rospy.sleep(5.0)

        self.confirm_to_proceed("go down with motor?")

        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move_rel"
        req.robot_name = "b_bot"
        req.relative_translation.y = .002
        req.velocity = .005
        res = self.urscript_client.call(req)
        wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(4.0))

        self.confirm_to_proceed("Go back up?")
      else: # Use the fat motor tool
        self.do_linear_push("b_bot", 4, direction="Y+", wait = True)
        rospy.sleep(1.0)
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move_rel"
        req.robot_name = "b_bot"
        req.relative_translation.y = -.0005
        req.velocity = .005
        res = self.urscript_client.call(req)
        wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(4.0))

        # self.confirm_to_proceed("Pushed. Turn on motor?")
        
        self.do_nut_fasten_action("set_screw", wait = False)
        rospy.sleep(1.5)

        # self.confirm_to_proceed("go down with motor?")

        self.do_linear_push("b_bot", 4, direction="Y+", wait = True)
        rospy.loginfo("Pushing in")
        rospy.sleep(4.0)
        self.do_nut_fasten_action("turn_all_off", wait = False)

        self.confirm_to_proceed("Go back up?")

      # Push on taskboard with c_bot
      self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command("c_bot", "close")
      push_pose_c = copy.deepcopy(self.place_poses[0])
      push_pose_c.pose.position.x = .05
      push_pose_c.pose.position.z = .03
      self.go_to_pose_goal("c_bot", push_pose_c, speed=1.0, move_lin=True)
      self.do_linear_push("c_bot", 5, wait = True)

      # Go up with b_bot tool
      req = o2ac_msgs.srv.sendScriptToURRequest()
      req.program_id = "lin_move_rel"
      req.robot_name = "b_bot"
      req.relative_translation.y = -.1
      req.velocity = .05
      res = self.urscript_client.call(req)
      wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(5.0))

      # Move away with c_bot tool
      self.go_to_pose_goal("c_bot", push_pose_c, speed=1.0, move_lin=True)
      self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      self.go_to_named_pose("discard_taskboard_tool","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command("b_bot", .06)
      self.send_gripper_command("b_bot", "open")
      rospy.sleep(3.0)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    if i in [12, 13, 121, 122, 131, 132]:
      if i in [12, 121, 122]:
        part_id = 12
        screw_size = 3
      else:
        part_id = 13
        screw_size = 4
      
      if i in [12, 13, 121, 131]:
        self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        screw_pick_pose = copy.deepcopy(self.pick_poses[part_id-1])
        screw_pick_pose.pose.position.y += .01
        screw_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi*45/180, pi*90/180))
        self.pick("a_bot",screw_pick_pose,0.001,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                                approach_height = 0.05, special_pick = False)
        
        self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
        # If we decide to use the feeder, there is self.place_screw_in_feeder(screw_size) and self.pick_screw_from_feeder(screw_size)
        self.put_screw_in_feeder(screw_size)
        self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      if i in [12, 13, 122, 132]:
        self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        #pick up the screw tool
        self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.do_change_tool_action("c_bot", equip=True, screw_size = screw_size)
        
        #pick up the screw from feeder
        self.pick_screw_from_feeder(screw_size)
        self.go_to_named_pose("screw_ready_high", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

        #screw on the cap
        screw_approach = copy.deepcopy(self.place_poses[part_id-1])
        screw_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, 0, 0))

        if i == 12:
          screw_approach.pose.position.z -= .002  #MAGIC NUMBER!
          screw_approach.pose.position.y -= .001  #MAGIC NUMBER!
        elif i == 13:
          screw_approach.pose.position.z -= .003  #MAGIC NUMBER!
          screw_approach.pose.position.y -= .001  #MAGIC NUMBER!
        
        screw_approach.pose.position.x -= 0.03
        self.go_to_pose_goal("c_bot", screw_approach, speed=0.5, end_effector_link="c_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)
        print(screw_approach)
        self.confirm_to_proceed("Proceed to screw_pose?")

        screw_pose = copy.deepcopy(screw_approach)
        screw_pose.pose.position.x = 0.007
        if screw_size == 3:
          screw_height = 0.007
        elif screw_size == 4:
          screw_height = 0.01
        self.do_screw_action("c_bot", screw_pose, screw_height = screw_height, screw_size = screw_size, stay_put_after_screwing=True)

        if screw_size == 3:
          screw_pose_manual_followup = copy.deepcopy(screw_approach)
          screw_pose_manual_followup.pose.position.x = 0.01
          
          self.set_motor("screw_tool_m3", direction = "tighten", wait=False, speed = 800, duration = 5)
          # self.horizontal_spiral_motion("c_bot", max_radius = .002, radius_increment = .001, spiral_axis="YZ")
          rospy.sleep(5.0)
          self.set_suction("screw_tool_m3", False, False)
          self.set_motor("screw_tool_m3", direction = "loosen", wait=False, speed = 300, duration = .5)  # Supposed to fix the bit getting stuck

        self.go_to_pose_goal("c_bot", screw_approach, speed=0.05, end_effector_link="c_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)

        self.go_to_named_pose("screw_ready_high", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.do_change_tool_action("c_bot", equip=False, screw_size = screw_size)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if i == 14:
      # self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1], approach_height = 0.05,
      #                         speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
      self.send_gripper_command("a_bot","close")
      approach_pose = copy.deepcopy(self.pick_poses[i-1])
      approach_pose.pose.position.z = 0.03
      self.go_to_pose_goal("a_bot", approach_pose, speed=1.0, move_lin=True)
      pickup_pose = copy.deepcopy(self.pick_poses[i-1])
      pickup_pose.pose.position.z = 0.007
      self.go_to_pose_goal("a_bot", pickup_pose, speed=0.05, move_lin=True)
      self.send_gripper_command("a_bot","open")

      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1], approach_height = 0.05,
                              speed_fast = 1.0, speed_slow = 0.04, gripper_command="easy_pick_only_inner",
                              lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", .002, radius_increment = .001)

      pose = copy.deepcopy(self.place_poses[i-1])
      pose.pose.position.z += .05
      self.go_to_pose_goal("a_bot", pose, speed=0.5, move_lin=True)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      # TODO: Try pushing with the a_bot's open inner gripper (it would save time)
      # Push down with the c_bot in case it is blocked
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command(gripper="b_bot", command=0.02)
      # TODO: Turn the pose around by 180 degrees to speed up the motion
      self.place("b_bot",self.place_poses[i-1],self.item_place_heights[i-1] - .01, approach_height = 0.02,
                      speed_fast = 1.0, speed_slow = 0.5, gripper_command="none",
                      lift_up_after_place = True)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if i == 15: 
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.send_gripper_command("b_bot", "open")
      self.pick("b_bot",self.pick_poses[i-1], self.item_pick_heights[i-1],
                      speed_fast = 1.0, speed_slow = 0.05, gripper_command="close",
                      approach_height = 0.04)
      
      handover_b = geometry_msgs.msg.PoseStamped()
      handover_b.header.frame_id = "workspace_center"
      handover_b.pose.position.x = 0.2
      handover_b.pose.position.y = .0
      handover_b.pose.position.z = 0.7
      handover_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, -pi/2))
      

      handover_a_approach = copy.deepcopy(handover_b)
      handover_a_approach.pose.position.y -= 0.05
      handover_a_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pi/2))
    
      self.go_to_pose_goal("b_bot", handover_b, speed=0.2)
      self.go_to_pose_goal("a_bot", handover_a_approach, speed=0.12)

      # Applied to the a_bot in workspace_center coordinates
      magic_x_offset = .001  
      magic_z_offset = .001

      handover_a  = copy.deepcopy(handover_a_approach)
      handover_a.pose.position.y += 0.05
      handover_a.pose.position.y += 0.023  # This is how much the gripper is pushed in
      handover_a.pose.position.x += magic_x_offset # MAGIC
      handover_a.pose.position.z += magic_z_offset # MAGIC

      self.send_gripper_command(gripper="a_bot", command="close")
      self.go_to_pose_goal("a_bot", handover_a, speed=0.03, move_lin= True)
      self.confirm_to_proceed("Is the gripper centered and in the end cap?")
      self.horizontal_spiral_motion("a_bot", .003, radius_increment = .001)
      self.send_gripper_command(gripper="a_bot", command="open")
      
      handover_b_retreat = copy.deepcopy(handover_a)
      handover_b_retreat.pose.position.y -= 0.017
      handover_b_retreat.pose.position.x -= magic_x_offset # MAGIC
      handover_b_retreat.pose.position.z -= magic_z_offset #MAGIC
      handover_b_retreat.pose.position.y += 0.03
      handover_b_retreat.pose.orientation = handover_b.pose.orientation
      
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(1.0)
      self.go_to_pose_goal("b_bot", handover_b_retreat, speed=0.02, move_lin= True)
      self.go_to_named_pose("back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      self.place("a_bot", self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.05, lift_up_after_place = False)

      #  self.send_gripper_command(gripper="precision_gripper_inner", command="open")
      self.horizontal_spiral_motion("a_bot", .001, radius_increment = .001)
      p = copy.deepcopy(self.place_poses[i-1])
      p.pose.position.z += 0.02
      self.go_to_pose_goal("a_bot", p, speed=0.02, move_lin= True)
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      #push with b
      self.send_gripper_command(gripper="b_bot", command="close")
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_pose_goal("b_bot", p, speed=1.0)
      self.do_linear_push("b_bot", 3, wait = True)
      self.go_to_pose_goal("b_bot", p, speed=1.0)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.log_to_debug_monitor("=== Subtask id {} end ===".format(i), "operation")

if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    taskboard.set_up_item_parameters()

    i = 1
    while(i):
      rospy.loginfo("Enter 11, 12, 121 to equip/unequip/discard nut_tool_m6")
      rospy.loginfo("Enter 13, 14, 141 to equip/unequip/discard nut_tool_m10")
      rospy.loginfo("Enter 15, 16 to equip/unequip belt placement tool")
      rospy.loginfo("Enter 17, 18, 181 to equip/unequip/discard retainer pin guide tool")
      rospy.loginfo("Enter 191, 192 to equip/unequip m4 screw tool")
      rospy.loginfo("Enter 2 to move robots to competition home (b_bot does not move)")
      rospy.loginfo("Enter 20 to move robots to home (b_bot moves home, too)")
      rospy.loginfo("Enter 21, 22 to move b_bot to set_screw_insert_pose / screw_ready")
      rospy.loginfo("Enter 31 to do m4 screw handover with b_bot")
      rospy.loginfo("Enter 32 to do m4 screw handover with b_bot")
      rospy.loginfo("Enter 40 to do a spiral motion with a_bot")
      rospy.loginfo("Enter 41 to do the belt circle motion with a_bot (this will not move to the pulley)")
      rospy.loginfo("Enter 42 to do the belt circle motion with two rotations")
      rospy.loginfo("Enter 5 to move the m6 nut tool to taskboard_part10")
      rospy.loginfo("Enter 82 to pick part 4 and tilt gripper up")
      rospy.loginfo("Enter 91, 92,... 915 to perform part 1, 2,... 15")
      rospy.loginfo("Enter start to start the task")
      rospy.loginfo("Enter x to exit")
      i = raw_input()

      if i == "start":
        taskboard.full_taskboard_task()
      if i == "11":
        taskboard.do_change_tool_action("c_bot", equip=True, screw_size = 66)
      if i == "12":
        taskboard.do_change_tool_action("c_bot", equip=False, screw_size = 66)
      if i == "15":
        taskboard.equip_unequip_belt_tool(equip=True)
      if i == "16":
        taskboard.equip_unequip_belt_tool(equip=False)
      if i == "17":
        tool_pickup_pose = geometry_msgs.msg.PoseStamped()
        tool_pickup_pose.header.frame_id = "retainer_pin_insertion_tool"
        tool_pickup_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
        tool_pickup_pose.pose.position.z = 0.03
        taskboard.go_to_pose_goal("b_bot", tool_pickup_pose, speed=.1)
      if i == "191":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      if i == "192":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      if i == "193":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 3)
      if i == "194":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 3)
      if i == "2":
        taskboard.go_to_named_pose("home","a_bot")
        taskboard.go_to_named_pose("home","c_bot")
      if i == "20":
        taskboard.go_to_named_pose("home","a_bot")
        taskboard.go_to_named_pose("home","b_bot")
        taskboard.go_to_named_pose("back","c_bot")
      if i == "21":
        taskboard.go_to_named_pose("set_screw_insert_pose", "b_bot")
      if i == "22":
        taskboard.go_to_named_pose("screw_ready", "b_bot")
      if i == "31":
        taskboard.pick_screw_from_precision_gripper(screw_size=4, robot_name="b_bot")
      if i == "32":
        taskboard.pick_screw_from_precision_gripper(screw_size=3, robot_name="b_bot")
      if i == "4":
        taskboard.go_to_named_pose("discard_taskboard_tool","b_bot", speed=1.0)
      if i == "5":
        taskboard.go_to_named_pose("home", "c_bot")
        place_pose = geometry_msgs.msg.PoseStamped()
        place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        place_pose.pose.position.z = .01
        place_pose.header.frame_id = "taskboard_part10"
        taskboard.go_to_pose_goal("c_bot", place_pose, speed=.1, move_lin=True, end_effector_link="c_bot_nut_tool_m6_tip_link")
        taskboard.confirm_to_proceed("Calibration good?")
        taskboard.go_to_named_pose("home", "c_bot")
      if i == "40":
        taskboard.horizontal_spiral_motion("a_bot", .05)
      if i == "41":
        taskboard.belt_circle_motion("a_bot")
      if i == "42":
        taskboard.belt_circle_motion("a_bot", rotations=2)
      if i == "82":
        taskboard.pick("a_bot",taskboard.pick_poses[3],taskboard.item_pick_heights[3]-0.026-0.06,
                                 speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                                 approach_height = 0.1, special_pick = True)
        # taskboard.tilt_up_gripper()

      if i in ["91","92","93","94","95","96","97","98","99","910","911","912","913","914","915", "9121", "9122", "9131", "9132"]:
        taskboard.do_task_number(int(i[1:]))
      if i == "x":
        break
      

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass

