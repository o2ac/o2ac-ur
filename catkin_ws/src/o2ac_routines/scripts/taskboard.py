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
from math import pi
from math import *

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
                      "M3 set screw", "M3 screw", 
                      "M4 screw", "Pulley", "Shaft"]
    
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))

    self.assy_reader = AssyReader("taskboard")


  def multiply_quaternion_msgs(self, q1_msg, q2_msg):
    q1 = [q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w]
    q2 = [q2_msg.x, q2_msg.y, q2_msg.z, q2_msg.w]
    return geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply(q1, q2))
    
  def  _example_objects(self):
    # This function spawns the objects into the tray as if they had been recognized by the vision node
    names = ["taskboard_idler_pulley_small", "bearing", "drive_shaft", "motor_pulley", "endcap"]
    offsets = {"bearing": [-.04, -.02, .001],
    "taskboard_idler_pulley_small": [.05, .06, .03], 
    "drive_shaft": [.03, -.06, .005], 
    "motor_pulley": [.01, .08, .005], 
    "endcap": [-.05, -.1, .005]}
    for name in names:
      collision_object = self.assy_reader.get_collision_object(name)
      if collision_object:
        collision_object.header.frame_id = "tray_center"
        if name == "bearing" or name == "taskboard_idler_pulley_small" or name == "endcap" or name == "motor_pulley":
          q_rotate = self.downward_orientation
        elif name == "drive_shaft":
          q_rotate = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
          
        offset = offsets[name]
        for poselist in [collision_object.mesh_poses, collision_object.subframe_poses]:
          for pose in poselist:
            pose.position.x += offset[0]
            pose.position.y += offset[1]
            pose.position.z += offset[2]
            pose.orientation = self.multiply_quaternion_msgs(pose.orientation, q_rotate)
        self.planning_scene_interface.add_object(collision_object)
        print("======== collision object: " + name)
        print(collision_object.mesh_poses)
        print(collision_object.subframe_names)
        print(collision_object.subframe_poses)
      else:
        rospy.logerr("Could not retrieve collision object:" + name)

  #####
  
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
    self.confirm_to_proceed("Current item: M3 set screw")
    self.do_task("M3 set screw")
    self.confirm_to_proceed("Current item: Belt")
    self.do_task("belt")
    self.confirm_to_proceed("Current item: M4 screw")
    self.do_task("M4 screw")
    self.confirm_to_proceed("Current item: M3 screw")
    self.do_task("M3 screw")
    self.confirm_to_proceed("Current item: Pulley")
    self.do_task("pulley")
    self.confirm_to_proceed("Current item: Bearing")
    self.do_task("bearing")
    self.confirm_to_proceed("Current item: Shaft")
    self.do_task("shaft")
    self.confirm_to_proceed("Current item: Idler pulley")
    self.do_task("idler pulley")
    
  
  def do_task(self, task_name):
    
    if task_name == "belt":
      # - Equip the belt tool with b_bot
      self.equip_tool("belt_tool")
      # - Pick up the belt with a_bot
      # - Tension the belt
      # - Move both arms to the taskboard
      # - Move a_bot next to the small pulley
      # - Move b_bot with the tool to slide the belt into the large pulley
      # - Open a_bot slightly and move away
      # - Move along the belt groove with b_bot and the tool to make sure it is in
      self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
  
      # Pick up the belt
      belt_pick_pose = copy.deepcopy(self.pick_poses[5])
      belt_pick_pose.pose.position.y += .055
      self.pick("b_bot", belt_pick_pose, grasp_height=0.0,
                      speed_fast = 1.0, speed_slow = 0.1, gripper_command="close")
      self.go_to_named_pose("taskboard_center_pose", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # Place the belt
      belt_place_intermediate = geometry_msgs.msg.PoseStamped()
      belt_place_intermediate.header.frame_id = "taskboard_large_pulley"
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
      belt_place_pose_final.header.frame_id = "taskboard_large_pulley"
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
      
      self.go_to_named_pose("home","b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
    if task_name == "M3 set screw":
      # This expects to be exactly above the set screw hole
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

      # Go up with b_bot tool
      req = o2ac_msgs.srv.sendScriptToURRequest()
      req.program_id = "lin_move_rel"
      req.robot_name = "b_bot"
      req.relative_translation.y = -.1
      req.velocity = .05
      res = self.urscript_client.call(req)
      wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(5.0))

      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    if task_name in ["M3 screw", "M4 screw"]:
      if task_name == ["M3 screw"]:
        screw_size = 3
      else:
        screw_size = 4
      # TODO
      if task_name in [12, 13, 121, 131]:
        self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
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
        #pick up the screw tool
        self.go_to_named_pose("tool_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.do_change_tool_action("b_bot", equip=True, screw_size = screw_size)
        
        #pick up the screw from feeder
        self.pick_screw_from_feeder(screw_size)
        self.go_to_named_pose("screw_ready_high", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

        #screw on the cap
        screw_approach = copy.deepcopy(self.place_poses[part_id-1])
        screw_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, 0, 0))

        if task_name == 12:
          screw_approach.pose.position.z -= .002  #MAGIC NUMBER!
          screw_approach.pose.position.y -= .001  #MAGIC NUMBER!
        elif task_name == 13:
          screw_approach.pose.position.z -= .003  #MAGIC NUMBER!
          screw_approach.pose.position.y -= .001  #MAGIC NUMBER!
        
        screw_approach.pose.position.x -= 0.03
        self.go_to_pose_goal("b_bot", screw_approach, speed=0.5, end_effector_link="b_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)
        print(screw_approach)
        self.confirm_to_proceed("Proceed to screw_pose?")

        screw_pose = copy.deepcopy(screw_approach)
        screw_pose.pose.position.x = 0.007
        if screw_size == 3:
          screw_height = 0.007
        elif screw_size == 4:
          screw_height = 0.01
        self.do_screw_action("b_bot", screw_pose, screw_height = screw_height, screw_size = screw_size, stay_put_after_screwing=True)

        if screw_size == 3:
          screw_pose_manual_followup = copy.deepcopy(screw_approach)
          screw_pose_manual_followup.pose.position.x = 0.01
          
          self.set_motor("screw_tool_m3", direction = "tighten", wait=False, speed = 800, duration = 5)
          # self.horizontal_spiral_motion("b_bot", max_radius = .002, radius_increment = .001, spiral_axis="YZ")
          rospy.sleep(5.0)
          self.set_suction("screw_tool_m3", False, False)
          self.set_motor("screw_tool_m3", direction = "loosen", wait=False, speed = 300, duration = .5)  # Supposed to fix the bit getting stuck

        self.go_to_pose_goal("b_bot", screw_approach, speed=0.05, end_effector_link="b_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)

        self.go_to_named_pose("screw_ready_high", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.go_to_named_pose("tool_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.do_change_tool_action("b_bot", equip=False, screw_size = screw_size)
      self.go_to_named_pose("back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if task_name == "Pulley":
      rospy.logerr("Pulley is not implemented yet!")
    if task_name == "Bearing":
      rospy.logerr("Bearing is not implemented yet!")
    if task_name == "Shaft":
      rospy.logerr("Shaft is not implemented yet!")
    if task_name == "Idler pulley":
      rospy.logerr("Idler pulley is not implemented yet!")

if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    taskboard.spawn_example_objects()

    i = 1
    while(i):
      rospy.loginfo("Enter 11, 12 to equip/unequip nut_tool_m6")
      rospy.loginfo("Enter 13, 14, 141 to equip/unequip/discard nut_tool_m10")
      rospy.loginfo("Enter 15, 16 to equip/unequip belt placement tool")
      rospy.loginfo("Enter 191, 192 to equip/unequip m4 screw tool")
      rospy.loginfo("Enter 2 to move robots to home")
      rospy.loginfo("Enter 8, 80, 81... to do example picks")
      rospy.loginfo("Enter 9, 90, 91... to plan motions with the picked object('s subframe)")
      rospy.loginfo("Enter 'belt', 'bearing', 'pulley', 'shaft', 'endcap'... to perform each subtask")
      rospy.loginfo("Enter start to start the task")
      rospy.loginfo("Enter x to exit")
      i = raw_input()

      if i == "start":
        taskboard.full_taskboard_task()
      if i == "y001":  # insert shaft sample
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "tray_center"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.position.z = 0.1
        taskboard.go_to_pose_goal("a_bot", pick_pose, speed=0.1, end_effector_link="a_bot_robotiq_85_tip_link", move_lin = False)
        # taskboard.pick("a_bot", ...)
        insert_pose = geometry_msgs.msg.PoseStamped()
        insert_pose.header.frame_id = "taskboard_small_shaft"
        insert_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        insert_pose.pose.position.z = 0.1
        taskboard.go_to_pose_goal("a_bot", insert_pose, speed=0.1, end_effector_link="a_bot_robotiq_85_tip_link", move_lin = False)
        taskboard.do_insert_action("a_bot")
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
      if i == "2":
        taskboard.go_to_named_pose("home","a_bot")
        taskboard.go_to_named_pose("home","b_bot")
      if i == "8":
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "move_group/drive_shaft"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.position.x = 0.03
        taskboard.simple_pick("a_bot", pick_pose, approach_height = 0.05, item_id_to_attach = "drive_shaft", lift_up_after_pick=True)
      if i == "80":
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "drive_shaft/front_tip"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.position.x = -0.03
        taskboard.simple_pick("a_bot", pick_pose, approach_height = 0.05, item_id_to_attach = "drive_shaft", lift_up_after_pick=True)
      if i == "81":
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "drive_shaft"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.position.x = 0.03
        taskboard.simple_pick("a_bot", pick_pose, approach_height = 0.05, item_id_to_attach = "drive_shaft", lift_up_after_pick=True)
      if i == "82":
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = "tray_center"
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.position.x = 0.035
        pick_pose.pose.position.z = 0.0
        taskboard.simple_pick("a_bot", pick_pose, approach_height = 0.05, item_id_to_attach = "drive_shaft", lift_up_after_pick=True)


      if i == "9":
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "taskboard_assy_part_07_front_hole"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        ps.pose.position.x = -0.03
        taskboard.go_to_pose_goal("a_bot", ps, speed=0.1, end_effector_link="drive_shaft/front_tip", move_lin = False)
      if i == "91":
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "taskboard_assy_part_07_front_hole"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        ps.pose.position.x = -0.03
        taskboard.go_to_pose_goal("a_bot", ps, speed=0.1, end_effector_link="drive_shaft", move_lin = False)
      if i == "92":
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "taskboard_assy_part_07_front_hole"
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        ps.pose.position.x = -0.05
        taskboard.go_to_pose_goal("a_bot", ps, speed=0.1, end_effector_link="a_bot_robotiq_85_tip_link", move_lin = False)
      if i == "x":
        break
      try:
        taskboard.do_task(i)
      except:
        pass

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
