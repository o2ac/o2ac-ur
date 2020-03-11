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
import tf
from math import pi
import math

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg


from o2ac_routines.base import O2ACCommonBase

class AssemblyClass(O2ACCommonBase):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)

    # Initialize variables
    self.screw_is_suctioned = dict()
    self.screw_is_suctioned["m3"] = False
    self.screw_is_suctioned["m4"] = False

    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")

  def set_up_item_parameters(self):
    # TODO: Publish the items to the scene, or do something equivalent. 
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    

  ################ ----- Routines  
  ################ 
  ################ 


  def fasten_motor_screw(self, screw_hole_number):
    self.log_to_debug_monitor("Fasten motor screw", "operation")
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "assembled_assy_part_02_motor_screw_hole_"+str(screw_hole_number)
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi*90/180, 0,0))
    
    self.pick_screw_from_feeder("b_bot", screw_size=3, screw_number="auto") # I commented this because this takes a long time in simulation
    self.go_to_named_pose("horizontal_screw_ready", "b_bot")
    self.go_to_pose_goal("b_bot", pose1, speed=0.3,end_effector_link="b_bot_screw_tool_m3_tip_link", move_lin=True)
    
    # TODO: add fastening action
    return

  def insert_bearing(self):
    self.log_to_debug_monitor("Insert bearing", "operation")
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

  ########

  def subtask_f(self):
    self.log_to_debug_monitor("SUBTASK F (motor plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask F start ===", "operation")
    self.place_plate_2_and_screw()
    self.log_to_debug_monitor("=== Subtask F end ===", "operation")

  def subtask_g(self):
    # ===== SUBTASK G (Placing and fastening the output (large) plate, for idle pulley set and clamping pulley set) =========
    self.log_to_debug_monitor("SUBTASK G (large plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask G (large plate) start ===", "operation")
    success = self.place_plate_3_and_screw()
    self.log_to_debug_monitor("=== Subtask G (large plate) end ===", "operation")
    return success

  def subtask_a(self, do_screwing=False):
    # ============= SUBTASK A (picking and inserting and fastening the motor shaft) =======================
    self.log_to_debug_monitor("SUBTASK A (motor)", "subtask")
    self.log_to_debug_monitor("=== Subtask A (motor) start ===", "operation")
    self.pick_motor()
    self.adjust_centering(go_fast=True, handover_motor_to_c_bot=True)
    self.insert_motor() # Joshua thinks this may be possible to do without impedance control, otherwise use insertion script in Y negative direction

    if do_screwing:
      self.do_change_tool_action("b_bot", equip=True, screw_size=3)
      self.go_to_named_pose("screw_ready", "b_bot")
      for i in range(1,7):
        self.fasten_motor_screw(screw_hole_number=i) # please ask Felix about the pick_screw function, I am not sure how he defined it
        self.do_change_tool_action("b_bot", equip=False, screw_size=3)
    self.log_to_debug_monitor("=== Subtask A (motor) end ===", "operation")

  def subtask_b(self):
    self.log_to_debug_monitor("SUBTASK B (motor pulley)", "subtask")
    self.log_to_debug_monitor("=== Subtask B (motor pulley) start ===", "operation")
    self.pick_motor_pulley()
    self.insert_motor_pulley()
    rospy.loginfo("todo: fasten motor pulley") # With the set screw
    self.log_to_debug_monitor("=== Subtask B (motor pulley) end ===", "operation")
    return

  def subtask_e(self, pick_from_holder=False, exit_before_spacer=False, force_continue_task_to_the_end=False, exit_on_level_2=False):
    rospy.loginfo("======== SUBTASK E ========")
    self.log_to_debug_monitor("SUBTASK E (idler pin)", "subtask")
    self.log_to_debug_monitor("=== Subtask E (idler pin) start ===", "operation")

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    # TODO: Reimplement this.
    
    self.go_to_pose_goal("b_bot", b_bot_retreat, speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.log_to_debug_monitor("=== Subtask E end ===", "operation")
    return True

  def subtask_c(self):
    # ==== SUBTASK C (clamping pulley set; everything but inserting and fastening clamping pulley) =================
    self.log_to_debug_monitor("SUBTASK C (bearing + shaft)", "subtask")
    self.log_to_debug_monitor("=== Subtask C start ===", "operation")

    self.pick_clamping_shaft()
    self.adjust_centering(go_fast=True)

    self.insert_clamping_shaft()   

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.log_to_debug_monitor("=== Subtask C end ===", "operation")

  ##### 

  def real_assembly_task(self):
    self.start_task_timer()
    self.log_to_debug_monitor(text="Assembly", category="task")
    self.picked_screw_counter["m4"] = 1
    self.picked_screw_counter["m3"] = 1

    self.subtask_c() # bearing shaft insertion

    # To prepare subtask E
    self.pick_retainer_pin_from_tray_and_place_in_holder(do_centering=False)
    self.confirm_to_proceed("press enter to proceed")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ## Equip screw tool for subtasks G, F
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    self.subtask_g()  # Large plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    self.subtask_f() # motor plate

    assy.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=assy.use_real_robot)
    assy.do_change_tool_action("b_bot", equip=False, screw_size=4)

    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.subtask_e(pick_from_holder=True) #idle pulley

    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.subtask_a() # motor
    
    # self.subtask_c() # bearing, clamping pulley set
    return

if __name__ == '__main__':
  try:
    rospy.loginfo("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    assy = AssemblyClass()
    assy.set_up_item_parameters()
    i = 1
    while i:
      rospy.loginfo("Enter 11 (12) to equip (unequip) m4 tool (b_bot).")
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (b_bot).")
      rospy.loginfo("Enter 2 to move the robots home to starting positions.")
      rospy.loginfo("Enter 30 to pick screw m3 from feeder with a_bot (31 for b_bot).")
      rospy.loginfo("Enter 40 to pick screw m4 from feeder with a_bot (41 for b_bot).")
      rospy.loginfo("Enter 91-94 for subtasks (Large plate, motor plate, idler pin, motor).")
      rospy.loginfo("Enter 95-98 for subtasks (motor pulley, bearing+shaft, clamp pulley, belt).")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '11':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if i == '12':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if i == '13':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=3)
      if i == '14':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=3)
      if i == '2':
        assy.go_to_named_pose("back", "a_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "b_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
      if i == '30':
        assy.go_to_named_pose("screw_pick_ready", "a_bot")
        assy.pick_screw_from_feeder("a_bot", screw_size=3, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "a_bot")
      if i == '31':
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw_from_feeder("b_bot", screw_size=3, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      if i == '40':
        assy.go_to_named_pose("screw_pick_ready", "a_bot")
        assy.pick_screw_from_feeder("a_bot", screw_size=4, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "a_bot")
      if i == '41':
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw_from_feeder("b_bot", screw_size=4, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i == '91':
        assy.subtask_g()  # Large plate
      elif i == '92':
        assy.subtask_f()  # Motor plate
      elif i == '93':
        assy.subtask_e() # 
      elif i == '94':
        assy.subtask_a() #
      elif i == '95':
        assy.subtask_c() #
      elif i == 'START' or i == 'start' or i == "9999":
        for i in [1,2]:
          rospy.loginfo("Starting set number " + str(i))
          assy.real_assembly_task()
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
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass
