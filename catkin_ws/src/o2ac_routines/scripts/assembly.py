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
import tf_conversions
import tf
from math import pi
import math

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg

from o2ac_assembly_handler.assy_reader import AssyReader
from o2ac_assembly_handler.assy import AssyHandler
from o2ac_routines.common import O2ACCommon

class AssemblyClass(O2ACCommon):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    
    self.set_assembly()
    
    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")

    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

  def set_assembly(self, assembly_name="wrs_assembly_1"):
    # Change the assembly
    return True

  ################ ----- Subroutines  

  def subtask_a(self, start_from_screwing=False):
    # ============= SUBTASK A (picking and inserting and fastening the motor shaft) =======================
    rospy.loginfo("======== SUBTASK A (motor) ========")
    rospy.logerr("Subtask A not implemented yet")
    return False

    # if start_from_screwing:
    #   self.do_change_tool_action("b_bot", equip=True, screw_size=3)
    #   self.go_to_named_pose("screw_ready", "b_bot")
    #   for i in range(1,7):
    #     self.fasten_motor_screw(screw_hole_number=i)
    #   self.do_change_tool_action("b_bot", equip=False, screw_size=3)

  def subtask_b(self):
    rospy.loginfo("======== SUBTASK B (motor pulley) ========")
    rospy.logerr("Subtask B not implemented yet")
    return

  def subtask_c(self):
    rospy.loginfo("======== SUBTASK C (output bearing + shaft) ========")
    rospy.logerr("Subtask C not implemented yet")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return False

  def subtask_d(self):
    # Fasten large pulley to output shaft
    rospy.loginfo("======== SUBTASK D (output pulley) ========")
    rospy.logerr("Subtask D not implemented yet")
    return False

  def subtask_e(self, pick_from_holder=False, exit_before_spacer=False, force_continue_task_to_the_end=False, exit_on_level_2=False):
    rospy.loginfo("======== SUBTASK E (idler pulley) ========")
    rospy.logerr("Subtask E not implemented yet")

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return True

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (small L-plate) ========")
    rospy.logerr("Subtask F not implemented yet")
    return False

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (large L-plate) ========")
    rospy.logerr("Subtask G not implemented yet")
    return False

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

  def real_assembly_task(self):
    self.start_task_timer()
    self.log_to_debug_monitor(text="Assembly", category="task")

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
    assy.set_assembly()
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