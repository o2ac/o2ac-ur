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
tau = 2.0*pi  # Part of math from Python 3.6
import math

from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg
import std_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.common import O2ACCommon

class AssemblyClass(O2ACCommon):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    
    # Initialize debug monitor
    if not self.assembly_database.db_name == "wrs_assembly_1":
      self.assembly_database.change_assembly("wrs_assembly_1")

    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

  ################ ----- Subroutines  

  def subtask_zero(self):
    self.unlock_base_plate()
    # Pick the base plate 
    # TODO

    # # Place it in the fixation after it is grasped (using UR script)
    # b_bot_wait_with_base_plate = [0, 0, 0, 0, 0, 0]
    # # rostopic echo /joint_states (Watch out for the order, joints 1 and 3 are not in order)
    # self.move_joints("b_bot", b_bot_wait_with_base_plate)

    # success_b = self.load_program(robot="b_bot", program_name="wrs2020/asm_place_base_plate.urp", recursion_depth=3)
    # if not success_b:
    #   rospy.logerr("Failed to load base plate placing program on b_bot")
    #   return False
    # print("Running belt pick on b_bot.")
    # if not self.execute_loaded_program(robot="b_bot"):
    #   rospy.logerr("Failed to execute base plate placing program on b_bot")
    #   return False
    # wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))

    # (Using ROS/Python)
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "workspace_center"
    place_pose.pose.position = geometry_msgs.msg.Point(-0.170, 0.013, 0.11)
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, 0))

    approach_pose = copy.deepcopy(place_pose)
    approach_pose.pose.position.z -= 0.05
    
    self.go_to_pose_goal("b_bot", approach_pose, speed=0.5, move_lin = True)
    self.go_to_pose_goal("b_bot", place_pose, speed=0.5, move_lin = True)
    self.open_gripper("b_bot")
    
    self.lock_base_plate()
    return True


  def subtask_a(self, start_from_screwing=False):
    # ============= SUBTASK A (picking and inserting and fastening the motor) =======================
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

  def subtask_e(self):
    rospy.loginfo("======== SUBTASK E (idler pulley) ========")
    rospy.logerr("Subtask E not implemented yet")

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return True

  def subtask_f(self):
    rospy.loginfo("======== SUBTASK F (motor panel (small L-plate)) ========")
    rospy.logerr("Subtask F not implemented yet")
    return False

  def subtask_g(self):
    rospy.loginfo("======== SUBTASK G (bearing panel (large L-plate)) ========")
    rospy.logerr("Subtask G not implemented yet")
    goal = self.look_and_get_grasp_point(2)  # bearing panel
    if not goal:
        rospy.logerr("Could not find bearing plate in tray. Breaking out.")
        return False
    # Grasp the plate at the correct position
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
      success = self.pick('screw_tool_' + screw_type, 'tools', 'screw_tool_m3_pickup_link', [-1.0, 0.0, 0.0], save_solution_to_file = 'mtc_pick_screw_tool')
    return success

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
      success = self.fasten('screw_tool_' + screw_type, screw_pickup_pose, object_subframe_to_place = screw_tool_tip_frame, save_solution_to_file = 'pick_screw')
    return success

  def mtc_place_object_in_tray_center(self, object_name):
    rospy.loginfo("======== PLACE TASK ========")
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = 'tray_center'
    target_pose.pose.position.x = -0.04
    target_pose.pose.position.y = 0.08
    target_pose.pose.orientation.w = 1
    self.place(object_name, target_pose, save_solution_to_file = 'place_' + object_name)

  def mtc_pickplace_l_panel(self):
    rospy.loginfo("======== PICKPLACE TASK ========")

    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = 'base/screw_hole_panel2_1'
    target_pose.pose.orientation.w = 1

    self.pick_place('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', robot_names = ['b_bot','a_bot'], force_robot_order = True, save_solution_to_file = 'pickplace')

  def mtc_pick_place_task(self):
    rospy.loginfo("======== PICK-PLACE TASK ========")
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
    pose.pose.orientation.w = 1
    success = self.pick_place('b_bot', 'panel_bearing', pose, 'panel_bearing/bottom_screw_hole_aligner_1')
    return success

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
    self.go_to_named_pose("feeder_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

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
      rospy.loginfo("Enter 91-94 for subtasks (Large plate, motor plate, idler pin, motor).")
      rospy.loginfo("Enter 95-98 for subtasks (motor pulley, bearing+shaft, clamp pulley, belt).")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        # speed=assy.speed_fastest, acceleration=assy.acc_fastest,
        assy.go_to_named_pose("home", "a_bot", force_ur_script=False)
        assy.go_to_named_pose("home", "b_bot", force_ur_script=False)
      if i == '11':
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if i == '12':
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if i == '13':
        assy.do_change_tool_action("b_bot", equip=True, screw_size=3)
      if i == '14':
        assy.do_change_tool_action("b_bot", equip=False, screw_size=3)
      if i == '25':
        assy.open_gripper('b_bot')
      if i == '26':
        assy.close_gripper('b_bot')
      if i == '27':
        assy.open_gripper('a_bot')
      if i == '28':
        assy.close_gripper('a_bot')
      if i == '30':
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
        assy.pick_screw_from_feeder("a_bot", screw_size=3)
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
      if i == '31':
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
        assy.pick_screw_from_feeder("b_bot", screw_size=3)
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
      if i == '40':
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
        assy.pick_screw_from_feeder("a_bot", screw_size=4)
        assy.go_to_named_pose("feeder_pick_ready", "a_bot")
      if i == '41':
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
        assy.pick_screw_from_feeder("b_bot", screw_size=4)
        assy.go_to_named_pose("feeder_pick_ready", "b_bot")
      if i == '68':
        assy.spawn_objects_for_demo()
      if i == '69':
        assy.pick('panel_bearing', robot_name = '', save_solution_to_file = 'pick_panel_bearing')
      if i == '70':
        assy.mtc_place_object_in_tray_center('panel_bearing')
      if i == '71':
        assy.mtc_pickplace_l_panel()
      if i == '72':
        assy.mtc_pick_screw_tool('m4')
      if i == '73':
        assy.mtc_suck_screw('m4')
      if i == '74':
        assy.release('panel_bearing', 'home', 'release_panel_bearing')
      if i == '75':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        assy.plan_wrs_subtask_b('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', save_solution_to_file = 'subassembly')
      if i == '80':
        rospy.loginfo("Loading")
        mp_res = assy.load_MTC_solution('subassembly')
        rospy.loginfo("Running")
        assy.execute_MTC_solution(mp_res.solution, speed = 0.2)
      if i == '800':
        assy.load_and_execute_program(robot="b_bot", program_name="wrs2020_push_motor_plate.urp", wait=True)
      if i == '801':
        assy.move_lin_rel("a_bot", relative_translation=[0, -0.01, 0], use_robot_base_csys=True, max_wait=5.0)
      if i == '802':
        assy.move_lin_rel("a_bot", relative_translation=[0,  0.01, 0], use_robot_base_csys=True, max_wait=5.0)
      if i == '81':
        assy.do_change_tool_action('b_bot', equip=True, screw_size=4)
      if i == '82':
        assy.pick_screw_from_feeder('b_bot', 4)
      if i == '83':
        assy.do_linear_push('a_bot', force=15, direction="Y-", max_approach_distance=0.05, forward_speed=0.003)
      if i == '84':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        assy.fasten_screw('b_bot', target_pose)
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
