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
import time
import numpy as np

from o2ac_msgs.srv import *
import moveit_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.assembly import O2ACAssembly
import o2ac_routines.helpers as helpers

from ur_control.constants import TERMINATION_CRITERIA

if __name__ == '__main__':
  try:
    rospy.init_node('o2ac_routines', anonymous=False)
    rospy.loginfo("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    c = O2ACAssembly()
    while True:
      rospy.loginfo("Enter 1 to move the robots home.")
      rospy.loginfo("Enter 11,12 to close,open grippers")
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (a_bot).")
      rospy.loginfo("Enter 15 (16) to equip (unequip) m4 tool (b_bot).")
      rospy.loginfo("Enter 31, 32 to pick screw m3, m4 from feeder.")
      rospy.loginfo("Enter 55, 551, 552 to spawn example parts.")
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
      tic_start = time.time()
      if i == '1':
        c.a_bot.go_to_named_pose("home")
        c.b_bot.go_to_named_pose("home")
      elif i == '1a':
        c.a_bot.go_to_named_pose("home")
      elif i == '1b':
        c.b_bot.go_to_named_pose("home")
      elif i == '11':
        c.a_bot.gripper.close(wait=False)
        c.b_bot.gripper.close()
      elif i == '12':
        c.a_bot.gripper.open(wait=False)
        c.b_bot.gripper.open()
      elif i == '13':
        c.do_change_tool_action("a_bot", equip=True, screw_size=3)
      elif i == '14':
        c.do_change_tool_action("a_bot", equip=False, screw_size=3)
      elif i == '15':
        c.do_change_tool_action("b_bot", equip=True, screw_size=4)
      elif i == '16':
        c.do_change_tool_action("b_bot", equip=False, screw_size=4)
      elif i == '21':
        c.take_tray_from_agv()
      elif i == '211':
        c.take_tray_from_agv(reverse_movement_for_calibration=True)
      elif i == '212':
        c.take_tray_from_agv_preplanned()
      elif i == '22':
        c.unload_drive_unit()
      elif i == '31':
        c.pick_screw_from_feeder("a_bot", screw_size=3)
      elif i == '32':
        c.pick_screw_from_feeder("b_bot", screw_size=4)
      elif i == "41":
        c.publish_part_in_assembled_position("base")
      elif i == "421":
        c.publish_part_in_assembled_position("panel_bearing")
      elif i == "422":
        c.publish_part_in_assembled_position("panel_motor")
      elif i == "43":
        c.publish_part_in_assembled_position("bearing")
      elif i == "44":
        c.publish_part_in_assembled_position("motor")
      elif i == "45":
        c.publish_part_in_assembled_position("shaft")
        c.publish_part_in_assembled_position("endcap")
      elif i == "46":
        c.publish_part_in_assembled_position("pulley")
      elif i == "51":
        grasp_pose = c.assembly_database.get_grasp_pose("panel_bearing", "default_grasp")
        res = c.plan_pick_place("panel_bearing", "a_bot", grasp_pose)
      elif i == "52":
        c.execute_MTC_solution(res.solution, speed=0.1)
      elif i == "53":
        c.pick(robot_name="a_bot", object_name="panel_bearing")
      elif i == "54":
        c.pick(robot_name="b_bot", object_name="base")
      elif i == '55':
        c.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=2)
      elif i == '551':
        c.spawn_objects_for_demo(layout_number=4)
      elif i == '552':
        c.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=1)
      elif i == '553':
        c.spawn_objects_for_demo(base_plate_in_tray=True, layout_number=3)
      elif i == '559':
        c.disable_scene_object_collisions()
      elif i == 'clean_scene':
        non_base_objects = ['bearing', 'bearing_spacer', 'end_cap', 'idler_pin', 'idler_pulley', 'idler_spacer', 'motor', 'motor_pulley', 'output_pulley', 'shaft']
        for o in non_base_objects:
          c.planning_scene_interface.remove_world_object(o)
      elif i == '550':
        c.planning_scene_interface.allow_collisions("base", "")
      elif i == '551':
        c.planning_scene_interface.disallow_collisions("base", "")
      elif i == '5599':
        c.allow_collisions_with_robot_hand("bearing", "b_bot")
      elif i == '55999':
        c.allow_collisions_with_robot_hand("bearing", "b_bot", False)
      elif i == '67':
        c.spawn_objects_for_closed_loop_test()
      elif i == '68':
        c.spawn_objects_for_demo()
      elif i == '69':
        result = c.do_plan_pick_action('panel_bearing', robot_name = '', save_solution_to_file = 'pick_panel_bearing')
        for solution in result.solution.sub_trajectory:
          scene_diff = solution.scene_diff
          planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
          planning_scene_diff_req.scene = scene_diff
          # self.apply_planning_scene_diff.call(planning_scene_diff_req)   # DEBUG: Update the scene pretending the action has been completed
      elif i == '691':
        rospy.loginfo("Loading 'pick_panel_bearing'")
        mp_res = c.load_MTC_solution('pick_panel_bearing')
        rospy.loginfo("Running")
        c.execute_MTC_solution(mp_res.solution, speed = 0.2)
      elif i == '70':
        c.mtc_place_object_in_tray_center('panel_bearing')
      elif i == '71':
        c.mtc_pickplace_l_panel()
      elif i == '72':
        c.mtc_pick_screw_tool('m4')
      elif i == '73':
        c.mtc_suck_screw('m4')
      elif i == '74':
        c.do_plan_release_action('panel_bearing', 'home', save_solution_to_file = 'release_panel_bearing')
      elif i == '75':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        c.do_plan_wrs_subtask_b_action('panel_bearing', target_pose, object_subframe_to_place = 'panel_bearing/bottom_screw_hole_aligner_1', save_solution_to_file = 'subassembly')
      elif i == '80':
        rospy.loginfo("Loading")
        mp_res = c.load_MTC_solution('subassembly')
        rospy.loginfo("Running")
        c.execute_MTC_solution(mp_res.solution, speed = 0.2)
      elif i == '800':
        c.b_bot.load_and_execute_program(program_name="wrs2020_push_motor_plate.urp", wait=True)
      elif i == '801':
        c.skill_server.move_lin_rel("a_bot", relative_translation=[0, -0.01, 0], relative_to_robot_base=True, max_wait=5.0)
      elif i == '802':
        c.skill_server.move_lin_rel("a_bot", relative_translation=[0,  0.01, 0], relative_to_robot_base=True, max_wait=5.0)
      elif i == '81':
        c.do_change_tool_action('b_bot', equip=True, screw_size=4)
      elif i == '82':
        c.do_change_tool_action('b_bot', equip=False, screw_size=4)
      elif i == '83':
        c.pick_screw_from_feeder('b_bot', 4)
      elif i == '84':
        c.skill_server.do_linear_push('a_bot', force=15, direction="Y-", max_approach_distance=0.05, forward_speed=0.003)
      elif i == '85':
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1
        c.fasten_screw('b_bot', target_pose)
      elif i == '90':
        c.subtask_zero() # Base plate
      elif i == '91':
        c.subtask_g()  # Large plate
      elif i == '92':
        c.subtask_f()  # Motor plate
      elif i == '93':
        c.subtask_c1() # bearing
      elif i == '931':
        c.pick_up_and_insert_bearing(task="assembly")
      elif i == '9311':
        c.align_bearing_holes(task="assembly")
      elif i == "93move":
        c.playback_sequence("bearing_orient")
        c.playback_sequence("bearing_move_to_assembly")
      elif i == '932':
        c.fasten_bearing(task="assembly")
      elif i == '9322':
        c.fasten_bearing(task="assembly", only_retighten=True)
      elif i == '933':
        c.insert_bearing(task="assembly")
      elif i == '94':
        c.subtask_a() # Motor
      elif i == '95':
        c.subtask_b() # motor pulley
      elif i == '96':
        c.subtask_e() # idler pin
      elif i == '97':
        c.subtask_c2() # shaft
        # 97: shaft, 98: clamp pulley, 99: belt).")
      elif i == '991':
        c.pick_and_store_belt()
      elif i == '97spawn':
        obj = c.assembly_database.get_collision_object("shaft")
        obj.header.frame_id = "b_bot_gripper_tip_link"
        obj.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, -tau/4, -tau/4))
        obj.pose = helpers.rotatePoseByRPY(0, 0, tau/2, obj.pose)
        obj.pose.position.x = -.006
        obj.pose.position.z = .0375
        c.planning_scene_interface.add_object(obj)
        c.b_bot.gripper.attach_object(obj.id)
      elif i == "899":
        center_plate_pose = geometry_msgs.msg.PoseStamped()
        center_plate_pose.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
        center_plate_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(60), -tau/4))
        c.a_bot.gripper.open(opening_width=0.07, wait=False)
        c.go_to_pose_goal("a_bot", center_plate_pose, move_lin=False, speed=1.0)
        c.a_bot.gripper.close(force = 100)
        c.a_bot.gripper.open()
      elif i == "898":
        c.a_bot.go_to_named_pose("home", force_ur_script=False)
      elif i == '100': # Test Force control be careful!!
        b_bot_starting_position = [1.7078, -1.5267, 2.0624, -2.1325, -1.6114, 1.7185]
        c.move_joints("b_bot", b_bot_starting_position)
        
        timeout = 20.0

        selection_matrix = [1.,1.,0.,1.,1.,1.]
        target_force =     [0.,0.,0.,0.,0.,0.]

        c.activate_ros_control_on_ur(robot="b_bot", recursion_depth=1)
        rospy.logwarn("STARTING Force Control with target_force: %s %s %s" % (str(target_force), "timeout", str(timeout)))
        c.b_bot.force_control(target_force=target_force, selection_matrix=selection_matrix, timeout=timeout, stop_on_target_force=True)
        rospy.logwarn("FINISHED Force Control")
      elif i == '101': # Test Force control be careful!!
        force = 10.0 #N
        direction = '-Z'

        c.b_bot.linear_push(force=force, direction=direction, timeout=20.0)
      elif i == "cb0":
        c.check_output_pulley_angle()
      elif i == "cb1":
        c.check_motor_pulley_angle()
      elif i == '102':
        b_bot_script_start_pose = [1.7094888, -1.76184906, 2.20651847, -2.03368343, -1.54728252, 0.96213197]
        c.move_joints("b_bot", b_bot_script_start_pose)
        
        # force = 1.0 #N
        # direction = '-X'

        # c.b_bot.linear_push(initial_pose=b_bot_starting_position, force=force, direction=direction, timeout=20.0)
      elif i == "print":  # Print collision objects
        c.print_objects_in_tray()
      elif i == "endcap":
        c.orient_shaft_end_cap()
      elif i == 'START' or i == 'start' or i == "9999":
        for i in [1,2]:
          rospy.loginfo("Starting set number " + str(i))
          c.competition_mode = True
          c.full_assembly_task()
          c.competition_mode = False
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
        c.full_assembly_task()
      if i == "reset":
        c.reset_scene_and_robots()
        c.reset_assembly_visualization()
      if i == "unload":
        c.unload_drive_unit()
      if i == "activate":
        c.a_bot.activate_ros_control_on_ur()
        c.b_bot.activate_ros_control_on_ur()
      elif i == 'x':
        break
      elif i == "":
        continue
      print("This took: %.3f seconds" % (time.time() - tic_start))
  except rospy.ROSInterruptException:
    pass
