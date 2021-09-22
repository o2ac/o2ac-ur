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
# Author: Felix von Drigalski, Cristian C. Beltran-Hernandez

from ur_control.constants import DONE, TERMINATION_CRITERIA
import o2ac_routines.helpers as helpers
from o2ac_routines.common import O2ACCommon
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_assembly_database.parts_reader import PartsReader
import moveit_task_constructor_msgs.msg
import moveit_msgs.msg
import std_msgs.msg
import o2ac_msgs.msg
import actionlib
from o2ac_msgs.srv import *
import numpy as np
import time
import math
from os import wait
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

        # Only used for MTC planning
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
        self.get_3d_poses_from_ssd()
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
        self.a_bot.move_lin_rel(relative_translation=[0, -0.05, .1])

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
        grasp_name = "big_holes_grasp" if self.assembly_database.db_name in ["wrs_assembly_2021", "wrs_assembly_2021_surprise"] else "default_grasp"
        success = self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=skip_initial_perception, use_b_bot_camera=use_b_bot_camera)
        if not success:
            rospy.logerr("Fail to grasp base. Trying again with different grasp (default_grasp)")
            grasp_name = "terminal_grasp"
            success = self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=skip_initial_perception, use_b_bot_camera=use_b_bot_camera)
            # if not success:
            #   rospy.logerr("Fail to grasp base. Trying again with different grasp (terminal_grasp)")
            #   grasp_name = "terminal_grasp"
            #   success = self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=skip_initial_perception, use_b_bot_camera=use_b_bot_camera)
            return False

        self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
        self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)

        if not self.use_real_robot:
            self.allow_collisions_with_robot_hand("base_fixture_top", "a_bot", allow=True)

        # self.confirm_to_proceed("finetune above pose")
        # print("q:", self.a_bot.robot_group.get_current_joint_values())
        # self.confirm_to_proceed("finetune in pose")

        # There is a risk of overextending the wrist joint if we don't use the joint pose
        if grasp_name == "big_holes_grasp":
            # above_base_drop = conversions.to_pose_stamped("assembled_part_01", [0.109, 0.069, 0.084, 0.004, -0.005, -0.708, 0.707])
            above_base_drop = [1.609, -1.446, 1.595, -1.7201, -1.5673, -1.5186]
            base_inserted = conversions.to_pose_stamped("assembled_part_01", [0.108, 0.008, 0.083, 0.004, -0.005, -0.708, 0.707])  # Taught
        elif grasp_name == "default_grasp":
            # Move to fixation
            above_base_drop = [1.57783019, -1.430060581, 1.67834741, -1.82884373, -1.56911117, 0.00590014457]
            base_drop = conversions.to_pose_stamped("assembled_part_01", [0.111, 0.007, 0.07, tau/4., 0, -tau/4.])
            base_inserted = conversions.to_pose_stamped("assembled_part_01", [0.108, -0.006, 0.067, 1.568, 0.103, -1.582])  # Taught
        else:
            return False

        seq = []
        seq.append(helpers.to_sequence_item(above_base_drop, 0.5, linear=False))
        if grasp_name == "default_grasp":
            seq.append(helpers.to_sequence_item(base_drop, 0.3))
        seq.append(helpers.to_sequence_item(base_inserted, 0.2))
        if not self.execute_sequence("a_bot", seq, "place base plate"):
            return False
        # self.a_bot.move_joints(above_base_drop, speed=0.5)
        # self.a_bot.go_to_pose_goal(base_drop, speed=0.3, move_lin = True)
        # self.a_bot.go_to_pose_goal(base_inserted, speed=0.05, move_lin = True)
        self.a_bot.gripper.open(opening_width=0.0425, velocity=0.05)
        self.a_bot.gripper.close(force=0, velocity=0.03, wait=False)
        self.a_bot.gripper.open(opening_width=0.0425)
        self.a_bot.gripper.forget_attached_item()
        self.a_bot.move_lin_rel([0, 0, 0.02])

        def set_base_plate():
            rospy.sleep(0.3)
            self.lock_base_plate()
            rospy.sleep(0.3)
            self.unlock_base_plate()
            rospy.sleep(0.3)
            self.lock_base_plate()

        def a_bot_return():
            self.a_bot.move_lin_rel(relative_translation=[0.05, -0.1, 0.02], speed=1.0)
            self.allow_collisions_with_robot_hand("base", "a_bot", allow=False)
            self.publish_part_in_assembled_position("base", marker_only=True)
        self.do_tasks_simultaneous(a_bot_return, set_base_plate)
        if not self.use_real_robot:
            self.allow_collisions_with_robot_hand("base_fixture_top", "a_bot", allow=False)
        return True

    def subtask_a(self, simultaneous=True):
        # ============= SUBTASK A (picking and inserting and fastening the motor) =======================
        rospy.loginfo("======== SUBTASK A (motor) ========")
        self.publish_status_text("Target: Motor")

        self.a_success = False
        self.b_success = False

        def b_task():
            self.b_success = self.pick_motor()
            if not self.b_success:
                rospy.logerr("Fail to pick motor")
                return False
            self.assembly_status.motor_placed_outside_of_tray = True
            self.b_success = self.orient_motor()
            if not self.b_success:
                rospy.logerr("Fail to orient motor")
                return False
            self.assembly_status.motor_oriented = self.b_success

        def a_task():
            self.a_success = self.do_change_tool_action("a_bot", equip=True, screw_size=3)
            self.a_success &= self.a_bot.go_to_named_pose("screw_ready")

        if simultaneous:
            self.do_tasks_simultaneous(a_task, b_task, timeout=120)
        else:
            b_task()
            a_task()

        if not self.a_success or not self.b_success:
            rospy.logerr("Fail to do subtask a, part 1 (a_bot:%s)(b_bot:%s)" % (self.a_success, self.b_success))
            self.do_change_tool_action("a_bot", equip=False, screw_size=3)
            self.a_bot.go_to_named_pose("home")
            self.b_bot.gripper.open()
            self.b_bot.go_to_named_pose("home")
            return False

        if not self.align_motor_pre_insertion():
            rospy.logerr("Fail to do subtask a, part 2")
            return False
        if not self.insert_motor("assembled_part_02_back_hole"):
            rospy.logerr("Fail to do subtask a, part 3")
            return False
        if not self.fasten_motor():
            rospy.logerr("Fail to do subtask a, part 4. Attempt Fallback once")
            if not self.fasten_motor_fallback():
                rospy.logerr("Fail to do fallback")
                self.do_change_tool_action("a_bot", equip=False, screw_size=3)
                self.a_bot.go_to_named_pose("home")
                self.b_bot.gripper.open()
                self.b_bot.go_to_named_pose("home")

        return True

    def subtask_b(self, simultaneous_execution=False):
        rospy.loginfo("======== SUBTASK B (motor pulley) ========")
        target_link = "assembled_part_05_center"
        self.a_bot_success = False
        self.b_bot_success = False

        def a_bot_task():
            if not self.pick_motor_pulley(robot_name="a_bot", attempt=5):
                return False
            if not self.orient_motor_pulley(target_link, robot_name="a_bot"):
                return False
            self.confirm_to_proceed("finetune")
            if not self.insert_motor_pulley(target_link, robot_name="a_bot"):
                return False
            self.a_bot.gripper.forget_attached_item()
            self.publish_part_in_assembled_position("motor_pulley", marker_only=True)
            self.a_bot_success = True

        def b_bot_task():
            if not self.equip_tool("b_bot", "set_screw_tool"):
                return False
            b_bot_approach_pose = conversions.to_pose_stamped(target_link, [0.006, -0.002, -0.072] + np.deg2rad([174.3, -87.6, -135.8]).tolist())
            if not self.b_bot.go_to_pose_goal(b_bot_approach_pose, speed=1.0, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link"):
                return False
            self.b_bot_success = True

        if simultaneous_execution:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180)
        else:
            a_bot_task()
            if self.a_bot_success:
                b_bot_task()

        if not self.a_bot_success or not self.b_bot_success:
            rospy.logerr("Fail to do motor pulley fastening (simultaneous=%s)  a_bot:%s b_bot:%s" % (simultaneous_execution, self.a_bot_success, self.b_bot_success))
            return False

        if not self.fasten_motor_pulley(target_link, simultaneous=simultaneous_execution):
            return False

        return True

    def subtask_c1(self):
        rospy.loginfo("======== SUBTASK C (bearing) ========")
        self.publish_status_text("Target: Bearing")
        self.unequip_tool("a_bot")
        success = False
        if self.pick_up_and_insert_bearing(task="assembly", robot_name="a_bot"):
            self.publish_part_in_assembled_position("bearing", marker_only=True)
            self.a_bot.go_to_named_pose("centering_area", speed=1.0)
            self.a_bot.gripper.forget_attached_item()
            self.b_bot.go_to_named_pose("home", speed=1.0)
            if self.align_bearing_holes(task="assembly"):
                self.b_bot.go_to_named_pose("home", speed=1.0)
                success = self.fasten_bearing(task="assembly", with_extra_retighten=True, robot_name="a_bot")
                self.unequip_tool('a_bot', 'screw_tool_m4')
        return success

    def subtask_c2(self, simultaneous_execution=True, skip_pick_end_cap=False, assemble_bearing_spacer=False):
        rospy.loginfo("======== SUBTASK C (output shaft) ========")
        if not simultaneous_execution:
            self.ab_bot.go_to_named_pose("home")

        self.allow_collisions_with_robot_hand("shaft", "b_bot", True)
        self.allow_collisions_with_robot_hand("end_cap", "a_bot", True)

        if not skip_pick_end_cap:
            self.publish_status_text("Target: end cap")
            if not self.pick_end_cap():
                return False
            if simultaneous_execution:
                self.a_bot.go_to_named_pose("centering_area", speed=1.0)

        self.a_bot_success = False
        self.b_bot_success = False

        def a_bot_task():
            if not self.orient_shaft_end_cap():
                return False
            self.a_bot_success = True

        def b_bot_task():
            if not self.pick_shaft():
                return False
            if not self.orient_shaft():
                return False
            self.b_bot_success = True

        if simultaneous_execution:
            self.publish_status_text("Target: shaft & end cap")
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=300)
        else:
            a_bot_task()
            self.publish_status_text("Target: shaft")
            b_bot_task()

        if not self.a_bot_success or not self.a_bot_success:
            rospy.logerr("Fail to assemble shaft")
            self.drop_in_tray("b_bot")
            self.b_bot.go_to_named_pose("home")
            self.drop_in_tray("a_bot")
            self.a_bot.go_to_named_pose("home")
            return False

        # pre_insertion_shaft = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.2, 0, 0, -tau/4.])
        # if not self.b_bot.go_to_pose_goal(pre_insertion_shaft, speed=0.3):
        pre_insertion_shaft = [1.78158, -0.98719, 2.42349, -4.57638, -1.78597, 0.00433]
        if not self.b_bot.move_joints(pre_insertion_shaft, speed=0.4):
            rospy.logerr("Fail to go to pre_insertion_shaft")
            return False

        above_pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.003, 0.002, 0.280]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(above_pre_insertion_end_cap, speed=0.6, move_lin=False):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False
        pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.003, 0.002, 0.245]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.3, move_lin=True):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False

        # self.confirm_to_proceed("insertion of end cap")
        if not self.insert_end_cap():
            rospy.logerr("failed to insert end cap. maybe")
            # return False
        self.despawn_object("end_cap")
        self.a_bot.gripper.forget_attached_item()

        # self.confirm_to_proceed("Did insertion succeed? Press Enter to open gripper")

        self.a_bot.gripper.send_command(0.06, velocity=0.01)
        self.a_bot.move_lin_rel([0, 0, 0.05], speed=0.3)
        self.a_bot.gripper.detach_object("end_cap")
        self.despawn_object("end_cap")

        self.confirm_to_proceed("prepare screw")

        if not self.fasten_end_cap():
            return False

        if not self.a_bot.go_to_named_pose("home"):
            return False

        # self.confirm_to_proceed("insert to bearing")
        if not self.align_shaft("assembled_part_07_inserted", pre_insert_offset=0.065):
            return False
        self.b_bot.gripper.forget_attached_item()

        self.a_bot_success = False
        self.b_bot_success = False

        def a_bot_task():
            if assemble_bearing_spacer:
                if not self.pick_bearing_spacer("a_bot"):
                    return False
                if not self.orient_bearing_spacer("a_bot"):
                    return False
                if not self.align_bearing_spacer_pre_insertion("a_bot"):
                    return False
                self.assembly_status.bearing_spacer_assembled = True
            self.a_bot_success = True

        def b_bot_task():
            self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot")
            self.despawn_object("shaft")
            if not self.insert_shaft("assembled_part_07_inserted", target=0.043):
                return False
            self.publish_part_in_assembled_position("shaft", marker_only=True)
            self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot", False)
            self.b_bot_success = True

        if simultaneous_execution:
            self.publish_status_text("Target: shaft & bearing spacer")
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=300)
        else:
            b_bot_task()

        if not self.a_bot_success or not self.b_bot_success:
            rospy.logerr("Fail to do simultaneous shaft insertion and bearing spacer")
            return False

        if not self.insert_bearing_spacer("assembled_part_07_inserted", "a_bot"):
            return False

        if not simultaneous_execution:
            if not self.b_bot.go_to_named_pose("home"):
                return False

            self.allow_collisions_with_robot_hand("end_cap", "a_bot", False)
            self.allow_collisions_with_robot_hand("shaft", "b_bot", False)

        return True

    def subtask_d(self):
        rospy.loginfo("======== SUBTASK D (output pulley) ========")

        self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
        self.b_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)

        self.allow_collisions_with_robot_hand("bearing_spacer", "a_bot", True)
        self.allow_collisions_with_robot_hand("output_pulley", "a_bot", True)

        self.publish_status_text("Target: bearing_spacer")

        if not self.pick_bearing_spacer():
            return False

        if not self.orient_bearing_spacer():
            rospy.logerr("Fail to complete the playback sequence bearing_spacer_orient")
            return False

        # # Move a_bot to hold shaft from end cap side
        self.a_bot.gripper.close()

        self.confirm_to_proceed("prepare a_bot")

        approach_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, -0.15] + np.deg2rad([-90, -90, -90]).tolist())
        # self.a_bot.go_to_pose_goal(approach_hold_pose)
        pre_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, 0.02] + np.deg2rad([-90, -90, -90]).tolist())
        # self.a_bot.go_to_pose_goal(pre_hold_pose)
        at_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.043, 0.000, 0.02] + np.deg2rad([-90, -90, -90]).tolist())
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

        self.publish_status_text("Target: output_pulley")

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

    def subtask_d_orquestrated(self):
        # bearing spacer
        if not self.assembly_status.bearing_spacer_assembled:
            # Debug/calibration place b_bot to hold shaft
            # approach_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, -0.15] + np.deg2rad([-90,-90,-90]).tolist())
            # self.b_bot.go_to_pose_goal(approach_hold_pose)
            # pre_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
            # self.b_bot.go_to_pose_goal(pre_hold_pose)
            # at_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.043, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
            # self.b_bot.go_to_pose_goal(at_hold_pose)

            if not self.pick_bearing_spacer("a_bot"):
                return False
            if not self.orient_bearing_spacer("a_bot"):
                return False
            if not self.align_bearing_spacer_pre_insertion("a_bot"):
                return False
            self.confirm_to_proceed("fine tune")
            if not self.insert_bearing_spacer("assembled_part_07_inserted", "a_bot"):
                return False

        # output pulley
        if not self.pick_output_pulley("a_bot"):
            return False
        if not self.orient_output_pulley("a_bot"):
            return False
        if not self.align_output_pulley_pre_insertion("a_bot"):
            return False
        self.confirm_to_proceed("fine tune")
        if not self.insert_output_pulley("assembled_part_07_inserted", "a_bot"):
            return False

        return True

        self.a_bot_success = False
        self.b_bot_success = False

        def a_task():
            # TODO: Orient output pulley screws with a_bot
            # if not self.check_output_pulley_angle():
            #     return False
            # self.a_bot_success = True
            self.a_bot_success = False

        def b_task():
            self.b_bot.move_lin_rel([-0.05, 0, 0], speed=1.0)
            if not self.equip_tool("b_bot", "padless_tool_m4"):
                return False
            self.b_bot_success = True

        self.do_tasks_simultaneous(a_task, b_task, timeout=180)

        if not self.a_bot_success or not self.b_bot_success:
            rospy.logerr("Fail to equip tool/align orient pulley angle a:%s b:%s" % (self.a_bot_success, self.b_bot_success))
            return False

        return self.fasten_output_pulley()

    def subtask_e(self):
        # Idler pulley
        rospy.loginfo("======== SUBTASK E (Idler pulley) ========")
        return self.subtask_e_urscript()

    def subtask_e_urscript(self):
        """ A hard-coded UR script version of the subtask. Expects the spacer and pulley to be in the correct locations
            in storage.
        """
        idler_pulley_store_pose = conversions.to_pose_stamped("left_centering_link", [-0.006, 0.003, 0.061, -tau/4, 0, 0])
        idler_spacer_store_pose = conversions.to_pose_stamped("left_centering_link", [-0.006, 0.002, 0.163, -tau/4, 0, 0])

        self.ab_bot.go_to_named_pose("home")

        self.pick_idler_pulley_assembly("a_bot")
        self.simple_place("a_bot", idler_pulley_store_pose, place_height=0.005, approach_height=0.15, axis="x", sign=-1)
        # self.orient_idler_pulley_assembly("a_bot", idler_pulley_store_pose, store=True)

        self.pick_idler_spacer("a_bot")
        self.simple_place("a_bot", idler_spacer_store_pose, place_height=0.0, approach_height=0.15, axis="x", sign=-1)
        # self.orient_idler_pulley_assembly("a_bot", idler_spacer_store_pose, store=True)

        self.a_bot.load_and_execute_program(program_name="wrs2020/asm_idler_pulley_v1.urp", skip_ros_activation=True)
        self.b_bot.load_and_execute_program(program_name="wrs2020/asm_idler_pulley_p1.urp", skip_ros_activation=True)

        # Go through pause dialogues
        self.confirm_to_proceed("Go through pause dialogs manually. Press enter after b_bot went home and a_bot holds the pulley at the ridge.")

        # TODO(cambel): move a_bot to some target pose w.r.t a frame id for any changes in the product arrangement

        # Equip padless tool
        self.equip_tool("b_bot", "padless_tool_m4")
        self.b_bot.go_to_named_pose("horizontal_screw_ready")
        self.b_bot.load_and_execute_program(program_name="wrs2020/asm_idler_pulley_p2.urp", skip_ros_activation=True)

        # Go through pause dialogues again
        self.confirm_to_proceed("Go through pause dialogs manually. Did both robots finish?")

        # When a_bot is finished:
        self.equip_nut_tool()
        self.fasten_idler_pulley_with_nut_tool(target_link="assembled_part_03_pulley_ridge_top")

        def a_bot_task():
            return self.unequip_nut_tool()

        def b_bot_task():
            success = self.playback_sequence("idler_pulley_return_screw_tool")
            if not success:
                rospy.logerr("Fail to complete idler_pulley_return_screw_tool")
            return success and self.unequip_tool("b_bot", "padless_tool_m4")

        if not self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60):
            return False

        return True

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

        def b_bot_task():  # Pick tool & screw, then wait
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

        def a_bot_task():  # Pick and orient panel
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
            self.pick_screw_from_feeder("b_bot", screw_size=4, realign_tool_upon_failure=True)

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
        if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel, approach_from_front=approach_from_front):
            # Fallback for screw 1
            rospy.logerr("Failed to fasten panel screw 1, trying to realign tool and retry.")
            self.realign_tool("b_bot", "screw_tool_m4")
            self.b_bot.go_to_named_pose("feeder_pick_ready")
            self.pick_screw_from_feeder("b_bot", screw_size=4)

            # Realign plate
            self.a_bot.gripper.close(force=100)
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
            if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel, approach_from_front=approach_from_front):
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
            self.pick_screw_from_feeder("b_bot", screw_size=4, realign_tool_upon_failure=True)
        self.do_tasks_simultaneous(a_bot_task3, b_bot_task3, timeout=180.0)
        if not self.tools.screw_is_suctioned.get("m4", False):
            rospy.logerr("Failed to pick second screw from feeder, could not fix the issue. Abort.")

        screw_target_pose.header.frame_id = part_name + "bottom_screw_hole_2"
        if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel, approach_from_front=approach_from_front):
            # Fallback for screw 2: Realign tool, recenter plate, try again
            rospy.logerr("Failed to fasten panel screw 2, trying to realign tool and retrying.")
            self.realign_tool("b_bot", "screw_tool_m4")
            self.b_bot.go_to_named_pose("feeder_pick_ready")
            self.pick_screw_from_feeder("b_bot", screw_size=4)

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
            self.a_bot.gripper.close(force=100)
            self.a_bot.gripper.open()
            if not self.a_bot.go_to_named_pose("home"):
                rospy.logerr("Failed to move a_bot home!")
                return False
            if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel, approach_from_front=approach_from_front):
                rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
                return False
        self.unlock_base_plate()
        rospy.sleep(0.5)
        self.lock_base_plate()
        self.allow_collisions_with_robot_hand(panel, "a_bot", allow=False)
        return True

    def panels_tasks_combined(self, simultaneous=True, pick_and_orient_insert_bearing=False,
                              pick_and_orient_insert_motor=False, do_base_plate_first=True):
        panels_order = ["panel_bearing", "panel_motor"]
        switch_panels_order = self.assembly_database.assembly_info.get("switched_motor_and_bearing", False)
        if switch_panels_order:
            panels_order = panels_order[::-1]

        print("self.assembly_status.completed_subtask_zero", self.assembly_status.completed_subtask_zero)
        if do_base_plate_first and not self.assembly_status.completed_subtask_zero:
            self.b_bot.go_to_named_pose("home")
            self.publish_status_text("Target: base plate")
            if not self.subtask_zero(skip_initial_perception=False):
                if not self.subtask_zero(skip_initial_perception=False):  # Try again
                    return False
            self.assembly_status.completed_subtask_zero = True
            self.publish_part_in_assembled_position("base", marker_only=True)
            self.a_bot.go_to_named_pose("home", speed=1.0)

        self.publish_status_text("Target: L-plates")

        # Pick bearing panel
        success = False
        for i in range(5):
            success = self.pick_panel_with_handover(panels_order[0])
            if success:
                break
        above_centering_joint_pose = [0.48, -2.05, 2.05, -1.55, -1.58, -1.09-(tau/2)]
        self.a_bot.move_joints(above_centering_joint_pose, speed=1.0)

        self.panel_bearing_pose = None
        self.b_bot_success = False

        # Store bearing panel, pick motor panel
        def a_bot_task():
            self.panel_bearing_pose = self.center_panel(panels_order[0], store=True)
            self.assembly_status.bearing_panel_placed_outside_of_tray = True

        def b_bot_task():
            self.b_bot_success = self.pick_panel_with_handover(panels_order[1], simultaneous=False)
            above_centering_joint_pose = [0.48, -2.05, 2.05, -1.55, -1.58, -1.09-(tau/2)]
            self.a_bot.move_joints(above_centering_joint_pose, speed=1.0)

        if simultaneous:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
        else:
            a_bot_task()
            b_bot_task()
        if not self.b_bot_success:
            rospy.logerr("Fail to do panels_assembly 1: simultaneous=%s b:%s" % (simultaneous, self.b_bot_success))
            self.return_l_plates()
            return False

        # Store motor panel, look at base plate with b_bot
        self.panel_motor_pose = None

        def a_bot_task():
            self.panel_motor_pose = self.center_panel(panels_order[1], store=True)
            self.assembly_status.motor_panel_placed_outside_of_tray = True

        def b_bot_task():
            if pick_and_orient_insert_motor:
                rospy.loginfo("Picking motor")
                self.assembly_status.motor_picked = self.pick_motor()
                self.b_bot.go_to_named_pose("centering_area", speed=1.0)

        if simultaneous:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=300)
        else:
            a_bot_task()
            b_bot_task()

        if not self.panel_motor_pose or (pick_and_orient_insert_motor and not self.assembly_status.motor_picked):
            rospy.logerr("Fail to do panels_assembly 2: simultaneous=%s a:%s b:%s" % (simultaneous, bool(self.panel_motor_pose), self.assembly_status.motor_picked))
            return False

        # Pick base plate with a_bot, prepare fastening with b_bot
        self.a_bot_success = False
        self.b_bot_success = False
        self.panel_bearing_picked = False

        def a_bot_task():
            if not self.assembly_status.completed_subtask_zero:
                self.publish_status_text("Target: base plate")
                if not self.subtask_zero(skip_initial_perception=False):
                    if not self.subtask_zero(skip_initial_perception=False):  # Try again
                        return False
            if simultaneous:
                if not self.place_panel("a_bot", panels_order[0], pick_again=True, pick_only=True, fake_position=True):
                    rospy.logerr("Fail to place bearing panel in simultaneous!!")
                    return False
                self.panel_bearing_picked = True
            self.a_bot_success = True

        def b_bot_task():
            if pick_and_orient_insert_motor and self.assembly_status.motor_picked:
                rospy.sleep(5)
                self.assembly_status.motor_oriented = self.orient_motor()
                if not self.assembly_status.motor_oriented:
                    self.assembly_status.motor_placed_outside_of_tray = True
                self.b_bot.move_lin_rel(relative_translation=[0, 0, 0.06], speed=0.3)
            start_time = rospy.get_time()
            while not self.b_bot_success and rospy.get_time()-start_time < 20:
                self.b_bot_success = self.do_change_tool_action("b_bot", equip=True, screw_size=4)
            self.b_bot.go_to_named_pose("screw_ready")

        if simultaneous:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=150)
        else:
            a_bot_task()
            b_bot_task()
        if not self.b_bot_success or not self.a_bot_success:
            rospy.logerr("Fail to do panels_assembly3: simultaneous=%s a:%s b:%s" % (simultaneous, self.a_bot_success, self.b_bot_success))
            if not self.do_change_tool_action("b_bot", equip=False, screw_size=4):
                raise  # Something is very wrong if this fails
            self.ab_bot.go_to_named_pose("home")
            self.return_l_plates()
            return False

        self.publish_status_text("Target: L-plates")
        self.a_bot_success = False
        self.b_bot_success = False

        def b_bot_task():
            self.b_bot_success = self.pick_screw_from_feeder("b_bot", screw_size=4)
            self.b_bot.go_to_named_pose("feeder_pick_ready")

        def a_bot_task():
            rospy.sleep(1)
            if not self.place_panel("a_bot", panels_order[0], pick_again=(not self.panel_bearing_picked), fake_position=True):
                return False
            if simultaneous:
                if not self.hold_panel_for_fastening(panels_order[0]):
                    return False
            self.a_bot_success = True

        self.publish_status_text("Target: fasten panel bearing")

        if simultaneous:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
        else:
            a_bot_task()
            b_bot_task()
        if not self.b_bot_success or not self.a_bot_success:
            rospy.logerr("Fail to do panels_assembly 4: simultaneous=%s a:%s b:%s" % (simultaneous, self.a_bot_success, self.b_bot_success))
            self.drop_in_tray("a_bot")
            if not self.do_change_tool_action("b_bot", equip=False, screw_size=4):
                raise  # Something is very wrong if this fails
            self.ab_bot.go_to_named_pose("home")
            self.return_l_plates()
            return False

        # Fasten plates
        self.publish_status_text("Target: %s" % panels_order[0])
        self.panel_motor_picked = False

        def a_bot_2nd_task():
            self.panel_motor_picked = self.place_panel("a_bot", panels_order[1], pick_again=True, pick_only=True, fake_position=True)
            return True

        def dummy_Task(): return True
        if not self.fasten_panel(panels_order[0], simultaneous=simultaneous, a_bot_task_2nd_screw=dummy_Task):
            return False

        self.a_bot_success = False
        self.b_bot_success = False

        def a_bot_task():
            if not self.place_panel("a_bot", panels_order[1], pick_again=True, fake_position=True):
                self.drop_in_tray("a_bot")
                return False
            if not self.hold_panel_for_fastening(panels_order[1]):
                return False
            self.a_bot_success = True

        def b_bot_task():
            self.pick_screw_from_feeder("b_bot", screw_size=4)
            self.b_bot_success = True

        self.publish_status_text("Target: %s" % panels_order[1])
        if simultaneous:
            self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
        else:
            a_bot_task()
            b_bot_task()

        if not self.b_bot_success or not self.a_bot_success:
            rospy.logerr("Fail to do panels_assembly 5: simultaneous=%s" % simultaneous)
            self.do_change_tool_action("b_bot", equip=False, screw_size=4)
            self.return_l_plates()
            return False

        if pick_and_orient_insert_bearing:
            def a_bot_2nd_task():
                self.publish_status_text("Target: Bearing")
                if not self.pick_bearing("a_bot"):
                    rospy.logerr("Fail to pick bearing (1). abort")
                    return False
                self.assembly_status.bearing_picked = True
                if not self.orient_bearing("assembly", "a_bot", part1=True, part2=True):
                    rospy.logerr("Fail to orient bearing (1). abort")
                    self.drop_in_tray("a_bot")
                    return False
                self.assembly_status.bearing_oriented = True
                self.use_storage_on_failure = True  # DANGER
                rospy.logwarn("Setting self.use_storage_on_failure to True")
                if not self.insert_bearing("assembled_part_07_inserted", robot_name="a_bot"):
                    rospy.logerr("Fail to insert bearing (1). Try again with more noise.")
                    if not self.pick_bearing("a_bot"):
                        rospy.logerr("Fail to pick bearing (1). abort")
                        return False
                    self.assembly_status.bearing_picked = True
                    if not self.orient_bearing("assembly", "a_bot", part1=True, part2=True):
                        rospy.logerr("Fail to orient bearing (1). abort")
                        self.drop_in_tray("a_bot")
                        return False
                    self.assembly_status.bearing_oriented = True
                    if not self.insert_bearing_fallback("assembled_part_07_inserted", robot_name="a_bot"):
                        rospy.logerr("Fail to insert bearing again. Abort.")
                        self.a_bot.move_lin_rel(relative_translation=[0.03, 0, 0], speed=0.05)
                        self.drop_in_tray("a_bot")
                        return False

                self.assembly_status.bearing_inserted_in_panel = True

                waypoints = []
                waypoints.append((self.a_bot.move_lin_rel([0.15, 0, 0.0], pose_only=True), 0, 1.0))
                waypoints.append((self.a_bot.move_lin_rel([0.1, -0.3, 0.2], pose_only=True), 0, 1.0))
                waypoints.append(("screw_ready", 0, 1.0))
                if not self.a_bot.move_joints_trajectory(waypoints):
                    rospy.logerr("Fail to go home")
                    self.a_bot.move_lin_rel(relative_translation=[0.1, 0, 0])
                    self.a_bot.move_lin_rel(relative_translation=[0.1, -0.2, 0.1])
                    self.a_bot.go_to_named_pose("home")
                    return False
        else:
            def a_bot_2nd_task(): return True

        if pick_and_orient_insert_motor and self.assembly_status.motor_oriented:
            def b_bot_2nd_task():
                self.publish_status_text("Target: Bearing & Motor")
                midpoint1 = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.25, 0.1, 0.4, tau/2,  0, radians(28)])
                waypoints = []
                waypoints.append((self.b_bot.compute_ik(midpoint1, timeout=0.02, retry=True), 0, 1.0))
                waypoints.append(("centering_area", 0, 1.0))
                self.b_bot.move_joints_trajectory(waypoints)
                if not self.align_motor_pre_insertion():
                    return False
                if not self.insert_motor("assembled_part_02_back_hole"):
                    rospy.logerr("Fail to insert motor!!")
                    if self.b_bot.is_protective_stopped():
                        rospy.logfatal("Something is very wrong. Trying to unlock and proceed")
                        self.b_bot.unlock_protective_stop()
                        self.b_bot.gripper.open()
                    self.b_bot.move_lin_rel(relative_translation=[-0.05, 0, 0], speed=0.05)
                    self.b_bot.move_lin_rel(relative_translation=[0, 0.05, 0.1])
                    self.b_bot.go_to_pose_goal(midpoint1)
                    self.b_bot.go_to_named_pose("centering_area")
                    # self.orient_motor_in_aid_edge()
                    place_pose = conversions.to_pose_stamped("right_centering_link", [0.0, 0, 0, 0, 0, 0])
                    self.simple_place("b_bot", place_pose, axis='x', sign=-1, approach_height=0.15, item_id_to_detach='motor', place_height=0.03)
                    self.assembly_status.motor_placed_outside_of_tray = True
                    self.assembly_status.motor_picked = False
                    self.assembly_status.motor_oriented = False
                    return False
                self.despawn_object("motor")
                self.publish_part_in_assembled_position("motor", marker_only=True)
                self.assembly_status.motor_inserted_in_panel = True
                return True
        else:
            self.assembly_status.motor_inserted_in_panel = False
            def b_bot_2nd_task(): return True

        # Optimitic approach
        # if not self.fasten_panel(panels_order[1], simultaneous=simultaneous, a_bot_task_2nd_screw=a_bot_2nd_task, unequip_tool_on_success=True, b_bot_2nd_task=b_bot_2nd_task):
        if not self.fasten_panel(panels_order[1], simultaneous=simultaneous, unequip_tool_on_success=True):
            self.do_change_tool_action("b_bot", equip=False, screw_size=4)
            return False

        self.do_change_tool_action("b_bot", equip=False, screw_size=4)

        rospy.loginfo("===== Panels assembly completed! =====")
        rospy.loginfo("===== Assembly status: %s =====" % str(vars(self.assembly_status)))

        del self.panel_bearing_pose
        del self.panel_motor_pose
        return True

    def subtask_h(self):
        # Attach belt
        rospy.loginfo("======== SUBTASK H (belt) ========")
        rospy.logerr("Subtask H not implemented yet")
        return False

    def subtask_i(self):
        # Insert motor cables
        rospy.loginfo("======== SUBTASK I (cables) ========")
        self.insert_motor_cables_without_tools_normal("red")
        self.insert_motor_cables_without_tools_normal("black")
        return False

    def exhibition_1(self):
        """ Fasten motor and bearing from their insertion position (motor has 2 screws already) 
        """
        def a_bot_task():
            self.fasten_motor("a_bot", part1=False)
            self.unequip_tool("a_bot")
            self.a_bot.go_to_named_pose("home")

        def b_bot_task():
            self.fasten_bearing("assembly", with_extra_retighten=False, robot_name="b_bot")
            self.unequip_tool("b_bot")
            self.b_bot.go_to_named_pose("home")

        self.do_tasks_simultaneous(a_bot_task, b_bot_task)

    def exhibition_2(self):
        """    insert motor pulley and shaft+endcap    """
        self.subtask_b(simultaneous_execution=True)
        self.unequip_tool("b_bot", "set_screw_tool")
        self.ab_bot.go_to_named_pose("home")

        self.pick_end_cap()

        def a_bot_task():
            self.orient_shaft_end_cap("a_bot")
            pass

        def b_bot_task():
            approach_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.100, -0.001, -0.005, tau/2., 0, 0])
            on_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [0.000, -0.001, -0.005, tau/2., 0, 0])
            grasp_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [0.012, -0.001, -0.005, tau/2., 0, 0])
            self.b_bot.go_to_pose_goal(approach_vgroove)
            self.b_bot.go_to_pose_goal(on_vgroove, move_lin=True, speed=0.1)
            self.b_bot.go_to_pose_goal(grasp_pose, move_lin=True, speed=0.05)
            self.b_bot.gripper.close()
            self.b_bot.go_to_pose_goal(on_vgroove, move_lin=True, speed=0.1)
            self.b_bot.go_to_pose_goal(approach_vgroove)
            self.b_bot.go_to_named_pose("home")
            # pre_insertion_shaft = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.2, 0, 0, -tau/4.])
            # if not self.b_bot.go_to_pose_goal(pre_insertion_shaft, speed=0.3):

        self.do_tasks_simultaneous(a_bot_task, b_bot_task)

        pre_insertion_shaft = [1.78158, -0.98719, 2.42349, -4.57638, -1.78597, 0.00433]
        if not self.b_bot.move_joints(pre_insertion_shaft, speed=0.4):
            rospy.logerr("Fail to go to pre_insertion_shaft")
            return False

        above_pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [0.000, 0.010, 0.290]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(above_pre_insertion_end_cap, speed=0.6, move_lin=False):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False
        pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [0.001, 0.011, 0.250]+np.deg2rad([-180, 90, -90]).tolist())
        if not self.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.3, move_lin=True):
            rospy.logerr("Fail to go to pre_insertion_end_cap")
            return False
        self.confirm_to_proceed('finetune')

        # self.confirm_to_proceed("insertion of end cap")
        if not self.insert_end_cap():
            rospy.logerr("failed to insert end cap. maybe")
            # return False
        self.despawn_object("end_cap")
        self.a_bot.gripper.forget_attached_item()

        # self.confirm_to_proceed("Did insertion succeed? Press Enter to open gripper")

        self.a_bot.gripper.send_command(0.06, velocity=0.01)
        self.a_bot.move_lin_rel([0, 0, 0.05], speed=0.3)
        self.a_bot.gripper.detach_object("end_cap")
        self.despawn_object("end_cap")

        self.confirm_to_proceed("prepare screw")

        if not self.fasten_end_cap():
            return False

        if not self.a_bot.go_to_named_pose("home"):
            return False

        if not self.align_shaft("assembled_part_07_inserted", pre_insert_offset=0.065):
            return False
        self.b_bot.gripper.forget_attached_item()

        self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot")
        self.despawn_object("shaft", collisions_only=True)
        self.confirm_to_proceed('finetune')
        if not self.insert_shaft("assembled_part_07_inserted", target=0.043):
            return False
        self.publish_part_in_assembled_position("shaft", marker_only=True)
        self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot", False)

        self.ab_bot.go_to_named_pose("home")
    ##############

    def mtc_pick_screw_tool(self, screw_type):
        rospy.loginfo("======== PICK TASK ========")
        success = False
        if screw_type in ['m3', 'm4']:
            return self.do_plan_pick_action('screw_tool_' + screw_type, 'tools', 'screw_tool_m3_pickup_link', [-1.0, 0.0, 0.0], save_solution_to_file='pick_screw_tool')

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
            return self.do_plan_fastening_action('screw_tool_' + screw_type, screw_pickup_pose, object_subframe_to_place=screw_tool_tip_frame, save_solution_to_file='pick_screw')

    def mtc_place_object_in_tray_center(self, object_name):
        rospy.loginfo("======== PLACE TASK ========")
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'tray_center'
        target_pose.pose.position.x = -0.04
        target_pose.pose.position.y = 0.08
        target_pose.pose.orientation.w = 1
        self.do_plan_place_action(object_name, target_pose, save_solution_to_file='place_' + object_name)

    def mtc_pickplace_l_panel(self):
        rospy.loginfo("======== PICKPLACE TASK ========")

        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = 'base/screw_hole_panel2_1'
        target_pose.pose.orientation.w = 1

        self.do_plan_pickplace_action('panel_bearing', target_pose, object_subframe_to_place='panel_bearing/bottom_screw_hole_aligner_1',
                                      robot_names=['b_bot', 'a_bot'], force_robot_order=True, save_solution_to_file='pickplace')

    def mtc_pick_place_task(self):
        rospy.loginfo("======== PICK-PLACE TASK ========")
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
        pose.pose.orientation.w = 1
        return self.do_plan_pickplace_action('b_bot', 'panel_bearing', pose, save_solution_to_file='panel_bearing/bottom_screw_hole_aligner_1')

    def update_assembly_display(self, assembly_status=None):
        if assembly_status is None:
            assembly_status = self.assembly_status

        if assembly_status.completed_subtask_zero:
            self.publish_part_in_assembled_position("base", marker_only=True)
        if assembly_status.completed_subtask_a:
            self.publish_part_in_assembled_position("motor", marker_only=True)
        if assembly_status.completed_subtask_b:
            self.publish_part_in_assembled_position("motor_pulley", marker_only=True)
        if assembly_status.completed_subtask_c1:
            self.publish_part_in_assembled_position("motor", marker_only=True)
        # if assembly_status.completed_subtask_d:
        #   self.publish_part_in_assembled_position("")
        # if assembly_status.completed_subtask_e:
        #   self.publish_part_in_assembled_position("")
        if assembly_status.completed_subtask_f:
            self.publish_part_in_assembled_position("panel_motor", marker_only=True)
        if assembly_status.completed_subtask_g:
            self.publish_part_in_assembled_position("panel_bearing", marker_only=True)

    def assemble_drive_unit_orchestrated(self, tray_name=None, simultaneous_execution=True):
        if not self.assembly_status.tray_placed_on_table and tray_name:
            if not self.pick_tray_from_agv_stack_calibration_long_side(tray_name=tray_name):
                rospy.logerr("Fail to pick and place tray. Abort!")
                return False
            self.assembly_status.tray_placed_on_table = True

        self.update_assembly_display()

        if not self.assembly_status.completed_subtask_f and not self.assembly_status.completed_subtask_g \
                and not self.assembly_status.bearing_panel_placed_outside_of_tray and not self.assembly_status.motor_panel_placed_outside_of_tray:

            print("Starting simultaneous Plates!")
            # L-plates and base plate
            success = self.panels_tasks_combined(simultaneous=simultaneous_execution,
                                                 pick_and_orient_insert_bearing=False,
                                                 pick_and_orient_insert_motor=False)
            if success:
                self.assembly_status.completed_subtask_zero = True
                self.assembly_status.completed_subtask_f = True
                self.assembly_status.completed_subtask_g = True
            else:
                rospy.logfatal("Fail to assemble panels... call a reset!")
                raise
        self.ab_bot.go_to_named_pose("home")

        self.update_assembly_display()

        return self.assemble_drive_unit(tray_name)

    def unload_assembled_unit(self, tray_name=None):
        self.publish_status_text("Unloading")
        self.unequip_tool("a_bot")
        self.unequip_tool("b_bot")
        self.reset_scene_and_robots()
        self.ab_bot.go_to_named_pose("home", speed=1.0)
        if not self.unload_drive_unit():
            rospy.logerr("Fail to unload drive unit. Abort!")
            return False
        if tray_name:
            self.publish_status_text("Returning tray")
            if not self.return_tray_to_agv_stack_calibration_long_side(tray_name):
                rospy.logerr("Fail to return tray. Abort!")
                return False
        return True

    def assemble_drive_unit(self, tray_name=None):
        # ======= Tray ========
        if not self.assembly_status.tray_placed_on_table:
            rospy.loginfo("=== Pick tray from AGV: START ===")
            if tray_name:
                if not self.pick_tray_from_agv_stack_calibration_long_side(tray_name=tray_name):
                    rospy.logerr("Fail to pick and place tray. Abort!")
                    return False
            rospy.loginfo("=== Pick tray from AGV: FINISH ===")

        self.update_assembly_display()

        # ======= Base Plate ========
        if not self.assembly_status.completed_subtask_zero:
            rospy.loginfo("=== subtask ZERO: START ===")
            self.b_bot.go_to_named_pose("home")
            self.publish_status_text("Target: base plate")
            if not self.subtask_zero(skip_initial_perception=False):
                if not self.subtask_zero(skip_initial_perception=False):  # Try again
                    rospy.logfatal("Fail to do subtask zero! call a reset")
                    raise  # Nothing to do other than reset
            self.a_bot.go_to_named_pose("home")
            rospy.loginfo("=== subtask ZERO: FINISH ===")

        self.update_assembly_display()

        # ======= L-Plates ========

        def do_panel(panel_name, placed_outside_of_tray, subtask_completed, start_with_fallback=False):
            if not subtask_completed:
                rospy.loginfo("=== subtask " + panel_name + ": START ===")
                if not placed_outside_of_tray:
                    for i in range(3):
                        success = self.pick_panel_with_handover(panel_name, simultaneous=True, rotate_on_failure=True)
                        if success:
                            break
                    if success:
                        self.center_panel(panel_name, store=True)
                        placed_outside_of_tray = True
                    else:
                        rospy.logerr("Could not pick bearing panel!")
                if placed_outside_of_tray:
                    success = self.place_panel("a_bot", panel_name, pick_again=True, fake_position=True)
                    if start_with_fallback:
                        self.center_panel_on_base_plate(panel_name)
                if success:
                    self.hold_panel_for_fastening(panel_name)
                    success = self.fasten_panel(panel_name, simultaneous=True, unequip_tool_on_success=True)
                subtask_completed = success
                rospy.loginfo("=== subtask " + panel_name + ": Finish (%s) ===" % success)
            return subtask_completed

        switch_panels_order = self.assembly_database.assembly_info.get("switched_motor_and_bearing", False)
        if switch_panels_order:
            do_panel("panel_motor", self.assembly_status.motor_panel_placed_outside_of_tray, self.assembly_status.completed_subtask_f)
            do_panel("panel_bearing", self.assembly_status.bearing_panel_placed_outside_of_tray, self.assembly_status.completed_subtask_g)
        else:
            do_panel("panel_bearing", self.assembly_status.bearing_panel_placed_outside_of_tray, self.assembly_status.completed_subtask_g, start_with_fallback=True)
            do_panel("panel_motor", self.assembly_status.motor_panel_placed_outside_of_tray, self.assembly_status.completed_subtask_f)

        self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
        self.do_change_tool_action("b_bot", equip=False, screw_size=4)

        self.update_assembly_display()

        # ======= Bearing ========
        if not self.assembly_status.completed_subtask_c1:
            rospy.loginfo("=== subtask C1: START ===")
            self.assembly_status.completed_subtask_c1 = self.subtask_c1()  # bearing
            rospy.loginfo("=== subtask C1: Finish (%s) ===" % self.assembly_status.completed_subtask_c1)

        self.update_assembly_display()

        # # ======= Motor ========
        # if not self.assembly_status.completed_subtask_a:
        #   rospy.loginfo("=== subtask A: START ===")
        #   self.assembly_status.completed_subtask_a = self.subtask_a() # motor
        #   rospy.loginfo("=== subtask A: Finish (%s) ===" % self.assembly_status.completed_subtask_a)
        # self.do_change_tool_action("a_bot", equip=False)

        # # ======= Motor Pulley ========
        # if self.assembly_status.completed_subtask_a and not self.assembly_status.completed_subtask_b:
        #   rospy.loginfo("=== subtask B: START ===")
        #   self.assembly_status.completed_subtask_b = self.subtask_b() # motor
        #   rospy.loginfo("=== subtask B: Finish (%s) ===" % self.assembly_status.completed_subtask_b)
        # self.do_change_tool_action("a_bot", equip=False)

        # # ======= Shaft ========
        # if self.assembly_status.completed_subtask_c1 and not self.assembly_status.completed_subtask_c2:
        #   rospy.loginfo("=== subtask C2: START ===")
        #   self.assembly_status.completed_subtask_c2 = self.subtask_c2() # shaft
        #   rospy.loginfo("=== subtask C2: Finish (%s) ===" % self.assembly_status.completed_subtask_c2)

        # # ======= Output Pulley ========
        # if self.assembly_status.completed_subtask_c2 and not self.assembly_status.completed_subtask_d:
        #   rospy.loginfo("=== subtask D: START ===")
        #   self.assembly_status.completed_subtask_d = self.subtask_d() # bearing spacer/output pulley
        #   rospy.loginfo("=== subtask D: Finish (%s) ===" % self.assembly_status.completed_subtask_d)

        # # ======= Idler Pulley ========
        # if not self.assembly_status.completed_subtask_e:
        #   rospy.loginfo("=== subtask E: START ===")
        #   self.assembly_status.completed_subtask_e = self.subtask_e()
        #   rospy.loginfo("=== subtask E: Finish (%s) ===" % self.assembly_status.completed_subtask_)

        # # ======= Motor Cables ========
        # if not self.assembly_status.completed_subtask_i1 or not self.assembly_status.completed_subtask_i2:
        #   rospy.loginfo("=== subtask I: START ===")
        #   success = self.subtask_i()
        #   rospy.loginfo("=== subtask E: Finish (%s) ===" % success)

        self.unload_assembled_unit(tray_name)
        rospy.loginfo("==== Finished. ====")

    def full_assembly_task(self, simultaneous_execution=True):
        self.ab_bot.go_to_named_pose("home")
        self.reset_scene_and_robots()
        orders = []
        orders.append({"tray_name": "tray1", "assembly_name": "wrs_assembly_2021_surprise", "status": self.get_first_order_status()})  # Top tray
        orders.append({"tray_name": "tray2", "assembly_name": "wrs_assembly_2021", "status": self.get_second_order_status()})  # Bottom tray

        simultaneous = [True, True]
        unload_right_away = [False, False]

        if not orders[0]["status"].tray_placed_on_table:
            print("get from AGV")

            def load_first_assembly():
                self.set_assembly(orders[0]["assembly_name"])
            # self.do_tasks_simultaneous(load_first_assembly, self.center_tray_stack, timeout=90)
            load_first_assembly()
            self.center_tray_stack()
        else:
            print("already on the table")
            self.set_assembly(orders[0]["assembly_name"])
        stack_center = [-0.03, 0]
        tray_heights = [0.03, -0.02]
        self.trays = {"tray%s" % (i+1): (stack_center+[tray_height], True) for i, tray_height in enumerate(tray_heights)}
        self.trays_return = {"tray%s" % (i+1): (stack_center+[tray_height], True) for i, tray_height in enumerate(tray_heights[::-1])}

        for i, order in enumerate(orders):
            self.assembly_status = order["status"]

            if self.assembly_status.tray_delivered_to_agv:
                rospy.loginfo("Order nr. " + str(i) + " already completed! Skipping.")
                continue
            if not self.assembly_status.tray_delivered_to_agv and self.assembly_status.assembly_unloaded:
                if not self.return_tray_to_agv_stack_calibration_long_side(order["tray_name"]):
                    rospy.logerr("Fail to return tray. Abort, call reset!")
                    return False
                continue

            self.set_assembly(order["assembly_name"])

            if i == 1 and not self.assembly_status.tray_placed_on_table:
                self.spawn_tray_stack(orientation_parallel=True, spawn_single_tray=True)
            if unload_right_away[i]:  # Start by unloading the tray into the AGV
                self.unload_assembled_unit(order["tray_name"])
            else:  # Normal execution
                if simultaneous[i]:
                    self.assemble_drive_unit_orchestrated(order["tray_name"], simultaneous_execution)
                else:
                    self.assemble_drive_unit(order["tray_name"])
        rospy.loginfo("==== Finished both tasks ====")
        return

    def get_first_order_status(self):
        """ A convenience function to define the status of the first order (to be used after a reset in the competition)
        """
        s = AssemblyStatus()
        s.tray_placed_on_table = True  # Needs to be True when doing second set only!

        s.bearing_panel_placed_outside_of_tray = False
        s.motor_panel_placed_outside_of_tray = False

        s.belt_placed_outside_of_tray = False

        s.motor_picked = False
        s.motor_oriented = False
        s.motor_placed_outside_of_tray = False
        s.motor_inserted_in_panel = False

        s.bearing_placed_outside_of_tray = False
        s.bearing_picked = False
        s.bearing_oriented = False
        s.bearing_inserted_in_panel = False
        s.bearing_holes_aligned = False
        s.bearing_spacer_assembled = False

        s.idler_pulley_spacer_placed_outside_of_tray = False
        s.idler_pulley_placed_outside_of_tray = False

        s.completed_subtask_zero = True  # Base
        s.completed_subtask_a = False  # Motor
        s.completed_subtask_b = False  # Motor pulley
        s.completed_subtask_c1 = True  # Bearing
        s.completed_subtask_c2 = False  # Shaft
        s.completed_subtask_d = False  # Fasten output pulley
        s.completed_subtask_e = False  # Output pulley
        s.completed_subtask_f = True  # Motor plate
        s.completed_subtask_g = False  # Bearing plate
        s.completed_subtask_h = False  # Belt
        s.completed_subtask_i1 = False  # Cable 1
        s.completed_subtask_i2 = False  # Cable 2

        s.assembly_unloaded = False

        s.tray_delivered_to_agv = False
        return s

    def get_second_order_status(self):
        """ A convenience function to define the status of the second order (to be used after a reset in the competition)
        """
        s = AssemblyStatus()
        s.tray_placed_on_table = False

        s.bearing_panel_placed_outside_of_tray = False
        s.motor_panel_placed_outside_of_tray = False

        s.belt_placed_outside_of_tray = False

        s.motor_picked = False
        s.motor_oriented = False
        s.motor_placed_outside_of_tray = False
        s.motor_inserted_in_panel = False

        s.bearing_placed_outside_of_tray = False
        s.bearing_picked = False
        s.bearing_oriented = False
        s.bearing_inserted_in_panel = False
        s.bearing_holes_aligned = False
        s.bearing_spacer_assembled = False

        s.idler_pulley_spacer_placed_outside_of_tray = False
        s.idler_pulley_placed_outside_of_tray = False

        s.completed_subtask_zero = False  # Base
        s.completed_subtask_a = False  # Motor
        s.completed_subtask_b = False  # Motor pulley
        s.completed_subtask_c1 = True  # Bearing
        s.completed_subtask_c2 = False  # Shaft
        s.completed_subtask_d = False  # Fasten output pulley
        s.completed_subtask_e = False  # Output pulley
        s.completed_subtask_f = False  # Motor plate
        s.completed_subtask_g = False  # Bearing plate
        s.completed_subtask_h = False  # Belt
        s.completed_subtask_i1 = False  # Cable 1
        s.completed_subtask_i2 = False  # Cable 2

        s.assembly_unloaded = False

        s.tray_delivered_to_agv = False
        return s

    def assemble_drive_unit_simultaneous(self):
        # This is the v2. "orchestrated" is v3.
        if not self.assembly_status.tray_placed_on_table:
            self.center_tray_stack()
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
            self.assembly_status.completed_subtask_f = self.subtask_f()  # Motor plate

        self.a_bot.go_to_named_pose("home", speed=self.speed_fastest, acceleration=self.acc_fastest)
        self.do_change_tool_action("b_bot", equip=False, screw_size=4)

        if self.assembly_status.completed_subtask_g:  # Bearing plate
            self.assembly_status.completed_subtask_c1 = self.subtask_c1()  # bearing
            # if self.assembly_status.completed_subtask_c1:
            #   self.assembly_status.completed_subtask_c2 = self.subtask_c2() # shaft
            #   if self.assembly_status.completed_subtask_c2:
            #     self.assembly_status.completed_subtask_e = self.subtask_e() # bearing spacer / output pulley

        self.ab_bot.go_to_named_pose("home")
        self.unload_drive_unit()
        self.return_tray_to_agv_stack_calibration_long_side("tray1")
        self.assembly_status = AssemblyStatus()
        rospy.loginfo("==== Finished.")
