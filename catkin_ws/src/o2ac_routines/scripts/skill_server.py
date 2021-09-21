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
# Author: Cristian C. Beltran-Hernandez

import signal
import sys
from ur_control import conversions
from o2ac_routines.assembly import O2ACAssembly
from o2ac_msgs.msg import HandoverAction, HandoverResult
from o2ac_msgs.msg import MoveToAction
from o2ac_msgs.msg import OrientAction, OrientResult
from o2ac_msgs.msg import FastenAction, FastenResult
from o2ac_msgs.msg import AlignBearingHolesAction, AlignBearingHolesResult
from o2ac_msgs.msg import InsertAction, InsertResult
from o2ac_msgs.msg import PickAction, PickResult
from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceResult
import actionlib
import rospy
import numpy as np
from math import pi
tau = 2*pi


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class SkillServer:
    """ Action servers for Flexbe Interface """

    def __init__(self):
        self.controller = O2ACAssembly()
        self.controller.reset_scene_and_robots()
        self.controller.competition_mode = True

        ns = 'o2ac_flexbe/'

        self.playback_action = actionlib.SimpleActionServer(ns + 'playback_sequence', PlayBackSequenceAction, self.execute_playback, False)
        self.playback_action.start()

        self.pick_action = actionlib.SimpleActionServer(ns + 'pick_object', PickAction, self.execute_pick, False)
        self.pick_action.start()

        self.insertion_action = actionlib.SimpleActionServer(ns + 'force_insertion', InsertAction, self.execute_insertion, False)
        self.insertion_action.start()

        self.align_bearing_holes_action = actionlib.SimpleActionServer(ns + 'align_bearing_holes', AlignBearingHolesAction, self.execute_align_bearing_holes, False)
        self.align_bearing_holes_action.start()

        self.fasten_action = actionlib.SimpleActionServer(ns + 'fasten', FastenAction, self.execute_fasten, False)
        self.fasten_action.start()

        self.orient_action = actionlib.SimpleActionServer(ns + 'orient', OrientAction, self.execute_orient, False)
        self.orient_action.start()

        self.handover_action = actionlib.SimpleActionServer(ns + 'handover', HandoverAction, self.execute_handover, False)
        self.handover_action.start()

        self.move_to_action = actionlib.SimpleActionServer(ns + 'move_to', MoveToAction, self.execute_move_to, False)
        self.move_to_action.start()

        # self.controller.assembly_database.change_assembly("taskboard")

    def execute_playback(self, goal):
        success = self.controller.playback_sequence(goal.sequence_name)
        if success:
            rospy.loginfo("Playback sequence completed successfully!")
        else:
            rospy.logerr("Playback sequence failed!")

        self.playback_action.set_succeeded(result=PlayBackSequenceResult(success))

    def execute_pick(self, goal):
        rospy.loginfo("Received a goal for picking")
        # TODO(cambel): given an object name, find the magic numbers for gripper.opening_width,
        # offset from ssd to pick_pose, parameters for picking (check close to border, etc) and others if needed

        robot_name = goal.robot_name
        object_name = goal.object_name

        self.controller.activate_led(robot_name)
        if object_name in ("base", "panel_bearing", "panel_motor"):
            if self.controller.use_real_robot:
                pick_pose = self.controller.get_large_item_position_from_top(object_name, robot_name)
            else:
                pick_pose = conversions.to_pose_stamped("tray_center", [0.02, -0.03, 0.001, 0.0, 0.0, tau/4])
                self.controller.spawn_object(object_name, pick_pose, pick_pose.header.frame_id)
            rospy.sleep(0.2)
            pick_pose = self.controller.assembly_database.get_grasp_pose(object_name, "default_grasp")
            pick_pose.header.frame_id = "move_group/" + pick_pose.header.frame_id
            pick_pose.header.stamp = rospy.Time(0)
            pick_pose = self.controller.listener.transformPose("world", pick_pose)
            self.controller.allow_collisions_with_robot_hand(object_name, robot_name, allow=True)
            success = self.controller.simple_pick(robot_name, object_pose=pick_pose, grasp_width=0.06, approach_height=0.05, grasp_height=0.005,
                                                  axis="z", item_id_to_attach=object_name, lift_up_after_pick=True, approach_with_move_lin=False,
                                                  speed_fast=1.0)
        else:
            pick_pose = self.controller.look_and_get_grasp_point(object_name)

            if not pick_pose:  # TODO(cambel): receive number of attempts
                rospy.logerr("Could not find bearing in tray. Skipping procedure.")
                self.pick_action.set_succeeded(result=PickResult(False, 0.0))
                return

            self.controller.vision.activate_camera(robot_name + "_inside_camera")
            pick_pose.pose.position.x -= 0.01  # MAGIC NUMBER
            pick_pose.pose.position.z = 0.0115
            success = self.controller.simple_pick(robot_name, pick_pose, gripper_force=100.0, approach_height=0.05, axis="z", approach_with_move_lin=False)

        if not self.controller.simple_gripper_check(robot_name, min_opening_width=0.005):
            rospy.logerr("Fail to grasp -> %s" % object_name)
            self.pick_action.set_aborted(result=PickResult(False))
            return

        if success:
            rospy.loginfo("Pickup completed successfully!")
            self.pick_action.set_succeeded(result=PickResult(success))
        else:
            rospy.logerr("Pickup failed!")
            self.pick_action.set_aborted(result=PickResult(success))

    def execute_insertion(self, goal):
        if goal.task_name == "taskboard":
            target_link = "taskboard_assy_part_07_inserted"
        elif goal.task_name == "assembly":
            target_link = "assembled_part_07_inserted"

        success = False
        if goal.object_name == "bearing":
            success = self.controller.insert_bearing(target_link=target_link)
        elif goal.object_name == "motor_pulley":
            success = self.controller.insert_motor_pulley(target_link=target_link)
        elif goal.object_name == "shaft":
            success = self.controller.align_shaft(target_link=target_link, pre_insert_offset=0.06)
            if success:
                success = self.controller.insert_shaft(target_link=target_link)
        elif goal.object_name == "end_cap":
            pre_insertion_shaft = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.2, 0, 0, -tau/4.])
            success = self.controller.b_bot.go_to_pose_goal(pre_insertion_shaft, speed=0.2)
            if not success:
                rospy.logerr("Fail to go to pre_insertion_shaft")
            if success:
                pre_insertion_end_cap = conversions.to_pose_stamped("tray_center", [-0.002, -0.001, 0.25]+np.deg2rad([-180, 90, -90]).tolist())
                if not self.controller.a_bot.go_to_pose_goal(pre_insertion_end_cap, speed=0.2, move_lin=False):
                    rospy.logerr("Fail to go to pre_insertion_end_cap")
                    success = False
            success = self.controller.insert_end_cap()
            self.controller.a_bot.go_to_named_pose("home")
        else:
            rospy.logerr("No insertion supported for object: %s" % goal.object_name)
            self.insertion_action.set_succeeded(result=InsertResult(success))
            return

        if success:
            rospy.loginfo("Insertion completed successfully!")
        else:
            rospy.logerr("Insertion failed!")

        self.insertion_action.set_succeeded(result=InsertResult(success))

    def execute_align_bearing_holes(self, goal):
        success = self.controller.align_bearing_holes(task=goal.task_name)

        if success:
            rospy.loginfo("Alignment completed successfully!")
        else:
            rospy.logerr("Alignment failed!")

        self.align_bearing_holes_action.set_succeeded(result=AlignBearingHolesResult(success))

    def execute_fasten(self, goal):
        if goal.object_name == "bearing":
            success = self.controller.fasten_bearing(task=goal.task_name)
        elif goal.object_name == "end_cap":
            success = self.controller.fasten_end_cap()
        elif goal.object_name == "panel_bearing":
            success = self.controller.fasten_panel(goal.object_name)
        elif goal.object_name == "panel_motor":
            success = self.controller.fasten_panel(goal.object_name)
        else:
            rospy.logerr("Unsupported object_name:%s for fasten skill" % goal.object_name)
            self.fasten_action.set_aborted(result=FastenResult(False))
            return

        if success:
            rospy.loginfo("Fasten completed successfully!")
        else:
            rospy.logerr("Fasten failed!")

        self.fasten_action.set_succeeded(result=FastenResult(success))

    def execute_orient(self, goal):
        if goal.object_name == "bearing":
            success = self.controller.orient_bearing()
        elif goal.object_name == "shaft":
            success = self.controller.orient_shaft()
        elif goal.object_name == "end_cap":
            success = self.controller.orient_shaft_end_cap()
        elif goal.object_name == "panel_bearing":
            success = self.controller.orient_panel(goal.object_name)
        elif goal.object_name == "panel_motor":
            success = self.controller.orient_panel(goal.object_name)
        else:
            rospy.logerr("Unsupported object_name: %s for orient skill" % goal.object_name)
            self.orient_action.set_aborted(result=OrientResult(False))

        if success:
            rospy.loginfo("Orient completed successfully!")
            self.orient_action.set_succeeded(result=OrientResult(True))
        else:
            rospy.logerr("Orient failed!")
            self.orient_action.set_aborted(result=OrientResult(False))

    def execute_handover(self, goal):
        success = False
        handover_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.15, 0.25, 0.0, tau/4, 0.0])
        handover_grasp = "grasp_7" if goal.object_name == "panel_bearing" else "grasp_9"
        success = self.controller.simple_handover(goal.from_robot_name, goal.to_robot_name, handover_pose, goal.object_name, handover_grasp)
        if success:
            rospy.loginfo("Handover completed successfully!")
        else:
            rospy.logerr("Handover failed!")

        self.handover_action.set_succeeded(result=HandoverResult(success))

    def execute_move_to(self, goal):
        robot = self.controller.active_robots[goal.robot_name]
        if goal.target_named_pose:
            robot.go_to_named_pose(goal.target_named_pose)
        elif goal.pose_type == "joints":
            robot.move_joints(goal.target_pose)
        elif goal.pose_type == "cartesian":
            pose_stamped = conversions.to_pose_stamped(goal.frame_id, goal.target_pose[:3] + np.deg2rad(goal.target_pose[3:]).tolist())
            move_lin = True if goal.motion_planner == "linear" else False
            robot.go_to_pose_goal(pose_stamped, move_lin=move_lin)
        else:
            raise ValueError("Invalid goal %s" % goal)


if __name__ == '__main__':
    rospy.init_node('skill_server_py')
    server = SkillServer()
    rospy.spin()
