#!/usr/bin/env python

from math import pi
tau = 2*pi
import numpy as np
import rospy
import actionlib

from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceResult
from o2ac_msgs.msg import PickAction, PickResult
from o2ac_msgs.msg import InsertAction, InsertResult
from o2ac_msgs.msg import AlignBearingHolesAction, AlignBearingHolesResult
from o2ac_msgs.msg import FastenAction, FastenResult
from o2ac_msgs.msg import OrientAction, OrientResult
from o2ac_msgs.msg import MoveToAction, MoveToResult
from o2ac_routines.common import O2ACCommon
from o2ac_routines.assembly import O2ACAssembly

from ur_control import conversions

import sys
import signal


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class SkillServer:
    def __init__(self):
        # self.controller = O2ACCommon()
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
        # TODO(cambel): given an object name, find the magic numbers for gripper.opening_width,
        # offset from ssd to pick_pose, parameters for picking (check close to border, etc) and others if needed

        # who decides which robot to use? for now let's assume that it is manual
        robot_name = goal.robot_name
        pick_pose = self.controller.look_and_get_grasp_point(goal.object_name)

        if not pick_pose:  # TODO(cambel): receive number of attempts
            rospy.logerr("Could not find bearing in tray. Skipping procedure.")
            self.pick_action.set_succeeded(result=PickResult(False, 0.0))
            return

        self.controller.vision.activate_camera(robot_name + "_inside_camera")
        pick_pose.pose.position.x -= 0.01  # MAGIC NUMBER
        pick_pose.pose.position.z = 0.0115
        success = self.controller.simple_pick(robot_name, pick_pose, gripper_force=100.0, approach_height=0.05, axis="z", approach_with_move_lin=False)

        if not self.controller.simple_gripper_check(robot_name, min_opening_width=0.005):
            rospy.logerr("Fail to grasp -> %s" % goal.object_name)
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
        else:
            rospy.logerr("Unsupported object_name: %s for orient skill" % goal.object_name)
            self.orient_action.set_aborted(result=OrientResult(False))

        if success:
            rospy.loginfo("Orient completed successfully!")
            self.orient_action.set_succeeded(result=OrientResult(True))
        else:
            rospy.logerr("Orient failed!")
            self.orient_action.set_aborted(result=OrientResult(False))

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
