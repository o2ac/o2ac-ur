#!/usr/bin/env python

import rospy
import actionlib

from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceResult
from o2ac_msgs.msg import PickUpAction, PickUpResult
from o2ac_msgs.msg import InsertAction, InsertResult
from o2ac_msgs.msg import AlignBearingHolesAction, AlignBearingHolesResult
from o2ac_routines.common import O2ACCommon

from ur_control import conversions


class SkillServer:
    def __init__(self):
        self.controller = O2ACCommon()

        self.playback_action = actionlib.SimpleActionServer('playback_sequence', PlayBackSequenceAction, self.execute_playback, False)
        self.playback_action.start()

        self.pickup_action = actionlib.SimpleActionServer('pickup_object', PickUpAction, self.execute_pickup, False)
        self.pickup_action.start()

        self.insertion_action = actionlib.SimpleActionServer('force_insertion', InsertAction, self.execute_insertion, False)
        self.insertion_action.start()

        self.align_bearing_holes_action = actionlib.SimpleActionServer('align_bearing_holes', AlignBearingHolesAction, self.execute_align_bearing_holes, False)
        self.align_bearing_holes_action.start()

        self.controller.assembly_database.change_assembly("taskboard")

    def execute_playback(self, goal):
        success = self.controller.playback_sequence(goal.sequence_name)
        if success:
            rospy.loginfo("Playback sequence completed successfully!")
        else:
            rospy.logerr("Playback sequence failed!")

        self.playback_action.set_succeeded(result=PlayBackSequenceResult(success))

    def execute_pickup(self, goal):
        pick_pose = self.controller.look_and_get_grasp_point(goal.object_name)

        if not pick_pose:
            rospy.logerr("Could not find bearing in tray. Skipping procedure.")
            self.pickup_action.set_succeeded(result=PickUpResult(False, 0.0))
            return

        self.controller.vision.activate_camera("b_bot_inside_camera")
        pick_pose.pose.position.x -= 0.01  # MAGIC NUMBER
        pick_pose.pose.position.z = 0.0115
        success = self.controller.simple_pick("b_bot", pick_pose, gripper_force=100.0, approach_height=0.05, axis="z")
        gripper_opening = self.controller.b_bot.gripper.opening_width

        if gripper_opening < 0.01:
            rospy.logerr("Fail to grasp %s" % goal.object_name)
            self.pickup_action.set_succeeded(result=PickUpResult(False, gripper_opening))
            return

        if success:
            rospy.loginfo("Pickup completed successfully!")
        else:
            rospy.logerr("Pickup failed!")

        self.pickup_action.set_succeeded(result=PickUpResult(success, gripper_opening))

    def execute_insertion(self, goal):
        success = False
        if goal.object_name == "bearing":
            success = self.controller.insert_bearing(task=goal.task_name)
        elif goal.object_name == "motor_pulley":
            success = self.controller.insert_motor_pulley(task=goal.task_name)
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


if __name__ == '__main__':
    rospy.init_node('skill_server_py')
    server = SkillServer()
    rospy.spin()
