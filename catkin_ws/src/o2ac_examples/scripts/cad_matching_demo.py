#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from math import pi

from std_srvs.srv import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from o2ac_msgs.srv import *
import actionlib
import o2ac_msgs.msg

from o2ac_routines.base import O2ACCommon
from o2ac_routines.vision import VisionProxy


def ros_service_proxy(service_name, service_type):
    proxy = None
    try:
        rospy.logdebug("wait for service %s", service_name)
        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("service error: " + str(e))
    return proxy


class VisionDemo(O2ACCommon):
    """
    This contains the routine used to run the vi task.
    """

    def __init__(self):
        super(VisionDemo, self).__init__()
        self._find_object = ros_service_proxy("find_object", FindObject)
        # Use this instead of waiting, so that simulation can be used
        rospy.sleep(.5)

    def find_object(
            self,
            expected_position,
            position_tolerance,
            object_id,
            camera):
        rospy.loginfo("SkillProxy.find_object() begin")
        rospy.loginfo("expected_position: ")
        rospy.loginfo(expected_position.pose.position)
        rospy.loginfo("position_tolerance = " + str(position_tolerance))
        rospy.loginfo("object_id = " + str(object_id))
        rospy.loginfo("camera = " + str(camera))
        try:
            res = self._find_object(
                expected_position,
                position_tolerance,
                object_id,
                camera)
            if res.success:
                return res.object_pose
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
        return None

    ################ ----- Routines
    ################
    ################

    def pick(
            self,
            robotname,
            object_pose,
            grasp_height,
            speed_fast,
            speed_slow,
            gripper_command="",
            approach_height=0.03):
        rospy.loginfo("pick() begin")
        rospy.loginfo("robotname = " + robotname)
        rospy.loginfo("object_pose: ")
        rospy.loginfo(object_pose)
        rospy.loginfo("grasp_height = " + str(grasp_height))
        rospy.loginfo("speed_fast = " + str(speed_fast))
        rospy.loginfo("speed_slow = " + str(speed_slow))

        self.active_robots[robotname].go_to_pose_goal(
            object_pose, speed=speed_fast)

    def place(
            self,
            robotname,
            object_pose,
            place_height,
            speed_fast,
            speed_slow,
            gripper_command="",
            approach_height=0.05,
            lift_up_after_place=True):
        rospy.loginfo("Going above place target")

    def run(self):
        try:
            vision = VisionProxy()

            tf_buffer = tf2_ros.Buffer(
                rospy.Duration(1200.0))  # tf buffer length
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            i = raw_input("Enter the number of the part to be picked up:")
            i = int(i)
            if i > 0:
                rospy.logdebug("find object")
                expected_position = geometry_msgs.msg.PoseStamped()
                # The part number (see the definition in
                # o2ac_parts_description)
                object_id = str(i)
                item_pose = vision.find_object(
                    expected_position,
                    position_tolerance=0.2,
                    object_id=object_id,
                    camera="b_bot_camera")
                if item_pose is not None:
                    # Transform pose from camera to workspace frame, so we can
                    # set the orientation more easily
                    target_frame = "workspace_center"
                    source_frame = item_pose.header.frame_id
                    transform = tf_buffer.lookup_transform(
                        target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
                    pick_pose = tf2_geometry_msgs.do_transform_pose(
                        item_pose, transform)

                    downward_orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(0, pi / 2, 0))
                    pick_pose.pose.orientation = downward_orientation

                    rospy.logdebug("pick object")
                    self.skill_server.publish_marker(
                        pick_pose, "pick_pose")  # debug
                    self.pick(
                        "b_bot",
                        pick_pose,
                        grasp_height=0.1,
                        approach_height=0.1,
                        speed_fast=0.2,
                        speed_slow=0.02,
                        gripper_command="easy_pick_only_inner")

                    # place_pose = pick_pose
                    # rospy.logdebug("place object")
                    # self.place("b_bot", place_pose, place_height = 0.03, approach_height = 0.05,
                    #                         speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                    #                         lift_up_after_place = False)

            else:
                rospy.loginfo("No valid entry. Skipping.")

            # rospy.logdebug("Go to home position")
            # self.go_to_named_pose("home_c", "c_bot")
            # self.go_to_named_pose("home_b", "b_bot")
            # self.go_to_named_pose("home_a", "a_bot")
            rospy.loginfo("============ Done!")

        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    demo = VisionDemo()
    demo.run()
