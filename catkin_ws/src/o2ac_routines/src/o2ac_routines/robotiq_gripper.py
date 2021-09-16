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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski

import actionlib
from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_routines import helpers
import rospy
import robotiq_msgs.msg
import numpy as np
import visualization_msgs.msg
from ur_control.controllers import GripperController  # Simulation only


class RobotiqGripper():
    def __init__(self, namespace, gripper_group, markers_scene):
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.ns = namespace
        self.gripper_group = gripper_group

        self.markers_scene = markers_scene

        self.opening_width = 0.0

        self.last_attached_object = (None, False)  # object name (string), with_collisions (bool)

        # Gripper
        if self.use_real_robot:
            self.sub_gripper_status_ = rospy.Subscriber("/%s/gripper_status" % self.ns, robotiq_msgs.msg.CModelCommandFeedback, self._gripper_status_callback)
            self.gripper = actionlib.SimpleActionClient('/%s/gripper_action_controller' % self.ns, robotiq_msgs.msg.CModelCommandAction)
        else:
            try:
                self.gripper = GripperController(namespace=self.ns, prefix=self.ns + '_', timeout=2.0, attach_link='o2ac_bots::%s_wrist_3_link' % self.ns)
            except Exception as e:
                rospy.logwarn("Fail to instantiate GripperController for simulation: " + str(e))
                rospy.logwarn("Instantiating dummy gripper, hoping for moveit Fake controllers")

                gripper_type = str(rospy.get_param(self.ns + "/gripper_type"))

                class GripperDummy():
                    pass
                self.gripper = GripperDummy()

                def close(wait=True):
                    return command(0.0, wait=wait)

                def open(opening_width=1.0, wait=True):
                    opening_width = opening_width if opening_width else 1.0
                    return command(opening_width, wait=wait)

                def command(cmd, wait=True):
                    if gripper_type == "85":
                        max_gap = 0.085
                        max_angle = 0.8028
                    elif gripper_type == "140":
                        max_gap = 0.140
                        max_angle = 0.69
                    distance = np.clip(cmd, 0, max_gap)
                    cmd_joints = gripper_group.get_current_joint_values()
                    cmd_joints[0] = (max_gap - distance) * max_angle / max_gap
                    gripper_group.set_joint_value_target(cmd_joints)
                    success, plan, planning_time, error = gripper_group.plan()
                    if success:
                        gripper_group.execute(plan, wait=wait)
                        current_joints = gripper_group.get_current_joint_values()
                        goal_joints = helpers.get_trajectory_joint_goal(plan, gripper_group.get_active_joints())
                        success = helpers.all_close(goal_joints, current_joints, 0.01)
                        # if not success:
                        #     rospy.logerr("Rviz move_group gripper to execute")
                        return success
                    rospy.logerr("Rviz move_group gripper failed: %s " % error)
                    return False
                setattr(GripperDummy, "close", lambda self, wait: close(wait))
                setattr(GripperDummy, "open", lambda self, opening_width, wait: open(opening_width, wait))
                setattr(GripperDummy, "command", lambda self, cmd, wait: command(cmd, wait))

    def _gripper_status_callback(self, msg):
        self.opening_width = msg.position  # [m]

    def close(self, force=40.0, velocity=1.0, wait=True):
        res = False
        if self.use_real_robot:
            res = self.send_command("close", force=force, velocity=velocity, wait=wait)
        else:
            res = self.gripper.close(wait=wait)
        if self.last_attached_object[0]:
            self.attach_object(object_to_attach=self.last_attached_object[0], with_collisions=self.last_attached_object[1])
        return res

    def open(self, velocity=1.0, wait=True, opening_width=None):
        res = False
        if self.use_real_robot:
            command = opening_width if opening_width else "open"
            res = self.send_command(command, wait=wait, velocity=velocity)
        else:
            res = self.gripper.open(opening_width, wait=wait)

        if self.last_attached_object[0]:
            self.detach_object(object_to_detach=self.last_attached_object[0])
        return res

    def send_command(self, command, force=40.0, velocity=1.0, wait=True):
        """
        gripper: a_bot or b_bot
        command: "open", "close" or opening width
        force: Gripper force in N. From 40 to 100
        velocity: Gripper speed. From 0.013 to 0.1
        attached_last_object: bool, Attach/detach last attached object if set to True

        Use a slow closing speed when using a low gripper force, or the force might be unexpectedly high.
        """
        res = False
        if self.use_real_robot:
            goal = robotiq_msgs.msg.CModelCommandGoal()
            goal.velocity = velocity
            goal.force = force
            if command == "close":
                goal.position = 0.0
            elif command == "open":
                goal.position = 0.140
            else:
                goal.position = command     # This sets the opening width directly
                rospy.logdebug(command)

            self.gripper.send_goal(goal)
            rospy.logdebug("Sending command " + str(command) + " to gripper: " + self.ns)
            if wait:
                self.gripper.wait_for_result(rospy.Duration(6.0))  # Default wait time: 6 s
                result = self.gripper.get_result()
                if result:
                    res = True
                else:
                    res = False
            else:
                res = True
        else:
            if command == "close":
                res = self.gripper.close(wait=wait)
            elif command == "open":
                res = self.gripper.open(wait=wait)
            else:
                res = self.gripper.command(command, wait=wait)

        if self.last_attached_object[0]:
            if command == "close":
                self.attach_object(object_to_attach=self.last_attached_object[0], with_collisions=self.last_attached_object[1])
            elif command == "open":
                self.detach_object(object_to_detach=self.last_attached_object[0])
            else:
                self.detach_object(object_to_detach=self.last_attached_object[0])
        return res

    def attach_object(self, object_to_attach=None, attach_to_link=None, with_collisions=False):
        if not self.last_attached_object[0] and not object_to_attach:
            return
        try:
            to_link = self.ns + "_ee_link" if attach_to_link is None else attach_to_link
            if with_collisions:
                self.gripper_group.attach_object(object_to_attach, to_link,
                                                 touch_links=[self.ns + "_gripper_tip_link",
                                                              self.ns + "_left_inner_finger_pad",
                                                              self.ns + "_left_inner_finger",
                                                              self.ns + "_left_inner_knuckle",
                                                              self.ns + "_right_inner_finger_pad",
                                                              self.ns + "_right_inner_finger",
                                                              self.ns + "_right_inner_knuckle"])
            else:
                self.markers_scene.attach_item(object_to_attach, to_link)
            self.last_attached_object = (object_to_attach, with_collisions)
        except Exception as e:
            rospy.logerr(object_to_attach + " could not be attached! robot_name = " + self.ns)
            print(e)

    def detach_object(self, object_to_detach):
        try:
            if self.last_attached_object[1]:
                self.gripper_group.detach_object(object_to_detach)
            else:
                self.markers_scene.detach_item(object_to_detach)
        except Exception as e:
            rospy.logerr(object_to_detach + " could not be detached! robot_name = " + self.ns)
            print(e)

    def forget_attached_item(self):
        self.last_attached_object = (None, False)
