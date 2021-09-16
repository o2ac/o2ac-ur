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
import rospy

import o2ac_msgs.msg
from o2ac_routines.helpers import check_for_real_robot, wait_for_UR_program

import std_srvs.srv  # toggleCollisions_client
import geometry_msgs.msg  # urscript_client


class SkillServerClient():
    def __init__(self):
        self.pick_screw_from_feeder_client = actionlib.SimpleActionClient('/o2ac_skills/pick_screw_from_feeder', o2ac_msgs.msg.pickScrewFromFeederAction)
        self.place_client = actionlib.SimpleActionClient('/o2ac_skills/place', o2ac_msgs.msg.placeAction)
        self.regrasp_client = actionlib.SimpleActionClient('/o2ac_skills/regrasp', o2ac_msgs.msg.regraspAction)
        self.screw_client = actionlib.SimpleActionClient('/o2ac_skills/screw', o2ac_msgs.msg.screwAction)
        self.change_tool_client = actionlib.SimpleActionClient('/o2ac_skills/change_tool', o2ac_msgs.msg.changeToolAction)

        self.publishMarker_client = rospy.ServiceProxy('/o2ac_skills/publishMarker', o2ac_msgs.srv.publishMarker)
        self.disable_markers = True

        self.toggleCollisions_client = rospy.ServiceProxy('/o2ac_skills/toggleCollisions', std_srvs.srv.SetBool)

        self.urscript_client = rospy.ServiceProxy('/o2ac_skills/sendScriptToUR', o2ac_msgs.srv.sendScriptToUR)
        self.use_real_robot = rospy.get_param("use_real_robot", False)

    @check_for_real_robot
    def pick_screw_from_feeder(self, robot_name, screw_size, realign_tool_upon_failure=True):
        """
        Picks a screw from one of the feeders. The screw tool already has to be equipped!
        Use this command to equip the screw tool: do_change_tool_action(self, "b_bot", equip=True, screw_size = 4)
        """
        goal = o2ac_msgs.msg.pickScrewFromFeederGoal()
        goal.robot_name = robot_name
        goal.screw_size = screw_size
        rospy.loginfo("Sending pickScrewFromFeeder action goal")
        rospy.logdebug(goal)

        self.pick_screw_from_feeder_client.send_goal(goal)
        rospy.logdebug("Waiting for result")
        self.pick_screw_from_feeder_client.wait_for_result(rospy.Duration(60.0))
        rospy.logdebug("Getting result")
        return self.pick_screw_from_feeder_client.get_result()

    def do_place_action(self, robot_name, pose_stamped, tool_name="", screw_size=0):
        # Call the place action
        goal = o2ac_msgs.msg.placeGoal()
        goal.robot_name = robot_name
        goal.item_pose = pose_stamped
        goal.tool_name = tool_name
        goal.screw_size = screw_size
        rospy.loginfo("Sending place action goal")
        rospy.logdebug(goal)

        self.place_client.send_goal(goal)
        rospy.logdebug("Waiting for result")
        self.place_client.wait_for_result(rospy.Duration(90.0))
        rospy.logdebug("Getting result")
        return self.place_client.get_result()

    def do_regrasp(self, giver_robot_name, receiver_robot_name, grasp_distance=.02):
        """The item goes from giver to receiver."""
        goal = o2ac_msgs.msg.regraspGoal()
        goal.giver_robot_name = giver_robot_name
        goal.receiver_robot_name = receiver_robot_name
        goal.grasp_distance = grasp_distance

        self.regrasp_client.send_goal(goal)
        rospy.loginfo("Performing regrasp with grippers " + giver_robot_name + " and " + receiver_robot_name)
        self.regrasp_client.wait_for_result(rospy.Duration(90.0))
        return self.regrasp_client.get_result()

    @check_for_real_robot
    def do_screw_action(self, robot_name, target_hole, screw_height=0.02,
                        screw_size=4, stay_put_after_screwing=False, loosen_and_retighten_when_done=True):
        goal = o2ac_msgs.msg.screwGoal()
        goal.target_hole = target_hole
        goal.screw_height = screw_height
        goal.screw_size = screw_size
        goal.robot_name = robot_name
        goal.stay_put_after_screwing = stay_put_after_screwing
        goal.loosen_and_retighten_when_done = loosen_and_retighten_when_done
        rospy.loginfo("Sending screw action goal.")
        self.screw_client.send_goal(goal)
        self.screw_client.wait_for_result()
        res = self.screw_client.get_result()
        try:
            return res.success
        except:
            print("failed to return screw result")
            print(res)
            return False

    def do_change_tool_action(self, robot_name, equip=True, screw_size=4):
        # DEPRECATED
        goal = o2ac_msgs.msg.changeToolGoal()
        goal.robot_name = robot_name
        goal.equip_the_tool = equip
        goal.screw_size = screw_size
        rospy.loginfo("Sending changeTool action goal.")
        self.change_tool_client.send_goal(goal)
        self.change_tool_client.wait_for_result()
        return self.change_tool_client.get_result()

    def publish_marker(self, pose_stamped, marker_type):
        # Publishes a marker to Rviz for visualization
        if self.disable_markers:
            return True
        req = o2ac_msgs.srv.publishMarkerRequest()
        req.marker_pose = pose_stamped
        req.marker_type = marker_type
        self.publishMarker_client.call(req)
        return True

    def toggle_collisions(self, collisions_on):
        """Turns collisions in MoveIt on and off. Use with caution!"""
        req = std_srvs.srv.SetBoolRequest()
        req.data = collisions_on
        res = self.toggleCollisions_client.call(req)
        return res.success

##### URScript with skill server? #####

    def move_lin_rel(self, robot_name, relative_translation=[0, 0, 0], relative_rotation=[0, 0, 0], acceleration=0.5, velocity=.03, relative_to_robot_base=False, wait=True, max_wait=30.0):
        '''
        Does a lin_move relative to the current position of the robot. Uses the robot's TCP.

        robot_name = "b_bot" for example
        relative_translation: translatory movement relative to current tcp position, expressed in robot's own base frame
        relative_rotation: rotatory movement relative to current tcp position, expressed in robot's own base frame
        relative_to_robot_base: If true, uses the robot_base coordinates for the relative motion (not workspace_center!)
        '''
        if rospy.is_shutdown():
            return False
        # Uses UR coordinates
        if not self.use_real_robot:
            return True
        # Directly calls the UR service
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.relative_translation.x = relative_translation[0]
        req.relative_translation.y = relative_translation[1]
        req.relative_translation.z = relative_translation[2]
        req.relative_rotation.x = relative_rotation[0]
        req.relative_rotation.y = relative_rotation[1]
        req.relative_rotation.z = relative_rotation[2]
        req.acceleration = acceleration
        req.velocity = velocity
        req.lin_move_rel_in_base_csys = relative_to_robot_base
        req.program_id = "lin_move_rel"
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(1.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(max_wait))
        return res.success

    def move_joints(self, group_name, joint_pose_goal, speed, acceleration):
        rospy.logdebug("Real robot is being used. Send joint command to robot controller directly via URScript.")
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "move_j"
        req.robot_name = group_name
        req.joint_positions = joint_pose_goal
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name, rospy.Duration.from_sec(20.0))
        return res.success

    def move_lin(self, group_name, pose_goal_stamped, end_effector_link, speed, acceleration, listener):
        rospy.logdebug("Real robot is being used. Send linear motion to robot controller directly via URScript.")
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move"
        req.robot_name = group_name
        req.target_pose = self.transformTargetPoseFromTipLinkToURTCP(pose_goal_stamped, group_name, end_effector_link, listener)
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name, rospy.Duration.from_sec(30.0))
        return res.success

    def transformTargetPoseFromTipLinkToURTCP(self, ps, robot_name, end_effector_link, listener):
        # This transforms a pose from the end_effector_link set in MoveIt to the TCP used in the UR controller.
        # It is used when sending commands to the UR controller directly, without MoveIt/ROS controllers.
        rospy.logdebug("Received pose to transform to TCP link:")
        rospy.logdebug(str(ps.pose.position.x) + ", " + str(ps.pose.position.y) + ", " + str(ps.pose.position.z))
        rospy.logdebug(str(ps.pose.orientation.x) + ", " + str(ps.pose.orientation.y) + ", " + str(ps.pose.orientation.z) + ", " + str(ps.pose.orientation.w))

        t = listener.lookupTransform(end_effector_link, robot_name + "_tool0", rospy.Time())

        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = ps.header.frame_id
        m.child_frame_id = "temp_goal_pose__"
        m.transform.translation.x = ps.pose.position.x
        m.transform.translation.y = ps.pose.position.y
        m.transform.translation.z = ps.pose.position.z
        m.transform.rotation.x = ps.pose.orientation.x
        m.transform.rotation.y = ps.pose.orientation.y
        m.transform.rotation.z = ps.pose.orientation.z
        m.transform.rotation.w = ps.pose.orientation.w
        listener.setTransform(m)

        m.header.frame_id = "temp_goal_pose__"
        m.child_frame_id = "temp_wrist_pose__"
        m.transform.translation.x = t[0][0]
        m.transform.translation.y = t[0][1]
        m.transform.translation.z = t[0][2]
        m.transform.rotation.x = t[1][0]
        m.transform.rotation.y = t[1][1]
        m.transform.rotation.z = t[1][2]
        m.transform.rotation.w = t[1][3]
        listener.setTransform(m)

        ps_wrist = geometry_msgs.msg.PoseStamped()
        ps_wrist.header.frame_id = "temp_wrist_pose__"
        ps_wrist.pose.orientation.w = 1.0

        ps_new = listener.transformPose(ps.header.frame_id, ps_wrist)

        rospy.logdebug("New pose:")
        rospy.logdebug(str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y) + ", " + str(ps_new.pose.position.z))
        rospy.logdebug(str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y) + ", " + str(ps_new.pose.orientation.z) + ", " + str(ps_new.pose.orientation.w))

        return ps_new

    def horizontal_spiral_motion(self, robot_name, max_radius=.01, radius_increment=.001, speed=0.02, spiral_axis="Z"):
        if rospy.is_shutdown():
            return False
        rospy.loginfo("Performing horizontal spiral motion at speed " + str(speed) + " and radius " + str(max_radius))
        if not self.use_real_robot:
            return True
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "spiral_motion"
        req.robot_name = robot_name
        req.max_radius = max_radius
        req.radius_increment = radius_increment
        req.velocity = speed
        req.spiral_axis = spiral_axis
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
        return res.success

    def do_insertion(self, robot_name, max_insertion_distance=0.0,
                     max_approach_distance=0.0, max_force=.0,
                     max_radius=0.0, radius_increment=.0,
                     peck_mode=False,
                     wait=True, horizontal=False):
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.program_id = "insert"
        if horizontal:
            req.program_id = "horizontal_insertion"

        #  Original defaults:
        # max_approach_distance = .1, max_force = 5,
        #                     max_radius = .001, radius_increment = .0001,
        req.max_insertion_distance = max_insertion_distance
        req.max_approach_distance = max_approach_distance
        req.max_force = max_force
        req.peck_mode = peck_mode
        req.max_radius = max_radius
        req.radius_increment = radius_increment
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(2.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
        return res.success

    def do_spiral_search(self, robot_name, max_insertion_distance=0.0,
                         max_approach_distance=0.0, max_force=.0,
                         max_radius=0.0, radius_increment=.0,
                         peck_mode=False, wait=True):
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.program_id = "spiral"

        #  Original defaults:
        # max_approach_distance = .1, max_force = 5,
        #                     max_radius = .001, radius_increment = .0001,
        req.max_insertion_distance = max_insertion_distance
        req.max_approach_distance = max_approach_distance
        req.max_force = max_force
        req.peck_mode = peck_mode
        req.max_radius = max_radius
        req.radius_increment = radius_increment
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(2.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
        return res.success

    def do_helix_motion(self, robot_name, max_force=50,
                        helix_forward_axis="Z+",
                        helix_forward_increment=0.01, helix_forward_limit=0.1,
                        max_radius=0.005, radius_increment=.005,
                        wait=True):
        if not self.use_real_robot:
            return True
        rospy.loginfo("Performing helix motion with radius " + str(max_radius) + " and forward limit " + str(helix_forward_limit))
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.program_id = "helix_motion"
        req.helix_forward_axis = helix_forward_axis
        req.helix_forward_increment = helix_forward_increment
        req.helix_forward_limit = helix_forward_limit
        req.max_force = max_force
        req.max_radius = max_radius
        req.radius_increment = radius_increment
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(2.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(60.0))
        return res.success

    def movelin_around_shifted_tcp(self, robot_name, wait=True, desired_twist=[0, 0, 0, 0, 0, 0], tcp_position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   velocity=0.1, acceleration=0.02):
        """
        Shifts the TCP to tcp_position (in the robot wrist frame) and moves by desired_twist.

        For the UR, this sets the TCP inside the UR controller and uses the movel command. The TCP is reset afterwards.

        The desired twist is the desired relative motion and should be in the coordinates of the shifted TCP. This method is used to 
        rotate around the tip of the workpiece during the alignment for adaptive insertion, when the robot touches the hole with the peg to find its position precisely.
        Using the position calculated from the robot link lengths and reported joint angles would introduce too much of an error.
        """
        if not robot_name == "b_bot":
            rospy.logerr("This is not yet implemented for non-UR robots!")
            return False
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.program_id = "tcp_movement"
        req.desired_twist = desired_twist
        req.tcp_pose = tcp_position
        req.velocity = velocity
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(2.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
        return res.success

    def set_tcp_in_ur(self, robot_name, tcp_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        """
        Change the TCP inside the UR controller (use with caution!)
        """
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.program_id = "set_tcp"
        req.tcp_pose = tcp_pose
        res = self.urscript_client.call(req)
        return res.success

    def do_linear_push(self, robot_name, force, wait=True, direction="Z+", max_approach_distance=0.1, forward_speed=0.0, acceleration=0.05, direction_vector=[0, 0, 0], use_base_coords=False):
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.max_force = force
        req.force_direction = direction
        req.direction_vector = direction_vector
        if sum(direction_vector) != 0:
            req.force_direction = "using_direction_vector"  # This overwrites the default argument
        req.max_approach_distance = max_approach_distance
        req.forward_speed = forward_speed
        req.use_base_coords = use_base_coords
        req.program_id = "linear_push"
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(1.0)
            wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
        return res.success
