#!/usr/bin/env python

# Copyright (c) 2020, OMRON SINIC X
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
# Author: Karoly Istvan Artur

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit_msgs.msg import Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import copy

import moveit_commander

import moveit_task_constructor_msgs.msg

hand_name = 'b_bot_robotiq_85'
hand_open_pose = 'open'
hand_close_pose = 'close'
hand = moveit_commander.MoveGroupCommander(hand_name)


def pick_place_client():
    client = actionlib.SimpleActionClient('plan_pick_place', moveit_task_constructor_msgs.msg.PlanPickPlaceAction)
    rospy.loginfo("wait for plan_pick_place server")
    client.wait_for_server()
    goal = moveit_task_constructor_msgs.msg.PlanPickPlaceGoal()
    goal.task_type = 0
    goal.arm_group_name = 'b_bot'
    goal.hand_group_name = 'b_bot_robotiq_85'
    goal.eef_name = 'b_bot_tip'
    goal.hand_frame = 'b_bot_gripper_tip_link'
    goal.object_id = 'Box_0'

    grasp = Grasp()

    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = 'Box_0'
    grasp_pose.pose.orientation.w = 1
    grasp_pose.pose.position.z = 0
    grasp.grasp_pose = grasp_pose

    approach_direction = Vector3Stamped()
    approach_direction.header.frame_id = 'Box_0'
    approach_direction.vector.x = 1
    grasp.pre_grasp_approach.direction = approach_direction
    grasp.pre_grasp_approach.min_distance = 0.0
    grasp.pre_grasp_approach.desired_distance = 0.01

    lift_direction = Vector3Stamped()
    lift_direction.header.frame_id = 'world'
    lift_direction.vector.z = 1
    grasp.post_grasp_retreat.direction = lift_direction
    grasp.post_grasp_retreat.min_distance = 0.0
    grasp.post_grasp_retreat.desired_distance = 0.01

    grasp_2 = copy.deepcopy(grasp)
    grasp_2.grasp_pose.pose.position.z = 0.1
    grasp_2.pre_grasp_approach.min_distance = 0.05
    grasp_2.pre_grasp_approach.desired_distance = 0.15

    hand_open = hand.get_named_target_values(hand_open_pose)
    hand_closed = hand.get_named_target_values(hand_close_pose)

    for (joint, value) in hand_open.items():
        joint_traj_point = JointTrajectoryPoint()
        joint_traj_point.positions.append(value)
        grasp.pre_grasp_posture.joint_names.append(joint)
        grasp.pre_grasp_posture.points.append(joint_traj_point)
        grasp_2.pre_grasp_posture.joint_names.append(joint)
        grasp_2.pre_grasp_posture.points.append(joint_traj_point)

    for (joint, value) in hand_closed.items():
        joint_traj_point = JointTrajectoryPoint()
        joint_traj_point.positions.append(value)
        grasp.grasp_posture.joint_names.append(joint)
        grasp.grasp_posture.points.append(joint_traj_point)
        grasp_2.grasp_posture.joint_names.append(joint)
        grasp_2.grasp_posture.points.append(joint_traj_point)

    goal.grasps.append(grasp)
    goal.grasps.append(grasp_2)

    place_pose = PoseStamped()
    place_pose.header.frame_id = 'tray_center'
    place_pose.pose.orientation.w = 1
    place_pose.pose.position.z = 0.3
    goal.place_locations = [PlaceLocation()]
    goal.place_locations[0].place_pose = place_pose

    place_direction = Vector3Stamped()
    place_direction.header.frame_id = 'world'
    place_direction.vector.z = -1
    goal.place_locations[0].pre_place_approach.direction = place_direction
    goal.place_locations[0].pre_place_approach.min_distance = 0.05
    goal.place_locations[0].pre_place_approach.desired_distance = 0.15

    retreat_direction = Vector3Stamped()
    retreat_direction.header.frame_id = 'Box_0'
    retreat_direction.vector.x = -1
    goal.place_locations[0].post_place_retreat.direction = retreat_direction
    goal.place_locations[0].post_place_retreat.min_distance = 0.05
    goal.place_locations[0].post_place_retreat.desired_distance = 0.15

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pick_place_client')
        result = pick_place_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
