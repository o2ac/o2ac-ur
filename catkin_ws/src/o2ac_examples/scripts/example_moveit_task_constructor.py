#!/usr/bin/env python
#
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

import sys

import rospy
import tf
import moveit_commander

from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init

import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

from math import pi

from o2ac_assembly_database.assembly_reader import AssemblyReader

if __name__ == '__main__':

    # Create a rosnode and connect to moveit and tf
    rospy.init_node('test')
    roscpp_init("load_co")
    planning_scene_interface = moveit_commander.PlanningSceneInterface()
    listener = tf.TransformListener()

    # Place collision object in the scene:
    assy_pose = geometry_msgs.msg.PoseStamped()
    assy_pose.header.frame_id = 'workspace_center'
    assy_pose.pose.position.x = -0.09
    assy_pose.pose.position.y = 0.11
    assy_pose.pose.position.z = 0.08
    quaternion = tf.transformations.quaternion_from_euler(pi / 2, 0, -pi / 2)
    assy_pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

    assy_name = 'assy_1'
    assembly_reader = AssemblyReader(assy_name)
    assembly_reader.publish_assembly_frames(assy_pose)

    pub = rospy.Publisher(
        '/collision_object',
        moveit_msgs.msg.CollisionObject,
        queue_size=100)

    collision_object = assembly_reader.collision_objects[0]
    collision_object.header.frame_id = 'workspace_center'
    # collision_object.mesh_poses[0].position = geometry_msgs.msg.Point(0.5,0,0.2)
    # quaternion = tf.transformations.quaternion_from_euler(0,pi/2,0)
    collision_object.mesh_poses[0].position = geometry_msgs.msg.Point(
        0.2, 0, 0)
    quaternion = tf.transformations.quaternion_from_euler(pi / 2, 0, -pi / 2)
    collision_object.mesh_poses[0].orientation = geometry_msgs.msg.Quaternion(
        *quaternion)

    transformer = tf.Transformer(True, rospy.Duration(10.0))
    collision_object_transform = geometry_msgs.msg.TransformStamped()
    collision_object_transform.header.frame_id = 'WORLD'
    collision_object_transform.child_frame_id = 'OBJECT'
    collision_object_transform.transform.translation = geometry_msgs.msg.Vector3(
        collision_object.mesh_poses[0].position.x,
        collision_object.mesh_poses[0].position.y,
        collision_object.mesh_poses[0].position.z)
    collision_object_transform.transform.rotation = geometry_msgs.msg.Quaternion(
        collision_object.mesh_poses[0].orientation.x,
        collision_object.mesh_poses[0].orientation.y,
        collision_object.mesh_poses[0].orientation.z,
        collision_object.mesh_poses[0].orientation.w)
    transformer.setTransform(collision_object_transform)

    for i in range(len(collision_object.subframe_names)):
        subframe_transform = geometry_msgs.msg.TransformStamped()
        subframe_transform.header.frame_id = 'OBJECT'
        subframe_transform.child_frame_id = collision_object.subframe_names[i]
        subframe_transform.transform.translation = geometry_msgs.msg.Vector3(
            collision_object.subframe_poses[i].position.x,
            collision_object.subframe_poses[i].position.y,
            collision_object.subframe_poses[i].position.z)
        subframe_transform.transform.rotation = geometry_msgs.msg.Quaternion(
            collision_object.subframe_poses[i].orientation.x,
            collision_object.subframe_poses[i].orientation.y,
            collision_object.subframe_poses[i].orientation.z,
            collision_object.subframe_poses[i].orientation.w)
        transformer.setTransform(subframe_transform)

    for i in range(len(collision_object.subframe_names)):
        (trans, rot) = transformer.lookupTransform(
            'WORLD', collision_object.subframe_names[i], rospy.Time(0))
        collision_object.subframe_poses[i].position = geometry_msgs.msg.Point(
            *trans)
        collision_object.subframe_poses[i].orientation = geometry_msgs.msg.Quaternion(
            *rot)

    print(collision_object.subframe_poses[0])

    rospy.sleep(0.2)
    pub.publish(collision_object)
    rospy.sleep(0.2)

    # # Grasping:
    # robot_name = "a_bot"
    # hand_name = 'panda_hand'
    # a_bot = moveit_commander.MoveGroupCommander(robot_name)

    # #Planners:
    # cartesian_planner = core.CartesianPath()
    # cartesian_planner.max_velocity_scaling_factor = 1
    # cartesian_planner.max_acceleration_scaling_factor = 1
    # cartesian_planner.step_size = 0.1

    # jointspace_planner = core.JointInterpolationPlanner()

    # sampling_planner = core.PipelinePlanner()
    # sampling_planner.goal_joint_tolerance = 0.00001

    # print('core functions, properties: ', dir(core))

    # # Task:
    # task = core.Task()

    # task.properties.update({'group': robot_name, 'eef': a_bot.get_end_effector_link(), 'hand': hand_name, 'hand_grasping_frame': hand_name, 'ik_frame': hand_name})

    # currstate = stages.CurrentState('current_state')
    # task.add(currstate)

    # predicate_filter = stages.PredicateFilter('predicate filter')
    # a = predicate_filter.setPredicate(predicate)
    # print('THE VALUE OF a IS: ' + str(a) + ' !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    # task.add(predicate_filter)

    # # CREATE WRAPPER FOR FILTER STAGE!!!!!!!!!!!!!!!

    # open_hand = stages.MoveTo("open hand", sampling_planner)
    # open_hand.group = hand_name
    # open_hand.setGoal('open')
    # task.add(open_hand)

    # connect = stages.Connect('move to pick', [(robot_name, sampling_planner)])
    # connect.timeout = 5
    # connect.properties.configureInitFrom(core.PARENT)
    # task.add(connect)

    # grasp = core.SerialContainer('pick object')
    # task.properties.exposeTo(grasp.properties, ['eef', 'hand', 'group', 'ik_frame'])
    # grasp.properties.configureInitFrom(core.PARENT, ['eef', 'hand', 'group', 'ik_frame'])

    # approach_object = stages.MoveRelative("approach_object", cartesian_planner)
    # approach_object.properties.update({'marker_ns': 'approach_object', 'link': hand_name})
    # approach_object.properties.configureInitFrom(core.PARENT, ['group'])
    # approach_object.min_distance = 0.01
    # approach_object.max_distance = 0.1
    # print(approach_object.properties.__getitem__('group')) #!!!!!!!!!!!!!!!!
    # approach_object.setDirection(geometry_msgs.msg.TwistStamped(header=std_msgs.msg.Header(frame_id = hand_name), twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(0,0,0.1))))
    # grasp.insert(approach_object)

    # generatepose = stages.GenerateGraspPose('generate grasp pose')
    # generatepose.properties.configureInitFrom(core.PARENT)
    # generatepose.properties.update({'marker_ns': 'grasp_pose'})
    # generatepose.pregrasp = 'open'
    # generatepose.object = 'base'
    # generatepose.angle_delta = pi/12
    # generatepose.setMonitoredStage(currstate)

    # #compute_ik = stages.ComputeIK('grasp pose IK', generatepose)

    # task.add(grasp)

    # if task.plan():
    #     task.publish(task.solutions[0])

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
