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

import moveit_msgs.msg
import geometry_msgs.msg

from math import pi

from o2ac_assembly_handler.assy import AssyHandler


if __name__ == '__main__':

    # Create a rosnode and connect to moveit and tf
    rospy.init_node('spawn_object')
    moveit_commander.roscpp_initialize(sys.argv)
    planning_scene_interface = moveit_commander.PlanningSceneInterface()
    listener = tf.TransformListener()

    # Get parameters:
    object_name = rospy.get_param('/o2ac_spawn_object/object_name')
    object_pose = rospy.get_param('/o2ac_spawn_object/object_pose')
    object_refrence_frame = rospy.get_param('/o2ac_spawn_object/object_reference_frame')
    assembly_name = rospy.get_param('/o2ac_spawn_object/assembly_name')

    # Create the assembly handler instance (loads and stores the assembly as a tf tree)
    assy_handler = AssyHandler(assembly_name)

    # Create a publisher for the '/collision_object' topic to spawn the object in the planning scene
    pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)

    # if the collision object is in the assembly, spawn it in the planning scene at the correct pose
    co_pose = geometry_msgs.msg.Pose()
    co_pose.position.x = object_pose[0]
    co_pose.position.y = object_pose[1]
    co_pose.position.z = object_pose[2]
    quaternion = tf.transformations.quaternion_from_euler(eval(str(object_pose[3])),eval(str(object_pose[4])),eval(str(object_pose[5])))
    co_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

    transformer = tf.Transformer(True, rospy.Duration(10.0))
    collision_object_transform = geometry_msgs.msg.TransformStamped()
    collision_object_transform.header.frame_id = 'WORLD'
    collision_object_transform.child_frame_id = object_name
    collision_object_transform.transform.translation.x = co_pose.position.x
    collision_object_transform.transform.translation.y = co_pose.position.y
    collision_object_transform.transform.translation.z = co_pose.position.z
    collision_object_transform.transform.rotation.x = co_pose.orientation.x
    collision_object_transform.transform.rotation.y = co_pose.orientation.y
    collision_object_transform.transform.rotation.z = co_pose.orientation.z
    collision_object_transform.transform.rotation.w = co_pose.orientation.w
    transformer.setTransform(collision_object_transform)

    collision_object = next(co for co in assy_handler.collision_objects if co.id == object_name)
    collision_object.header.frame_id = object_refrence_frame
    collision_object.mesh_poses[0] = co_pose

    subframe_poses = []

    for (subframe_name, subframe_pose) in zip(collision_object.subframe_names, collision_object.subframe_poses):
        subframe_transform = geometry_msgs.msg.TransformStamped()
        subframe_transform.header.frame_id = object_name
        subframe_transform.child_frame_id = subframe_name
        subframe_transform.transform.translation.x = subframe_pose.position.x
        subframe_transform.transform.translation.y = subframe_pose.position.y
        subframe_transform.transform.translation.z = subframe_pose.position.z
        subframe_transform.transform.rotation.x = subframe_pose.orientation.x
        subframe_transform.transform.rotation.y = subframe_pose.orientation.y
        subframe_transform.transform.rotation.z = subframe_pose.orientation.z
        subframe_transform.transform.rotation.w = subframe_pose.orientation.w

        transformer.setTransform(subframe_transform)

        (trans,rot) = transformer.lookupTransform('WORLD', subframe_name, rospy.Time(0))

        subframe_pose = geometry_msgs.msg.Pose()
        subframe_pose.position = geometry_msgs.msg.Point(*trans)
        subframe_pose.orientation = geometry_msgs.msg.Quaternion(*rot)

        subframe_poses.append(subframe_pose)

    collision_object.subframe_poses = subframe_poses

    rospy.sleep(0.2)
    pub.publish(collision_object)
    rospy.sleep(0.2)
