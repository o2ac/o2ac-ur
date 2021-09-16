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

from o2ac_assembly_database.assembly_reader import AssemblyReader


if __name__ == '__main__':

    # Create a rosnode and connect to moveit and tf
    rospy.init_node('test')
    moveit_commander.roscpp_initialize(sys.argv)
    planning_scene_interface = moveit_commander.PlanningSceneInterface()
    listener = tf.TransformListener()

    # Create the assembly pose to place the assembly at the desired pose in
    # the planning scene
    assy_pose = geometry_msgs.msg.PoseStamped()
    assy_pose.header.frame_id = 'workspace_center'
    quaternion = tf.transformations.quaternion_about_axis(pi / 2, (1, 0, 0))
    assy_pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

    # Create the assembly handler instance (loads and stores the assembly as a tf tree) and
    # publish the assembly frames to tf
    assy_name = 'wrs_assembly_2020'
    assembly_reader = AssemblyReader(assy_name)
    assembly_reader.publish_assembly_frames(assy_pose)

    # Create a publisher for the '/collision_object' topic to spawn objects in
    # the planning scene
    pub = rospy.Publisher(
        '/collision_object',
        moveit_msgs.msg.CollisionObject,
        queue_size=100)

    # Get the frame names of all frames in the assembly
    assy_frames = assembly_reader.assembly_tree.getFrameStrings()
    assy_frames = map(lambda x: '_'.join([assy_name, x]), assy_frames)

    # Loop through the loaded collision objects (stored in the assembly handler instance after constructing it) and
    # if the collision object is in the assembly, spawn it in the planning
    # scene at the correct pose
    for collision_object in assembly_reader.collision_objects:
        co_frame = assembly_reader.lookup_frame(collision_object.id)
        if co_frame in assy_frames:
            rospy.sleep(0.2)
            (trans, rot) = listener.lookupTransform(
                '/world', co_frame, rospy.Time(0))
            collision_object.mesh_poses[0].position = geometry_msgs.msg.Point(
                *trans)
            collision_object.mesh_poses[0].orientation = geometry_msgs.msg.Quaternion(
                *rot)
            rospy.sleep(0.2)
            pub.publish(collision_object)
            rospy.sleep(0.2)
