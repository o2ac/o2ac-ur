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

import yaml

import tf2_ros
import rospy

import geometry_msgs.msg

from o2ac_assembly_handler.assy_reader import AssyReader

class AssyHandler():
    '''
    Load/store collision objects and assembly tree, publish assembly target frames to tf
    '''

    def __init__(self, assembly_name="wrs_assembly_1"):
        self._reader = AssyReader(assembly_name)
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.collision_objects = self._reader.get_collision_objects_with_subframes()
        self.assembly_tree = self._reader.get_assembly_tree(self.collision_objects)

    def change_assembly(self, assembly_name):
        self._reader.change_assembly(assembly_name)
        self.collision_objects = self._reader.get_collision_objects_with_subframes()
        self.assembly_tree = self._reader.get_assembly_tree(self.collision_objects)

    def publish_target_frames(self, assy_pose = None):
        '''
        Publish assembly target frames for tf

        This function constructs a list of tf transforms in the assembly tree and
        publishes the list with a static transform broadcaster

        The input 'assy_pose' is a geometry_msgs.msg.PoseStamped object that describes the
        desired pose of the assembly. If it is not set, the assembly is placed at the origin of the 'world' frame
        '''

        # Find the base of the assembly tree
        all_frames_dict = yaml.load(self.assembly_tree._buffer.all_frames_as_yaml())
        assembly_base_frame = next((properties['parent'] for (frame, properties) in all_frames_dict.items() if properties['parent'] not in self.assembly_tree.getFrameStrings()),None)

        base_transform = geometry_msgs.msg.TransformStamped()

        mating_transforms = []

        # Set the base transform for the assembly to place it at the pose given by input 'assy_pose'
        if assy_pose == None:
            base_transform.header.frame_id = 'world'
            base_transform.child_frame_id = '_'.join([self._reader.assy_name,assembly_base_frame])
            base_transform.transform.rotation.w = 1
        else:
            base_transform.header.frame_id = assy_pose.header.frame_id
            base_transform.child_frame_id = '_'.join([self._reader.assy_name,assembly_base_frame])
            base_transform.transform.translation.x = assy_pose.pose.position.x
            base_transform.transform.translation.y = assy_pose.pose.position.y
            base_transform.transform.translation.z = assy_pose.pose.position.z
            base_transform.transform.rotation = assy_pose.pose.orientation
        mating_transforms = [base_transform]

        # Get all transforms in the assembly tree as a list and start publishing it
        for (frame, properties) in all_frames_dict.items():
            (trans,rot) = self.assembly_tree.lookupTransform(properties['parent'],frame,rospy.Time(0))
            mating_transform = geometry_msgs.msg.TransformStamped()
            mating_transform.header.frame_id = '_'.join([self._reader.assy_name, properties['parent']])
            mating_transform.child_frame_id = '_'.join([self._reader.assy_name, frame])
            mating_transform.transform.translation = geometry_msgs.msg.Vector3(*trans)
            mating_transform.transform.rotation = geometry_msgs.msg.Quaternion(*rot)
            mating_transforms.append(mating_transform)
        self._broadcaster.sendTransform(mating_transforms)
        rospy.sleep(0.2)

    def lookup_frame(self, collision_object_id):
        return self._reader.lookup_frame(collision_object_id)

if __name__ == '__main__':
  print("Testing assembly handler.")
  try:
    c = AssyHandler()
    rospy.init_node('o2ac_assy', anonymous=True)
    while not rospy.is_shutdown():
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to do things with the Assembly Handler: ")
      rospy.loginfo("1: Publish TF tree")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.publish_target_frames()
      if r == 'x':
        break
  except Exception as e:
    print("Received error:" + type(e))