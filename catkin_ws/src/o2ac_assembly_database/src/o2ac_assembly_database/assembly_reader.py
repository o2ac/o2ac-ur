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
import os
import csv

import tf2_ros
import tf.transformations
import tf
import rospy

from math import pi, cos, sin

from math import pi

import geometry_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_routines import conversions

class AssemblyReader(PartsReader):
    '''
    Load stored collision objects. This class also allows publishing a tree to TF,
    containing the frames of the completed assembly.
    This is used to parametrize the assembly procedure.
    '''

    def __init__(self, assembly_name=""):
        super(AssemblyReader, self).__init__(assembly_name)
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        if self.db_name: # = If a database is loaded
          self.assembly_tree = self.get_assembly_tree(self.collision_objects)
          self._upload_grasps_to_param_server(assembly_name)

    def change_assembly(self, assembly_name="wrs_assembly_1"):
        self.load_db(assembly_name)
        self.assembly_tree = self.get_assembly_tree(self.collision_objects)

    def get_frame_mating(self, base_object, child_object):
        '''
        This function returns the mating between two parts in the assembly as a geometry_msgs/Transform message

        'base_object' input is a string specifying the name of the object onto which the child object will be placed
        'child_object' input is a string specifying the name of the object to be placed onto the base object

        If no mating is defined between the two object, or the objects are not part of the loaded assembly the function returns None
        '''
        base_object_id = next((part['id'] for part in self._parts_list if part['name'] == base_object), None)
        child_object_id = next((part['id'] for part in self._parts_list if part['name'] == child_object), None)
        transform = next((mating_transform for mating_transform in self.mating_transforms if int(mating_transform.header.frame_id.split('_')[1]) == base_object_id and int(mating_transform.child_frame_id.split('_')[1]) == child_object_id), None)
        if transform is not None:
            frame_mating = get_transform(parent_frame=base_object + '/' + '_'.join(transform.header.frame_id.split('_')[2:]), 
                                         child_frame=child_object + '/' + '_'.join(transform.child_frame_id.split('_')[2:]),
                                         transform=transform.transform)
            return frame_mating
        else:
            return None

    def publish_assembly_frames(self, assembly_pose = None):
        '''
        Publish assembly target frames for tf

        This function constructs a list of tf transforms in the assembly tree and
        publishes the list with a static transform broadcaster.

        The input 'assembly_pose' is a geometry_msgs.msg.PoseStamped object that describes the
        desired pose of the assembly. If it is not set, the assembly is placed at the origin of the 'world' frame
        '''

        # Find the base of the assembly tree
        all_frames_dict = yaml.load(self.assembly_tree._buffer.all_frames_as_yaml())
        assembly_base_frame = next((properties['parent'] for (frame, properties) in all_frames_dict.items() if properties['parent'] not in self.assembly_tree.getFrameStrings()),None)

        base_transform = geometry_msgs.msg.TransformStamped()

        mating_transforms = []

        # Set the base transform for the assembly to place it at the pose given by input 'assembly_pose'
        if assembly_pose == None:
            base_transform.header.frame_id = 'world'
            base_transform.child_frame_id = '_'.join([self.db_name,assembly_base_frame])
            base_transform.transform.rotation.w = 1
        else:
            base_transform.header.frame_id = assembly_pose.header.frame_id
            base_transform.child_frame_id = '_'.join([self.db_name,assembly_base_frame])
            base_transform.transform.translation = conversions.to_vector3(conversions.from_point(assembly_pose.pose.position))
            base_transform.transform.rotation = assembly_pose.pose.orientation
        mating_transforms = [base_transform]

        # Get all transforms in the assembly tree as a list and start publishing it
        for (frame, properties) in all_frames_dict.items():
            (trans,rot) = self.assembly_tree.lookupTransform(properties['parent'],frame,rospy.Time(0))
            mating_transform = get_transform('_'.join([self.db_name, properties['parent']]), '_'.join([self.db_name, frame]), transform=conversions.to_transform(trans + rot))
            mating_transforms.append(mating_transform)
        self._broadcaster.sendTransform(mating_transforms)
        rospy.sleep(0.2)

    def lookup_frame(self, collision_object_id):
        return next(('_'.join([self.db_name, 'part', str(part['id']).zfill(2)]) for part in self._parts_list if part['name'] == collision_object_id))

    def get_assembly_tree(self, collision_objects = []):
        '''
        Return the assembly target represented as a tree of tf transforms
        '''
        
        self.mating_transforms = self._read_frames_to_mate_csv()
        if not self.mating_transforms:
            rospy.loginfo("No assembly tree defined")
            return False
        base_id = int(self.mating_transforms[0].header.frame_id.split('_')[2])
        base_name = next((part['name'] for part in self._parts_list if part['id'] == base_id), None)
        base_object = next((collision_object for collision_object in self.collision_objects if collision_object.id == base_name), None)
        assembly_tree = self._collision_object_to_tf_tree(base_object)
        for mating_transform in self.mating_transforms:
            child_id = int(mating_transform.child_frame_id.split('_')[2])
            child_name = next((part['name'] for part in self._parts_list if part['id'] == child_id), None)
            child_object = next((collision_object for collision_object in self.collision_objects if collision_object.id == child_name), None)
            tree_2 = self._collision_object_to_tf_tree(child_object)
            mating_transform.header.frame_id = mating_transform.header.frame_id.replace('assy_', '')
            mating_transform.child_frame_id = mating_transform.child_frame_id.replace('assy_', '')
            self._add_part_to_assembly_tree(assembly_tree, tree_2, mating_transform)
        
        return assembly_tree

    def _read_frames_to_mate_csv(self):

        # Load frames to mate from csv file
        mating_file = os.path.join(self._directory, 'frames_to_mate.csv')
        frame_matings = []
        with open(mating_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                if row:
                    if row[0][0] == '#':
                        continue
                    row_stripped = []
                    for el in row:
                        row_stripped.append(el.strip())
                    frame_matings.append(row_stripped)

        # Create transforms
        transforms = []
        for row in frame_matings:
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.frame_id = row[0]
            transform.child_frame_id = row[1]
            transform.transform.translation = conversions.to_vector3(conversions.to_float(row[5:8]))
            quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(row[2:5]))
            transform.transform.rotation = conversions.to_quaternion(quaternion)
            transforms.append(transform)

        return transforms

    def _collision_object_to_tf_tree(self, collision_object):
        '''
        Create a tf tree of the object, based on the transformations between the object frame and its subframes
        '''
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        collision_object_tree = tf.Transformer(True, rospy.Duration(10.0))

        # Get the transformation between the collision object and the frame indicated in its header, from the pose of the mesh
        part_id = next((str(part['id']) for part in self._parts_list if collision_object.id == part['name']), '')
        collision_object_frame_id = '_'.join(['part', part_id.zfill(2)])
        transform_pose = geometry_msgs.msg.Transform()
        transform_pose.translation = conversions.to_vector3(conversions.from_point(collision_object.mesh_poses[0].position))
        transform_pose.rotation = collision_object.mesh_poses[0].orientation
        collision_object_transform = get_transform(collision_object.header.frame_id, collision_object_frame_id, transform=transform_pose)

        # The transform describing the collision object pose in its reference frame is the same in the relative and absolute description
        transformer.setTransform(collision_object_transform)
        collision_object_tree.setTransform(collision_object_transform)

        for (subframe_name, subframe_pose) in zip(collision_object.subframe_names, collision_object.subframe_poses):
            # Get the transformaion between the subframes and the frame indicated in the collision object's header, from 'subframe poses'
            transform_pose = geometry_msgs.msg.Transform()
            transform_pose.translation = conversions.to_vector3(conversions.from_point(subframe_pose.position))
            transform_pose.rotation = subframe_pose.orientation
            subframe_transform = get_transform(collision_object.header.frame_id, '_'.join([collision_object_frame_id, subframe_name]), transform=transform_pose)

            transformer.setTransform(subframe_transform)

            # Calculate the transformation between the subframes and the collision object (relative, not in described in the same frame)
            subframe_transform_relative = get_transform(collision_object_transform.child_frame_id, subframe_transform.child_frame_id, transformer)

            # Add the relative transforms to the tf tree
            collision_object_tree.setTransform(subframe_transform_relative)

        return collision_object_tree

    def _add_part_to_assembly_tree(self, tree_1, tree_2, mating_transform):
        '''
        Add 'tree_2' to 'tree_1' using the mating_transform.

        The input 'tree_1' is the tf tree in which the assembly is being built and 'tree_2' is the tf tree of the object
        that we want to add to the assembly.

        The input 'mating_transform' gives the transformation between a subframe of an object in the assembly tree (parent object) and
        a subframe of the object to be added to the assembly (child object)
        '''

        # Create transformer, which is a separate helper tree, to compute the transform between the child and parent objects
        transformer = tf.Transformer(True, rospy.Duration(10.0))

        # Get the subframes to mate
        tree_1_subframe_to_mate = mating_transform.header.frame_id
        tree_2_subframe_to_mate = mating_transform.child_frame_id

        # Get the frames of the parent and the child object
        parent_object_frame = yaml.load(tree_1._buffer.all_frames_as_yaml())[mating_transform.header.frame_id]['parent']
        child_object_frame = yaml.load(tree_2._buffer.all_frames_as_yaml())[mating_transform.child_frame_id]['parent']

        # Get the transform describing the frame to mate of the parent object relative to the parent object frame
        tree_1_subframe_transform = get_transform(parent_object_frame, tree_1_subframe_to_mate, tree_1)

        # Add the transform to transformer and add the mating transform as well
        transformer.setTransform(tree_1_subframe_transform)
        transformer.setTransform(mating_transform)

        # Get the transform describing the child object frame in the frame to mate of the child object and add it to transformer
        tree_2_root_relative_to_own_subframe = get_transform(tree_2_subframe_to_mate, child_object_frame, tree_2)

        transformer.setTransform(tree_2_root_relative_to_own_subframe)

        # Get the transform describing the child object frame in the parent object frame
        tree_2_relative_to_tree_1 = get_transform(parent_object_frame, child_object_frame, transformer)

        # Add the child object frame to tree_1 with the correct transform
        tree_1.setTransform(tree_2_relative_to_tree_1)

        # Add the subframes of the child object to tree_1
        tree_2_frames = [frame for frame in tree_2.getFrameStrings() if frame != child_object_frame and frame != 'world']
        for tree_2_frame in tree_2_frames:
            if tree_2_frame != child_object_frame:
                tree_2_frame_transform = get_transform(child_object_frame, tree_2_frame, tree_2)

                tree_1.setTransform(tree_2_frame_transform)

def get_transform(parent_frame, child_frame, transformer=None, transform=None):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    
    if transform is None:
        assert transformer is not None, "Transformer needs to be defined!"
        (trans,rot) = transformer.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        t.transform = conversions.to_transform(trans + rot)
    else:
        t.transform = transform
    return t


if __name__ == '__main__':
  print("Testing assembly handler.")
  try:
    c = AssemblyReader()
    rospy.init_node('o2ac_assembly_database', anonymous=True)
    while not rospy.is_shutdown():
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to do things with the Assembly Database: ")
      rospy.loginfo("1: Publish TF tree")
      rospy.loginfo("2: Publish parts to the scene")
      r = raw_input()
      if r == '1':
        c.publish_assembly_frames()
      if r == '2':
        print("Not yet implemented")
      if r == 'x':
        break
  except Exception as e:
    print("Received error:" + type(e))
