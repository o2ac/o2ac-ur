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

import os
import yaml
import csv

import rospy
import rospkg
import tf

import geometry_msgs.msg
import moveit_msgs.msg
import shape_msgs.msg

from math import pi, cos, sin

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    import pyassimp

class AssyReader():
    '''
    This class can be used for loading the parts of an assembly as 'CollisionObject' messages,
    to be published to the MoveIt planning scene, as well as creating a TF tree representation of the assembly.
    '''

    def __init__(self, assembly_name="assy_1"):
        self._rospack = rospkg.RosPack()
        self.assy_name = assembly_name
        self._directory = os.path.join(self._rospack.get_path('o2ac_assembly_handler'), 'config', assembly_name)
        self._parts_list = self._read_parts_list()
        self.collision_objects, self.grasps = self.get_collision_objects_with_metadata()

    def change_assembly(self, assembly_name):
        '''
        Switch between assemblies
        '''
        self.assy_name = assembly_name
        self._directory = os.path.join(self._rospack.get_path('o2ac_assembly_handler'), 'config', assembly_name)
        self._parts_list = self._read_parts_list()
    
    def get_collision_object(self, object_name):
        '''
        This returns the collision object (including subframes) for an object_name.
        '''
        collision_object = next((c_obj for c_obj in self.collision_objects if c_obj.id == object_name), None)
        return collision_object

    def _tofloat(self, val):
        if type(val)==float:
            return val
        elif type(val)==str:
            return(float(eval(val)))
        else:
            return(float(val))

    def _read_parts_list(self):
        path = os.path.join(self._directory, 'parts_list.yaml')
        with open(path, 'r') as file_open:
            parts_list = yaml.load(file_open)
        return parts_list['parts_list']

    def lookup_frame(self, collision_object_id):
        return next(('_'.join([self.assy_name, 'part', str(part['id']).zfill(2)]) for part in self._parts_list if part['name'] == collision_object_id))

    def _read_subframe_csv(self, part_id):
        '''
        Read the information of the file 'extra_frames.csv' in the object metadata directory,
        and return the subframe names and poses for the part referred to by input 'part_id'
        '''

        # Path to subframe definitions
        subframes_file = os.path.join(self._directory, 'object_metadata', 'extra_frames.csv')

        # Read the subframe definitions in the extra_frames.csv file
        subframe_definitions = []
        with open(subframes_file, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                for row in reader:
                    row_stripped = []
                    for el in row:
                        row_stripped.append(el.strip())
                    subframe_definitions.append(row_stripped)

        # Get the subframes that belong to the given part 
        relevant_subframes = [row for row in subframe_definitions if row[0] == str(part_id)]

        # Create subframe names and subframe poses
        subframe_names = [row[1] for row in relevant_subframes]
        subframe_poses = []

        # Create the transformation between the world and the collision object        
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        collision_object_transform = geometry_msgs.msg.TransformStamped()
        collision_object_transform.header.frame_id = 'WORLD'
        collision_object_transform.child_frame_id = 'OBJECT'
        collision_object_transform.transform.rotation.w = 1
        transformer.setTransform(collision_object_transform)

        for row in relevant_subframes:
            # Create the transformations between the collision object and the given subframe
            subframe_transform = geometry_msgs.msg.TransformStamped()
            subframe_transform.header.frame_id = 'OBJECT'
            subframe_transform.child_frame_id = row[1]
            subframe_transform.transform.translation.x = eval(row[5])
            subframe_transform.transform.translation.y = eval(row[6])
            subframe_transform.transform.translation.z = eval(row[7])
            quaternion = tf.transformations.quaternion_from_euler(eval(row[2]),eval(row[3]),eval(row[4]))
            subframe_transform.transform.rotation = geometry_msgs.msg.Quaternion(*quaternion)
 
            transformer.setTransform(subframe_transform)

            # Get the pose of the subframe in the world frame and add the subframe pose to the subframe poses of the collision object
            (trans,rot) = transformer.lookupTransform('WORLD', row[1], rospy.Time(0))

            subframe_pose = geometry_msgs.msg.Pose()
            subframe_pose.position = geometry_msgs.msg.Point(*trans)
            subframe_pose.orientation = geometry_msgs.msg.Quaternion(*rot)

            subframe_poses.append(subframe_pose)
        
        return (subframe_names, subframe_poses)

    def _read_object_metadata(self, object_name):
        '''Read and return the object metadata including the subframes and the grasp points
        of the object reffered to by input 'object_name'
        '''
        metadata_directory = os.path.join(self._directory, 'object_metadata')
        objects_with_metadta = [f.split('.')[0] for f in os.listdir(metadata_directory) if os.path.isfile(os.path.join(metadata_directory, f))]
        subframe_names = []
        subframe_poses = []
        grasp_names = []
        grasp_transforms = []
        if object_name in objects_with_metadta:
            with open(os.path.join(metadata_directory, object_name + '.yaml'), 'r') as f:
                data = yaml.load(f)
            subframes = data['extra_frames']
            grasps = data['grasp_points']

            # Create the transformation between the world and the collision object        
            transformer = tf.Transformer(True, rospy.Duration(10.0))
            collision_object_transform = geometry_msgs.msg.TransformStamped()
            collision_object_transform.header.frame_id = 'WORLD'
            collision_object_transform.child_frame_id = 'OBJECT'
            collision_object_transform.transform.rotation.w = 1
            transformer.setTransform(collision_object_transform)

            for subframe in subframes:
                subframe_names.append(subframe['subframe_name'])

                # Create the transformations between the collision object and the given subframe
                subframe_transform = geometry_msgs.msg.TransformStamped()
                subframe_transform.header.frame_id = 'OBJECT'
                subframe_transform.child_frame_id = subframe['subframe_name']
                subframe_transform.transform.translation.x = float(subframe['pose_xyzrpy'][0])
                subframe_transform.transform.translation.y = float(subframe['pose_xyzrpy'][1])
                subframe_transform.transform.translation.z = float(subframe['pose_xyzrpy'][2])
                quaternion = tf.transformations.quaternion_from_euler(self._tofloat(subframe['pose_xyzrpy'][3]),self._tofloat(subframe['pose_xyzrpy'][4]),self._tofloat(subframe['pose_xyzrpy'][5]))
                subframe_transform.transform.rotation = geometry_msgs.msg.Quaternion(*quaternion)
    
                transformer.setTransform(subframe_transform)

                # Get the pose of the subframe in the world frame and add the subframe pose to the subframe poses of the collision object
                (trans,rot) = transformer.lookupTransform('WORLD', subframe['subframe_name'], rospy.Time(0))

                subframe_pose = geometry_msgs.msg.Pose()
                subframe_pose.position = geometry_msgs.msg.Point(*trans)
                subframe_pose.orientation = geometry_msgs.msg.Quaternion(*rot)

                subframe_poses.append(subframe_pose)

            for grasp in grasps:
                grasp_names.append(grasp['grasp_name'])

                # Create the transformations between the collision object and the given grasp
                grasp_transform = geometry_msgs.msg.TransformStamped()
                grasp_transform.header.frame_id = 'OBJECT'
                grasp_transform.child_frame_id = grasp['grasp_name']
                grasp_transform.transform.translation.x = float(grasp['pose_xyzrpy'][0])
                grasp_transform.transform.translation.y = float(grasp['pose_xyzrpy'][1])
                grasp_transform.transform.translation.z = float(grasp['pose_xyzrpy'][2])
                quaternion = tf.transformations.quaternion_from_euler(self._tofloat(grasp['pose_xyzrpy'][3]),self._tofloat(grasp['pose_xyzrpy'][4]),self._tofloat(grasp['pose_xyzrpy'][5]))
                grasp_transform.transform.rotation = geometry_msgs.msg.Quaternion(*quaternion)
    
                transformer.setTransform(grasp_transform)

                # Get the pose of the grasp in the world frame and add the grasp pose to the grasp poses of the collision object
                (trans,rot) = transformer.lookupTransform('WORLD', grasp['grasp_name'], rospy.Time(0))

                grasp_trans = geometry_msgs.msg.Transform()
                grasp_trans.translation = geometry_msgs.msg.Vector3(*trans)
                grasp_trans.rotation = geometry_msgs.msg.Quaternion(*rot)

                grasp_transforms.append(grasp_trans)

        else:
            print('\nObject \'' + object_name + '\' has no metadata defined!\nReturning empty metadta information!\n')
        return (subframe_names, subframe_poses, grasp_names, grasp_transforms)


    def _make_collision_object_from_mesh(self, name, pose, filename, scale=(1, 1, 1)):
        '''
        Reimplementation of '__make_mesh()' function from 'moveit_commander/planning_scene_interface.py'

        This function does not need to create a 'PlanningSceneInterface' object in order to work
        '''
        # Set the properties apart from the mesh
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.mesh_poses = [pose.pose]

        # Try to load the mesh and set it for the collision object
        if pyassimp is False:
            raise Exception("Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
        try:
            scene = pyassimp.load(filename)
        except:
            print('Could not load mesh file ' + os.path.basename(os.path.normpath(filename)) + ' for part ' + name)
            return co
        if not scene.meshes or len(scene.meshes) == 0:
            raise Exception("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise Exception("There are no faces in the mesh")

        mesh = shape_msgs.msg.Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = shape_msgs.msg.MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = shape_msgs.msg.MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise Exception("Unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            point = geometry_msgs.msg.Point()
            point.x = vertex[0]*scale[0]
            point.y = vertex[1]*scale[1]
            point.z = vertex[2]*scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        pyassimp.release(scene)
        return co

    def get_collision_objects_with_metadata(self):
        '''
        Return a list of collision objects of the parts listed in 'parts list'

        This function loops through the loaded 'parts list' and for each part
        it searches for the 3D representation (mesh) described in the 'cad' field
        of the 'parts list file', creates a collision object from the mesh and
        returns a list of such collision objects

        The returned collision objects are placed at the origin of the frame 'world'
        Use the MOVE option of the collision object once it is in the planning scene
        to move it to a different pose
        '''

        # List of collision objects to return
        collision_objects = []
        grasps = []

        for part in self._parts_list:
            # Find mesh
            mesh_file = os.path.join(self._directory, 'meshes', part['cad'])

            # Set the pose of the collision object for the origin of the 'world' frame
            posestamped = geometry_msgs.msg.PoseStamped()
            posestamped.header.frame_id = 'world'
            collision_object_pose = geometry_msgs.msg.Pose()
            collision_object_pose.orientation.w = 1
            posestamped.pose = collision_object_pose

            # subframe_names, subframe_poses = self._read_subframe_csv(part['id'])
            subframe_names, subframe_poses, grasp_names, grasp_trans = self._read_object_metadata(part['name'])

            grasp_transforms = []
            for (name, transform) in zip(grasp_names, grasp_trans):
                grasp_transform = geometry_msgs.msg.TransformStamped()
                grasp_transform.header.frame_id = 'world'
                grasp_transform.child_frame_id = name
                grasp_transform.transform = transform
                grasp_transforms.append(grasp_transform)

            collision_object = self._make_collision_object_from_mesh(part['name'], posestamped, mesh_file, scale=(0.001, 0.001, 0.001))

            collision_object.subframe_names = subframe_names
            collision_object.subframe_poses = subframe_poses

            grasps.append({'part_name': part['name'], 'grasps': grasp_transforms})

            # Here do we need the 'type' field of collision object (can be filled in from parts_list)?

            collision_objects.append(collision_object)

        return (collision_objects, grasps)

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
            transform.transform.translation.x = eval(row[5])
            transform.transform.translation.x = eval(row[6])
            transform.transform.translation.x = eval(row[7])
            quaternion = tf.transformations.quaternion_from_euler(eval(row[2]), eval(row[3]), eval(row[4])).tolist()
            transform.transform.rotation = geometry_msgs.msg.Quaternion(*quaternion)
            transforms.append(transform)

        return transforms

    def _collision_object_to_tf_tree(self, collision_object):
        '''
        Create a tf tree of the object, based on the transformations between the object frame and its subframes
        '''
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        collision_object_tree = tf.Transformer(True, rospy.Duration(10.0))

        # Get the transformation between the collision object and the frame indicated in its header, from the pose of the mesh
        collision_object_transform = geometry_msgs.msg.TransformStamped()
        collision_object_transform.header.frame_id = collision_object.header.frame_id
        part_id = next((str(part['id']) for part in self._parts_list if collision_object.id == part['name']), '')
        collision_object_frame_id = '_'.join(['part', part_id.zfill(2)])
        collision_object_transform.child_frame_id = collision_object_frame_id
        collision_object_transform.transform.translation.x = collision_object.mesh_poses[0].position.x
        collision_object_transform.transform.translation.y = collision_object.mesh_poses[0].position.y
        collision_object_transform.transform.translation.z = collision_object.mesh_poses[0].position.z
        collision_object_transform.transform.rotation = collision_object.mesh_poses[0].orientation

        # The transform describing the collision object pose in its reference frame is the same in the relative and absolute description
        transformer.setTransform(collision_object_transform)
        collision_object_tree.setTransform(collision_object_transform)

        for (subframe_name, subframe_pose) in zip(collision_object.subframe_names, collision_object.subframe_poses):
            # Get the transformaion between the subframes and the frame indicated in the collision objet's header, from 'subframe poses'
            subframe_transform = geometry_msgs.msg.TransformStamped()
            subframe_transform.header.frame_id = collision_object.header.frame_id
            subframe_transform.child_frame_id = '_'.join([collision_object_frame_id, subframe_name])
            subframe_transform.transform.translation.x = subframe_pose.position.x
            subframe_transform.transform.translation.y = subframe_pose.position.y
            subframe_transform.transform.translation.z = subframe_pose.position.z
            subframe_transform.transform.rotation = subframe_pose.orientation
            transformer.setTransform(subframe_transform)

            # Calculate the transformation between the subframes and the collision object (relative, not in described in the same frame)
            (trans,rot) = transformer.lookupTransform(collision_object_transform.child_frame_id, subframe_transform.child_frame_id, rospy.Time(0))
            subframe_transform_relative = geometry_msgs.msg.TransformStamped()
            subframe_transform_relative.header.frame_id = collision_object_transform.child_frame_id
            subframe_transform_relative.child_frame_id = subframe_transform.child_frame_id
            subframe_transform_relative.transform.translation = geometry_msgs.msg.Vector3(*trans)
            subframe_transform_relative.transform.rotation = geometry_msgs.msg.Quaternion(*rot)

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
        (trans, rot) = tree_1.lookupTransform(parent_object_frame, tree_1_subframe_to_mate, rospy.Time(0))
        tree_1_subframe_transform = geometry_msgs.msg.TransformStamped()
        tree_1_subframe_transform.header.frame_id = parent_object_frame
        tree_1_subframe_transform.child_frame_id = tree_1_subframe_to_mate
        tree_1_subframe_transform.transform.translation = geometry_msgs.msg.Vector3(*trans)
        tree_1_subframe_transform.transform.rotation = geometry_msgs.msg.Quaternion(*rot)

        # Add the transform to transformer and add the mating transform as well
        transformer.setTransform(tree_1_subframe_transform)
        transformer.setTransform(mating_transform)

        # Get the transform describing the child object frame in the frame to mate of the child object and add it to transformer
        (trans,rot) = tree_2.lookupTransform(tree_2_subframe_to_mate, child_object_frame, rospy.Time(0))
        tree_2_root_relative_to_own_subframe = geometry_msgs.msg.TransformStamped()
        tree_2_root_relative_to_own_subframe.header.frame_id = tree_2_subframe_to_mate
        tree_2_root_relative_to_own_subframe.child_frame_id = child_object_frame
        tree_2_root_relative_to_own_subframe.transform.translation = geometry_msgs.msg.Vector3(*trans)
        tree_2_root_relative_to_own_subframe.transform.rotation = geometry_msgs.msg.Quaternion(*rot)

        transformer.setTransform(tree_2_root_relative_to_own_subframe)

        # Get the transform describing the child object frame in the parent object frame
        (trans,rot) = transformer.lookupTransform(parent_object_frame, child_object_frame, rospy.Time(0))
        tree_2_relative_to_tree_1 = geometry_msgs.msg.TransformStamped()
        tree_2_relative_to_tree_1.header.frame_id = parent_object_frame
        tree_2_relative_to_tree_1.child_frame_id = child_object_frame
        tree_2_relative_to_tree_1.transform.translation = geometry_msgs.msg.Vector3(*trans)
        tree_2_relative_to_tree_1.transform.rotation = geometry_msgs.msg.Quaternion(*rot)

        # Add the child object frame to tree_1 with the correct transform
        tree_1.setTransform(tree_2_relative_to_tree_1)

        # Add the subframes of the child object to tree_1
        tree_2_frames = [frame for frame in tree_2.getFrameStrings() if frame != child_object_frame and frame != 'world']
        for tree_2_frame in tree_2_frames:
            if tree_2_frame != child_object_frame:
                (trans,rot) = tree_2.lookupTransform(child_object_frame, tree_2_frame, rospy.Time(0))
                tree_2_frame_transform = geometry_msgs.msg.TransformStamped()
                tree_2_frame_transform.header.frame_id = child_object_frame
                tree_2_frame_transform.child_frame_id = tree_2_frame
                tree_2_frame_transform.transform.translation = geometry_msgs.msg.Vector3(*trans)
                tree_2_frame_transform.transform.rotation = geometry_msgs.msg.Quaternion(*rot)

                tree_1.setTransform(tree_2_frame_transform)

    def get_assembly_tree(self, collision_objects = []):
        '''
        Return the assembly target represented as a tree of tf transforms
        '''
        
        mating_transforms = self._read_frames_to_mate_csv()
        base_id = int(mating_transforms[0].header.frame_id.split('_')[2])
        base_name = next((part['name'] for part in self._parts_list if part['id'] == base_id), None)
        base_object = next((collision_object for collision_object in self.collision_objects if collision_object.id == base_name), None)
        assy_tree = self._collision_object_to_tf_tree(base_object)
        for mating_transform in mating_transforms:
            child_id = int(mating_transform.child_frame_id.split('_')[2])
            child_name = next((part['name'] for part in self._parts_list if part['id'] == child_id), None)
            child_object = next((collision_object for collision_object in self.collision_objects if collision_object.id == child_name), None)
            tree_2 = self._collision_object_to_tf_tree(child_object)
            mating_transform.header.frame_id = mating_transform.header.frame_id.replace('assy_', '')
            mating_transform.child_frame_id = mating_transform.child_frame_id.replace('assy_', '')
            self._add_part_to_assembly_tree(assy_tree, tree_2, mating_transform)
        
        return assy_tree
