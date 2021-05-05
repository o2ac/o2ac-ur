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
# Author: Karoly Istvan Artur, Felix von Drigalski

import os
import yaml

import rospy
import rospkg
import tf

import geometry_msgs.msg
import moveit_msgs.msg
import shape_msgs.msg

from ur_control import conversions

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    import pyassimp

class PartsReader(object):
    '''
    Loads collision objects with metadata from a database in the config folder, 
    for easy publishing to the MoveIt planning scene.
    The grasp data for each object can be published to the parameter server.
    '''

    def __init__(self, db_name=""):
        self._rospack = rospkg.RosPack()
        self.db_name = db_name
        if self.db_name:
            self.load_db(db_name)
        

    def load_db(self, db_name):
        '''
        Switch between assemblies
        '''
        rospy.loginfo("Loading new parts database: " + db_name)
        self.db_name = db_name
        self._directory = os.path.join(self._rospack.get_path('o2ac_assembly_database'), 'config', db_name)
        self._parts_list = self._read_parts_list()
        self._collision_objects, self._grasps = self.get_collision_objects_with_metadata()
        rospy.loginfo("Done loading.")
    
    def get_collision_object(self, object_name):
        '''
        This returns the collision object (including subframes) for an object_name.
        '''
        for c_obj in self._collision_objects:
            if c_obj.id == object_name:
                return c_obj
        rospy.logerr("Could not find collision object with id " + str(object_name))
        return None
    
    def get_grasp_pose(self, object_name, grasp_name):
        """
        Returns a geometry_msgs.msg.PoseStamped object in the frame of the object

        grasp_name is generally the format "grasp_0"
        """
        grasp_pose_stamped = geometry_msgs.msg.PoseStamped()
        grasp_pose_stamped.header.frame_id = object_name
        
        object_grasps = next((part for part in self._grasps if part["part_name"] == object_name), None)
        for (_grasp_name, grasp_pose) in zip(object_grasps['grasp_names'], object_grasps['grasp_poses']):
            if _grasp_name == grasp_name:
                grasp_pose_stamped.pose = grasp_pose
                return grasp_pose_stamped
        rospy.logerr("Did not find grasp_name " + grasp_name + " for object " + object_name)
        return None

    #### Converters 

    def id_to_name(self, id_num):
        """
        Returns the name of the object with the given id number.
        Returns an empty string on failure.
        """
        for obj in self._parts_list:
            if obj["id"] == id_num:
                return obj["name"]
        rospy.logerr("Could not find object with id " + str(id_num))
        return ""
    
    def name_to_id(self, name):
        """
        Returns the id of the object with the given name.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["name"] == name:
                return obj["id"]
        rospy.logerr("Could not find object with name " + str(name))
        return False
    
    def id_to_type(self, id_num):
        """
        Returns the type of the object with the given id number.
        Returns an empty string on failure.
        """
        for obj in self._parts_list:
            if obj["id"] == id_num:
                return obj["type"]
        rospy.logerr("Could not find object with id " + str(id_num))
        return ""
    
    def type_to_id(self, type):
        """
        Returns the id of the object with the given type.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["type"] == type:
                return obj["id"]
        rospy.logerr("Could not find object with type " + str(type))
        return False
    
    def name_to_type(self, name):
        """
        Returns the type of the object with the given name.
        Returns False on failure.
        """
        for obj in self._parts_list:
            if obj["name"] == name:
                return obj["type"]
        rospy.logerr("Could not find object with name " + str(name))
        return False

    #### 

    def _upload_grasps_to_param_server(self, namespace):
        '''Upload grasps to the ROS param server
        Hierarchical params on the param server can be stored as dictionaries
        All of these grasps can be retrieved by requesting the parent parameter from the rosparam server
        '''
        for part in self._grasps:
            for (grasp_name, grasp_pose) in zip(part['grasp_names'], part['grasp_poses']):
              d = {'position': conversions.from_point(grasp_pose.position).tolist(),
               'orientation': conversions.from_quaternion(grasp_pose.orientation).tolist()}
              param_name = '/'.join(['', namespace, part['part_name'], grasp_name])
              rospy.set_param(param_name, d)
              
    def _read_parts_list(self):
        path = os.path.join(self._directory, 'parts_list.yaml')
        with open(path, 'r') as file_open:
            parts_list = yaml.load(file_open)
        return parts_list['parts_list']

    def _read_object_metadata(self, object_name):
        '''Read and return the object metadata including the subframes and the grasp points
        of the object referred to by input 'object_name'
        '''
        metadata_directory = os.path.join(self._directory, 'object_metadata')
        objects_with_metadata = [f.split('.')[0] for f in os.listdir(metadata_directory) if os.path.isfile(os.path.join(metadata_directory, f))]
        subframe_names = []
        subframe_poses = []
        grasp_names = []
        grasp_poses = []
        if object_name in objects_with_metadata:
            with open(os.path.join(metadata_directory, object_name + '.yaml'), 'r') as f:
                data = yaml.load(f)
            subframes = data['subframes']
            grasps = data['grasp_points']

            # Create the transformation between the world and the collision object        
            transformer = tf.Transformer(True, rospy.Duration(10.0))
            collision_object_transform = geometry_msgs.msg.TransformStamped()
            collision_object_transform.header.frame_id = 'WORLD'
            collision_object_transform.child_frame_id = 'OBJECT'
            collision_object_transform.transform.rotation.w = 1
            transformer.setTransform(collision_object_transform)

            for subframe in subframes:
                subframe_names.append(subframe['name'])

                # Create the transformations between the collision object and the given subframe
                # Currently the subframes poses in the CollosionObject message are interpreted to be poses in the frame specified by the frame_id of the header of the CollosionObject
                # However, we want to define the subframe poses relative to the frame of the object, for ease of use
                # So we have to make this transformation to tell the pose of the subframes, that were defined in the object's frame, in the frame specified by the frame_if of the header of the CollosionObject
                subframe_transform = geometry_msgs.msg.TransformStamped()
                subframe_transform.header.frame_id = 'OBJECT'
                subframe_transform.child_frame_id = subframe['name']
                subframe_transform.transform.translation = conversions.to_vector3(conversions.to_float(subframe['pose_xyzrpy'][:3]) )
                quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(subframe['pose_xyzrpy'][3:]))
                subframe_transform.transform.rotation = conversions.to_quaternion(quaternion)
    
                transformer.setTransform(subframe_transform)

                # Get the pose of the subframe in the world frame and add the subframe pose to the subframe poses of the collision object
                (trans,rot) = transformer.lookupTransform('WORLD', subframe['name'], rospy.Time(0))

                subframe_pose = conversions.to_pose(trans+rot)

                subframe_poses.append(subframe_pose)

            for grasp in grasps:
                grasp_names.append(grasp['grasp_name'])

                # Create the pose message for the grasp
                grasp_pose = geometry_msgs.msg.Pose()
                grasp_pose.position = conversions.to_vector3(conversions.to_float(grasp['pose_xyzrpy'][:3]) )
                quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(grasp['pose_xyzrpy'][3:]))
                grasp_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion.tolist())

                grasp_poses.append(grasp_pose)

        else:
            rospy.logwarn('Object \'' + object_name + '\' has no metadata defined! \n \
                           Returning empty metadata information! \n')
        return (subframe_names, subframe_poses, grasp_names, grasp_poses)

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
            rospy.logerr('Could not load mesh file ' + os.path.basename(os.path.normpath(filename)) + ' for part ' + name)
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
        Return a list of collision objects of the parts listed in 'parts list' along with their metadata (subframes, grasps)

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

            subframe_names, subframe_poses, grasp_names, grasp_poses = self._read_object_metadata(part['name'])

            collision_object = self._make_collision_object_from_mesh(part['name'], posestamped, mesh_file, scale=(0.001, 0.001, 0.001))

            collision_object.subframe_names = subframe_names
            collision_object.subframe_poses = subframe_poses

            grasps.append({'part_name': part['name'], 'grasp_names': grasp_names, 'grasp_poses': grasp_poses})

            # Here do we need the 'type' field of collision object (can be filled in from parts_list)?

            collision_objects.append(collision_object)

        return (collision_objects, grasps)
