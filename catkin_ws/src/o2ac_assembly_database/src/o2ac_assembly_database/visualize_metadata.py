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
import os
import rospy

import moveit_commander
import tf
import tf2_ros
import numpy as np
import copy

import visualization_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg

from o2ac_assembly_database.assembly_reader import AssemblyReader
import o2ac_routines.helpers as helpers
from math import pi


class MetadataVisualizer():
    '''Class for object metadata visualization
    '''

    def __init__(self):

        # The links defined for the move group in the srdf
        # self._gripper_links = ['robotiq_85_base_link',
        # 'robotiq_85_left_inner_knuckle_link',
        # 'robotiq_85_left_finger_tip_link',
        # 'robotiq_85_left_knuckle_link',
        # 'robotiq_85_left_finger_link',
        # 'robotiq_85_right_inner_knuckle_link',
        # 'robotiq_85_right_finger_tip_link',
        # 'robotiq_85_right_knuckle_link',
        # 'robotiq_85_right_finger_link']

        self._gripper_links = ['b_bot_base',
                               'b_bot_left_inner_knuckle',
                               'b_bot_left_outer_knuckle',
                               'b_bot_left_inner_finger',
                               'b_bot_left_inner_finger_pad',
                               'b_bot_right_inner_knuckle',
                               'b_bot_right_outer_knuckle',
                               'b_bot_right_inner_finger',
                               'b_bot_right_inner_finger_pad']

        self._robot_commander = moveit_commander.RobotCommander()
        self._gripper_visual = self._robot_commander.get_robot_markers({}, self._gripper_links)

        self._pub = rospy.Publisher('o2ac_metadata_visualizer', visualization_msgs.msg.MarkerArray, queue_size=100)
        self._transformer = tf.Transformer(True, rospy.Duration(10.0))

    def _to_transform_stamped(self, parent_frame, child_frame, child_pose):
        '''Create a transform of geometry_msgs.msg.TransformStamped() type
        given the name of the parent frame as input 'parent_frame',
        the name of the child frame as input 'child_frame' and
        the pose of the child frame in the parent frame
        '''
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = child_pose.position.x
        transform.transform.translation.y = child_pose.position.y
        transform.transform.translation.z = child_pose.position.z
        transform.transform.rotation = child_pose.orientation
        return transform

    def add_object_marker(self, assembly_name, mesh_file_name, object_pose, mesh_pose, namespace='object', marker_array_to_append_to=""):
        '''Add a marker of an object from o2ac_assembly_database to a MarkerArray to be displayed in rviz
        The assembly name is the name of the assembly (folder) in which the mesh file of the object can be found
        The mesh_file_name should be the name of a mesh of one objects in the o2ac_assembly_database package
        The input 'object_pose' describes the object pose in the 'world' frame.
        'marker_pose' is the mesh marker's position in the object pose frame.
        Namespace is used for namespacing the marker if multiple objects are visualized. Ny default it is the name of the object,
        or simply "object" if there is only one object visualized
        If marker_array_to_append_to is not specified, a new array is returned.
        '''
        if not marker_array_to_append_to:
            marker_array_to_append_to = visualization_msgs.msg.MarkerArray()

        self._transformer.setTransform(self._to_transform_stamped('world', namespace, object_pose))

        mesh_path = os.path.join('o2ac_assembly_database', 'config', assembly_name, 'meshes', mesh_file_name)
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "world"
        marker.ns = namespace
        marker.id = 0
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.pose = mesh_pose
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.mesh_resource = 'package://' + mesh_path

        marker_array_to_append_to.markers.append(marker)

        return(marker_array_to_append_to)

    def add_frames(self, frame_names, poses, reference_frame='object', marker_array_to_append_to=""):
        '''Add markers visualizing coordinate frames to a MarkerArray
        The 'frame_names' are also displayed as text in the visualization
        The input 'poses' is a list of poses for the frames relative to the 'reference_frame'
        '''

        global_poses = []
        for (name, pose) in zip(frame_names, poses):
            self._transformer.setTransform(self._to_transform_stamped(reference_frame, name, pose))

        i = 0
        if not marker_array_to_append_to:
            marker_array_to_append_to = visualization_msgs.msg.MarkerArray()

        transformer = tf.Transformer(True, rospy.Duration(10.0))

        frame_names = []

        for frame in self._transformer.getFrameStrings():
            (trans, rot) = self._transformer.lookupTransform('world', frame, rospy.Time(0))
            frame_transform = geometry_msgs.msg.TransformStamped()
            frame_transform.header.frame_id = 'world'
            frame_transform.child_frame_id = frame
            frame_transform.transform.translation = geometry_msgs.msg.Vector3(*trans)
            frame_transform.transform.rotation = geometry_msgs.msg.Quaternion(*rot)
            transformer.setTransform(frame_transform)

            y_transform = geometry_msgs.msg.TransformStamped()
            y_transform.header.frame_id = frame
            y_transform.child_frame_id = frame + '_y'
            y_transform.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_about_axis(pi/2, (0, 0, 1)).tolist())
            transformer.setTransform(y_transform)

            z_transform = geometry_msgs.msg.TransformStamped()
            z_transform.header.frame_id = frame
            z_transform.child_frame_id = frame + '_z'
            z_transform.transform.rotation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_about_axis(-pi/2, (0, 1, 0)).tolist())
            transformer.setTransform(z_transform)

            global_pose = geometry_msgs.msg.Pose()
            global_pose.position = geometry_msgs.msg.Point(*trans)
            global_pose.orientation = geometry_msgs.msg.Quaternion(*rot)
            global_poses.append(global_pose)
            frame_names.append(frame)

            pose_marker = visualization_msgs.msg.Marker()
            pose_marker.header.frame_id = 'world'
            pose_marker.ns = reference_frame + '/' + 'frames'
            pose_marker.pose = global_pose
            pose_marker.id = 0
            pose_marker.type = pose_marker.ARROW
            pose_marker.action = pose_marker.ADD
            pose_marker.scale.x = .02
            pose_marker.scale.y = .002
            pose_marker.scale.z = .002
            pose_marker.color.a = .8

            arrow_x = visualization_msgs.msg.Marker()
            arrow_y = visualization_msgs.msg.Marker()
            arrow_z = visualization_msgs.msg.Marker()
            arrow_x = copy.deepcopy(pose_marker)
            arrow_y = copy.deepcopy(pose_marker)
            arrow_z = copy.deepcopy(pose_marker)

            arrow_x.id = i
            i += 1
            arrow_y.id = i
            i += 1
            arrow_z.id = i
            i += 1

            arrow_x.color.r = 1.0
            arrow_y.color.g = 1.0
            arrow_z.color.b = 1.0

            (trans, rot) = transformer.lookupTransform('world', frame + '_y', rospy.Time(0))
            arrow_y.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

            (trans, rot) = transformer.lookupTransform('world', frame + '_z', rospy.Time(0))
            arrow_z.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

            marker_array_to_append_to.markers.append(arrow_x)
            marker_array_to_append_to.markers.append(arrow_y)
            marker_array_to_append_to.markers.append(arrow_z)

        i = 0
        for (frame_name, global_pose) in zip(frame_names, global_poses):
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = 'world'
            marker.ns = reference_frame + '/' + 'frame_names'
            marker.id = i
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker.scale.z = 0.01
            marker.pose = global_pose
            marker.text = frame_name
            marker_array_to_append_to.markers.append(marker)
            i += 1

        return(marker_array_to_append_to)

    def publish_gripper_marker_array(self, frame='world'):
        '''Publish markers to visualize the gripper.
        The gripper is shown with 'gripper_tip_link' moved to 'frame'.
        '''

        g_transform = geometry_msgs.msg.TransformStamped()
        (g_trans, g_rot) = self._transformer.lookupTransform('world', frame, rospy.Time(0))
        g_transform.header.frame_id = 'world'
        g_transform.child_frame_id = frame
        g_transform.transform.translation = geometry_msgs.msg.Vector3(*g_trans)
        g_transform.transform.rotation = geometry_msgs.msg.Quaternion(*g_rot)

        transformer = tf.Transformer(True, rospy.Duration(10.0))

        transformer.setTransform(g_transform)

        tip_link = self._robot_commander.get_link('gripper_tip_link')
        tip_link_translation = tf.transformations.translation_matrix((tip_link.pose().pose.position.x, tip_link.pose().pose.position.y, tip_link.pose().pose.position.z))
        tip_link_rotation = tf.transformations.quaternion_matrix((tip_link.pose().pose.orientation.x, tip_link.pose().pose.orientation.y,
                                                                 tip_link.pose().pose.orientation.z, tip_link.pose().pose.orientation.w))
        gripper_base_in_tip_link = tf.transformations.inverse_matrix(np.matmul(tip_link_translation, tip_link_rotation))

        translation = tf.transformations.translation_from_matrix(gripper_base_in_tip_link).tolist()
        quaternion = tf.transformations.quaternion_from_matrix(gripper_base_in_tip_link).tolist()

        gripper_base_transform = geometry_msgs.msg.TransformStamped()
        gripper_base_transform.header.frame_id = frame
        gripper_base_transform.child_frame_id = 'gripper_base'
        gripper_base_transform.transform.translation = geometry_msgs.msg.Vector3(*translation)
        gripper_base_transform.transform.rotation = geometry_msgs.msg.Quaternion(*quaternion)

        transformer.setTransform(gripper_base_transform)

        poses = []
        for link in self._gripper_links:
            robot_link = self._robot_commander.get_link(link)
            transformer.setTransform(self._to_transform_stamped('gripper_base', link, robot_link.pose().pose))
            (trans, rot) = transformer.lookupTransform('world', link, rospy.Time(0))
            global_pose = geometry_msgs.msg.Pose()
            global_pose.position = geometry_msgs.msg.Point(*trans)
            global_pose.orientation = geometry_msgs.msg.Quaternion(*rot)
            poses.append(global_pose)

        i = 0
        for (pose, marker) in zip(poses, self._gripper_visual.markers):
            marker.header.frame_id = 'world'
            marker.ns = 'gripper'
            marker.pose = pose
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.id = i
            i += 1

        rospy.sleep(1)
        self._pub.publish(self._gripper_visual)

    def add_grasp_viz(self, grasps, namespace='', marker_array_to_append_to=""):
        '''Add to a MarkerArray markers visualizing the input 'grasps'.
        The arrows point along the 'x' axis of the grasp frames, with an offset of -0.15 from the origin along the 'x' axis.
        Two pad markers are spaced apart on the 'y' axis. Opening width is not considered.
        The input 'namespace' is used when grasps of multiple objects are visualized at once.
        '''
        if not marker_array_to_append_to:
            marker_array_to_append_to = visualization_msgs.msg.MarkerArray()
        i = 0
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        grasp_counter = 0
        for grasp in grasps:
            grasp_counter += 1
            g_transform = geometry_msgs.msg.TransformStamped()
            (g_trans, g_rot) = self._transformer.lookupTransform('world', grasp, rospy.Time(0))
            g_transform.header.frame_id = 'world'
            g_transform.child_frame_id = grasp
            g_transform.transform.translation = geometry_msgs.msg.Vector3(*g_trans)
            g_transform.transform.rotation = geometry_msgs.msg.Quaternion(*g_rot)
            transformer.setTransform(g_transform)

            arrow_transform = geometry_msgs.msg.TransformStamped()
            arrow_transform.header.frame_id = grasp
            arrow_transform.child_frame_id = 'arrow_' + str(i)
            arrow_transform.transform.translation.x = -0.15
            arrow_transform.transform.rotation.w = 1
            transformer.setTransform(arrow_transform)

            gripper_pad_transform_l = copy.deepcopy(arrow_transform)
            gripper_pad_transform_l.child_frame_id = 'pad_l_' + str(i)
            gripper_pad_transform_l.transform.translation.y -= 0.01
            transformer.setTransform(gripper_pad_transform_l)

            gripper_pad_transform_r = copy.deepcopy(arrow_transform)
            gripper_pad_transform_r.child_frame_id = 'pad_r_' + str(i)
            gripper_pad_transform_r.transform.translation.y += 0.01
            transformer.setTransform(gripper_pad_transform_r)

            (trans, rot) = transformer.lookupTransform('world', 'arrow_' + str(i), rospy.Time(0))
            (trans_pad_l, rot_pad_l) = transformer.lookupTransform('world', 'pad_l_' + str(i), rospy.Time(0))
            (trans_pad_r, rot_pad_r) = transformer.lookupTransform('world', 'pad_r_' + str(i), rospy.Time(0))

            arrow_marker = visualization_msgs.msg.Marker()
            arrow_marker.header.frame_id = 'world'
            arrow_marker.ns = namespace + '/arrows'
            arrow_marker.id = i
            arrow_marker.type = arrow_marker.ARROW
            arrow_marker.action = arrow_marker.ADD

            arrow_marker.pose.position = geometry_msgs.msg.Point(*trans)
            arrow_marker.pose.orientation = geometry_msgs.msg.Quaternion(*rot)

            arrow_marker.scale.x = 0.1
            arrow_marker.scale.y = 0.005
            arrow_marker.scale.z = 0.005

            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 1.0

            marker_array_to_append_to.markers.append(arrow_marker)

            gripper_pad_marker_l = visualization_msgs.msg.Marker()
            gripper_pad_marker_l.header.frame_id = 'world'
            gripper_pad_marker_l.ns = namespace + '/gripper_pads'
            gripper_pad_marker_l.id = i
            gripper_pad_marker_l.type = gripper_pad_marker_l.CUBE
            gripper_pad_marker_l.action = gripper_pad_marker_l.ADD

            gripper_pad_marker_l.pose.position = geometry_msgs.msg.Point(*trans_pad_l)
            gripper_pad_marker_l.pose.orientation = geometry_msgs.msg.Quaternion(*rot_pad_l)

            gripper_pad_marker_l.scale.x = 0.03
            gripper_pad_marker_l.scale.y = 0.002
            gripper_pad_marker_l.scale.z = 0.02

            gripper_pad_marker_l.color.a = 1.0
            gripper_pad_marker_l.color.r = 0.9
            gripper_pad_marker_l.color.g = 0.4
            gripper_pad_marker_l.color.b = 0.4

            marker_array_to_append_to.markers.append(gripper_pad_marker_l)

            i += 1

            gripper_pad_marker_r = copy.deepcopy(gripper_pad_marker_l)
            gripper_pad_marker_l.color.a = 1.0
            gripper_pad_marker_l.color.r = 0.4
            gripper_pad_marker_l.color.g = 0.4
            gripper_pad_marker_l.color.b = 0.9
            gripper_pad_marker_r.id = i
            gripper_pad_marker_l.pose.position = geometry_msgs.msg.Point(*trans_pad_r)
            gripper_pad_marker_l.pose.orientation = geometry_msgs.msg.Quaternion(*rot_pad_r)
            marker_array_to_append_to.markers.append(gripper_pad_marker_r)

            i += 1

            text_marker = visualization_msgs.msg.Marker()
            text_marker.header.frame_id = 'world'
            text_marker.ns = namespace + '/grasp_names'
            text_marker.id = i
            text_marker.type = text_marker.TEXT_VIEW_FACING
            text_marker.action = text_marker.ADD
            text_marker.pose = arrow_marker.pose
            # text_marker.pose = gripper_pad_marker_l.pose

            # text_marker.pose.position = geometry_msgs.msg.Point(0,0,-.1)
            # text_marker.pose.position = geometry_msgs.msg.Point(*trans)
            # text_marker.pose.orientation = geometry_msgs.msg.Quaternion(*rot)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            text_marker.scale.z = .01
            text_marker.text = grasp
            marker_array_to_append_to.markers.append(text_marker)
            i += 1
        return(marker_array_to_append_to)


if __name__ == '__main__':
    rospy.init_node('visualize_metadata', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    db_name = rospy.get_param('visualize_metadata/db_name')
    object_name = rospy.get_param('visualize_metadata/object_name')
    gripper_at_grasp = rospy.get_param('visualize_metadata/gripper_at_grasp')
    only_subframes = rospy.get_param('visualize_metadata/only_subframes')
    only_grasps = rospy.get_param('visualize_metadata/only_grasps')

    assembly_reader = AssemblyReader(db_name)

    marker_array = visualization_msgs.msg.MarkerArray()

    if object_name:
        mesh_name = next((part['cad'] for part in assembly_reader._parts_list if part['name'] == object_name))

        co = next(collision_object for collision_object in assembly_reader._collision_objects if collision_object.id == object_name)

        viz = MetadataVisualizer()

        grasp_names = next(element['grasp_names'] for element in assembly_reader._grasps if element['part_name'] == co.id)
        grasp_poses = next(element['grasp_poses'] for element in assembly_reader._grasps if element['part_name'] == co.id)

        frame_names = copy.copy(co.subframe_names)
        frame_poses = copy.copy(co.subframe_poses)

        frame_names.extend(grasp_names)
        frame_poses.extend(grasp_poses)

        object_pose = geometry_msgs.msg.Pose()
        object_pose.orientation.w = 1

        marker_array = viz.add_object_marker(db_name, mesh_name, object_pose, object_pose, marker_array_to_append_to=marker_array)

        if only_subframes:
            marker_array = viz.add_frames(co.subframe_names, co.subframe_poses, marker_array_to_append_to=marker_array)
        elif only_grasps:
            marker_array = viz.add_frames(grasp_names, grasp_poses, marker_array_to_append_to=marker_array)
            if gripper_at_grasp:
                viz.publish_gripper_marker_array(gripper_at_grasp)
        else:
            marker_array = viz.add_frames(frame_names, frame_poses, marker_array_to_append_to=marker_array)
            if gripper_at_grasp:
                viz.publish_gripper_marker_array(gripper_at_grasp)
            marker_array = viz.add_grasp_viz(grasp_names, marker_array_to_append_to=marker_array)

    else:  # Show all objects
        offset = 0.4
        row_counter = 0
        object_pose = geometry_msgs.msg.Pose()
        for co in assembly_reader._collision_objects:
            row_counter += 1
            mesh_name = next((part['cad'] for part in assembly_reader._parts_list if part['name'] == co.id))

            viz = MetadataVisualizer()

            grasp_names = next(element['grasp_names'] for element in assembly_reader._grasps if element['part_name'] == co.id)
            grasp_poses = next(element['grasp_poses'] for element in assembly_reader._grasps if element['part_name'] == co.id)

            frame_names = copy.copy(co.subframe_names)
            frame_poses = copy.copy(co.subframe_poses)

            object_pose.position.x += offset
            object_pose.orientation.w = 1
            if row_counter % 5 == 0:
                object_pose.position.x = 0
                object_pose.position.y += offset

            mesh_pose = copy.deepcopy(object_pose)  # Defined in world
            if not helpers.pose_msg_is_identity(co.mesh_poses[0]):
                mesh_pose.position.x += co.mesh_poses[0].position.x
                mesh_pose.position.y += co.mesh_poses[0].position.y
                mesh_pose.position.z += co.mesh_poses[0].position.z
                mesh_pose.orientation = helpers.multiply_quaternion_msgs(mesh_pose.orientation, co.mesh_poses[0].orientation)

            marker_array = viz.add_object_marker(db_name, mesh_name, object_pose, mesh_pose, co.id, marker_array)

            if only_subframes:
                names = []
                for name in frame_names:
                    names.append(co.id + '/' + name)
                marker_array = viz.add_frames(names, frame_poses, co.id, marker_array)
            elif only_grasps:
                frame_names = copy.copy(grasp_names)
                for name in frame_names:
                    name = co.id + '/' + name
                marker_array = viz.add_frames(frame_names, grasp_poses, co.id, marker_array)
            else:

                frame_names.extend(grasp_names)
                frame_poses.extend(grasp_poses)

                for name in frame_names:
                    name = co.id + '/' + name

                marker_array = viz.add_frames(frame_names, frame_poses, co.id, marker_array)

                frame_names = copy.copy(grasp_names)
                for name in frame_names:
                    name = co.id + '/' + name
                marker_array = viz.add_grasp_viz(frame_names, co.id, marker_array)

    rospy.sleep(1)
    rospy.loginfo("Publishing visualization markers.")
    while not rospy.is_shutdown():
        viz._pub.publish(marker_array)
        rospy.sleep(2)
    rospy.loginfo("Done.")
