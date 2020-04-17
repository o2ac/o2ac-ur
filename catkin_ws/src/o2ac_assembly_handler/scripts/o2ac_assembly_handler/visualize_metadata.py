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

import moveit_commander
import tf
import tf2_ros

import visualization_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg

from o2ac_assembly_handler.assy import AssyHandler

class MetadataVisualizer():
    '''Class for object metadata visualization
    '''

    def __init__(self):

        # The links defined for the move group in the srdf
        self._links = ['robotiq_85_base_link',
        'robotiq_85_left_inner_knuckle_link',
        'robotiq_85_left_finger_tip_link',
        'robotiq_85_left_knuckle_link',
        'robotiq_85_left_finger_link',
        'robotiq_85_right_inner_knuckle_link',
        'robotiq_85_right_finger_tip_link',
        'robotiq_85_right_knuckle_link',
        'robotiq_85_right_finger_link']

        self._robot_commander = moveit_commander.RobotCommander()
        self._gripper_visual = self._robot_commander.get_robot_markers({},self._links)

        self._pub = rospy.Publisher('gripper_marker_array', visualization_msgs.msg.MarkerArray, queue_size=100)

    def _create_transform(self, parent_frame, child_frame, child_pose):
        '''Create a transformation of geometry_msgs.msg.TransformStamped() type
        given the name of the parent frame as input 'parent_frame',
        the name of the child frame as input 'child_frame and
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

    def publish_marker_array(self, frame = 'world'):
        '''Publish the MarkerArray object to visualize the gripper
        the pose of the gripper visual is set so, that the frame 'robotiq_85_tip_link'
        is identical to the frame given in input 'frame'
        '''
        transformer = tf.Transformer(True, rospy.Duration(10.0))
        tip_link = self._robot_commander.get_link('robotiq_85_tip_link')
        transformer.setTransform(self._create_transform(tip_link.pose().header.frame_id, 'robotiq_85_tip_link', tip_link.pose().pose))

        poses = []
        for link in self._links:
            robot_link = self._robot_commander.get_link(link)
            transformer.setTransform(self._create_transform(tip_link.pose().header.frame_id, link, robot_link.pose().pose))
            (trans,rot) = transformer.lookupTransform('robotiq_85_tip_link', link, rospy.Time(0))
            relative_pose = geometry_msgs.msg.Pose()
            relative_pose.position = geometry_msgs.msg.Point(*trans)
            relative_pose.orientation = geometry_msgs.msg.Quaternion(*rot)
            poses.append(relative_pose)
        
        i = 0
        for (pose, marker) in zip(poses, self._gripper_visual.markers):
            marker.header.frame_id = frame
            marker.pose = pose
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.id = i
            i += 1

        rospy.sleep(1)
        self._pub.publish(self._gripper_visual)
    
    def publish_arrows(self, grasps):
        '''Publish an array of arrow markers pointing to the tf frames of grasps given by the input 'grasps'
        '''
        marker_array = visualization_msgs.msg.MarkerArray()
        i = 0
        for grasp in grasps:
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = grasp
            marker.ns = 'arrows'
            marker.id = i
            marker.type = marker.ARROW
            marker.action = marker.ADD

            marker.pose.position.x = -0.15
            marker.pose.orientation.w = 1

            marker.scale.x = 0.1
            marker.scale.y = 0.005
            marker.scale.z = 0.005

            marker.color.a = 1.0
            marker.color.r = 1.0
            
            marker_array.markers.append(marker)

            i += 1
        rospy.sleep(1)
        self._pub.publish(marker_array)



if __name__ == '__main__':
    rospy.init_node('visualize_metadata', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    assy_name = 'wrs_assembly_1'
    assy_handler = AssyHandler(assy_name)
    co = assy_handler.collision_objects[1]

    pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
    rospy.sleep(1)
    pub.publish(co)
    rospy.sleep(1)

    grasps = next(element['grasps'] for element in assy_handler.grasps if element['part_name'] == co.id)
    for grasp in grasps:
        rospy.sleep(0.3)
        tf_broadcaster.sendTransform(grasp)

    viz = MetadataVisualizer()
    viz.publish_marker_array('grasp_9')
    viz.publish_arrows(['grasp_6', 'grasp_7', 'grasp_8', 'grasp_10', 'grasp_11', 'grasp_12'])
