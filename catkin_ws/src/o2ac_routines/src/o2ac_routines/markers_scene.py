#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
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
# Author: Cristian C. Beltran-Hernandez

import copy
import rospy
import visualization_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from std_msgs.msg import ColorRGBA


class MarkersScene():
    def __init__(self, listener):
        self.listener = listener
        self.marker_publisher = rospy.Publisher("o2ac_assembly_markers", visualization_msgs.msg.Marker, queue_size=100)
        self.parts_database = PartsReader("wrs_assembly_2021", load_meshes=False, verbose=False)  # TODO this should be a param somewhere
        self.published_items = {}

    def spawn_item(self, item_name, pose_stamped, attach=False, color=None):
        color = color if color else ColorRGBA(0.2, 0.9, 0.2, 1.0)  # GREEN
        item_marker = self.parts_database.get_visualization_marker(item_name, pose_stamped.pose, pose_stamped.header.frame_id, color, frame_locked=attach)
        self.marker_publisher.publish(item_marker)
        self.published_items.update({item_name: copy.deepcopy(pose_stamped)})

    def despawn_item(self, item_name):
        marker = visualization_msgs.msg.Marker()
        marker.id = self.parts_database.name_to_id(item_name)
        marker.action = marker.DELETE
        try:
            self.marker_publisher.publish(marker)
            del self.published_items[item_name]
        except Exception as e:
            rospy.logerr("Try to delete item: %s but it does not exist" % item_name)

    def attach_item(self, item_name, to_link):
        current_pose = self.published_items[item_name]
        tries = 0
        current_pose.header.stamp = rospy.Time.now()
        while tries < 10:
            try:
                self.listener.waitForTransform(to_link, current_pose.header.frame_id, current_pose.header.stamp, rospy.Duration(1))
                new_pose = self.listener.transformPose(to_link, current_pose)
                break
            except:
                return False
        color = ColorRGBA(1., 0.0, 1., 1.0)  # Purple
        self.spawn_item(item_name, new_pose, attach=True, color=color)

    def detach_item(self, item_name):
        current_pose = self.published_items.get(item_name, None)
        if current_pose:
            tries = 0
            current_pose.header.stamp = rospy.Time.now()
            while tries < 10:
                try:
                    self.listener.waitForTransform("world", current_pose.header.frame_id, current_pose.header.stamp, rospy.Duration(1))
                    new_pose = self.listener.transformPose("world", current_pose)
                    break
                except:
                    return False
            self.spawn_item(item_name, new_pose, attach=False)
        else:
            rospy.logerr("Try to detach item: %s but it does not exist" % item_name)

    def delete_all(self):
        marker = visualization_msgs.msg.Marker()
        marker.action = marker.DELETEALL
        self.marker_publisher.publish(marker)
        self.published_items = {}
