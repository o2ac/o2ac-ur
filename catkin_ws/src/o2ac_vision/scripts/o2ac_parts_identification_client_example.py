#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Science and 
#    Technology (AIST) nor the names of its contributors may be used 
#    to endorse or promote products derived from this software without 
#    specific prior written permission.
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
# Author: Toshio Ueshiba

# This script shows how to call the vision actions.

import rospy
import argparse
import actionlib
import o2ac_msgs.msg


def get_2d_poses_from_ssd_client():
    client = actionlib.SimpleActionClient(
        "get_2d_poses_from_ssd", o2ac_msgs.msg.get2DPosesFromSSDAction
    )
    client.wait_for_server()

    goal = o2ac_msgs.msg.get2DPosesFromSSDGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def belt_detection_client():
    client = actionlib.SimpleActionClient(
        "belt_detection", o2ac_msgs.msg.beltDetectionAction
    )
    client.wait_for_server()

    goal = o2ac_msgs.msg.get2DPosesFromSSDGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":

    try:
        rospy.init_node("test_client_py")

        results = get_2d_poses_from_ssd_client()
        print "pose estimation result"
        for j in range(len(results.get_2d_poses_from_ssd_result_list)):
            print "--------------------------------"
            print "confidence: ", results.get_2d_poses_from_ssd_result_list[
                j
            ].confidence
            print "rotation [deg(ccw)]: ", results.get_2d_poses_from_ssd_result_list[
                j
            ].rotation
            print "center [j,i]: ", results.get_2d_poses_from_ssd_result_list[j].center
            # print "3d pose: ",
            # results.get_2d_poses_from_ssd_result_list[j].pose
            print "class id: ", results.get_2d_poses_from_ssd_result_list[j].class_id
            # print "grasp points: ",
            # results.get_2d_poses_from_ssd_result_list[j].grasp_points

        results = belt_detection_client()
        print "\nbelt detection result"
        print "grasp points: ", results.grasp_points

    except rospy.ROSInterruptException:
        pass
