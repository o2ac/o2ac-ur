#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X Corp.
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
#  * Neither the name of OMRON SINIC X Corp. nor the names of its
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
# Author: Felix von Drigalski

import rospy
import geometry_msgs.msg
import tf_conversions
import tf
import actionlib
from math import pi

import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv

import o2ac_routines.helpers

import o2ac_msgs.msg   # This is needed to advertise the actions
import sensor_msgs.msg # This is needed to receive images from the camera
import cv_bridge       # This offers conversion methods between OpenCV and ROS formats
                       # See here: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    
                       # Note that a similar package exists for PCL:
                       # http://wiki.ros.org/pcl_ros

import cv2
import sys
import rospkg
import o2ac_ssd
import o2ac_pose_estimation
from cv_bridge import CvBridge

ssd_detection = o2ac_ssd.ssd_detection()
pose_estimation = o2ac_pose_estimation.pose_estimation()

class O2ACVision(object):
    # This class advertises the vision actions that we will call during the tasks.

    def __init__(self):
    	rospy.init_node('o2ac_vision_', anonymous=False)
    	self.listener = tf.TransformListener()

    	# This creates the action server.nSee this tutorial, and also check out base.py: 
    	# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
        self.pose_estimation_test_server = actionlib.SimpleActionServer("poseEstimationTest", o2ac_msgs.msg.poseEstimationTestAction, 
            execute_cb = self.pose_estimation_test_callback, auto_start = True)
        
        rospy.loginfo("O2AC_vision has started up!")

    def pose_estimation_test_callback(self, goal):
        rospy.loginfo("Received a request to detect object named " + goal.object_id)
        action_result = o2ac_msgs.msg.poseEstimationTestResult()

        # First, obtain the image from the camera and convert it
        # image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
        #image_msg = rospy.wait_for_message("/camera/color/image_raw", sensor_msgs.msg.Image, 1.0)
        image_msg = rospy.wait_for_message("/b_bot_outside_camera_throttled/color/image_raw/compressed", sensor_msgs.msg.CompressedImage, 1.0)
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        cv2.imwrite("rgb.png", cv_image)

        # Detect the object
        ssd_results = self.detect_object_in_image(cv_image)

        # Estimate the pose
        pose_estimation_results = self.estimate_pose_in_image(cv_image, ssd_results)
        """
        for j in range(len(ssd_results)):
            SSDResult_msg = o2ac_msgs.msg.SSDResult()
            ssd_results[j]["bbox"][2] = ssd_results[j]["bbox"][0] + ssd_results[j]["bbox"][2]
            ssd_results[j]["bbox"][3] = ssd_results[j]["bbox"][1] + ssd_results[j]["bbox"][3]
            SSDResult_msg.bbox = ssd_results[j]["bbox"]
        """

        # Return
        #o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
        #self.ssd_test_server.set_succeeded(action_result)

### =======

    def detect_object_in_image(self, cv_image):
        ssd_results = ssd_detection.object_detection(cv_image)
        return ssd_results

    def estimate_pose_in_image(self, cv_image, ssd_results):
        pose_estimation_results = pose_estimation.pose_estimation(cv_image, ssd_results)
        return pose_estimation_results


if __name__ == '__main__':
    try:
        c = O2ACVision()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass