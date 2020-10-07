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
import time
import rospkg
import actionlib
import numpy as np
from math import pi, radians
import cv2
import cv_bridge  # This offers conversion methods between OpenCV
                  # and ROS formats
                  # See here:
                  #   http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
                  # Note that a similar package exists for PCL:
                  #   http://wiki.ros.org/pcl_ros

from sensor_msgs   import msg as smsg
from geometry_msgs import msg as gmsg
from o2ac_msgs     import msg as omsg

import o2ac_routines.helpers
import o2ac_ssd
from pose_estimation_func import template_matching
from pose_estimation_func import FastGraspabilityEvaluation

ssd_detection = o2ac_ssd.ssd_detection()

class O2ACVision(object):
    # This class advertises the vision actions that we will call during the tasks.

    def __init__(self):
        # Setup subscriber for input RGB image
        self.image_sub = rospy.Subscriber('/image', smsg.Image,
                                          self.image_subscriber_callback)

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher('~result_image',
                                         smsg.Image, queue_size=1)

        # Determine whether the server works with continuous mode or not.
        self.cont = rospy.get_param('~cont', False)

        if self.cont:
            self.results_pub \
                = rospy.Publisher('~detection_results',
                                  omsg.EstimatedPosesArray, queue_size=1)
        else:
            self.axserver \
                = actionlib.SimpleActionServer("poseEstimation",
                                               omsg.poseEstimationAction,
                                               auto_start = False)
            self.axserver.register_goal_callback(self.goal_callback)
            self.axserver.register_preempt_callback(self.preempt_callback)
            self.axserver.start()

        rospy.loginfo("O2AC_vision has started up!")

    def goal_callback(self):
        self.axserver.accept_new_goal()
        rospy.loginfo("Received a request to detect object")

    def preempt_callback(self):
        self.axserver.set_preempted()
        rospy.loginfo("o2ac_msgs.msg.poseEstimationAction preempted")

    def image_subscriber_callback(self, image):
        # First, obtain the image from the camera and convert it
        bridge = cv_bridge.CvBridge()
        im_in  = bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        im_vis = im_in.copy()

        if self.cont:
            estimatedPoses_msg = omsg.EstimatedPosesArray()
            estimatedPoses_msg.header = image.header
            estimatedPoses_msg.results, im_vis \
                = self.get_estimation_results(im_in, im_vis)
            self.results_pub.publish(estimatedPoses_msg)

            # Publish images visualizing results
            self.image_pub.publish(bridge.cv2_to_imgmsg(im_vis,
                                                        encoding="rgb8"))

        elif self.axserver.is_active():
            action_result = omsg.poseEstimationResult()
            action_result.results, im_vis \
                = self.get_estimation_results(im_in, im_vis)
            self.axserver.set_succeeded(action_result)

            # Publish images visualizing results
            self.image_pub.publish(bridge.cv2_to_imgmsg(im_vis,
                                                        encoding="rgb8"))


### =======

    def get_estimation_results(self, im_in, im_vis):
        # Object detection
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        # Pose estimation
        estimatedPoses_msgs      = []
        apply_2d_pose_estimation = [8,9,10,14]
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]
        apply_grasp_detection    = [6]
        for ssd_result in ssd_results:
            target = ssd_result["class"]
            estimatedPoses_msg = omsg.EstimatedPoses()
            estimatedPoses_msg.confidence = ssd_result["confidence"]
            estimatedPoses_msg.class_id   = target
            estimatedPoses_msg.bbox       = ssd_result["bbox"]

            if target in apply_2d_pose_estimation:
                rospy.loginfo("Target id is %d. Apply the 2d pose estimation",
                              target)
                pose, im_vis = self.estimate_pose_in_image(im_in, im_vis,
                                                           ssd_result)
                estimatedPoses_msg.poses = [pose]

            elif target in apply_3d_pose_estimation:
                rospy.loginfo("Target id is %d. Apply the 3d pose estimation",
                              target)

            elif target in apply_grasp_detection:
                rospy.loginfo("Target id is %d. Apply grasp detection", target)
                estimatedPoses_msg.poses, im_vis \
                    = self.grasp_detection_in_image(im_in, im_vis, ssd_result)

            estimatedPoses_msgs.append(estimatedPoses_msg)

        return estimatedPoses_msgs, im_vis

    def detect_object_in_image(self, cv_image, im_vis):
        ssd_results, im_vis = ssd_detection.object_detection(cv_image, im_vis)
        return ssd_results, im_vis

    def estimate_pose_in_image(self, im_in, im_vis, ssd_result):

        start = time.time()

        rospack = rospkg.RosPack()
        temp_root = rospack.get_path("wrs_dataset") + "/data/templates"
        # Name of template infomation
        temp_info_name = "template_info.json"
        # downsampling rate
        ds_rate = 1.0/2.0

        # RGB -> Gray
        if im_in.shape[2] == 3:
            im_in = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)

        # template matching
        tm = template_matching(im_in, ds_rate, temp_root, temp_info_name)
        center, ori = tm.compute( ssd_result )

        elapsed_time = time.time() - start
        rospy.loginfo("Processing time[msec]: %d", 1000*elapsed_time)

        im_vis = tm.get_result_image(ssd_result, ori, center, im_vis)

        bbox = ssd_result["bbox"]

        return gmsg.Pose2D(x=center[1] - bbox[0],
                           y=center[0] - bbox[1],
                           theta=radians(ori)), \
               im_vis

    def grasp_detection_in_image(self, im_in, im_vis, ssd_result):

        start = time.time()

        # Generation of a hand template
        im_hand = np.zeros( (60,60), np.float )
        hand_width = 20 #in pixel
        im_hand[0:10,20:20+hand_width] = 1
        im_hand[50:60,20:20+hand_width] = 1

        # Generation of a colision template
        im_collision = np.zeros( (60,60), np.float )
        im_collision[10:50,20:20+hand_width] = 1

        param_fge = {"ds_rate": 0.5,
                     "target_lower":[0, 150, 100],
                     "target_upper":[15, 255, 255],
                     "fg_lower": [0, 0, 100],
                     "fg_upper": [179, 255, 255],
                     "hand_rotation":[0,45,90,135],
                     "threshold": 0.01}

        fge = FastGraspabilityEvaluation(im_in, im_hand, im_collision,
                                         param_fge)
        results = fge.main_proc()
        im_vis  = fge.visualization(im_vis)
        cv2.imwrite("reslut_grasp_points.png", im_vis)

        bbox = ssd_result["bbox"]

        return [ gmsg.Pose2D(x=result[1] - bbox[0],
                             y=result[0] - bbox[1],
                             theta=-radians(result[2]))
                 for result in results ], \
               im_vis


if __name__ == '__main__':
    rospy.init_node('o2ac_vision', anonymous=False)
    c = O2ACVision()
    rospy.spin()
