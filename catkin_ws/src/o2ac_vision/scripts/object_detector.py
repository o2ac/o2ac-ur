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
import cv2
import numpy as np
import time
import rospkg
import o2ac_ssd
from sensor_msgs import msg as smsg
from o2ac_msgs   import msg as omsg
from cv_bridge   import CvBridge
from pose_estimation_func import template_matching
from pose_estimation_func import FastGraspabilityEvaluation

ssd_detection = o2ac_ssd.ssd_detection()

class ObjectDetector(object):

    def __init__(self):
    	rospy.init_node('object_detector', anonymous=False)

        # Setup subscriber for RGB image
        self.image_sub   = rospy.Subscriber('/image', smsg.Image,
                                            self.image_subscriber_callback)
        self.results_pub = rospy.Publisher('~results',
                                           omsg.ObjectDetectionResults,
                                           queue_size=1)
        self.image_pub   = rospy.Publisher('~result_image', smsg.Image,
                                           queue_size=1)

        rospy.loginfo("object_detector has started up!")

    def image_subscriber_callback(self, image_msg):
        objectDetectionResults_msg = omsg.ObjectDetectionResults()
        objectDetectionResults_msg.header = image_msg.header

        # First, obtain the image from the camera and convert it
        bridge = CvBridge()
        im_in  = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        im_vis = im_in.copy()

        # Object detection in image (confidence, class_id, bbox)
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        # Pose estimation
        apply_2d_pose_estimation = [8,9,10,14]
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]
        apply_grasp_detection = [6]
        for ssd_result in ssd_results:
            target = ssd_result["class"]
            poseEstimationResult_msg = omsg.poseEstimationResult()
            poseEstimationResult_msg.confidence = ssd_result["confidence"]
            poseEstimationResult_msg.class_id = target
            poseEstimationResult_msg.bbox = ssd_result["bbox"]

            if target in apply_2d_pose_estimation:
                print("Target id is ", target, " apply the 2d pose estimation")
                pose_estimation_results, im_vis = \
                    self.estimate_pose_in_image(im_in, im_vis, ssd_result)
                poseEstimationResult_msg.rotation = pose_estimation_results[1]
                poseEstimationResult_msg.center.append(pose_estimation_results[0][0])
                poseEstimationResult_msg.center.append(pose_estimation_results[0][1])
            elif target in apply_3d_pose_estimation:
                print("Target id is ", target, " apply the 3d pose estimation")
            else:
                print("Target id is ", target, " NO Solution")

            objectDetectionResults_msg.pose_estimation_results.append(poseEstimationResult_msg)

        # Belt detection in image
        grasp_points, im_vis = self.grasp_detection_in_image( im_in, im_vis )
        print("grasp_result")
        print(grasp_points)
        for grasp_point in grasp_points:
            GraspPoint_msg = o2ac_msgs.msg.GraspPoint()
            GraspPoint_msg.grasp_point = grasp_point
            objectDetectionResults_msg.grasp_points.append(GraspPoint_msg)

        # Publish object detection results
        self.results_pub.publish(objectDetectionResults_msg)

        # Publish an image for visualizing results
        self.image_pub.publish(bridge.cv2_to_imgmsg(im_vis,
                                                    encoding='passthrough'))

    def detect_object_in_image(self, im_in, im_vis):
        ssd_results, im_vis = ssd_detection.object_detection(im_in, im_vis)
        return ssd_results, im_vis

    def grasp_detection_in_image(self, im_in, im_vis):

        start = time.time()

        # Generation of a hand template
        im_hand = np.zeros( (60,60), np.float )
        hand_width = 20 #in pixel
        im_hand[0:10,20:20+hand_width] = 1
        im_hand[50:60,20:20+hand_width] = 1

        # Generation of a colision template
        im_collision = np.zeros( (60,60), np.float )
        im_collision[10:50,20:20+hand_width] = 1

        param_fge = {"ds_rate": 0.5, "target_lower":[0, 150, 100], "target_upper":[15, 255, 255],
                    "fg_lower": [0, 0, 100], "fg_upper": [179, 255, 255], "hand_rotation":[0,45,90,135], "threshold": 0.01}

        fge = FastGraspabilityEvaluation( im_in, im_hand, im_collision, param_fge )
        results = fge.main_proc()
        im_vis  = fge.visualization(im_vis)

        return results, im_vis

    def estimate_pose_in_image(self, im_in, im_vis, ssd_result):

        start = time.time()

        rospack = rospkg.RosPack()
        temp_root = rospack.get_path("o2ac_vision") + "/dataset/data/templates"
        # Name of template infomation
        temp_info_name = "template_info.json"
        # downsampling rate
        ds_rate = 1.0/2.0

        # RGB -> Gray
        if im_in.shape[2] == 3:
            im_in = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)

        class_id = ssd_result["class"]

        tm = template_matching( im_in, ds_rate, temp_root, temp_info_name )

        center = ori = 0.0

        # template matching
        center, ori = tm.compute( ssd_result )
        result = (center, ori)

        elapsed_time = time.time() - start
        print( "Processing time[msec]: ", 1000*elapsed_time )

        im_vis = tm.get_result_image(ssd_result, ori, center, im_vis)

        return result, im_vis


if __name__ == '__main__':
    try:
        c = ObjectDetector()
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
