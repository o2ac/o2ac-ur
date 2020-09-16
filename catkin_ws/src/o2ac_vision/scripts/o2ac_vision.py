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
import numpy as np
import time
import rospkg
import message_filters
import threading
import o2ac_ssd
from geometry_msgs     import msg as gmsg
from sensor_msgs       import msg as smsg
from cv_bridge import CvBridge
from pose_estimation_func import template_matching
from pose_estimation_func import FastGraspabilityEvaluation

ssd_detection = o2ac_ssd.ssd_detection()

class O2ACVision(object):
    _Colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0),
               (255, 255, 0), (255, 0, 255), (0, 255, 255))

    # This class advertises the vision actions that we will call during the tasks.

    def __init__(self):
    	rospy.init_node('o2ac_vision_', anonymous=False)
    	self.listener = tf.TransformListener()

        # Setup synchronized ROS subscribers for camera info and image topics
        self.camera_info_sub = message_filters.Subscriber('/camera_info',
                                                          smsg.CameraInfo)
        self.image_sub = message_filters.Subscriber('/image', smsg.Image)
        self.depth_sub = message_filters.Subscriber('/depth', smsg.Image)
        sync = message_filters.ApproximateTimeSynchronizer(
                [self.camera_info_sub, self.image_sub, self.depth_sub],
                10, 0.01)
        sync.registerCallback(self.synced_images_callback)

    	# This creates the action server.nSee this tutorial, and also check out base.py:
    	# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
        self.pose_estimation_server = actionlib.SimpleActionServer("poseEstimationTest", o2ac_msgs.msg.poseEstimationTestAction, auto_start = False)
        self.pose_estimation_server.register_goal_callback(self.goal_callback)
        self.pose_estimation_server.register_preempt_callback(self.preempt_callback)
        self.pose_estimation_server.start()

        self.belt_detection_server = actionlib.SimpleActionServer("beltDetectionTest", o2ac_msgs.msg.beltDetectionTestAction, auto_start = False)
        self.belt_detection_server.register_goal_callback(self.belt_detection_goal_callback)
        self.belt_detection_server.register_preempt_callback(self.belt_detection_preempt_callback)
        self.belt_detection_server.start()

        self.image_pub = rospy.Publisher('~image', smsg.Image, queue_size=1)

        rospy.loginfo("O2AC_vision has started up!")

    def goal_callback(self):
        self.object_id = self.pose_estimation_server.accept_new_goal().object_id
        rospy.loginfo("Received a request to detect object named "
                      + self.object_id)

    def preempt_callback(self):
        rospy.loginfo("o2ac_msgs.msg.poseEstimationTestAction preempted")
        self.pose_estimation_server.set_preempted()

    def synced_images_callback(self, camera_info, image, depth):
        """
        Callback for storing camera_info and RGB/depth images from camera node
        """
        if not self.pose_estimation_server.is_active():
            return

        action_result = o2ac_msgs.msg.poseEstimationTestResult()

        # First, obtain the image from the camera and convert it
        image_msg = rospy.wait_for_message("/b_bot_outside_camera_throttled/color/image_raw/compressed", sensor_msgs.msg.CompressedImage, 1.0)
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        #im_in = cv2.resize( cv_image, None, fx=0.35, fy=0.35, interpolation=cv2.INTER_NEAREST )
        im_in = cv_image.copy()

        # Object detection in image (confidence, class_id, bbox)
        ssd_results = self.detect_object_in_image(im_in)

        # Pose estimation
        #
        apply_2d_pose_estimation = [8,9,10,14]
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]
        apply_grasp_detection = [6]
        for ssd_result in ssd_results:
            target = ssd_result["class"]
            poseEstimationResult_msg = o2ac_msgs.msg.poseEstimationResult()
            poseEstimationResult_msg.confidence = ssd_result["confidence"]
            poseEstimationResult_msg.class_id = target

            # Pose estimation
            #
            apply_2d_pose_estimation = [8,9,10,14]
            apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]
            apply_grasp_detection = [6]
            for ssd_result in ssd_results:
                target = ssd_result["class"]
                poseEstimationResult_msg = o2ac_msgs.msg.poseEstimationResult()
                poseEstimationResult_msg.confidence = ssd_result["confidence"]
                poseEstimationResult_msg.class_id = target
                poseEstimationResult_msg.bbox = ssd_result["bbox"]

            elif target in apply_3d_pose_estimation:
                print("Target id is ", target, " apply the 3d pose estimation")
                # poseEstimationResult_msg.3d_pose = ***

                elif target in apply_3d_pose_estimation:
                    print("Target id is ", target, " apply the 3d pose estimation")

                else:
                    print("Target id is ", target, " NO Solution")

        # Return
        # o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
        self.pose_estimation_server.set_succeeded(action_result)

                self.draw_bbox(im_in,
                               poseEstimationResult_msg.class_id,
                               poseEstimationResult_msg.bbox)

            # Return
            # o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
            self.pose_estimation_server.set_succeeded(action_result)

            # Draw bbox and publish the result image
            imsg = CvBridge().cv2_to_imgmsg(im_in, encoding='passthrough')
            self.image_pub.publish(imsg)

        elif self.belt_detection_server.is_active():
            action_result = o2ac_msgs.msg.beltDetectionTestResult()

        # Belt detection in image
        grasp_points = self.grasp_detection_in_image( im_in )
        print("grasp_result")
        print(grasp_points)
        for grasp_point in grasp_points:
            GraspPoint_msg = o2ac_msgs.msg.GraspPoint()
            GraspPoint_msg.grasp_point = grasp_point
            action_result.grasp_points.append(GraspPoint_msg)

        # Return
        # o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
        self.belt_detection_server.set_succeeded(action_result)

### =======

    def detect_object_in_image(self, cv_image):
        ssd_results = ssd_detection.object_detection(cv_image)
        return ssd_results

    def grasp_detection_in_image( self, im_in ):

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

        im_result = fge.visualization()
        cv2.imwrite("reslut_grasp_points.png", im_result )

        return results

    def estimate_pose_in_image(self, im_in, ssd_result):

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
        im_result = tm.get_result_image( ssd_result, ori, center )

        name ="tm_result"+str(class_id)+".png"
        print( "Save", name )
        cv2.imwrite( name, im_result )

        return result

    def draw_bbox(self, image, id, bbox):
        idx = id % len(O2ACVision._Colors)
        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                      O2ACVision._Colors[idx], 3)
        cv2.putText(image, str(id + 1), (bbox[0] + 5, bbox[3] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, O2ACVision._Colors[idx], 2,
                    cv2.LINE_AA)


if __name__ == '__main__':
    try:
        c = O2ACVision()
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
