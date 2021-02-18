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

#import o2ac_routines.helpers
import sensor_msgs.msg
import o2ac_msgs.msg
import geometry_msgs.msg

import o2ac_routines.helpers
from o2ac_vision.pose_estimation_func import TemplateMatching
from o2ac_vision.pose_estimation_func import FastGraspabilityEvaluation

import o2ac_vision.o2ac_ssd
ssd_detection = o2ac_vision.o2ac_ssd.ssd_detection()

class O2ACVisionServer(object):
    """
    Advertises the vision actions that we will call during the tasks:
    - 2D Pose estimation
    - Belt detection
    - Cable tip detection
    - Bearing angle detection
    - Tooltip/hole tracking
    """
    # TODO (felixvd): Add the actions besides 2D pose estimation to this class

    def __init__(self):
    	rospy.init_node('o2ac_vision_server', anonymous=False)

        # Setup subscriber for RGB image
        self.image_sub = rospy.Subscriber('/camera_multiplexer/image', sensor_msgs.msg.Image,
                                          self.image_subscriber_callback)

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher('~result_image',
                                         sensor_msgs.msg.Image, queue_size=1)

        # Load parameters for detecting graspabilities
        default_param_fge = {"ds_rate": 0.5,
                             "target_lower":[0, 150, 100],
                             "target_upper":[45, 255, 255],
                             "fg_lower": [0, 0, 100],
                             "fg_upper": [179, 255, 255],
                             "hand_rotation":[0,45,90,135],
                             "threshold": 0.01}
        self.param_fge = rospy.get_param('~param_fge', default_param_fge)

        self.bridge = cv_bridge.CvBridge()

        # Determine whether the detection server works in continuous mode or not.
        self.continuous_streaming = rospy.get_param('~continuous_streaming', False)
        if self.continuous_streaming:
            # Setup publisher for object detection results
            self.results_pub \
                = rospy.Publisher('~detection_results',
                                  o2ac_msgs.msg.EstimatedPosesArray, queue_size=1)
            rospy.logwarn("Localization action server is not running because SSD results are being streamed! Turn off continuous mode to use localization.")
        else:
            self.pose_estimation_action_server = actionlib.SimpleActionServer("poseEstimation", o2ac_msgs.msg.poseEstimationAction,
                execute_cb = self.pose_estimation_goal_callback, auto_start=False)
            self.pose_estimation_action_server.start()    
        
        self.belt_detection_action_server = actionlib.SimpleActionServer("beltDetection", o2ac_msgs.msg.beltDetectionAction,
            execute_cb = self.belt_detection_callback, auto_start=False)
        self.belt_detection_action_server.start()    
        
        self.front_view_angle_detection_action_server = actionlib.SimpleActionServer("frontViewAngleDetection", o2ac_msgs.msg.frontViewAngleDetectionAction,
            execute_cb = self.front_view_angle_detection_callback, auto_start=False)
        self.front_view_angle_detection_action_server.start()    

        rospy.loginfo("O2AC_vision has started up!")

    def pose_estimation_goal_callback(self):
        self.pose_estimation_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect object")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()
        
        action_result = o2ac_msgs.msg.poseEstimationResult()
        action_result.results, im_vis = self.get_pose_estimation_results(im_in, im_vis)
        self.pose_estimation_action_server.set_succeeded(action_result)

        # Publish result visualization
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

    def belt_detection_callback(self, goal):
        self.belt_detection_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect belt grasp points")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()

        action_result = o2ac_msgs.msg.beltDetectionResult()
        # Get bounding boxes, then belt grasp points
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis) 
        poses_2d, im_vis = self.belt_grasp_detection_in_image(im_in, im_vis, ssd_result)
        # TODO: Transform results to 3D PoseStamped (ideally to tray_center frame)

        self.belt_detection_action_server.set_succeeded(action_result)

    def front_view_angle_detection_callback(self, goal):
        self.front_view_angle_detection_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect angle of " + goal.item_id)
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()

        action_result = o2ac_msgs.msg.frontViewAngleDetectionResult()
        # Object detection
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)
        poses_2d, im_vis = self.belt_grasp_detection_in_image(im_in, im_vis, ssd_result)
        # TODO: Transform results to 3D PoseStamped (ideally to tray_center frame)

        self.front_view_angle_detection_action_server.set_succeeded(action_result)

    def image_subscriber_callback(self, image):
        # Save the camera image locally
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        self.last_rgb_image = image
        
        # Pass image to SSD if continuous display is turned on
        if self.continuous_streaming:
            # with self.lock:
            im_in  = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            im_vis = im_in.copy()

            estimatedPoses_msg = o2ac_msgs.msg.EstimatedPosesArray()
            estimatedPoses_msg.header = image.header
            estimatedPoses_msg.results, im_vis = self.get_pose_estimation_results(im_in, im_vis)
            self.results_pub.publish(estimatedPoses_msg)

            # Publish images visualizing results
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))


### =======

    def get_pose_estimation_results(self, im_in, im_vis):
        """
        Finds an object's bounding box on the tray, then attempts to find its pose.
        Can also find grasp poses for the belt.
        """
        # Object detection
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        # Pose estimation
        estimatedPoses_msgs      = []
        apply_2d_pose_estimation = [8,9,10,14]             # Small items --> Neural Net
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]  # Large items --> CAD matching
        apply_grasp_detection    = [6]                     # Belt --> Fast Grasp Estimation
        for ssd_result in ssd_results:
            target = ssd_result["class"]
            estimatedPoses_msg = o2ac_msgs.msg.EstimatedPoses()
            estimatedPoses_msg.confidence = ssd_result["confidence"]
            estimatedPoses_msg.class_id   = target
            estimatedPoses_msg.bbox       = ssd_result["bbox"]

            if target in apply_2d_pose_estimation:
                rospy.logdebug("Target id is %d. Apply the 2d pose estimation",
                              target)
                pose, im_vis = self.estimate_pose_in_image(im_in, im_vis,
                                                           ssd_result)
                estimatedPoses_msg.poses = [pose]

            elif target in apply_3d_pose_estimation:
                rospy.logdebug("Target id is %d. Apply the 3d pose estimation",
                              target)

            elif target in apply_grasp_detection:
                rospy.logdebug("Target id is %d. Apply grasp detection", target)
                estimatedPoses_msg.poses, im_vis \
                    = self.belt_grasp_detection_in_image(im_in, im_vis, ssd_result)

            estimatedPoses_msgs.append(estimatedPoses_msg)

        return estimatedPoses_msgs, im_vis

    def detect_object_in_image(self, cv_image, im_vis):
        ssd_results, im_vis = ssd_detection.object_detection(cv_image, im_vis)
        return ssd_results, im_vis

    def estimate_pose_in_image(self, im_in, im_vis, ssd_result):
        """
        2D pose estimation (x, y, theta) 
        """
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
        tm = TemplateMatching(im_in, ds_rate, temp_root, temp_info_name)
        center, orientation = tm.compute( ssd_result )

        elapsed_time = time.time() - start
        rospy.logdebug("Processing time[msec]: %d", 1000*elapsed_time)

        im_vis = tm.get_result_image(ssd_result, orientation, center, im_vis)

        bbox = ssd_result["bbox"]

        return geometry_msgs.msg.Pose2D(x=center[1] - bbox[0],
                                      y=center[0] - bbox[1],
                                      theta=radians(orientation)), \
               im_vis

    def belt_grasp_detection_in_image(self, im_in, im_vis, ssd_result):
        """
        Fast Grasp Evaluation, used for detecting belt grasp poses in an RGB image.
        The Pose2D objects returned are in (u,v) image coordinates.
        """
        start = time.time()

        # Generation of a hand template
        im_hand = np.zeros( (60,60), np.float )
        hand_width = 20 #in pixel
        im_hand[0:10,20:20+hand_width] = 1
        im_hand[50:60,20:20+hand_width] = 1

        # Generation of a collision template
        im_collision = np.zeros( (60,60), np.float )
        im_collision[10:50,20:20+hand_width] = 1

        bbox = ssd_result["bbox"]
        margin = 30
        top_bottom = slice(bbox[1] - margin, bbox[1] + bbox[3] + margin)
        left_right = slice(bbox[0] - margin, bbox[0] + bbox[2] + margin)

        fge = FastGraspabilityEvaluation(im_in[top_bottom, left_right],
                                         im_hand, im_collision, self.param_fge)
        results = fge.main_proc()

        elapsed_time = time.time() - start
        rospy.logdebug("Belt detection processing time[msec]: %d", 1000*elapsed_time)

        im_vis[top_bottom, left_right] = fge.visualization(im_vis[top_bottom, left_right])

        return [ geometry_msgs.msg.Pose2D(x=result[1] - margin,
                                          y=result[0] - margin,
                                          theta=-radians(result[2]))
                 for result in results ], \
               im_vis

    def detect_angle_from_front_view(self, im_in, im_vis):
        """
        Receives a frontal view of the bearing or large pulley, 
        returns the angle by which the item needs to be rotated to be at the target position.
        A negative angle means that the object needs to be turned counter-clockwise.
        """

        # TODO: Return the angle for the bearing

        # TODO: Return the angle for the large pulley

if __name__ == '__main__':
    rospy.init_node('o2ac_vision_server', anonymous=False)
    c = O2ACVisionServer()
    rospy.spin()
