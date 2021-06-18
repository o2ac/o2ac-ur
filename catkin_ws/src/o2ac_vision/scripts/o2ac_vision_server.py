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
import tf
import tf_conversions
import time
import copy
import os
from datetime import datetime
import rospkg
import actionlib
import numpy as np
from math import pi, radians, degrees
tau = 2.0*pi  # Part of math from Python 3.6
import cv2
import cv_bridge  # This offers conversion methods between OpenCV
                  # and ROS formats
                  # See here:
                  #   http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
                  # Note that a similar package exists for PCL:
                  #   http://wiki.ros.org/pcl_ros

import o2ac_routines.helpers
import sensor_msgs.msg
import o2ac_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import std_msgs.msg

from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient

from o2ac_vision.pose_estimation_func import TemplateMatching
from o2ac_vision.pose_estimation_func import FastGraspabilityEvaluation
from o2ac_vision.pose_estimation_func import ShaftAnalysis
from o2ac_vision.pose_estimation_func import PickCheck

from o2ac_vision.cam_utils import O2ACCameraHelper

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

    def __init__(self):
        rospy.init_node('o2ac_vision_server', anonymous=False)

        camera_name = rospy.get_param('~camera_name', "camera_multiplexer")
        self.cam_helper = O2ACCameraHelper()
        # tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0)) #tf buffer length
        # self.listener = tf2_ros.TransformListener(tf_buffer)
        self.listener = tf.TransformListener()

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher('~result_image',
                                         sensor_msgs.msg.Image, queue_size=1)
        self.marker_pub = rospy.Publisher('~result_markers', visualization_msgs.msg.Marker, queue_size=1)
        self.marker_array_pub = rospy.Publisher('~result_marker_arrays', visualization_msgs.msg.MarkerArray, queue_size=1)
        
        # Parameters for the localization server
        self._models = (
               '00-ALL',                        # 0
               '01-BASE',                       # 1
               '02-PANEL',                      # 2
               '03-PANEL2',                     # 3
               '04_37D-GEARMOTOR-50-70',        # 4
               '05_MBRFA30-2-P6',               # 5
               '06_MBT4-400',                   # 6
               '07_SBARB6200ZZ_30',             # 7
               '08_KZAF1075NA4WA55GA20AA0',     # 8
               '09_EDCS10',                     # 9
               '10_CLBPS10_17_4',               # 10
               '11_MBRAC60-2-10',               # 11
               '12_CLBUS6-9-9.5',               # 12
               '13_MBGA30-2',                   # 13
               '14_BGPSL6-9-L30-F8',            # 14
              )

        self._nposes  = rospy.get_param('~nposes',  2)  # Number of pose candidates to be returned from localization
        self._timeout = rospy.get_param('~timeout', 10)

        # Setup clients for depth filtering and localization
        self._dfilter = DepthFilterClient('depth_filter')
        self._dfilter.window_radius = 2
        self._localizer = LocalizationClient('localization')

        # Load parameters for detecting graspabilities
        default_param_fge = {"ds_rate": 0.5,
                             "n_grasp_point": 50,
                             "threshold": 0.01}
        self.param_fge = rospy.get_param('~param_fge', default_param_fge)

        self.rospack = rospkg.RosPack()
        background_file = self.rospack.get_path("o2ac_vision") + "/config/shaft_background.png"
        self.shaft_bg_image = cv2.imread(background_file, 0) 

        # Setup camera image subscribers
        self.cam_info_sub = rospy.Subscriber('/' + camera_name + '/camera_info', sensor_msgs.msg.CameraInfo,
                                          self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber('/' + camera_name + '/depth', sensor_msgs.msg.Image,
                                          self.depth_image_sub_callback)
        self.image_sub = rospy.Subscriber('/' + camera_name + '/image', sensor_msgs.msg.Image,
                                          self.image_subscriber_callback)
        self.bridge = cv_bridge.CvBridge()

        # Determine whether the detection server works in continuous mode or not.
        self.continuous_streaming = rospy.get_param('~continuous_streaming', False)
        if self.continuous_streaming:
            # Setup publisher for object detection results
            self.results_pub = rospy.Publisher('~detection_results', o2ac_msgs.msg.Estimated2DPosesArray, queue_size=1)
            rospy.logwarn("Localization action server is not running because SSD results are being streamed! Turn off continuous mode to use localization.")
        else:
            # Setup the localization 
            self.get_2d_poses_from_ssd_action_server = actionlib.SimpleActionServer("~get_2d_poses_from_ssd", o2ac_msgs.msg.get2DPosesFromSSDAction,
                execute_cb = self.get_2d_poses_from_ssd_goal_callback, auto_start=False)
            self.get_2d_poses_from_ssd_action_server.start()

            self.get_3d_poses_from_ssd_action_server = actionlib.SimpleActionServer("~get_3d_poses_from_ssd", o2ac_msgs.msg.get3DPosesFromSSDAction,
                execute_cb = self.get_3d_poses_from_ssd_goal_callback, auto_start=False)
            self.get_3d_poses_from_ssd_action_server.start()

            self.localization_server = actionlib.SimpleActionServer("~localize_object", o2ac_msgs.msg.localizeObjectAction,
                execute_cb=self.localization_callback, auto_start = False)
            self.localization_server.register_preempt_callback(self.localization_preempt_callback)
            self.localization_server.start()
        
        self.belt_detection_action_server = actionlib.SimpleActionServer("~belt_detection", o2ac_msgs.msg.beltDetectionAction,
            execute_cb = self.belt_detection_callback, auto_start=False)
        self.belt_detection_action_server.start()
        
        self.angle_detection_action_server = actionlib.SimpleActionServer("~detect_angle", o2ac_msgs.msg.detectAngleAction,
            execute_cb = self.angle_detection_callback, auto_start=False)
        self.angle_detection_action_server.start()
        # Action client to forward the calculation to the Python3 node
        self._py3_axclient = actionlib.SimpleActionClient('/o2ac_py3_vision_server/internal/detect_angle', o2ac_msgs.msg.detectAngleAction)
        
        self.shaft_notch_detection_action_server = actionlib.SimpleActionServer("~detect_shaft_notch", o2ac_msgs.msg.shaftNotchDetectionAction,
            execute_cb = self.shaft_notch_detection_callback, auto_start=False)
        self.shaft_notch_detection_action_server.start()

        self.pick_success_action_server = actionlib.SimpleActionServer("~check_pick_success", o2ac_msgs.msg.checkPickSuccessAction,
            execute_cb = self.pick_success_callback, auto_start=False)
        self.pick_success_action_server.start()
        
        # For visualization
        self.pose_marker_id_counter = 0
        self.pose_marker_array = 0
        self._depth_image_ros = None
        rospy.loginfo("O2AC_vision has started up!")

    def camera_info_callback(self, cam_info_message):
        self._camera_info = cam_info_message

    def depth_image_sub_callback(self, depth_image_msg):
        self._depth_image_ros = depth_image_msg

    def get_3d_poses_from_ssd_goal_callback(self, goal):
        self.get_3d_poses_from_ssd_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect objects via SSD")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()
        
        action_result = o2ac_msgs.msg.get3DPosesFromSSDResult()
        poses_2d_with_id, im_vis, upside_down_list = self.get_2d_poses_from_ssd(im_in, im_vis)

        action_result.poses = []
        action_result.class_ids = []
        action_result.upside_down = []
        for (array, upside_down) in zip(poses_2d_with_id, upside_down_list):
            for pose2d in array.poses:
                action_result.class_ids.append(array.class_id)
                p3d = self.convert_pose_2d_to_3d(pose2d)
                if p3d:
                    action_result.poses.append(p3d)
                    action_result.upside_down.append(upside_down)
        self.get_3d_poses_from_ssd_action_server.set_succeeded(action_result)
        # Publish result visualization
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "3d_poses_from_ssd")

    def get_2d_poses_from_ssd_goal_callback(self, goal):
        self.get_2d_poses_from_ssd_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect objects via SSD")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()
        
        action_result = o2ac_msgs.msg.get2DPosesFromSSDResult()
        action_result.results, im_vis, action_result.upside_down = self.get_2d_poses_from_ssd(im_in, im_vis)

        self.get_2d_poses_from_ssd_action_server.set_succeeded(action_result)

        # Publish result visualization
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "2d_poses_from_ssd")

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
        
        poses_3d = []
        for p2d in poses_2d:
            p3d = self.convert_pose_2d_to_3d(p2d)
            if p3d:
                poses_3d.poses.append(p3d)
        
        action_result.grasp_points = poses_3d
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.belt_detection_action_server.set_succeeded(action_result)
        self.write_to_log(im_in, im_vis, "belt_detection")

    def angle_detection_callback(self, goal):
        self.angle_detection_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect angle of " + goal.item_id)
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        # im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        # im_vis = im_in.copy()
        
        # Pass action goal to Python3 node
        action_goal = goal
        action_goal.rgb_image = copy.deepcopy(self.last_rgb_image)
        self._py3_axclient.send_goal(action_goal)
        
        if (not self._py3_axclient.wait_for_result(rospy.Duration(3.0))):
            rospy.logerr("Angle detection timed out.")
            self._py3_axclient.cancel_goal()  # Cancel goal if timeout expired
        
        action_result = self._py3_axclient.get_result()
        self.angle_detection_action_server.set_succeeded(action_result)

    def shaft_notch_detection_callback(self, goal):
        self.shaft_notch_detection_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect shaft notch")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")

        action_result = o2ac_msgs.msg.shaftNotchDetectionResult()
        
        top, bottom, im_vis = self.shaft_notch_detection(im_in)
        print("top", top, "bottom", bottom)
        
        action_result.shaft_notch_detected_at_top = top
        action_result.shaft_notch_detected_at_bottom = bottom
        action_result.no_shaft_notch_detected = (not top and not bottom)

        self.shaft_notch_detection_action_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "shaft_notch_detection")

    def pick_success_callback(self, goal):
        self.pick_success_action_server.accept_new_goal()
        rospy.loginfo("Received a request to detect pick success for item: " + goal.item_id)
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")

        action_result = o2ac_msgs.msg.checkPickSuccessResult()
        
        pick_successful, im_vis = self.check_pick_success(im_in, goal.item_id)

        action_result.item_is_picked = pick_successful
        action_result.success = True
        
        self.pick_success_action_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "pick_success")

    def localization_callback(self, goal):
        rospy.loginfo("Received a request to localize objects via CAD matching")
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        im_in  = self.bridge.imgmsg_to_cv2(self.last_rgb_image, desired_encoding="bgr8")
        im_vis = im_in.copy()
        # Apply SSD first to get the object's bounding box
        detection_results, im_vis = self.get_2d_poses_from_ssd(im_in, im_vis)

        localization_result = o2ac_msgs.msg.localizeObjectResult()
        localization_result.succeeded = False

        for result in detection_results:
            # Only pass to CAD matching if object is found in the RGB image
            if goal.item_id == self.item_id(result.class_id):
                rospy.loginfo("Seen object " + str(goal.item_id) + " in tray. Apply CAD matching.")
                localization_result.detected_poses, \
                localization_result.confidences     \
                    = self.localize(goal.item_id, result.bbox, result.poses)

                if localization_result.detected_poses:
                    localization_result.succeeded = True
                    self.localization_server.set_succeeded(localization_result)
                    return
                else:
                    rospy.logerr("Could not not localize object " + str(goal.item_id) + " although it was seen by SSD.")
                break
        rospy.logerr("Could not not localize object " + str(goal.item_id) + ". Might not be detected by SSD.")
        self.localization_server.set_aborted(localization_result)
        self.write_to_log(im_in, im_vis, "localization")
    
    def localization_preempt_callback(self):
        rospy.loginfo("o2ac_msgs.msg.localizeObjectAction preempted")
        self.localization_server.set_preempted()

    def image_subscriber_callback(self, image):
        # Save the camera image locally
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        self.last_rgb_image = image
        
        # Pass image to SSD if continuous display is turned on
        if self.continuous_streaming:
            # with self.lock:
            im_in  = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            im_vis = im_in.copy()

            Estimated2DPoses_msg = o2ac_msgs.msg.Estimated2DPosesArray()
            Estimated2DPoses_msg.header = image.header
            Estimated2DPoses_msg.results, im_vis = self.get_2d_poses_from_ssd(im_in, im_vis)
            self.results_pub.publish(Estimated2DPoses_msg)

            # Publish images visualizing results
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

### ======= Localization helpers

    def item_id(self, class_id):
        return self._models[class_id]

    def localize(self, item_id, bbox, poses2d):
        self._dfilter.roi = (bbox[0],           bbox[1],
                             bbox[0] + bbox[2], bbox[1] + bbox[3])
        while not self._dfilter.capture():  # Load PLY data to the localizer
            pass
        self._localizer.send_goal(item_id, self._nposes, poses2d)
        return self._localizer.wait_for_result(rospy.Duration(self._timeout))

### =======

    def get_2d_poses_from_ssd(self, im_in, im_vis):
        """
        Finds an object's bounding box on the tray, then attempts to find its pose.
        Can also find grasp poses for the belt.

        Returns all bounding boxes with confidences, and returns all the 2D poses 
        """
        # Object detection
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        self.reset_pose_markers()        

        # Pose estimation
        estimated_poses_array    = []                      # Contains all results
        apply_2d_pose_estimation = [8,9,10,14]             # Small items --> Neural Net
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]  # Large items --> CAD matching
        apply_grasp_detection    = [6]                     # Belt --> Fast Grasp Estimation

        item_flipped_over = []
        for ssd_result in ssd_results:
            target = ssd_result["class"]
            estimated_poses_msg = o2ac_msgs.msg.Estimated2DPoses() # Stores the result for one item/class id
            estimated_poses_msg.confidence = ssd_result["confidence"]
            estimated_poses_msg.class_id   = target
            estimated_poses_msg.bbox       = ssd_result["bbox"]

            if target in apply_2d_pose_estimation:
                rospy.loginfo("Seeing object id %d. Apply 2D pose estimation",
                              target)
                pose, im_vis = self.estimate_pose_in_image(im_in, im_vis,
                                                           ssd_result)
                estimated_poses_msg.poses = [pose]
                # Publish result markers
                poses_3d = []
                for p2d in estimated_poses_msg.poses:
                    p3d = self.convert_pose_2d_to_3d(p2d)
                    if not p3d:
                        rospy.logwarn("Could not find pose for class " + str(target) + "!")
                        continue
                    poses_3d.append(p3d)
                    # rospy.loginfo("Found pose for class " + str(target) + ": " + str(poses_3d[-1].pose.position.x) + ", " + str(poses_3d[-1].pose.position.y) + ", " + str(poses_3d[-1].pose.position.z))
                    # rospy.loginfo("Found pose for class " + str(target) + ": %2f, %2f, %2f", 
                    #                poses_3d[-1].pose.position.x, poses_3d[-1].pose.position.y, poses_3d[-1].pose.position.z)
                if not poses_3d:
                    rospy.logwarn("Could not find poses for class " + str(target) + "!")
                    continue
                self.add_markers_to_pose_array(poses_3d)

            elif target in apply_3d_pose_estimation:
                rospy.loginfo("Seeing object id %d. Apply 3D pose estimation",
                              target)
                x = int(estimated_poses_msg.bbox[0] + round(estimated_poses_msg.bbox[2]/2))
                y = int(estimated_poses_msg.bbox[1] + round(estimated_poses_msg.bbox[3]/2))
                # Publish markers at bbox centers
                p2d = geometry_msgs.msg.Pose2D(x=x, y=y)
                estimated_poses_msg.poses = [p2d]
                p3d = self.convert_pose_2d_to_3d(p2d)
                if not p3d:
                    rospy.logwarn("Could not find pose for class " + str(target) + "!")
                    continue
                poses_3d = [p3d]
                # rospy.loginfo("Found pose for class " + str(target) + ": " + str(poses_3d[-1].pose.position.x) + ", " + str(poses_3d[-1].pose.position.y) + ", " + str(poses_3d[-1].pose.position.z))
                # rospy.loginfo("Found pose for class " + str(target) + ": %2f, %2f, %2f", 
                #                    poses_3d[-1].pose.position.x, poses_3d[-1].pose.position.y, poses_3d[-1].pose.position.z)
                self.add_markers_to_pose_array(poses_3d)

            elif target in apply_grasp_detection:
                rospy.loginfo("Seeing object id %d (belt). Apply grasp detection", target)
                estimated_poses_msg.poses, im_vis \
                    = self.belt_grasp_detection_in_image(im_in, im_vis, ssd_result)
                rospy.loginfo("Saw " + str(len(estimated_poses_msg.poses)) + " belt grasp poses.")
                # Publish result markers
                poses_3d = []
                for p2d in estimated_poses_msg.poses:
                    p3d = self.convert_pose_2d_to_3d(p2d)
                    if p3d:
                        poses_3d.append(p3d)
                self.publish_belt_grasp_pose_markers(poses_3d)
                rospy.loginfo("Done with belt grasp detection")

            estimated_poses_array.append(estimated_poses_msg)
            item_flipped_over.append(ssd_result["state"])
        
        self.publish_stored_pose_markers()
        return estimated_poses_array, im_vis, item_flipped_over

    def detect_object_in_image(self, cv_image, im_vis):
        ssd_results, im_vis = ssd_detection.object_detection(cv_image, im_vis)
        return ssd_results, im_vis

    def estimate_pose_in_image(self, im_in, im_vis, ssd_result):
        """
        2D pose estimation (x, y, theta) 
        """
        start = time.time()

        temp_root = self.rospack.get_path("wrs_dataset") + "/data/templates"
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

        return geometry_msgs.msg.Pose2D(x=center[1],
                                      y=center[0],
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
        im_hand[20:20+hand_width,0:10] = 1
        im_hand[20:20+hand_width,50:60] = 1

        bbox = ssd_result["bbox"]
        margin = 30
        top_bottom = slice(max(bbox[1] - margin, 0), min(bbox[1] + bbox[3] + margin, 480))
        left_right = slice(max(bbox[0] - margin, 0), min(bbox[0] + bbox[2] + margin, 640))

        fge = FastGraspabilityEvaluation(im_in[top_bottom, left_right],
                                         im_hand, self.param_fge)
        results = fge.main_proc()

        elapsed_time = time.time() - start
        rospy.logdebug("Belt detection processing time[msec]: %d", 1000*elapsed_time)

        im_vis[top_bottom, left_right] = fge.visualization(im_vis[top_bottom, left_right])

        # Subtracting tau/4 (90 degrees) to the rotation result to match the gripper_tip_link orientation
        return [ geometry_msgs.msg.Pose2D(x=result[1] +(bbox[0] - margin),
                                          y=result[0] +(bbox[1] - margin),
                                          theta=radians(result[2])-tau/4)
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

    def shaft_notch_detection(self, im_in):
        """
        Detects the notch in the supplied image (looking at the shaft in the V-jig).

        Return values:
        top:    True if notch is seen at top of image
        bottom: True if notch is seen at bottom of image
        im_vis: Result visualization
        """
        # Convert to grayscale
        if im_in.shape[2] == 3: im_gray = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY) 

        bbox = [350,100,100,400] # ROI(x,y,w,h) of shaft area
        sa = ShaftAnalysis( self.shaft_bg_image, im_gray, bbox ) 
        res = sa.main_proc() # threshold.
        im_vis = sa.get_result_image()
        # TODO(cambel): How to know where is the notch?
        if res == 1:
            return False, False, im_vis 
        front = (res == 3)
        back = (res == 2)
        return front, back, im_vis

    def check_pick_success(self, im_in, item_id):
        """ item_id is the object name, not the ID.
        """
        if item_id == "belt":
            class_id = 6
        pc = PickCheck(ssd_detection)
        pick_successful = pc.check( im_in, class_id )
        im_vis = pc.get_im_result()
        return pick_successful, im_vis
        

### ========

    def convert_pose_2d_to_3d(self, pose_2d):
        """
        Convert a 2D pose to a 3D pose in the tray using the current depth image.
        Returns a PoseStamped in tray_center.
        """
        p3d = geometry_msgs.msg.PoseStamped()
        p3d.header.frame_id = self._camera_info.header.frame_id
        if self._depth_image_ros is None:
            rospy.logerr("No depth image found")
            return
        depth_image = self.bridge.imgmsg_to_cv2(self._depth_image_ros, desired_encoding="passthrough")
        xyz = self.cam_helper.project_2d_to_3d_from_images(pose_2d.x, pose_2d.y, [depth_image])
        if not xyz:
            return None
        p3d.pose.position.x = xyz[0]
        p3d.pose.position.y = xyz[1]
        p3d.pose.position.z = xyz[2]
        
        # Transform position to tray_center
        try:
            p3d.header.stamp = rospy.Time(0.0)
            p3d = self.listener.transformPose("tray_center", p3d)
        except Exception as e:
            rospy.logerr('Pose transform failed(): {}'.format(e))
        
        p3d.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, -pose_2d.theta+tau/4))
        return p3d

    def write_to_log(self, img_in, img_out, action_name):
        now = datetime.now()
        timeprefix = now.strftime("%Y-%m-%d_%H:%M:%S")
        folder = os.path.join(self.rospack.get_path("o2ac_vision"), "log")
        cv2.imwrite(os.path.join(folder, timeprefix + "_" + action_name + "_in.png") , img_in)
        cv2.imwrite(os.path.join(folder, timeprefix + "_" + action_name + "_out.jpg") , img_out)

### ========  Visualization

    def publish_belt_grasp_pose_markers(self, grasp_poses_3d):
        # Clear the namespace first
        self.clear_markers(namespace='/belt_grasp_poses')
        
        # Then make array of grasp pose markers

        marker_array = visualization_msgs.msg.MarkerArray()
        show_arrows = False
        grasp_width = 0.01
        i = 0
        for idx, grasp_pose in enumerate(grasp_poses_3d):
            # print("treating grasp pose: " + str(idx))
            # print(grasp_pose.pose.position)
            # Set an auxiliary transform to display the gripper pads (otherwise they are not rotated)
            base_frame_name = grasp_pose.header.frame_id
            helper_frame_name = 'belt_grasp_pose_helper_frame_' + str(idx)
            auxiliary_transform = geometry_msgs.msg.TransformStamped()
            auxiliary_transform.header.frame_id = base_frame_name
            auxiliary_transform.child_frame_id = helper_frame_name
            auxiliary_transform.transform.translation = geometry_msgs.msg.Vector3(grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z)
            auxiliary_transform.transform.rotation = grasp_pose.pose.orientation
            self.listener.setTransform(auxiliary_transform)

            # Neutral PoseStamped
            p0 = geometry_msgs.msg.PoseStamped()
            p0.header.frame_id = helper_frame_name
            p0.header.stamp = rospy.Time(0.0)
            p0.pose.position = geometry_msgs.msg.Point(0, 0, 0)
            p0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))

            # Define first marker
            i += 1
            center_marker = visualization_msgs.msg.Marker()
            center_marker.ns = '/belt_grasp_poses'
            center_marker.id = i
            center_marker.type = center_marker.SPHERE
            center_marker.action = center_marker.ADD
            
            center_marker.header = copy.deepcopy(grasp_pose.header)

            # Get pose from auxiliary transform
            p = copy.deepcopy(p0)
            p = self.listener.transformPose(base_frame_name, p)
            center_marker.pose = p.pose
            # print("center_marker after transform: ")
            # print(center_marker.pose.position)

            center_marker.scale.x = 0.005
            center_marker.scale.y = 0.005
            center_marker.scale.z = 0.005

            center_marker.color.a = 0.8
            center_marker.color.r = 0.2
            center_marker.color.g = 0.9
            center_marker.color.b = 0.2
            marker_array.markers.append(center_marker)

            # Repeat for the next markers
            i += 1
            gripper_pad_marker = copy.deepcopy(center_marker)
            gripper_pad_marker.id = i
            gripper_pad_marker.type = gripper_pad_marker.CUBE

            p = copy.deepcopy(p0)
            p.pose.position = geometry_msgs.msg.Point(0, grasp_width, 0)
            p = self.listener.transformPose(base_frame_name, p)
            gripper_pad_marker.pose = p.pose
            
            gripper_pad_marker.scale.x = 0.03
            gripper_pad_marker.scale.y = 0.002
            gripper_pad_marker.scale.z = 0.02

            gripper_pad_marker.color.a = 0.5

            marker_array.markers.append(gripper_pad_marker)

            
            i += 1
            gripper_pad_marker2 = copy.deepcopy(gripper_pad_marker)
            gripper_pad_marker2.id = i

            p = copy.deepcopy(p0)
            p.pose.position = geometry_msgs.msg.Point(0, -grasp_width, 0)
            p = self.listener.transformPose(base_frame_name, p)
            gripper_pad_marker2.pose = p.pose

            gripper_pad_marker2.color.r = 0.9
            gripper_pad_marker2.color.g = 0.2
            gripper_pad_marker2.color.b = 0.2
            marker_array.markers.append(gripper_pad_marker2)

            if show_arrows:
                arrow_marker = copy.deepcopy(center_marker)
                arrow_marker.type = visualization_msgs.msg.Marker.ARROW
                arrow_marker.color = std_msgs.msg.ColorRGBA(0, 0, 0, 0.8)
                arrow_marker.scale = geometry_msgs.msg.Vector3(.05, .005, .005)

                i += 1
                arrow_x = copy.deepcopy(arrow_marker)
                arrow_x.id = i
                # Pose is already OK, arrow points along x

                i += 1
                arrow_y = copy.deepcopy(arrow_marker)
                arrow_y.id = i
                p = copy.deepcopy(p0)
                p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,tau/4))
                p = self.listener.transformPose(base_frame_name, p)
                arrow_y.pose = p.pose

                i += 1
                arrow_z = copy.deepcopy(arrow_marker)
                arrow_z.id = i
                p = copy.deepcopy(p0)
                p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-tau/4,0))
                p = self.listener.transformPose(base_frame_name, p)
                arrow_z.pose = p.pose
                
                arrow_x.color.r = 1.0
                arrow_y.color.g = 1.0
                arrow_z.color.b = 1.0

                marker_array.markers.append(arrow_x)
                marker_array.markers.append(arrow_y)
                marker_array.markers.append(arrow_z)
        # print(" i went up to " + str(i))
        self.marker_array_pub.publish(marker_array)

    def clear_markers(self, namespace="/belt_grasp_poses"):
        delete_marker_array = visualization_msgs.msg.MarkerArray()
        del_marker = visualization_msgs.msg.Marker()
        del_marker.ns = namespace
        del_marker.action = del_marker.DELETEALL
        for i in range(1):
            dm = copy.deepcopy(del_marker)
            dm.id = i
            delete_marker_array.markers.append(dm)
        self.marker_array_pub.publish(delete_marker_array)
    
    def reset_pose_markers(self):
        self.clear_markers(namespace="/objects")
        self.pose_marker_id_counter = 0
        self.pose_marker_array = visualization_msgs.msg.MarkerArray()
    
    def publish_stored_pose_markers(self):
        self.marker_array_pub.publish(self.pose_marker_array)

    def add_markers_to_pose_array(self, poses_3d):
        """
        Add markers to the pose array.
        """
        i = self.pose_marker_id_counter
        for idx, viz_pose in enumerate(poses_3d):
            # print("treating pose: " + str(idx))
            # print(viz_pose.pose.position)
            # Set an auxiliary transform to display the arrows (otherwise they are not rotated)
            base_frame_name = viz_pose.header.frame_id
            helper_frame_name = 'viz_pose_helper_frame_' + str(idx)
            auxiliary_transform = geometry_msgs.msg.TransformStamped()
            auxiliary_transform.header.frame_id = base_frame_name
            auxiliary_transform.child_frame_id = helper_frame_name
            auxiliary_transform.transform.translation = geometry_msgs.msg.Vector3(viz_pose.pose.position.x, viz_pose.pose.position.y, viz_pose.pose.position.z)
            auxiliary_transform.transform.rotation = viz_pose.pose.orientation
            self.listener.setTransform(auxiliary_transform)

            # Neutral PoseStamped
            p0 = geometry_msgs.msg.PoseStamped()
            p0.header.frame_id = helper_frame_name
            p0.header.stamp = rospy.Time(0.0)
            p0.pose.position = geometry_msgs.msg.Point(0, 0, 0)
            p0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))

            ## Draw arrows
            arrow_marker = visualization_msgs.msg.Marker()
            arrow_marker.ns = "/objects"
            arrow_marker.id = i
            arrow_marker.type = visualization_msgs.msg.Marker.ARROW
            arrow_marker.action = arrow_marker.ADD
            arrow_marker.header = viz_pose.header
            
            arrow_marker.color = std_msgs.msg.ColorRGBA(0, 0, 0, 0.8)
            arrow_marker.scale = geometry_msgs.msg.Vector3(.05, .005, .005)

            i += 1
            arrow_x = copy.deepcopy(arrow_marker)
            arrow_x.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))
            p = self.listener.transformPose(base_frame_name, p)
            arrow_x.pose = p.pose

            i += 1
            arrow_y = copy.deepcopy(arrow_marker)
            arrow_y.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,tau/4))
            p = self.listener.transformPose(base_frame_name, p)
            arrow_y.pose = p.pose

            i += 1
            arrow_z = copy.deepcopy(arrow_marker)
            arrow_z.id = i
            p = copy.deepcopy(p0)
            p.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-tau/4,0))
            p = self.listener.transformPose(base_frame_name, p)
            arrow_z.pose = p.pose
            
            arrow_x.color.r = 1.0
            arrow_y.color.g = 1.0
            arrow_z.color.b = 1.0

            self.pose_marker_array.markers.append(arrow_x)
            self.pose_marker_array.markers.append(arrow_y)
            self.pose_marker_array.markers.append(arrow_z)
        self.pose_marker_id_counter = i

if __name__ == '__main__':
    rospy.init_node('o2ac_vision_server', anonymous=False)
    c = O2ACVisionServer()
    rospy.spin()
