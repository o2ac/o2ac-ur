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
import message_filters
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

import sensor_msgs.msg
import o2ac_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import std_msgs.msg
import std_srvs.srv

from aist_depth_filter     import DepthFilterClient
from aist_new_localization import LocalizationClient
from aist_model_spawner    import ModelSpawnerClient

from o2ac_vision.pose_estimation_func import MotorOrientation, ShaftHoleDetection, TemplateMatching
from o2ac_vision.pose_estimation_func import FastGraspabilityEvaluation
from o2ac_vision.pose_estimation_func import ShaftAnalysis
from o2ac_vision.pose_estimation_func import PickCheck
from o2ac_vision.pose_estimation_func import PulleyScrewDetection

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

        # Load parameters for detecting graspabilities
        default_param_fge = {"ds_rate": 0.5,
                             "n_grasp_point": 50,
                             "threshold": 0.01}
        self._param_fge = rospy.get_param('~param_fge', default_param_fge)


        # Load parameters for localization
        self._param_localization = rospy.get_param('~localization_parameters')
        self._models = tuple(part_id
                             for part_id
                             in sorted(list(self._param_localization.keys())))

        self.rospack = rospkg.RosPack()
        background_file = self.rospack.get_path("o2ac_vision") + "/config/shaft_background.png"
        self.shaft_bg_image = cv2.imread(background_file, 0)

        shaft_w_hole_file  = self.rospack.get_path("o2ac_vision") + "/config/shaft_w_hole.png"
        shaft_wo_hole_file = self.rospack.get_path("o2ac_vision") + "/config/shaft_wo_hole.png"
        sdh_bbox = rospy.get_param("shaft_hole_detection/bbox", [350,100,100,100])
        self.shaft_hole_detector = ShaftHoleDetection(shaft_w_hole_file, shaft_wo_hole_file, sdh_bbox)

        pulley_template_filepath = self.rospack.get_path("wrs_dataset") + "/data/pulley/pulley_screw_temp.png"
        self.pulley_screws_template_image = cv2.imread(pulley_template_filepath, 0)
        self.pulley_screw_detection_stream_active = False
        self.activate_pulley_screw_detection_service = rospy.Service("~activate_pulley_screw_detection", std_srvs.srv.SetBool, self.set_pulley_screw_detection_callback)

        # Determine whether the detection server works in continuous mode or not.
        self.continuous_streaming = rospy.get_param('~continuous_streaming',
                                                    False)

        if self.continuous_streaming:
            rospy.logwarn("Localization action server is not running because SSD results are being streamed! Turn off continuous mode to use localization.")
        else:
            # Clients for depth filtering and localization
            self._dfilter   = DepthFilterClient('depth_filter')
            self._localizer = LocalizationClient('localization')
            self._spawner   = ModelSpawnerClient('model_spawner')

            # Action server for 2D localization by SSD
            self.get_2d_poses_from_ssd_server \
                = actionlib.SimpleActionServer(
                    "~get_2d_poses_from_ssd",
                    o2ac_msgs.msg.get2DPosesFromSSDAction, auto_start=False)
            self.get_2d_poses_from_ssd_server.register_goal_callback(
                self.get_2d_poses_from_ssd_goal_callback)
            self.get_2d_poses_from_ssd_server.start()

            # Action server for 3D localization by SSD
            self.get_3d_poses_from_ssd_server \
                = actionlib.SimpleActionServer(
                    "~get_3d_poses_from_ssd",
                    o2ac_msgs.msg.get3DPosesFromSSDAction, auto_start=False)
            self.get_3d_poses_from_ssd_server.register_goal_callback(
                self.get_3d_poses_from_ssd_goal_callback)
            self.get_3d_poses_from_ssd_server.start()

            # Action server for 3D localization by CAD matching
            self.localization_server \
                = actionlib.SimpleActionServer(
                    "~localize_object",
                    o2ac_msgs.msg.localizeObjectAction, auto_start=False)
            self.localization_server.register_goal_callback(
                self.localization_callback)
            self.localization_server.start()

            # Action server for belt detection
            self.belt_detection_server \
                = actionlib.SimpleActionServer(
                    "~belt_detection",
                    o2ac_msgs.msg.beltDetectionAction,  auto_start=False)
            self.belt_detection_server.register_goal_callback(
                self.belt_detection_callback)
            self.belt_detection_server.start()

            # Action server for angle detection
            self.angle_detection_server \
                = actionlib.SimpleActionServer(
                    "~detect_angle",
                    o2ac_msgs.msg.detectAngleAction, auto_start=False)
            self.angle_detection_server.register_goal_callback(
                self.angle_detection_callback)
            self.angle_detection_server.start()

            # Action server for shaft hole detection
            self.shaft_hole_detection_server \
                = actionlib.SimpleActionServer(
                    "~detect_shaft_hole",
                    o2ac_msgs.msg.shaftHoleDetectionAction, auto_start=False)
            self.shaft_hole_detection_server.register_goal_callback(
                self.shaft_hole_detection_callback)
            self.shaft_hole_detection_server.start()

            # Action server for checking pick success
            self.pick_success_server \
                = actionlib.SimpleActionServer(
                    "~check_pick_success",
                    o2ac_msgs.msg.checkPickSuccessAction, auto_start=False)
            self.pick_success_server.register_goal_callback(
                self.pick_success_callback)
            self.pick_success_server.start()

        # Action client to forward the calculation to the Python3 node
        self._py3_axclient \
            = actionlib.SimpleActionClient(
                '/o2ac_py3_vision_server/internal/detect_angle',
                o2ac_msgs.msg.detectAngleAction)

        # Setup subscribers of camera topics and synchronizer.
        self.camera_info_sub \
            = message_filters.Subscriber('/camera_info',
                                         sensor_msgs.msg.CameraInfo)
        self.image_sub \
            = message_filters.Subscriber('/image', sensor_msgs.msg.Image)
        self.depth_sub \
            = message_filters.Subscriber('/depth', sensor_msgs.msg.Image)
        sync = message_filters.ApproximateTimeSynchronizer(
                [self.camera_info_sub, self.image_sub, self.depth_sub],
                10, 0.01)
        sync.registerCallback(self.synced_images_callback)

        self.bridge = cv_bridge.CvBridge()

        # Setup publisher for object detection results
        self.results_pub = rospy.Publisher('~detection_results',
                                           o2ac_msgs.msg.Estimated2DPosesArray,
                                           queue_size=1)
        self.pulley_screw_detection_pub = rospy.Publisher('~activate_pulley_screw_detection',
                                           std_msgs.msg.Bool,
                                           queue_size=1)

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher('~result_image',
                                         sensor_msgs.msg.Image, queue_size=1)

        # For visualization
        self.cam_helper             = O2ACCameraHelper()
        self.listener               = tf.TransformListener()
        self.pose_marker_id_counter = 0
        self.pose_marker_array      = 0
        self._camera_info           = None
        self._depth                 = None
        self.marker_array_pub = rospy.Publisher(
                                        '~result_marker_arrays',
                                        visualization_msgs.msg.MarkerArray,
                                        queue_size=1)

        rospy.loginfo("O2AC_vision has started up!")

    ### ======= Callbacks of action servers

    def get_2d_poses_from_ssd_goal_callback(self):
        print("get_2d_poses_from_ssd called")
        self.get_2d_poses_from_ssd_server.accept_new_goal()

    def get_3d_poses_from_ssd_goal_callback(self):
        self.get_3d_poses_from_ssd_server.accept_new_goal()

    def localization_callback(self):
        self._localization_goal = self.localization_server.accept_new_goal()

    def belt_detection_callback(self):
        self.belt_detection_server.accept_new_goal()

    def angle_detection_callback(self):
        self._angle_detection_goal \
            = self.angle_detection_server.accept_new_goal()

    def shaft_hole_detection_callback(self):
        self.shaft_hole_detection_server.accept_new_goal()

    def pick_success_callback(self):
        self.pick_success_server.accept_new_goal()

### ======= Callbacks of subscribed topics

    def synced_images_callback(self, camera_info, image, depth):
        rospy.loginfo_throttle(5, "synced_images heartbeat")
        self._camera_info = camera_info
        self._depth       = depth

        im_in  = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        im_vis = im_in.copy()

        if self.pulley_screw_detection_stream_active:
            msg = std_msgs.msg.Bool()
            msg.data = self.detect_pulley_screws(im_in, im_vis)
            self.pulley_screw_detection_pub.publish(msg)

            # Publish images visualizing results
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

        # Pass image to SSD if continuous display is turned on
        if self.continuous_streaming:
            poses2d_array = o2ac_msgs.msg.Estimated2DPosesArray()
            poses2d_array.header = image.header
            poses2d_array.results, im_vis = self.get_2d_poses_from_ssd(im_in,
                                                                       im_vis)
            self.results_pub.publish(poses2d_array)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

        elif self.get_2d_poses_from_ssd_server.is_active():
            self.execute_get_2d_poses_from_ssd(im_in, im_vis)

        elif self.get_3d_poses_from_ssd_server.is_active():
            self.execute_get_3d_poses_from_ssd(im_in, im_vis)

        elif self.localization_server.is_active():
            self.execute_localization(im_in, im_vis)

        elif self.belt_detection_server.is_active():
            self.execute_belt_detection(im_in, im_vis)

        elif self.angle_detection_server.is_active():
            self.execute_angle_detection(im_in, im_vis)

        elif self.shaft_hole_detection_server.is_active():
            self.execute_shaft_hole_detection(im_in)

        elif self.pick_success_server.is_active():
            self.execute_pick_success(im_in)

    def set_pulley_screw_detection_callback(self, msg):
        self.pulley_screw_detection_stream_active = msg.data
        res = std_srvs.srv.SetBoolResponse()
        res.success = True
        return res

### ======= Process active goals of action servers

    def execute_get_2d_poses_from_ssd(self, im_in, im_vis):
        rospy.loginfo("Executing get_2d_poses_from_ssd action")

        action_result = o2ac_msgs.msg.get2DPosesFromSSDResult()
        action_result.results, im_vis = self.get_2d_poses_from_ssd(im_in,
                                                                   im_vis)
        self.get_2d_poses_from_ssd_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

    def execute_get_3d_poses_from_ssd(self, im_in, im_vis):
        rospy.loginfo("Executing get_3d_poses_from_ssd action")

        poses2d_array, im_vis = self.get_2d_poses_from_ssd(im_in, im_vis)
        action_result = o2ac_msgs.msg.get3DPosesFromSSDResult()
        action_result.poses       = []
        action_result.class_ids   = []
        action_result.upside_down = []
        for poses2d in poses2d_array:
            for pose2d in poses2d.poses:
                p3d = self.convert_pose_2d_to_3d(pose2d)
                if p3d:
                    action_result.class_ids.append(poses2d.class_id)
                    action_result.poses.append(p3d)
                    action_result.upside_down.append(poses2d.upside_down)
        self.get_3d_poses_from_ssd_server.set_succeeded(action_result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

    def execute_localization(self, im_in, im_vis):
        rospy.loginfo("Executing localization action")

        # Apply SSD first to get the object's bounding box
        poses2d_array, im_vis = self.get_2d_poses_from_ssd(im_in, im_vis)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))

        # If item_id is specified, keep only results with the id.
        goal = self.localization_server.current_goal.get_goal()
        if goal.item_id != '':
            poses2d_array = [poses2d for poses2d in poses2d_array
                             if self.item_id(poses2d.class_id) == goal.item_id]

        self._spawner.delete_all()

        # Execute localization for each item with 2D poses detected by SSD.
        action_result = o2ac_msgs.msg.localizeObjectResult()
        for poses2d in poses2d_array:
            poses3d = o2ac_msgs.msg.Estimated3DPoses()
            poses3d.class_id = poses2d.class_id
            poses3d.poses    = self.localize(self.item_id(poses2d.class_id),
                                             poses2d.bbox, poses2d.poses,
                                             im_in.shape)
            if poses3d.poses:
                action_result.detected_poses.append(poses3d)
                # Spawn URDF models
                for n, pose in enumerate(poses3d.poses.poses):
                    self._spawner.add(self.item_id(poses3d.class_id),
                                      geometry_msgs.msg.PoseStamped(
                                          poses3d.poses.header, pose),
                                      '{:02d}_'.format(n))

        if action_result.detected_poses:
            self.localization_server.set_succeeded(action_result)
        else:
            rospy.logerr("Could not not localize object %s. Might not be detected by SSD.",
                         goal.item_id)
            self.localization_server.set_aborted(action_result)
            self.write_to_log(im_in, im_vis, "localization")

    def execute_belt_detection(self, im_in, im_vis):
        rospy.loginfo("Executing belt grasp points detection")

        # Get bounding boxes, then belt grasp points
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        # Remain only results with class id of the belt.
        poses2d = []
        for ssd_result in ssd_results:
            if ssd_result['class'] == 6:
                p2d, im_vis = self.belt_grasp_detection_in_image(im_in, im_vis,
                                                                 ssd_result)
                poses2d += p2d

        action_result = o2ac_msgs.msg.beltDetectionResult()
        for pose2d in poses2d:
            pose3d = self.convert_pose_2d_to_3d(pose2d)
            if pose3d:
                action_result.grasp_points.append(pose3d)
        self.belt_detection_server.set_succeeded(action_result)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "belt_detection")

    def execute_angle_detection(self, im_in, im_vis):
        goal = self.angle_detection_server.current_goal.get_goal()
        rospy.loginfo("Executing angle detection for item: %s", goal.item_id)

        if goal.get_motor_from_top:
            action_result = o2ac_msgs.msg.detectAngleResult()
            action_result.succeeded, action_result.motor_rotation_flag, im_vis = self.motor_angle_detection_from_top(im_in, im_vis)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
            self.write_to_log(im_in, im_vis, "motor_orientation_detection")
        else:  # Pass action goal to Python3 node
            action_goal = goal
            action_goal.rgb_image = self.bridge.cv2_to_imgmsg(im_in)
            self._py3_axclient.send_goal(action_goal)

            if (not self._py3_axclient.wait_for_result(rospy.Duration(3.0))):
                rospy.logerr("Angle detection timed out.")
                self._py3_axclient.cancel_goal()  # Cancel goal if timeout expired

            action_result = self._py3_axclient.get_result()
        self.angle_detection_server.set_succeeded(action_result)

    def execute_shaft_hole_detection(self, im_in):
        rospy.loginfo("Received a request to detect shaft hole")

        has_hole, im_vis = self.shaft_hole_detection(im_in)
        rospy.loginfo("shaft tip has hole? %s" % has_hole)

        action_result = o2ac_msgs.msg.shaftHoleDetectionResult()
        action_result.has_hole = has_hole
        self.shaft_hole_detection_server.set_succeeded(action_result)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "shaft_hole_detection")

    def execute_pick_success(self, im_in):
        goal = self.pick_success_server.current_goal.get_goal()
        rospy.loginfo("Received a request to detect pick success for item: %s" % goal.item_id)
        # TODO (felixvd): Use Threading.Lock() to prevent race conditions here
        action_result = o2ac_msgs.msg.checkPickSuccessResult()
        action_result.item_is_picked, im_vis \
            = self.check_pick_success(im_in, goal.item_id)
        action_result.success = True
        self.pick_success_server.set_succeeded(action_result)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "pick_success")

### ======= Localization helpers

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
        poses2d_array            = []                      # Contains all results
        apply_2d_pose_estimation = [8,9,10,14]             # Small items --> Neural Net
        apply_3d_pose_estimation = [1,2,3,4,5,7,11,12,13]  # Large items --> CAD matching
        apply_grasp_detection    = [6]                     # Belt --> Fast Grasp Estimation

        for ssd_result in ssd_results:
            target  = ssd_result["class"]
            poses2d = o2ac_msgs.msg.Estimated2DPoses() # Stores the result for one item/class id
            poses2d.class_id    = target
            poses2d.confidence  = ssd_result["confidence"]
            poses2d.bbox        = ssd_result["bbox"]
            poses2d.upside_down = ssd_result["state"]

            if target in apply_2d_pose_estimation:
                rospy.loginfo("Seeing object id %d. Apply 2D pose estimation",
                              target)
                pose2d, im_vis = self.estimate_2d_pose_in_image(im_in, im_vis,
                                                                ssd_result)
                poses2d.poses = [pose2d]

            elif target in apply_3d_pose_estimation:
                rospy.loginfo("Seeing object id %d. Apply 3D pose estimation",
                              target)
                pose2d = geometry_msgs.msg.Pose2D(
                            int(poses2d.bbox[0] + round(poses2d.bbox[2]/2)),
                            int(poses2d.bbox[1] + round(poses2d.bbox[3]/2)),
                            0)
                poses2d.poses = [pose2d]

            elif target in apply_grasp_detection:
                rospy.loginfo("Seeing object id %d (belt). Apply grasp detection",
                              target)
                poses2d.poses, im_vis \
                    = self.belt_grasp_detection_in_image(im_in, im_vis,
                                                         ssd_result)
            else:
                continue

            poses2d_array.append(poses2d)

            # Publish result markers
            poses3d = []
            for pose2d in poses2d.poses:
                pose3d = self.convert_pose_2d_to_3d(pose2d)
                if pose3d:
                    poses3d.append(pose3d)
                    rospy.loginfo("Found pose for class %d: (%f, %f, %f)",
                                  target,
                                  pose3d.pose.position.x,
                                  pose3d.pose.position.y,
                                  pose3d.pose.position.z)
            if poses3d:
                if target in apply_grasp_detection:
                    self.publish_belt_grasp_pose_markers(poses3d)
                else:
                    self.add_markers_to_pose_array(poses3d)
                    self.publish_stored_pose_markers()
            else:
                rospy.logwarn("Could not find pose for class %d!", target)

        return poses2d_array, im_vis

    def detect_object_in_image(self, im_in, im_vis):
        ssd_results, im_vis = ssd_detection.object_detection(im_in, im_vis)
        return ssd_results, im_vis

    def estimate_2d_pose_in_image(self, im_in, im_vis, ssd_result):
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
        center, orientation = tm.compute(ssd_result)

        elapsed_time = time.time() - start
        rospy.logdebug("Processing time[msec]: %d", 1000*elapsed_time)

        im_vis = tm.get_result_image(ssd_result, orientation, center, im_vis)

        bbox = ssd_result["bbox"]

        return geometry_msgs.msg.Pose2D(center[1], center[0],
                                        radians(orientation)), \
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
        top_bottom = slice(max(bbox[1] - margin, 0),
                           min(bbox[1] + bbox[3] + margin, 480))
        left_right = slice(max(bbox[0] - margin, 0),
                           min(bbox[0] + bbox[2] + margin, 640))

        fge = FastGraspabilityEvaluation(im_in[top_bottom, left_right],
                                         im_hand, self._param_fge)
        results = fge.main_proc()

        elapsed_time = time.time() - start
        rospy.logdebug("Belt detection processing time[msec]: %d",
                       1000*elapsed_time)

        im_vis[top_bottom, left_right] = fge.visualization(im_vis[top_bottom,
                                                                  left_right])

        # Subtracting tau/4 (90 degrees) to the rotation result to match the gripper_tip_link orientation
        return [ geometry_msgs.msg.Pose2D(result[1] + (bbox[0] - margin),
                                          result[0] + (bbox[1] - margin),
                                          radians(result[2])-tau/4)
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

    def shaft_hole_detection(self, im_in):
        """
        Detects the hole in the supplied image (looking at the shaft in the V-jig).

        Return values:
        has_hole: True if hole is seen
        im_vis: Result visualization
        """

        # Convert to grayscale
        if im_in.shape[2] == 3:
            im_gray = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)
        else:
            im_gray = im_in

        has_hole, im_vis = self.shaft_hole_detector.main_proc(im_gray)  # if True hole is observed in im_in
        return has_hole, im_vis

    def check_pick_success(self, im_in, item_id):
        """ item_id is the object name, not the ID.
        """
        if item_id == "belt":
            class_id = 6
        pc = PickCheck(ssd_detection)
        pick_successful = pc.check( im_in, class_id )
        im_vis = pc.get_im_result()
        return pick_successful, im_vis

    def localize(self, item_id, bbox, poses2d, shape):
        # (u0, v0)/(u1, v1): upper-left/lower-right corner of bbox
        margin = 30
        u0     = bbox[0] - margin
        v0     = bbox[1] - margin
        u1     = bbox[0] + bbox[2] + margin
        v1     = bbox[1] + bbox[3] + margin

        # Setup ROI for depth filter.
        self._dfilter.roi = (u0, v0, u1, v1)

        if len(poses2d) == 1 and poses2d[0].theta == 0:
            # If the bounding box includes a image corner, give up
            # localization because we cannot estimate its center.
            if (u0 < 0 or u1 > shape[1]) and (v0 < 0 or v1 > shape[0]):
                rospy.logwarn('Cannot localize[%s] because the bounding box includes a corner of image.', item_id)
                return None

            # Estimate the object center as the bouinding box center.
            poses2d[0].x = 0.5*(u0 + u1)
            poses2d[0].y = 0.5*(v0 + v1)

            aspect_ratio = self._param_localization[item_id]['aspect_ratio']

            # Correct object center if the bounding box intersects with
            # the image border.
            if v0 < 0:
                if v1 - v0 < u1 - u0:
                    poses2d[0].y = v1 - 0.5*(u1 - u0)*aspect_ratio
                else:
                    poses2d[0].y = v1 - 0.5*(u1 - u0)/aspect_ratio
            elif v1 > shape[0]:
                if v1 - v0 < u1 - u0:
                    poses2d[0].y = v0 + 0.5*(u1 - u0)*aspect_ratio
                else:
                    poses2d[0].y = v0 + 0.5*(u1 - u0)/aspect_ratio

            if u0 < 0:
                if u1 - u0 < v1 - v0:
                    poses2d[0].x = u1 - 0.5*(v1 - v0)*aspect_ratio
                else:
                    poses2d[0].x = u1 - 0.5*(v1 - v0)/aspect_ratio
            elif u1 > shape[1]:
                if u1 - u0 < v1 - v0:
                    poses2d[0].x = u0 + 0.5*(v1 - v0)*aspect_ratio
                else:
                    poses2d[0].x = u0 + 0.5*(v1 - v0)/aspect_ratio

        if u0 < 0:
            u0 = 0
        if v0 < 0:
            v0 = 0
        for pose2d in poses2d:
            pose2d.x -= u0
            pose2d.y -= v0

        self._localizer.send_goal_with_target_frame(item_id, 'tray_center',
                                                    rospy.Time.now(), poses2d)
        return self._localizer.wait_for_result()

    def item_id(self, class_id):
        """ Returns the name (item_id) of the item's id number (class_id) of the SSD.
        """
        return self._models[class_id - 1]

    def detect_pulley_screws(self, im_in, im_vis):
        # Convert to grayscale
        if im_in.shape[2] == 3:
            im_gray = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)
        else:
            im_gray = im_in
        bbox = [300,180,200,120]   # (x,y,w,h)      bbox of search area
        s = PulleyScrewDetection(im_gray, self.pulley_screws_template_image, bbox)
        score, detected = s.main_proc()
        print("Screws detected: ", detected)
        print("Score: ", score)
        # TODO: Draw green rectangle around bbox if detected, red if not. Display score.
        return detected

    def motor_angle_detection_from_top(self, im_in, im_vis):
        """
        When looking at the motor from the top, detects the cables and returns their position.
        
        Return values:
        motor_seen: bool (False if no motor in view)
        motor_rotation_flag: int (0:right, 1:left, 2:top, 3:bottom  (documented in pose_estimation_func))
        im_vis: Result visualization
        """        
        ssd_results, im_vis = self.detect_object_in_image(im_in, im_vis)

        m = MotorOrientation()
        motor_rotation_flag = m.main_proc(im_in, ssd_results)  # if True hole is observed in im_in
        im_vis = m.draw_im_vis(im_vis)
        motor_seen = motor_rotation_flag is not None
        return motor_seen, motor_rotation_flag, im_vis

### ========

    def convert_pose_2d_to_3d(self, pose2d):
        """
        Convert a 2D pose to a 3D pose in the tray using the current depth image.
        Returns a PoseStamped in tray_center.
        """
        p3d = geometry_msgs.msg.PoseStamped()
        p3d.header.frame_id = self._camera_info.header.frame_id
        if self._depth is None:
            rospy.logerr("No depth image found")
            return
        depth = self.bridge.imgmsg_to_cv2(self._depth,
                                          desired_encoding="passthrough")
        xyz = self.cam_helper.project_2d_to_3d_from_images(self._camera_info,
                                                           pose2d.x, pose2d.y,
                                                           [depth])
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

        p3d.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, -pose2d.theta+tau/4))
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
