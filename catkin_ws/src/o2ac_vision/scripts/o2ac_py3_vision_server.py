#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X Corp.
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

from o2ac_vision.bearing_pose_estimation import (
    BearingPoseEstimator,
    InPlaneRotationEstimator,
    get_templates,
)
import std_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import o2ac_msgs.msg
import cv_bridge  # This offers conversion methods between OpenCV
import cv2
import rospy
import os
import copy
from datetime import datetime
import rospkg
import actionlib
import numpy as np
from math import pi, radians, degrees

tau = 2.0 * pi  # Part of math from Python 3.6
# and ROS formats
# See here:
#   http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# Note that a similar package exists for PCL:
#   http://wiki.ros.org/pcl_ros


class O2ACBearingPoseEstimationServer(object):
    """
    This should be part of the O2ACVisionServer class, but since it uses Python3 we
    separate it from the Python2 code. In Noetic, this can be moved into o2ac_vision_server.py.

    Advertises the actions that use Python3:
    - Bearing angle detection
    """

    def __init__(self):
        rospy.init_node("o2ac_py3_vision_server", anonymous=False)
        self.bridge = cv_bridge.CvBridge()

        # Setup publisher for output result image
        self.image_pub = rospy.Publisher(
            "/o2ac_vision_server/result_image", sensor_msgs.msg.Image, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/o2ac_vision_server/result_markers",
            visualization_msgs.msg.Marker,
            queue_size=1,
        )
        self.marker_array_pub = rospy.Publisher(
            "/o2ac_vision_server/result_marker_arrays",
            visualization_msgs.msg.MarkerArray,
            queue_size=1,
        )

        self.angle_detection_action_server = actionlib.SimpleActionServer(
            "~internal/detect_angle",
            o2ac_msgs.msg.detectAngleAction,
            execute_cb=self.angle_detection_callback,
            auto_start=False,
        )
        self.angle_detection_action_server.start()

        template_filename = os.path.join(
            rospkg.RosPack().get_path("o2ac_vision"),
            "config",
            "bearing_template_image.png",
        )

        self.bearing_template = cv2.imread(template_filename, cv2.IMREAD_GRAYSCALE)
        rospy.loginfo("o2ac_py3_vision_server started up")

    def angle_detection_callback(self, goal):
        rospy.loginfo("Received a request to detect bearing angle (py3)")
        # im_in  = self.bridge.imgmsg_to_cv2(goal.rgb_image,
        # desired_encoding="bgr8")  # Requires Python2 --> use numpy directly
        im_in = np.frombuffer(goal.rgb_image.data, dtype=np.uint8).reshape(
            goal.rgb_image.height, goal.rgb_image.width, -1
        )
        im_in2 = cv2.cvtColor(im_in, cv2.COLOR_RGB2GRAY)
        im_vis = im_in.copy()

        if goal.item_id == "bearing":
            estimator = BearingPoseEstimator(
                self.bearing_template, im_in2, [200, 100, 320, 280]
            )
            rotation, translation = estimator.main_proc(threshold=4.0, ds=3.0)

        if goal.item_id == "motor":
            # src_pts = np.array([[287,94], [470,91], [285,270], [490,265]], dtype=np.float32)
            # destination points (rectified corner points)
            # dst_pts = np.array([[287,94], [470,91], [287,270], [470,265]], dtype=np.float32)
            # mat = cv2.getPerspectiveTransform(src_pts, dst_pts)

            bbox = [270, 180, 240, 240]  # Set bounding box(x,y,w,h)
            template_path = os.path.join(
                rospkg.RosPack().get_path("wrs_dataset"), "data/motor_front/"
            )
            im_templates = get_templates(template_path, "name")
            # im_template = cv2.imread(template_path, 0)  # Read template image
            # (use option "0")

            estimator = InPlaneRotationEstimator(
                im_templates, im_in2, bbox
            )  # Define pose estimation class
            # estimator = PoseEstimator( im_template, im_in2, bbox, mat )  # Define pose estimation class
            # rotation, translation = estimator.main_proc( threshold=5.0,
            # ds=10.0 )  # Do registration
            rotation, translation, mse = estimator.main_proc(ds=10.0)  # Do registration

        action_result = o2ac_msgs.msg.detectAngleResult()
        if rotation:
            action_result.succeeded = True
            action_result.rotation_angle = rotation
            self.angle_detection_action_server.set_succeeded(action_result)
        else:
            action_result.succeeded = False
            self.angle_detection_action_server.set_aborted(action_result)
        im_vis = estimator.get_result_image()
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_vis))
        self.write_to_log(im_in, im_vis, "angle_detection")

    def write_to_log(self, img_in, img_out, action_name):
        now = datetime.now()
        timeprefix = now.strftime("%Y-%m-%d_%H:%M:%S")
        rospack = rospkg.RosPack()
        folder = os.path.join(rospack.get_path("o2ac_vision"), "log")
        cv2.imwrite(
            os.path.join(folder, timeprefix + "_" + action_name + "_in.png"), img_in
        )
        cv2.imwrite(
            os.path.join(folder, timeprefix + "_" + action_name + "_out.jpg"), img_out
        )


if __name__ == "__main__":
    rospy.init_node("o2ac_py3_vision_server", anonymous=False)
    c = O2ACBearingPoseEstimationServer()
    rospy.spin()
