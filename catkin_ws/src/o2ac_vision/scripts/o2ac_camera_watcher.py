#!/usr/bin/env python

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

# This file only checks that the cameras are alive, and restarts them if necessary
# This is required because the depth stream of the realsense cameras sometimes fails for no reason.
# Restarting the driver may fix it.

import rospy
import os
import thread
from math import pi, radians
import sensor_msgs.msg

import std_srvs.srv

import cv2
import cv_bridge

import sys
import signal


def signal_handler(sig, frame):
    print("You pressed Ctrl+C!")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

CAMERA_IDS = {
    "a_bot_inside_camera": 0,
    "a_bot_outside_camera": 1,
    "b_bot_inside_camera": 2,
    "b_bot_outside_camera": 3,
    "scene_camera": 4,
}


class O2ACWatcher(object):
    def __init__(self):
        # Load camera names to determine which cameras will be checked
        default_camera_names = {"camera_multiplexer": True}
        self.camera_names = rospy.get_param("~camera_names", default_camera_names)
        self.bridge = cv_bridge.CvBridge()

        self.reset_services = {}
        if "camera_multiplexer" in self.camera_names:
            self.b_bot_outside_camera = rospy.Subscriber(
                "/camera_multiplexer/depth",
                sensor_msgs.msg.Image,
                self.camera_multiplexer_callback,
            )
            # Keeps track of which camera is active:
            self.camera_info_sub = rospy.Subscriber(
                "/camera_multiplexer/camera_info",
                sensor_msgs.msg.CameraInfo,
                self.camera_info_callback,
            )
            self.current_camera_name = ""
            self.last_image = sensor_msgs.msg.Image()
            self.multiplexer_camera_names = rospy.get_param(
                "/camera_multiplexer/camera_names", default_camera_names
            )
            for cam in self.multiplexer_camera_names:
                rospy.wait_for_service("/%s/realsense2_camera/reset" % cam)
                self.reset_services.update(
                    {
                        cam: rospy.ServiceProxy(
                            "/%s/realsense2_camera/reset" % cam, std_srvs.srv.Empty
                        )
                    }
                )

        # Setup subscribers for input depth images
        if "a_bot_inside_camera" in self.camera_names:
            self.a_bot_inside_camera = rospy.Subscriber(
                "/a_bot_inside_camera/aligned_depth_to_color/image_raw",
                sensor_msgs.msg.Image,
                self.a_bot_inside_camera_callback,
            )
            rospy.wait_for_service("/a_bot_inside_camera/realsense2_camera/reset")
            self.a_bot_inside_camera_reset = rospy.ServiceProxy(
                "/a_bot_inside_camera/realsense2_camera/reset", std_srvs.srv.Empty
            )
            self.reset_services.update(
                {"a_bot_inside_camera": self.a_bot_inside_camera_reset}
            )
        if "a_bot_outside_camera" in self.camera_names:
            self.a_bot_outside_camera = rospy.Subscriber(
                "/a_bot_outside_camera/aligned_depth_to_color/image_raw",
                sensor_msgs.msg.Image,
                self.a_bot_outside_camera_callback,
            )
            rospy.wait_for_service("/a_bot_outside_camera/realsense2_camera/reset")
            self.a_bot_outside_camera_reset = rospy.ServiceProxy(
                "/a_bot_outside_camera/realsense2_camera/reset", std_srvs.srv.Empty
            )
            self.reset_services.update(
                {"a_bot_outside_camera": self.a_bot_outside_camera_reset}
            )
        if "b_bot_inside_camera" in self.camera_names:
            self.b_bot_inside_camera = rospy.Subscriber(
                "/b_bot_inside_camera/aligned_depth_to_color/image_raw",
                sensor_msgs.msg.Image,
                self.b_bot_inside_camera_callback,
            )
            rospy.wait_for_service("/b_bot_inside_camera/realsense2_camera/reset")
            self.b_bot_inside_camera_reset = rospy.ServiceProxy(
                "/b_bot_inside_camera/realsense2_camera/reset", std_srvs.srv.Empty
            )
            self.reset_services.update(
                {"b_bot_inside_camera": self.b_bot_inside_camera_reset}
            )
        if "b_bot_outside_camera" in self.camera_names:
            self.b_bot_outside_camera = rospy.Subscriber(
                "/b_bot_outside_camera/aligned_depth_to_color/image_raw",
                sensor_msgs.msg.Image,
                self.b_bot_outside_camera_callback,
            )
            rospy.wait_for_service("/b_bot_outside_camera/realsense2_camera/reset")
            self.b_bot_outside_camera_reset = rospy.ServiceProxy(
                "/b_bot_outside_camera/realsense2_camera/reset", std_srvs.srv.Empty
            )
            self.reset_services.update(
                {"b_bot_outside_camera": self.b_bot_outside_camera_reset}
            )

        # Initialize times
        self.last_msg_times = dict()
        for cam in self.camera_names:
            self.last_msg_times[cam] = rospy.Time.now()
        self.last_full_depth_image_time = rospy.Time.now()

        rospy.loginfo("o2ac_camera_watcher has started up. Waiting a few more seconds.")
        rospy.sleep(5)
        rospy.loginfo("o2ac_camera_watcher has started watching for dead cameras!")
        self.check_status_loop()

    def a_bot_inside_camera_callback(self, image):
        self.last_msg_times["a_bot_inside_camera"] = rospy.Time.now()

    def a_bot_outside_camera_callback(self, image):
        self.last_msg_times["a_bot_outside_camera"] = rospy.Time.now()

    def b_bot_inside_camera_callback(self, image):
        self.last_msg_times["b_bot_inside_camera"] = rospy.Time.now()

    def b_bot_outside_camera_callback(self, image):
        self.last_msg_times["b_bot_outside_camera"] = rospy.Time.now()

    def camera_multiplexer_callback(self, image):
        self.last_msg_times["camera_multiplexer"] = rospy.Time.now()
        self.last_image = image

    def camera_info_callback(self, cam_info):
        # Extracts camera name from the header by stripping prefix "calibrated_" and suffix "_color_optical_frame"
        # Ex.: "calibrated_b_bot_inside_camera_color_optical_frame" -->
        # "b_bot_inside_camera"
        self.current_camera_name = cam_info.header.frame_id[11:-20]  # str

    def check_status_loop(self):
        r = rospy.Rate(1)
        softreset = True
        while not rospy.is_shutdown():
            camera_reset_required = False

            assert not rospy.is_shutdown(), "Did ros die?"
            now = rospy.Time.now()

            for cam in self.camera_names:
                # Check if depth image empty
                if self.last_image.data:
                    last_depth_image_cv = self.bridge.imgmsg_to_cv2(
                        self.last_image, desired_encoding="passthrough"
                    )
                    s = cv2.sumElems(last_depth_image_cv)
                    sum_depth_vals = s[0]
                    if sum_depth_vals:
                        self.last_full_depth_image_time = (
                            now  # We just care about the "active" camera
                        )

                rospy.sleep(0.01)
                time_since_last_nonempty_depth_img = (
                    now - self.last_full_depth_image_time
                )
                time_since_last_msg = now - self.last_msg_times.get(cam, rospy.Time())
                camera_reset_required |= (
                    time_since_last_msg.secs > 3
                    or time_since_last_nonempty_depth_img.secs > 3
                )
                if time_since_last_msg.secs > 3:
                    rospy.logerr("(NO DEPTH IMAGE RECEIVED)")
                elif time_since_last_nonempty_depth_img.secs > 3:
                    rospy.logerr("(DEPTH IMAGE WAS EMPTY, ASSUMING ERROR)")

                if camera_reset_required:
                    if cam == "camera_multiplexer":
                        cam_name = self.current_camera_name
                    else:
                        cam_name = cam

                    rospy.logerr("RESETTING CAMERA: " + cam_name)
                    if rospy.is_shutdown():
                        break

                    # Retrieve ID used to restart camera node

                    cam_num = CAMERA_IDS.get(cam_name, -1)
                    if cam_num < 0:
                        rospy.logerr("Could not find " + cam_name)
                        continue

                    if softreset:
                        reset = self.reset_services.get(cam_name, None)
                        if reset:
                            rospy.logwarn("Soft resetting camera: %s" % cam_name)
                            reset()
                            # Ensure that the camera has time to start up, to
                            # avoid an infinite loop
                            rospy.sleep(10)
                            softreset = False  # Next time a reset is required, try the hard reset
                        else:
                            rospy.logerr(
                                "Softreset service not found. camera: %s" % cam_name
                            )
                            softreset = False  # Next time a reset is required, try the hard reset
                    else:
                        # Restart camera by killing the nodes and spawning a
                        # roslaunch process
                        os.system(
                            "rosnode kill /"
                            + cam_name
                            + "/realsense2_camera /"
                            + cam_name
                            + "/realsense2_camera_manager"
                        )
                        rospy.sleep(1)
                        command = (
                            "roslaunch o2ac_scene_description osx_bringup_cam"
                            + str(cam_num)
                            + ".launch initial_reset:=true"
                        )
                        rospy.loginfo("Executing command: " + command)
                        assert not rospy.is_shutdown(), "Did ros die?"
                        thread.start_new_thread(os.system, (command,))
                        assert not rospy.is_shutdown(), "Did ros die?"
                        rospy.loginfo("Sleeping for 15 seconds to let camera restart")
                        # Ensure that the camera has time to start up, to avoid
                        # an infinite loop
                        rospy.sleep(15)
                        rospy.loginfo(
                            "Camera watcher active again (waited 15 seconds for camera "
                            + str(self.current_camera_name)
                            + " to restart)"
                        )
                        softreset = (
                            True  # Next time a reset is required, try the soft reset
                        )
            r.sleep()

        rospy.logerr("Stopping o2ac_camera_watcher (rospy shutdown)")


if __name__ == "__main__":
    rospy.init_node("o2ac_camera_watcher", anonymous=False)
    c = O2ACWatcher()
