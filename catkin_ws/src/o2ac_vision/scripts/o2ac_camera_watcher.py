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
import datetime
import os
import thread
import rospkg
from math import pi, radians
from sensor_msgs   import msg as smsg

class O2ACWatcher(object):
    
    def __init__(self):
        # Load camera names to determine which cameras will be checked
        default_camera_names = {"b_bot_inside_camera": True,
                             "b_bot_outside_camera": True}
        self.camera_names = rospy.get_param('~camera_names', default_camera_names)

        # Setup subscribers for input depth images
        if "a_bot_inside_camera" in self.camera_names:
            self.a_bot_inside_camera = rospy.Subscriber("/a_bot_inside_camera/aligned_depth_to_color/image_raw", smsg.Image, self.a_bot_inside_camera_callback)
        if "a_bot_outside_camera" in self.camera_names:
            self.a_bot_outside_camera = rospy.Subscriber("/a_bot_outside_camera/aligned_depth_to_color/image_raw", smsg.Image, self.a_bot_outside_camera_callback)
        if "b_bot_inside_camera" in self.camera_names:
            self.b_bot_inside_camera = rospy.Subscriber("/b_bot_inside_camera/aligned_depth_to_color/image_raw", smsg.Image, self.b_bot_inside_camera_callback)
        if "b_bot_outside_camera" in self.camera_names:
            self.b_bot_outside_camera = rospy.Subscriber("/b_bot_outside_camera/aligned_depth_to_color/image_raw", smsg.Image, self.b_bot_outside_camera_callback)

        # Initialize times
        self.last_msg_times = dict()
        for cam in self.camera_names:
            self.last_msg_times[cam] = datetime.datetime.now()
        
        rospy.loginfo("o2ac_camera_watcher has started up. Waiting a few more seconds.")
        rospy.sleep(5)
        rospy.loginfo("o2ac_camera_watcher has started watching for dead cameras!")
        self.check_status_loop()

    def a_bot_inside_camera_callback(self, image):
        self.last_msg_times["a_bot_inside_camera"] = datetime.datetime.now()

    def a_bot_outside_camera_callback(self, image):
        self.last_msg_times["a_bot_outside_camera"] = datetime.datetime.now()

    def b_bot_inside_camera_callback(self, image):
        self.last_msg_times["b_bot_inside_camera"] = datetime.datetime.now()

    def b_bot_outside_camera_callback(self, image):
        self.last_msg_times["b_bot_outside_camera"] = datetime.datetime.now()

    def check_status_loop(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            now = datetime.datetime.now()
            if rospy.is_shutdown():
                    break
            for cam in self.camera_names:
                if rospy.is_shutdown():
                    break
                time_since_last_msg = now - self.last_msg_times[cam]
                if time_since_last_msg > datetime.timedelta(milliseconds=1000):
                    rospy.logerr("NO DEPTH IMAGE RECEIVED FROM CAMERA: " + cam)
                    rospy.logerr("RESETTING CAMERA: " + cam)
                    if rospy.is_shutdown():
                        break
                    # print("time since last message: ", str(time_since_last_msg), "cam: ", cam)
                    if cam == "a_bot_inside_camera":
                        cam_num = 0
                    if cam == "a_bot_outside_camera":
                        cam_num = 1
                    if cam == "b_bot_inside_camera":
                        cam_num = 2
                    if cam == "b_bot_outside_camera":
                        cam_num = 3
                        
                    # Restart camera by killing the nodes and spawning a roslaunch process
                    os.system("rosnode kill /" + cam + "/realsense2_camera /" + cam + "/realsense2_camera_manager")
                    rospy.sleep(1)
                    command = "roslaunch o2ac_scene_description osx_bringup_cam" + str(cam_num) + ".launch initial_reset:=true"
                    rospy.loginfo("Executing command: " + command)
                    thread.start_new_thread(os.system, (command,))
                    if rospy.is_shutdown():
                        break
                    rospy.loginfo("Sleeping for 15 seconds to let camera restart")
                    rospy.sleep(15)  # Ensure that the camera has time to start up, to avoid an infinite loop
                    rospy.loginfo("Camera watcher active again (waited 15 seconds for camera " + str(cam) + " to restart)")
                    if rospy.is_shutdown():
                        break
            r.sleep()

        rospy.logerr("Stopping o2ac_camera_watcher (rospy shutdown)")

if __name__ == '__main__':
    rospy.init_node('o2ac_camera_watcher', anonymous=False)
    c = O2ACWatcher()
    rospy.spin()
