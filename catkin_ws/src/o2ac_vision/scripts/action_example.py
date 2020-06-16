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

class ActionExampleClass(object):
  # Use a class like this to advertise vision-based functions.

  def __init__(self):
    rospy.init_node('action_example', anonymous=False)
    self.listener = tf.TransformListener()
    
    # This creates the action server. See this tutorial, and also check out base.py: 
    # http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
    self.detect_object_action_server = actionlib.SimpleActionServer("detectObject", o2ac_msgs.msg.detectObjectAction, 
        execute_cb = self.detect_object_callback, auto_start = True)
    
    rospy.loginfo("ActionExampleClass has started up!")

  def detect_object_callback(self, goal):
    # Goal is of type o2ac_msgs.msg.detectObjectActionRequest
    # It is defined in o2ac_msgs/msg/detectObject.action
    rospy.loginfo("Received a request to detect object named " + goal.object_id)
    
    # Actions need to do two things:
    # 1. Set the status of the action (success, failure)
    # 2. Return a result. The result is of type o2ac_msgs.msg.detectObjectActionResult
    action_result = o2ac_msgs.msg.detectObjectActionResult() # We fill the fields of this object

    # First, obtain the image from the camera and convert it
    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0) # This topic name is probably incorrect
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    detected_pose = self.detect_object_in_image(cv_image)
    if not detected_pose:   # If "object_pose" is empty, this evaluates to "True"
      self.detect_object_action_server.set_aborted(action_result) # This sets the action status to "Aborted" (= Failed)
    else:
      action_result.detected_pose.pose = detected_pose # Note the difference between Pose and PoseStamped
    action_result.detected_pose.header.frame_id = goal.camera_id + "_camera_color_frame"  # This is probably incorrect, but you get the idea
    
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")

    self.detect_object_action_server.set_succeeded(action_result)
    
  def detect_object_in_image(self, cv_image):
    # Use OpenCV to detect the object here
    success = True
    if not success:
      object_pose = []
    return object_pose

if __name__ == '__main__':
  try:
    c = ActionExampleClass()
    while not rospy.is_shutdown():
      rospy.sleep(.1)
  except rospy.ROSInterruptException:
    pass
