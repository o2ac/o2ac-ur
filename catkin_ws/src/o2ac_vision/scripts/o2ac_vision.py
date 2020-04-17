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
from o2ac_routines.base import O2ACBase


import o2ac_msgs.msg   # This is needed to advertise the actions
import sensor_msgs.msg # This is needed to receive images from the camera
import cv_bridge       # This offers conversion methods between OpenCV and ROS formats
                       # See here: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    
                       # Note that a similar package exists for PCL:
                       # http://wiki.ros.org/pcl_ros

class O2ACVision(object):
  # This class advertises the vision actions that we will call during the tasks.

  def __init__(self):
    rospy.init_node('o2ac_vision', anonymous=False)
    self.listener = tf.TransformListener()
    
    # This creates the action server.nSee this tutorial, and also check out base.py: 
    # http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29
    self.detect_object_action_server = actionlib.SimpleActionServer("detectObject", o2ac_msgs.msg.detectObjectAction, 
        execute_cb = self.detect_object_callback, auto_start = True)
    self.detect_orientation_from_front_view_server = actionlib.SimpleActionServer("detectOrientationFromFrontView", o2ac_msgs.msg.detectOrientationFromFrontViewAction, 
        execute_cb = self.detect_orientation_from_front_view_callback, auto_start = True)
    self.get_grasp_candidates_server = actionlib.SimpleActionServer("getGraspCandidates", o2ac_msgs.msg.getGraspCandidatesAction, 
        execute_cb = self.get_grasp_candidates_callback, auto_start = True)
    self.detect_cable_tip_server = actionlib.SimpleActionServer("detectCableTip", o2ac_msgs.msg.detectCableTipAction, 
        execute_cb = self.detect_cable_tip_callback, auto_start = True)
    self.detect_object_tip_server = actionlib.SimpleActionServer("detectObjectTip", o2ac_msgs.msg.detectObjectTipAction, 
        execute_cb = self.detect_object_tip_callback, auto_start = True)
    self.get_tip_distance_to_hole_server = actionlib.SimpleActionServer("getTipDistanceToHole", o2ac_msgs.msg.getTipDistanceToHoleAction, 
        execute_cb = self.get_tip_distance_to_hole_callback, auto_start = True)
    self.adjust_pulley_angle_server = actionlib.SimpleActionServer("adjustPulleyAngle", o2ac_msgs.msg.adjustPulleyAngleAction, 
        execute_cb = self.adjust_pulley_angle_callback, auto_start = True)

    self.use_robots = False
    if self.use_robots:
      self.robots = O2ACBase()

    rospy.loginfo("O2AC_vision has started up!")

  ### ======= Helper functions

  def move_camera_to_pose(self, target_pose, camera_id="a_bot_outside", camera_frame=""):
    # TODO (felixvd): Move the camera to the target pose
    if not camera_frame:
      # construct the link frame
    
    return self.robots.go_to_pose_goal(robot_name, goal_pose)

  def turn_pulley(self, pulley_name, angle):
    # TODO (felixvd): 
    return self._turn_pulley_client.get_result()

  ### ======= Action server definitions

  def detect_object_callback(self, goal):
    rospy.loginfo("Received a request to detect object named " + goal.object_id)
    action_result = o2ac_msgs.msg.detectObjectActionResult()

    # First, obtain the image from the camera and convert it
    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect the object
    detected_pose = self.detect_object_in_image(cv_image)
    if not detected_pose:
      self.detect_object_action_server.set_aborted(action_result)
    else:
      action_result.detected_pose.pose = detected_pose
    action_result.detected_pose.header.frame_id = goal.camera_id + "_camera_color_frame"
    
    # Return
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
    self.detect_object_action_server.set_succeeded(action_result)
  
  def detect_orientation_from_front_view_callback(self, goal):
    rospy.loginfo("Received a request to detect orientation of " + goal.object_id)
    action_result = o2ac_msgs.msg.detectOrientationFromFrontViewActionResult()

    # First, obtain the image from the camera and convert it
    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect orientation
    # TODO: Decide which return parameters would be useful here. The angle by which the
    # object should be turned to arrive at the target position would be best.
    if goal.object_id == "bearing":
      goal.angle_offset = self.detect_bearing_angle(cv_image)
    elif goal.object_id == "large_pulley":
      goal.angle_offset = self.detect_large_pulley_angle(cv_image)
    
    # Return
    self.detect_orientation_from_front_view_server.set_succeeded(action_result)

  def detect_orientation_from_front_view_callback(self, goal):
    rospy.loginfo("Received a request to detect grasp candidates for belt")
    action_result = o2ac_msgs.msg.getGraspCandidatesActionResult()

    # First, obtain the image from the camera and convert it
    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect grasp candidates
    if goal.object_id == "bearing":
      goal.grasp_candidates = self.detect_bearing_angle(cv_image)
    elif goal.object_id == "large_pulley":
      goal.grasp_candidates = self.detect_large_pulley_angle(cv_image)
    
    # Return
    self.get_grasp_candidates_server.set_succeeded(action_result)

  def detect_cable_tip_callback(self, goal):
    rospy.loginfo("Received a request to detect the tip of the cable")
    action_result = o2ac_msgs.msg.detectCableTipActionResult()

    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect the cable tip
    detected_point = self.detect_cable_tip_in_image(cv_image)
    if not detected_point:
      self.detect_cable_tip_server.set_aborted(action_result)
    else:
      action_result.detected_point = detected_point
    action_result.detected_point.header.frame_id = goal.camera_id + "_camera_color_frame"
    
    # Return
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
    self.detect_cable_tip_server.set_succeeded(action_result)
  
  def detect_object_tip_callback(self, goal):
    rospy.loginfo("Received a request to detect the tip of the object")
    action_result = o2ac_msgs.msg.detectObjectTipActionResult()

    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect the object tip
    detected_point = self.detect_object_tip_in_image(cv_image)
    if not detected_point:
      self.detect_object_tip_server.set_aborted(action_result)
    else:
      action_result.detected_pose = detected_point
    
    # Return
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
    self.detect_object_cable_tip_server.set_succeeded(action_result)

  def get_tip_distance_to_hole_callback(self, goal):
    rospy.loginfo("Received a request to detect the tooltip distance to target hole")
    action_result = o2ac_msgs.msg.getTipDistanceToHoleActionResult()

    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect the image
    target_hole_pose = self.detect_object_in_image(cv_image)
    if not target_hole_pose:
      self.get_tip_distance_to_hole_server.set_aborted(action_result)
    else:
      action_result.detected_pose = target_hole_pose
    
    # Return
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
    self.get_tip_distance_to_hole_server.set_succeeded(action_result)

  def adjust_pulley_angle_callback(self, goal):
    rospy.loginfo("Received a request to adjust pulley " + goal.pulley_name)
    action_result = o2ac_msgs.msg.adjustPulleyAngleActionResult()

    image_msg = rospy.wait_for_message("/" + goal.camera_id + "/color", sensor_msgs.msg.Image, 1.0)
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

    # Detect the cable tip
    success = self.adjust_pulley_angle(goal.pulley_name, goal.camera_id)
    if not success:
      self.adjust_pulley_angle_server.set_aborted(action_result)
    else:
      action_result.detected_point = detected_point
    action_result.detected_point.header.frame_id = goal.camera_id + "_camera_color_frame"
    
    # Return
    o2ac_routines.helpers.publish_marker(action_result.detected_pose, "pose")
    self.adjust_pulley_angle_server.set_succeeded(action_result)

  ### ======= Implementations
    
  def detect_object_in_image(self, cv_image):
    # TODO: Detect the object here
    success = True
    if not success:
      object_pose = []
    return object_pose
  
  def detect_cable_tip_in_image(self, cv_image):
    # TODO: Detect the cable tip here
    success = True
    if not success:
      object_pose = []
    return object_pose
  
  def detect_object_tip_in_image(self, cv_image):
    # TODO: Detect the object tip here
    success = True
    if not success:
      object_pose = []
    return object_pose
  
  def detect_belt_grasp_candidates(self, cv_image):
    # TODO: Detect the belt grasp candidates here. 
    # Each grasp candidate is a pose. The x-axis should point into the tray, the y-axis along the belt.
    success = True
    if not success:
      grasp_candidates = []
    return grasp_candidates

  def detect_bearing_angle(self, cv_image):
    # TODO: Detect the difference between current and target angle of the bearing
    success = True
    if not success:
      angle = []
    return angle
  
  def detect_motor_pulley_screw_hole(self, cv_image):
    # TODO: Detect the difference between current and target angle of the motor pulley
    #       The set screw hole should be pointing upwards (facing the camera).
    success = True
    if not success:
      angle = []
    return angle
  
  def detect_large_pulley_angle(self, cv_image):
    # TODO: Detect the difference between current and target angle of the large pulley
    #       The screws should be pointing upwards (facing the camera).
    success = True
    if not success:
      angle = []
    return angle

  def get_tip_distance_to_hole(self, cv_image):
    # TODO: Detect the tooltip and estimate the distance between it and the presumed target.
    success = True
    if not success:
      target_pose = []
    return target_pose
  
  def adjust_pulley_angle(self, pulley_name, camera_id):
    # TODO: Find the set screw hole or screws and turn them so they face the camera
    success = True
    return success

if __name__ == '__main__':
  try:
    c = O2ACVision()
    while not rospy.is_shutdown():
      rospy.sleep(.1)
  except rospy.ROSInterruptException:
    pass