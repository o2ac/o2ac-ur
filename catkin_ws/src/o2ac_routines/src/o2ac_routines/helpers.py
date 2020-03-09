#!/usr/bin/env python

import sys
import rospy 
import actionlib
from math import *

import visualization_msgs.msg  # For marker visualization

import o2ac_msgs
import o2ac_msgs.msg
import o2ac_msgs.srv

from math import pi
from moveit_commander.conversions import pose_to_list

import ur_msgs.msg
import ur_dashboard_msgs.msg

helper_fct_marker_id_count = 0

def is_program_running(topic_namespace, service_client):
  req = ur_dashboard_msgs.srv.IsProgramRunningRequest()
  res = []
  try:
    res = service_client.call(req)
  except:
    pass

  if res:
    return res.program_running
  else:
    rospy.logerr("No response received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
    return True

def wait_for_UR_program(topic_namespace = "", timeout_duration = rospy.Duration.from_sec(20.0)):
  """Waits for the UR to finish executing a program."""
  rospy.logdebug("Waiting for UR program to finish.")
  # Only run this after sending custom URScripts and not the regular motion commands, or this call will not terminate before the timeout.
  rospy.sleep(1.0)
  t_start = rospy.Time.now()
  time_passed = rospy.Time.now() - t_start
  
  service_client = rospy.ServiceProxy(topic_namespace + "/ur_hardware_interface/dashboard/program_running", ur_dashboard_msgs.srv.IsProgramRunning)
  while is_program_running(topic_namespace, service_client) and not rospy.is_shutdown():
    rospy.sleep(.1)
    time_passed = rospy.Time.now() - t_start
    if time_passed > timeout_duration:
      rospy.loginfo("Timeout reached.")
      return False
  rospy.logdebug("UR Program has terminated.")
  return True

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

# This is a helper function for publishMarker. Publishes a TF-like frame.
def publishPoseMarker(marker_publisher, marker_pose_stamped):
  marker = visualization_msgs.msg.Marker()
  marker.header = marker_pose.header
  marker.header.stamp = rospy.Time.now()
  marker.pose = marker_pose.pose

  marker.ns = "markers"
  helper_fct_marker_id_count = 0
  marker.id = helper_fct_marker_id_count = 0
  marker.lifetime = rospy.Duration()
  marker.action = visualization_msgs.msg.Marker.ADD

  # This draws a TF-like frame.
  marker.type = visualization_msgs.msg.Marker.ARROW
  marker.scale.x = .1
  marker.scale.y = .01
  marker.scale.z = .01
  marker.color.a = .8

  arrow_x = visualization_msgs.msg.Marker()
  arrow_y = visualization_msgs.msg.Marker()
  arrow_z = visualization_msgs.msg.Marker()
  arrow_x = marker; arrow_y = marker; arrow_z = marker;
  helper_fct_marker_id_count += 1
  arrow_x.id = helper_fct_marker_id_count = 0
  helper_fct_marker_id_count += 1
  arrow_y.id = helper_fct_marker_id_count = 0
  helper_fct_marker_id_count += 1
  arrow_z.id = helper_fct_marker_id_count = 0
  arrow_x.color.r = 1.0
  arrow_y.color.g = 1.0
  arrow_z.color.b = 1.0

  rotatePoseByRPY(0, 0, M_PI/2, arrow_y.pose)
  rotatePoseByRPY(0, -M_PI/2, 0, arrow_z.pose)

  marker_publisher.publish(arrow_x)
  marker_publisher.publish(arrow_y)
  marker_publisher.publish(arrow_z)
  return True

def publishMarker(marker_publisher, marker_pose_stamped, marker_type):
    marker = visualization_msgs.msg.Marker()
    marker.header = marker_pose.header
    # marker.header.stamp = rospy.Time.now()
    marker.pose = marker_pose.pose

    marker.ns = "markers"
    marker.id = helper_fct_marker_id_count = 0
    marker.lifetime = rospy.Duration(60.0)
    marker.action = visualization_msgs.msg.Marker.ADD

    if (marker_type == "pose"):
      publishPoseMarker(marker_pose)

      # Add a flat sphere
      marker.type = visualization_msgs.msg.Marker.SPHERE
      marker.scale.x = .01
      marker.scale.y = .05
      marker.scale.z = .05
      marker.color.g = 1.0
      marker.color.a = 0.8
      marker_publisher.publish(marker)
      return true
    if (marker_type == "place_pose"):
      publishPoseMarker(marker_pose)

      # Add a flat sphere
      marker.type = visualization_msgs.msg.Marker.SPHERE
      marker.scale.x = .01
      marker.scale.y = .05
      marker.scale.z = .05
      marker.color.g = 1.0
      marker.color.a = 0.8
      marker_publisher.publish(marker)
      return true
    if (marker_type == "pick_pose"):
      publishPoseMarker(marker_pose)

      # Add a flat sphere
      marker.type = visualization_msgs.msg.Marker.SPHERE
      marker.scale.x = .01
      marker.scale.y = .05
      marker.scale.z = .05
      marker.color.r = 0.8
      marker.color.g = 0.4
      marker.color.a = 0.8
    elif (marker_type == ""):
      marker.type = visualization_msgs.msg.Marker.SPHERE
      marker.scale.x = .02
      marker.scale.y = .1
      marker.scale.z = .1
      
      marker.color.g = 1.0
      marker.color.a = 0.8
    else:
      rospy.warn("No useful marker message received.")
    marker_publisher.publish(marker)
    
    if (helper_fct_marker_id_count > 50):
      helper_fct_marker_id_count = 0
    return True
