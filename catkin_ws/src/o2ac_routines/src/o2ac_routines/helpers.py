#!/usr/bin/env python

import sys
import rospy
import geometry_msgs.msg
import actionlib
import tf
from math import *
tau = 2.0*pi  # Part of math from Python 3.6

import visualization_msgs.msg  # For marker visualization

import o2ac_msgs
import o2ac_msgs.msg
import o2ac_msgs.srv

import moveit_task_constructor_msgs.msg

from math import pi
import moveit_commander
from moveit_commander.conversions import pose_to_list
from o2ac_assembly_handler.assy import AssyHandler

import ur_msgs.msg
import ur_dashboard_msgs.msg
import moveit_msgs.msg

helper_fct_marker_id_count = 0

def upload_mtc_modules_initial_params():
  '''
  Set parameters that are needed for the initialization of the mtc_modules node
  '''
  rospy.set_param('mtc_modules/arm_group_names', ['a_bot','b_bot'])
  rospy.set_param('mtc_modules/hand_group_names', ['a_bot_robotiq_85','b_bot_robotiq_85'])
  rospy.set_param('mtc_modules/grasp_parameter_location', 'wrs_assembly_1')
  rospy.set_param('mtc_modules/lift_direction_reference_frame', 'world')
  rospy.set_param('mtc_modules/lift_direction', [0.0, 0.0, 1.0])
  rospy.set_param('mtc_modules/approach_place_direction_reference_frame', 'world')
  rospy.set_param('mtc_modules/approach_place_direction', [0.0, 0.0, -1.0])
  rospy.set_param('mtc_modules/retreat_direction_reference_frame', '')
  rospy.set_param('mtc_modules/retreat_direction', [-1.0, 0.0, 0.0])
  rospy.set_param('mtc_modules/support_surfaces', ['tray_center', 'screw_tool_holder_long'])

def spawn_objects(assembly_name, object_names, object_poses, object_reference_frame):
  '''
  Spawn collision objects in the planning scene

  This function uses the o2ac_assembly_handler module to spawn objects in the scene. The assembly, its objects and their metadata
  has to be set up inside the o2ac_assembly_handler module.

  Given a list of object names from an assembly, this functions spawns the listed objects in the corresponding poses in input 'object_poses'.
  The inputs 'object_names' and 'object_poses' must have the same lengths.
  The object poses are lists of floats in [x,y,z,r,p,y] format and are relative to the object_reference_frame
  '''
  moveit_commander.roscpp_initialize(sys.argv)
  assy_handler = AssyHandler(assembly_name)
  planning_scene_interface = moveit_commander.PlanningSceneInterface()
  transformer = tf.Transformer(True, rospy.Duration(10.0))

  for (object_name, object_pose) in zip(object_names, object_poses):
    co_pose = geometry_msgs.msg.Pose()
    co_pose.position.x = object_pose[0]
    co_pose.position.y = object_pose[1]
    co_pose.position.z = object_pose[2]
    quaternion = tf.transformations.quaternion_from_euler(eval(str(object_pose[3])),eval(str(object_pose[4])),eval(str(object_pose[5])))
    co_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

    collision_object_transform = geometry_msgs.msg.TransformStamped()
    collision_object_transform.header.frame_id = 'WORLD'
    collision_object_transform.child_frame_id = object_name
    collision_object_transform.transform.translation.x = co_pose.position.x
    collision_object_transform.transform.translation.y = co_pose.position.y
    collision_object_transform.transform.translation.z = co_pose.position.z
    collision_object_transform.transform.rotation.x = co_pose.orientation.x
    collision_object_transform.transform.rotation.y = co_pose.orientation.y
    collision_object_transform.transform.rotation.z = co_pose.orientation.z
    collision_object_transform.transform.rotation.w = co_pose.orientation.w
    transformer.setTransform(collision_object_transform)

    collision_object = next(co for co in assy_handler.collision_objects if co.id == object_name)
    collision_object.header.frame_id = object_reference_frame
    collision_object.mesh_poses[0] = co_pose

    subframe_poses = []

    for (subframe_name, subframe_pose) in zip(collision_object.subframe_names, collision_object.subframe_poses):
        subframe_transform = geometry_msgs.msg.TransformStamped()
        subframe_transform.header.frame_id = object_name
        subframe_transform.child_frame_id = subframe_name
        subframe_transform.transform.translation.x = subframe_pose.position.x
        subframe_transform.transform.translation.y = subframe_pose.position.y
        subframe_transform.transform.translation.z = subframe_pose.position.z
        subframe_transform.transform.rotation.x = subframe_pose.orientation.x
        subframe_transform.transform.rotation.y = subframe_pose.orientation.y
        subframe_transform.transform.rotation.z = subframe_pose.orientation.z
        subframe_transform.transform.rotation.w = subframe_pose.orientation.w

        transformer.setTransform(subframe_transform)

        (trans,rot) = transformer.lookupTransform('WORLD', subframe_name, rospy.Time(0))

        subframe_pose = geometry_msgs.msg.Pose()
        subframe_pose.position = geometry_msgs.msg.Point(*trans)
        subframe_pose.orientation = geometry_msgs.msg.Quaternion(*rot)

        subframe_poses.append(subframe_pose)

    collision_object.subframe_poses = subframe_poses

    planning_scene_interface.add_object(collision_object)

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


# Below are some ugly helper functions that should really be replaced. 
def publish_marker(marker_pose_stamped, marker_type):
  publisher = rospy.Publisher("vision_markers", visualization_msgs.msg.Marker, queue_size = 100)
  rospy.sleep(0.5)
  return publish_marker(publisher, marker_pose_stamped, marker_type)

def publish_marker(marker_publisher, marker_pose_stamped, marker_type):
  marker = visualization_msgs.msg.Marker()
  marker.header = marker_pose.header
  # marker.header.stamp = rospy.Time.now()
  marker.pose = marker_pose.pose

  marker.ns = "markers"
  marker.id = 0
  marker.lifetime = rospy.Duration(60.0)
  marker.action = visualization_msgs.msg.Marker.ADD

  if (marker_type == "pose"):
    publish_pose_marker(marker_pose)

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
    publish_pose_marker(marker_pose)

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
    publish_pose_marker(marker_pose)

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
    rospy.warn("No supported marker message received.")
  marker_publisher.publish(marker)
  
  if (helper_fct_marker_id_count > 50):
    helper_fct_marker_id_count = 0
  return True

# This is a helper function for publish_marker. Publishes a TF-like frame. Should probably be replaced by rviz_visual_tools
def publish_pose_marker(marker_publisher, marker_pose_stamped):
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

  rotatePoseByRPY(0, 0, tau/4, arrow_y.pose)
  rotatePoseByRPY(0, -tau/4, 0, arrow_z.pose)

  marker_publisher.publish(arrow_x)
  marker_publisher.publish(arrow_y)
  marker_publisher.publish(arrow_z)
  return True
