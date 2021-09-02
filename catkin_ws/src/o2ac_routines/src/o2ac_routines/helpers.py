#!/usr/bin/env python

import os
import sys
import copy

from numpy.lib.function_base import append
import rospkg
import rosbag
import rospy
import random
import json
import numpy as np
import geometry_msgs.msg
import actionlib
import tf
from math import pi, cos, sin, sqrt, atan2
tau = 2.0*pi  # Part of math from Python 3.6

import visualization_msgs.msg  # For marker visualization

import o2ac_msgs
import o2ac_msgs.msg
import o2ac_msgs.srv

import moveit_task_constructor_msgs.msg

import moveit_commander
from moveit_commander.conversions import pose_to_list

from ur_control import conversions, transformations

import ur_msgs.msg
import ur_dashboard_msgs.msg
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String, Float64MultiArray
import trajectory_msgs.msg
helper_fct_marker_id_count = 0

def save_task_plan(func):
  '''Decorator that optionally save the solution to a plan.'''

  def wrap(*args, **kwargs):
      save_solution_to_file = kwargs.pop("save_solution_to_file", None)
      result = func(*args, **kwargs)
      
      if result is None:
        rospy.logerr("No solution from server")
        return

      if result.success and save_solution_to_file:
        path = rospkg.RosPack().get_path('o2ac_routines') + '/MP_solutions/'
        with open(path + save_solution_to_file,'wb') as f:
          pickle.dump(result, f)
        rospy.loginfo("Writing solution to: %s" % save_solution_to_file)
      return result  
  return wrap


def upload_mtc_modules_initial_params():
  '''
  Set parameters that are needed for the initialization of the mtc_modules node
  '''
  rospy.set_param('mtc_modules/arm_group_names', ['a_bot','b_bot'])
  rospy.set_param('mtc_modules/hand_group_names', ['a_bot_robotiq_85','b_bot_robotiq_85'])
  rospy.set_param('mtc_modules/grasp_parameter_location', 'wrs_assembly_2020')
  rospy.set_param('mtc_modules/lift_direction_reference_frame', 'world')
  rospy.set_param('mtc_modules/lift_direction', [0.0, 0.0, 1.0])
  rospy.set_param('mtc_modules/approach_place_direction_reference_frame', 'world')
  rospy.set_param('mtc_modules/approach_place_direction', [0.0, 0.0, -1.0])
  rospy.set_param('mtc_modules/retreat_direction_reference_frame', '')
  rospy.set_param('mtc_modules/retreat_direction', [-1.0, 0.0, 0.0])
  rospy.set_param('mtc_modules/support_surfaces', ['tray_center', 'screw_tool_holder_long'])

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

def rotateQuaternionByRPY(roll, pitch, yaw, in_quat):
  """
  Apply RPY rotation in the rotated frame (the one to which the quaternion has rotated the reference frame).
  
  Input: geometry_msgs.msg.Quaternion
  Output: geometry_msgs.msg.Quaternion rotated by roll, pitch, yaw in its frame
  """
  if type(in_quat) == type(geometry_msgs.msg.Quaternion()):
    q_in = [in_quat.x, in_quat.y, in_quat.z, in_quat.w]
  else:
    q_in = in_quat
  q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  q_rotated = tf.transformations.quaternion_multiply(q_in, q_rot)

  return geometry_msgs.msg.Quaternion(*q_rotated)

def rotateQuaternionByRPYInUnrotatedFrame(roll, pitch, yaw, in_quat):
  """
  Apply RPY rotation in the reference frame of the quaternion.
  
  Input: geometry_msgs.msg.Quaternion
  Output: geometry_msgs.msg.Quaternion rotated by roll, pitch, yaw in its frame
  """
  q_in = [in_quat.x, in_quat.y, in_quat.z, in_quat.w]
  q_rot = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  q_rotated = tf.transformations.quaternion_multiply(q_rot, q_in)

  return geometry_msgs.msg.Quaternion(*q_rotated)

def rotateTranslationByRPY(roll, pitch, yaw, in_point):
  matrix = tf.transformations.euler_matrix(roll, pitch, yaw)
  xyz = np.array([in_point.x, in_point.y, in_point.z, 1]).reshape((4,1))
  xyz_new = np.dot(matrix, xyz)
  return geometry_msgs.msg.Point(*xyz_new[:3])

def rotateTranslationByQuat(quat, in_point):
  try:
    if type(quat) == type(geometry_msgs.msg.Quaternion()):
      q = [quat.x, quat.y, quat.z, quat.w]
    else:
      q = quat
    if type(in_point) == type(geometry_msgs.msg.Point()):
      p = [in_point.x, in_point.y, in_point.z]
    else:
      p = in_point
  except:
    pass # header doesn't exist so the object is probably not posestamped
  
  matrix = tf.transformations.quaternion_matrix(q)
  xyz = np.array([p[0], p[1], p[2], 1]).reshape((4,1))
  xyz_new = np.dot(matrix, xyz)
  return geometry_msgs.msg.Point(*xyz_new[:3])

# RPY rotations are applied in the frame of the pose.
def rotatePoseByRPY(roll, pitch, yaw, in_pose):
  # Catch if in_pose is a PoseStamped instead of a Pose.
  try:
    if type(in_pose) == type(geometry_msgs.msg.PoseStamped()):
      outpose = copy.deepcopy(in_pose)
      outpose.pose = rotatePoseByRPY(roll, pitch, yaw, in_pose.pose)
      return outpose
  except:
    pass # header doesn't exist so the object is probably not posestamped
  
  rotated_pose = copy.deepcopy(in_pose)
  rotated_pose.orientation = rotateQuaternionByRPY(roll, pitch, yaw, rotated_pose.orientation)
  return rotated_pose

# Taken from the transformations library: https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1772
def vector_norm(data, axis=None, out=None):
    data = np.array(data, dtype=np.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return sqrt(np.dot(data, data))
        data *= data
        out = np.atleast_1d(np.sum(data, axis=axis))
        np.sqrt(out, out)
        return out
    data *= data
    np.sum(data, axis=axis, out=out)
    np.sqrt(out, out)
    return None

# Taken from the transformations library: https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py#L1818
def angle_between_vectors(v0, v1, directed=True, axis=0):
    """Return angle between vectors.
    If directed is False, the input vectors are interpreted as undirected axes,
    i.e. the maximum angle is pi/2.
    >>> a = angle_between_vectors([1, -2, 3], [-1, 2, -3])
    >>> np.allclose(a, math.pi)
    True
    >>> a = angle_between_vectors([1, -2, 3], [-1, 2, -3], directed=False)
    >>> np.allclose(a, 0)
    True
    >>> v0 = [[2, 0, 0, 2], [0, 2, 0, 2], [0, 0, 2, 2]]
    >>> v1 = [[3], [0], [0]]
    >>> a = angle_between_vectors(v0, v1)
    >>> np.allclose(a, [0, 1.5708, 1.5708, 0.95532])
    True
    >>> v0 = [[2, 0, 0], [2, 0, 0], [0, 2, 0], [2, 0, 0]]
    >>> v1 = [[0, 3, 0], [0, 0, 3], [0, 0, 3], [3, 3, 3]]
    >>> a = angle_between_vectors(v0, v1, axis=1)
    >>> np.allclose(a, [1.5708, 1.5708, 1.5708, 0.95532])
    True
    """
    v0 = np.array(v0, dtype=np.float64, copy=False)
    v1 = np.array(v1, dtype=np.float64, copy=False)
    dot = np.sum(v0 * v1, axis=axis)
    dot /= vector_norm(v0, axis=axis) * vector_norm(v1, axis=axis)
    dot = np.clip(dot, -1.0, 1.0)
    return np.arccos(dot if directed else np.fabs(dot))

def getOrientedFlatGraspPoseFromXAxis(pre_grasp_pose):
  """ Used to obtain the grasp pose for e.g. the motor, where only the direction of the x-axis matters.
      
      Input: PoseStamped or Pose of motor_center in stationary frame (e.g. tray_center)
      Output: PoseStamped or Pose, z-axis aligned with frame, x-axis in XY-plane
  """
  try:
    if type(pre_grasp_pose) == type(geometry_msgs.msg.PoseStamped()):
      outpose = copy.deepcopy(pre_grasp_pose)
      outpose.pose = getOrientedFlatGraspPoseFromXAxis(pre_grasp_pose.pose)
      return outpose
  except Exception as e:
    rospy.logerr("Error!")
    rospy.logerr(e)
    pass

  # Take (1,0,0) in motor/center, rotate to tray center
  p_x_object = geometry_msgs.msg.Point(1.0, 0, 0)
  p_x_object = rotateTranslationByQuat(pre_grasp_pose.orientation, p_x_object)

  # Project to XY-plane
  p_xy_in_header_frame = copy.deepcopy(p_x_object)
  p_xy_in_header_frame.z = 0.0

  # Get angle between projected object axis and tray_center x-axis
  theta = angle_between_vectors([p_xy_in_header_frame.x, p_xy_in_header_frame.y, p_xy_in_header_frame.z], [1.0, 0, 0])
  outpose = copy.deepcopy(pre_grasp_pose)
  outpose.orientation = tf.transformations.quaternion_from_euler(0, 0, theta)
  
  outpose = rotatePoseByRPY(0, tau/4, 0, outpose)

  return outpose

# # Returns the angle between two quaternions
# def quaternionDistance(q1, q2):
#   # q1, q2 are lists
#   tf::Quaternion q1tf, q2tf
#   tf::quaternionMsgToTF(q1, q1tf)
#   tf::quaternionMsgToTF(q2, q2tf)
#   return 2*q1tf.angle(q2tf)

# double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
# {
#   tf::Point tp1, tp2
#   tf::pointMsgToTF(p1, tp1)
#   tf::pointMsgToTF(p2, tp2)
#   return tfDistance(tp1, tp2)
# }

def multiply_quaternion_msgs(q1_msg, q2_msg):
  q1 = [q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w]
  q2 = [q2_msg.x, q2_msg.y, q2_msg.z, q2_msg.w]
  return geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply(q1, q2))

def pose_msg_is_identity(pose):
  """ Returns true if the pose is close to identity """
  p_identity = geometry_msgs.msg.Pose()
  p_identity.orientation.w = 1.0
  return pose_dist(pose, p_identity) < 1e-4

def pose_dist(p1, p2):
  """
  Returns Euclidean distance of two geometry_msgs.msg.Pose objects defined in the same frame.
  """
  v1 = [p1.position.x, p1.position.y, p1.position.z]
  v2 = [p2.position.x, p2.position.y, p2.position.z]
  vd = [v1[0]-v2[0], v1[1]-v2[1], v1[2]-v2[2]]
  return norm2(vd[0], vd[1], vd[2])

def interpolate_between_poses(p1, p2, ratio):
  """ Returns a point between two poses defined by ratio.
      Poses need to be defined in the same frame.

  Input: Two geometry_msgs.msg.Pose objects
         ratio between 0.0 and 1.0
  Output: Point between the poses. p1 if ratio == 0.0, p2 if ratio == 1.0
  """
  p_out = copy.deepcopy(p1)
  p_out.position.x = p1.position.x + (p2.position.x - p1.position.x) * ratio
  p_out.position.y = p1.position.y + (p2.position.y - p1.position.y) * ratio
  p_out.position.z = p1.position.z + (p2.position.z - p1.position.z) * ratio
  if not all_close(p1.orientation, p2.orientation, 0.001):
    rospy.logwarn("Orientation interpolation of two poses is not implemented!! (Use slerp)")
  return p_out

def norm2(a, b, c=0.0):
  return sqrt(a**2 + b**2 + c**2)

def ur_axis_angle_to_quat(axis_angle):
  # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
  angle = norm2(*axis_angle)
  axis_normed = [axis_angle[0]/angle, axis_angle[1]/angle, axis_angle[2]/angle]
  s = sin(angle/2)
  return [s*axis_normed[0], s*axis_normed[1], s*axis_normed[2], cos(angle/2)]   #xyzw

def quat_to_ur_axis_angle(quaternion):
  # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
  # quaternion must be [xyzw]
  angle = 2*atan2(norm2(quaternion[0], quaternion[1], quaternion[2]), quaternion[3])
  if abs(angle) > 1e-6:
    axis_normed = [ quaternion[0]/sin(angle/2), quaternion[1]/sin(angle/2), quaternion[2]/sin(angle/2) ]
  else:
    axis_normed = 0.0
  return [axis_normed[0]*angle, axis_normed[1]*angle, axis_normed[2]*angle]

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list or type(goal) is np.ndarray:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    position_allclose = all_close(conversions.from_point(goal.position), conversions.from_point(actual.position), tolerance) 
    quaternion_allclose = all_close(conversions.from_quaternion(goal.orientation), conversions.from_quaternion(actual.orientation), tolerance)
    if not quaternion_allclose:  # Check for the second orientation
      quaternion_allclose = all_close(conversions.from_quaternion(goal.orientation), (-1)*conversions.from_quaternion(actual.orientation), tolerance)
    return position_allclose and quaternion_allclose

  return True


# Below are some ugly helper functions that should really be replaced. 
def publish_marker(marker_pose_stamped, marker_type="", namespace="", marker_topic="o2ac_markers"):
  publisher = rospy.Publisher(marker_topic, visualization_msgs.msg.Marker, queue_size = 100)
  rospy.sleep(0.5)
  return publish_marker_(publisher, marker_pose_stamped, marker_type, namespace)

def publish_marker_(marker_publisher, marker_pose_stamped, marker_type="", namespace=""):
  marker = visualization_msgs.msg.Marker()
  if not marker_type:
    marker_type = "pose"
  marker.header = marker_pose_stamped.header
  # marker.header.stamp = rospy.Time.now()
  marker.pose = marker_pose_stamped.pose

  marker.ns = namespace
  if not marker.ns:
    marker.ns = "markers"
  helper_fct_marker_id_count = np.random.randint(2147483647)  # int32
  marker.id = helper_fct_marker_id_count
  marker.lifetime = rospy.Duration(60.0)
  marker.action = visualization_msgs.msg.Marker.ADD

  if (marker_type == "pose"):
    publish_pose_marker_(marker_publisher, marker_pose_stamped, namespace=namespace, helper_fct_marker_id_count=helper_fct_marker_id_count)

    # Add a flat sphere
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.scale.x = .01
    marker.scale.y = .05
    marker.scale.z = .05
    marker.color.g = 1.0
    marker.color.a = 0.8
    marker_publisher.publish(marker)
    return True
  if (marker_type == "place_pose"):
    publish_pose_marker_(marker_publisher, marker_pose_stamped, namespace=namespace, helper_fct_marker_id_count=helper_fct_marker_id_count)

    # Add a flat sphere
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.scale.x = .01
    marker.scale.y = .05
    marker.scale.z = .05
    marker.color.g = 1.0
    marker.color.a = 0.8
    marker_publisher.publish(marker)
    return True
  if (marker_type == "pick_pose"):
    publish_pose_marker_(marker_publisher, marker_pose_stamped, namespace=namespace, helper_fct_marker_id_count=helper_fct_marker_id_count)

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
  
  if (helper_fct_marker_id_count > 100):
    helper_fct_marker_id_count = 0
  return True

# This is a helper function for publish_marker. Publishes a TF-like frame. Should probably be replaced by rviz_visual_tools
def publish_pose_marker_(marker_publisher, marker_pose_stamped, namespace="", helper_fct_marker_id_count=0):
  marker = visualization_msgs.msg.Marker()
  marker.header = marker_pose_stamped.header
  marker.header.stamp = rospy.Time.now()
  marker.pose = marker_pose_stamped.pose

  marker.ns = namespace
  if not marker.ns:
    marker.ns = "markers"
  marker.id = helper_fct_marker_id_count
  marker.lifetime = rospy.Duration()
  marker.action = visualization_msgs.msg.Marker.ADD

  # This draws a TF-like frame.
  marker.type = visualization_msgs.msg.Marker.ARROW
  marker.scale.x = .1
  marker.scale.y = .01
  marker.scale.z = .01
  marker.color.a = .8

  arrow_x = copy.deepcopy(marker)
  arrow_y = copy.deepcopy(marker)
  arrow_z = copy.deepcopy(marker)
  
  helper_fct_marker_id_count += 1
  arrow_x.id = copy.deepcopy(helper_fct_marker_id_count)
  helper_fct_marker_id_count += 1
  arrow_y.id = copy.deepcopy(helper_fct_marker_id_count)
  helper_fct_marker_id_count += 1
  arrow_z.id = copy.deepcopy(helper_fct_marker_id_count)
  
  arrow_x.color.r = 1.0
  arrow_y.color.g = 1.0
  arrow_z.color.b = 1.0

  arrow_y.pose = rotatePoseByRPY(0, 0, tau/4, arrow_y.pose)
  arrow_z.pose = rotatePoseByRPY(0, -tau/4, 0, arrow_z.pose)

  marker_publisher.publish(arrow_x)
  marker_publisher.publish(arrow_y)
  marker_publisher.publish(arrow_z)
  return True

# =========

def get_direction_index(direction):
  DIRECTION_INDEX = {'X':0, 'Y':1, 'Z':2}
  return DIRECTION_INDEX.get(direction.upper())

def get_target_force(direction, force):
  validate_direction(direction)

  res = [0.,0.,0.,0.,0.,0.]
  if 'Z' in direction: # hack
    sign = -1. if '+' in direction else 1.
  else:
    sign = 1. if '+' in direction else -1.
  res[get_direction_index(direction[1])] = force * sign

  return np.array(res)

def validate_direction(direction):
  VALID_DIRECTIONS = ('+X', '+Y', '+Z', '-X', '-Y', '-Z')
  assert direction in VALID_DIRECTIONS, "Invalid direction: %s" % direction

def get_orthogonal_plane(direction):
  if direction == "X":
    return "YZ"
  elif direction == "Y":
    return "XZ"
  elif direction == "Z":
    return "XY"
  else:
    raise ValueError("Invalid direction: %s" % direction)

def get_random_valid_direction(plane):
  if plane == "XZ":
      return random.choice(["+X","-X","+Z","-Z"])
  elif plane == "YZ":
      return random.choice(["+Y","-Y","+Z","-Z"])
  elif plane == "XY":
      return random.choice(["+X","-X","+Y","-Y"])
  else:
      raise ValueError("Invalid value for plane: %s" % plane)

def ordered_joint_values_from_dict(joints_dict, joints_name_list):
  return conversions.to_float([joints_dict.get(q) for q in joints_name_list])

def check_for_real_robot(func):
    '''Decorator that validates the real robot is used or no'''
  
    def wrap(*args, **kwargs):
        if args[0].use_real_robot:
          return func(*args, **kwargs)
        rospy.logwarn("Ignoring function %s since no real robot is being used" % func.__name__)
        return True
    return wrap

def lock_impedance(func):
    '''Decorator that locks resources while being used. Assumes there is a self.vision_lock accessible in the decorated method'''
    def wrap(*args, **kwargs):
        result = False
        # print("== waiting for lock ==", func.__name__)
        try:
            args[0].impedance_lock.acquire()
            # print("Lock acquired", func.__name__)
            result = func(*args, **kwargs)
        except Exception as e:
          print("(lock_impedance) received an exception", func.__name__, e)
        finally:
          args[0].impedance_lock.release()
          # print("Lock released", func.__name__)
        return result
    return wrap

def lock_vision(func):
    '''Decorator that locks resources while being used. Assumes there is a self.vision_lock accessible in the decorated method'''
    def wrap(*args, **kwargs):
        result = False
        print("== waiting for lock ==", func.__name__)
        try:
            args[0].vision_lock.acquire()
            print("Lock acquired", func.__name__)
            result = func(*args, **kwargs)
        except Exception as e:
          print("(lock_vision) received an exception", func.__name__, e)
        finally:
          args[0].vision_lock.release()
          print("Lock released", func.__name__)
        return result
    return wrap

def get_trajectory_duration(plan):
  time_from_start = plan.joint_trajectory.points[-1].time_from_start
  duration = rospy.Time(time_from_start.secs, time_from_start.nsecs)
  return duration.to_sec()

def get_trajectory_joint_goal(plan, joints_order=None):
  if joints_order is not None:
    joint_values = []
    for joint in joints_order:
      i = plan.joint_trajectory.joint_names.index(joint)
      joint_values.append(plan.joint_trajectory.points[-1].positions[i])
    return joint_values
  return plan.joint_trajectory.points[-1].positions

def to_robot_state(move_group, joints):
  moveit_robot_state = move_group.get_current_state()
  moveit_robot_state.joint_state.header.stamp = rospy.Time.now()
  active_joints = move_group.get_active_joints()
  temp_joint_values = list(moveit_robot_state.joint_state.position)
  for i in range(len(active_joints)):
    temp_joint_values[moveit_robot_state.joint_state.name.index(active_joints[i])] = joints[i]
  moveit_robot_state.joint_state.position = temp_joint_values
  return moveit_robot_state

def to_sequence_gripper(action, gripper_opening_width=0.14, gripper_force=40, gripper_velocity=0.03, pre_callback=None, post_callback=None, wait=True):
  item = {
    "pose_type": "gripper",
    "gripper":
            {
              "action": action,
              "open_width": gripper_opening_width,
              "force": gripper_force,
              "velocity": gripper_velocity,
              "pre_callback": pre_callback,
              "post_callback": post_callback,
              "wait": wait,
            }
    }
  return ["waypoint", item]

def to_sequence_item_relative(pose, relative_to_base=False, relative_to_tcp=False, speed=0.5, acc=0.25):
  if relative_to_tcp:
    pose_type = 'relative-tcp'
  elif relative_to_base:
    pose_type = 'relative-base'
  else:
    pose_type = 'relative-world'
  item  = {"pose": pose,
           "pose_type": pose_type}
  item.update({"speed": speed, "acc": acc})
  return ["waypoint", item]

def to_sequence_item(pose, speed=0.5, acc=0.25, linear=True, end_effector_link=None):
  if isinstance(pose, geometry_msgs.msg.PoseStamped):
    item           = {"pose": conversions.from_point(pose.pose.position).tolist() + np.rad2deg(transformations.euler_from_quaternion(conversions.from_quaternion(pose.pose.orientation))).tolist(),
                      "pose_type": "task-space-in-frame",
                      "frame_id": pose.header.frame_id,
                      "move_linear": linear,
                      "end_effector_link": end_effector_link,
                     }
  if isinstance(pose, str):
    item           = {"pose": pose,
                      "pose_type": "named-pose",
                     }
  if isinstance(pose, list): # Assume joint angles
    # if not explicitly defined, use linear motion
      item       = {"pose": pose,
                    "pose_type": "joint-space-goal-cartesian-lin-motion" if linear else "joint-space",
                    }
  item.update({"speed": speed, "acc": acc})

  return ["waypoint", item]

def to_sequence_trajectory(trajectory, blend_radiuses=0.0, speed=0.5, default_frame="world"):
  sequence_trajectory = []
  blend_radiuses = blend_radiuses if isinstance(blend_radiuses, list) else np.zeros_like(trajectory)+blend_radiuses
  for i, (t, br) in enumerate(zip(trajectory, blend_radiuses)):
    if isinstance(speed, list):
      spd = speed[i]
    else:
      spd = speed if i != len(trajectory) - 1 else 0.2

    if isinstance(t, geometry_msgs.msg.PoseStamped):
      sequence_trajectory.append([t, br, spd])
    elif isinstance(t, list):
      sequence_trajectory.append([conversions.to_pose_stamped(default_frame, t), br, spd])
  return ["trajectory", sequence_trajectory]

def to_sequence_joint_trajectory(trajectory, blend_radiuses=0.0, speed=0.5, end_effector_link="", linear=False):
  sequence_trajectory = []
  blend_radiuses = blend_radiuses if isinstance(blend_radiuses, list) else np.zeros_like(trajectory)+blend_radiuses
  speeds = speed if isinstance(speed, list) else np.zeros_like(trajectory)+speed
  for waypoint, br, spd in zip(trajectory, blend_radiuses, speeds):
    sequence_trajectory.append([waypoint, br, spd])
  return ["joint_trajectory", sequence_trajectory, end_effector_link, linear]

def to_sequence_item_dual_arm(pose1, pose2, speed, acc=None, planner="OMPL"):
  item = {"pose": conversions.from_pose_to_list(pose1.pose),
          "pose2": conversions.from_pose_to_list(pose2.pose),
          "pose_type": "task-space-in-frame",
          "frame_id": pose1.header.frame_id,
          "planner": planner,
         }
  item.update({"speed":speed})
  item.update({"acc":acc})
  return ["waypoint", item]

def to_sequence_item_master_slave(master, slave, pose, slave_relative_pose, speed):
  item = {
      "pose": conversions.from_pose_to_list(pose.pose),
      "pose_type": "master-slave",
      "master_name": master,
      "slave_name": slave,
      "slave_relation": slave_relative_pose,
      "frame_id": pose.header.frame_id,
      "speed": speed,
  }
  return ["waypoint", item]

def get_plan_full_path(name):
  rp = rospkg.RosPack()
  return rp.get_path("o2ac_routines") + "/config/saved_plans/" + name

def load_sequence_plans(name):
  bagfile = get_plan_full_path(name)
  sequence = []
  if not os.path.exists(bagfile):
    raise Exception("Sequence: %s does not exist" % bagfile)
  with rosbag.Bag(bagfile, 'r') as bag:
    for (topic, msg, ts) in bag.read_messages():
      if topic in ("robot_name", "initial_joint_configuration"):
        sequence.append(msg.data)
      elif topic == "gripper_action":
        sequence.append(json.loads(msg.data))
      else:
        sequence.append(msg)
  return sequence

def save_sequence_plans(name, plans):
  bagfile = get_plan_full_path(name)
  if os.path.exists(bagfile):
    os.remove(bagfile)

  # Make sure the directory exists before trying to open a file
  saved_plans_directory = os.path.dirname(bagfile)
  if not os.path.exists(saved_plans_directory):
    os.makedirs(saved_plans_directory)
  
  with rosbag.Bag(bagfile, 'w') as bag:
    bag.write(topic="robot_name", msg=String(data=plans[0]))
    bag.write(topic="initial_joint_configuration", msg=Float64MultiArray(data=plans[1]))
    for plan in plans[2:]:
      if isinstance(plan, dict):
        bag.write(topic="gripper_action", msg=String(json.dumps(plan)))
      else:
        bag.write(topic="plan", msg=plan)

def create_tray_collision_object(id, pose, frame_id):
  tray_co = moveit_msgs.msg.CollisionObject()
  tray_co.header.frame_id = frame_id
  tray_co.id = id
  tray_co.primitives = [SolidPrimitive()]
  tray_co.primitive_poses = [pose] 
  tray_co.primitives[0].type = SolidPrimitive.BOX
  tray_co.primitives[0].dimensions = [.255, .375, 0.05]
  tray_co.operation = tray_co.ADD
  tray_co.subframe_names = ["center"]
  tray_co.subframe_poses = [pose]
  return tray_co

def combine_plans(a_bot_plan, b_bot_plan):
  assert a_bot_plan.joint_trajectory.header.frame_id == b_bot_plan.joint_trajectory.header.frame_id
  plan = moveit_msgs.msg.RobotTrajectory()
  plan.joint_trajectory.header = a_bot_plan.joint_trajectory.header
  plan.joint_trajectory.joint_names = a_bot_plan.joint_trajectory.joint_names + b_bot_plan.joint_trajectory.joint_names
  a_num_points = len(a_bot_plan.joint_trajectory.points)
  b_num_points = len(b_bot_plan.joint_trajectory.points)
  print("a_bot # points:", a_num_points)
  print("b_bot # points:", b_num_points)
  if a_num_points == b_num_points or a_num_points < b_num_points:
    for i in range(a_num_points):
      plan.joint_trajectory.points.append(concat_joint_trajectory_point(a_bot_plan.joint_trajectory.points[i], b_bot_plan.joint_trajectory.points[i]))

  if a_num_points < b_num_points:
    diff = a_num_points - b_num_points
    for i in range(diff, 0):
      plan.joint_trajectory.points.append(concat_joint_trajectory_point(a_bot_plan.joint_trajectory.points[-1], b_bot_plan.joint_trajectory.points[i]))

  if a_num_points > b_num_points:
    for i in range(b_num_points):
      plan.joint_trajectory.points.append(concat_joint_trajectory_point(a_bot_plan.joint_trajectory.points[i], b_bot_plan.joint_trajectory.points[i]))
    diff = b_num_points - a_num_points
    for i in range(diff, 0):
      plan.joint_trajectory.points.append(concat_joint_trajectory_point(a_bot_plan.joint_trajectory.points[i], b_bot_plan.joint_trajectory.points[-1]))
  return plan

def concat_joint_trajectory_point(point1, point2):
  point = trajectory_msgs.msg.JointTrajectoryPoint()
  point.positions = point1.positions + point2.positions
  point.velocities = point1.velocities + point2.velocities
  point.accelerations = point1.accelerations + point2.accelerations
  point.effort = point1.effort + point2.effort
  point.time_from_start = point1.time_from_start
  return point

def stack_plans(plans):
  staked_plan = copy.deepcopy(plans.pop(0))
  for plan in plans:
    staked_plan = copy.deepcopy(stack_two_plans(staked_plan, plan))
  return staked_plan

def stack_two_plans(plan1, plan2):
  # Stack 2 plans of the same move group
  plan = copy.deepcopy(plan1)
  plan1_duration = plan1.joint_trajectory.points[-1].time_from_start
  plan2.joint_trajectory.points.pop(0)
  for point in plan2.joint_trajectory.points:
    new_point = copy.deepcopy(point)
    new_point.time_from_start += plan1_duration
    plan.joint_trajectory.points.append(new_point)
  print("----")
  print("plan1:", len(plan1.joint_trajectory.points), get_trajectory_duration(plan1))
  print("plan2:", len(plan2.joint_trajectory.points), get_trajectory_duration(plan2))
  print("plan res:", len(plan.joint_trajectory.points), get_trajectory_duration(plan))
  return plan

def save_single_plan(filename, plan):
  bagfile = get_plan_full_path(filename)
  if os.path.exists(bagfile):
    os.remove(bagfile)

  # Make sure the directory exists before trying to open a file
  saved_plans_directory = os.path.dirname(bagfile)
  if not os.path.exists(saved_plans_directory):
    os.makedirs(saved_plans_directory)
  
  with rosbag.Bag(bagfile, 'w') as bag:
    bag.write(topic="plan", msg=plan)

def load_single_plan(filename):
  bagfile = get_plan_full_path(filename)
  plan = None
  if not os.path.exists(bagfile):
    rospy.logwarn("Plan: %s does not exist" % bagfile)
    return None
  with rosbag.Bag(bagfile, 'r') as bag:
    for (topic, msg, ts) in bag.read_messages():
      plan = msg
  return plan
