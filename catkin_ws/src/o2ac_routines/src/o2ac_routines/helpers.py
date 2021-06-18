#!/usr/bin/env python

import sys
import copy

from numpy.lib.function_base import append
import rospy
import random
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

helper_fct_marker_id_count = 0

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

def spawn_objects(assembly_database, object_names, object_poses, object_reference_frame):
  '''
  Spawn collision objects in the planning scene

  This function uses the o2ac_assembly_database module to spawn objects in the scene. The assembly, its objects and their metadata
  has to be set up inside the o2ac_assembly_database module.

  Given a list of object names from an assembly, this functions spawns the listed objects in the corresponding poses in input 'object_poses'.
  The inputs 'object_names' and 'object_poses' must have the same lengths.
  The object poses are lists of floats in [x,y,z,r,p,y] format and are relative to the object_reference_frame
  '''
  moveit_commander.roscpp_initialize(sys.argv)
  planning_scene_interface = moveit_commander.PlanningSceneInterface()
  transformer = tf.Transformer(True, rospy.Duration(10.0))

  for (object_name, object_pose) in zip(object_names, object_poses):
    co_pose = geometry_msgs.msg.Pose()
    co_pose.position = geometry_msgs.msg.Point(*object_pose[:3])
    quaternion = tf.transformations.quaternion_from_euler(*conversions.to_float(object_pose[3:]))
    co_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

    collision_object = next((co for co in assembly_database._collision_objects if co.id == object_name), None)
    assert collision_object is not None, "Collision object for '%s' does not exist or names do not match" % object_name

    # Create copy to avoid modifying the original
    collision_object_copy = moveit_msgs.msg.CollisionObject()
    collision_object_copy.header.frame_id = object_reference_frame
    collision_object_copy.pose = co_pose
    
    # Shallow copy the rest
    collision_object_copy.operation = collision_object.operation
    collision_object_copy.type = collision_object.type
    collision_object_copy.id = collision_object.id
    collision_object_copy.primitives = collision_object.primitives
    collision_object_copy.primitive_poses = collision_object.primitive_poses
    collision_object_copy.meshes = collision_object.meshes
    collision_object_copy.mesh_poses = collision_object.mesh_poses
    
    planning_scene_interface.add_object(collision_object_copy)

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
  q_in = [in_quat.x, in_quat.y, in_quat.z, in_quat.w]
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

# // Returns the angle between two quaternions
# double quaternionDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) 
# { 
#   tf::Quaternion q1tf, q2tf
#   tf::quaternionMsgToTF(q1, q1tf)
#   tf::quaternionMsgToTF(q2, q2tf)
#   return 2*q1tf.angle(q2tf) 
# }

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
  angle = 2*math.atan2(norm2(quaternion[0], quaternion[1], quaternion[2]), quaternion[3])
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

def get_direction_index(direction):
  DIRECTION_INDEX = {'X':0, 'Y':1, 'Z':2}
  return DIRECTION_INDEX.get(direction.upper())

def get_target_force(direction, force):
  validate_direction(direction)

  res = [0.,0.,0.,0.,0.,0.]
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

def get_trajectory_duration(plan):
  time_from_start = plan.joint_trajectory.points[-1].time_from_start
  duration = rospy.Time(time_from_start.secs, time_from_start.nsecs)
  return duration.to_sec()

def get_trajectory_joint_goal(plan):
  return plan.joint_trajectory.points[-1].positions

def to_robot_state(move_group, joints):
  joint_state = JointState()
  joint_state.header.stamp = rospy.Time.now()
  joint_state.name = move_group.get_active_joints()
  joint_state.position = joints
  moveit_robot_state = RobotState()
  moveit_robot_state.joint_state = joint_state
  return moveit_robot_state

def to_sequence_gripper(gripper, gripper_opening_width=0.14, gripper_force=40, gripper_velocity=0.03):
  item = {
    "pose_type": "gripper",
    "gripper":
            {
              "action":gripper,
              "width": gripper_opening_width,
              "force": gripper_force,
              "velocity": gripper_velocity,
            }
    }
  return ["waypoint", item]

def to_sequence_item(pose, speed=0.5, acc=0.25):
  if isinstance(pose, geometry_msgs.msg.PoseStamped):
    item           = {"pose": conversions.from_point(pose.pose.position).tolist() + np.rad2deg(transformations.euler_from_quaternion(conversions.from_quaternion(pose.pose.orientation))).tolist(),
                      "pose_type": "task-space-in-frame",
                      "frame_id": pose.header.frame_id,
                     }
  if isinstance(pose, str):
    item           = {"pose": pose,
                      "pose_type": "named-pose",
                     }
  if isinstance(pose, list): # Assume joint angles
    item       = {"pose": pose,
                  "pose_type": "joint-space-goal-cartesian-lin-motion",
                  }
  item.update({"speed": speed, "acc": acc})

  return ["waypoint", item]

def to_sequence_trajectory(trajectory, blend_radiuses, speed=0.5, default_frame="world"):
  sequence_trajectory = []
  for t, br in zip(trajectory, blend_radiuses):
    if isinstance(t, geometry_msgs.msg.PoseStamped):
      sequence_trajectory.append([t, br])
    elif isinstance(t, list):
      sequence_trajectory.append([conversions.to_pose_stamped(default_frame, t), br])
  return ["trajectory", [sequence_trajectory, speed]]
