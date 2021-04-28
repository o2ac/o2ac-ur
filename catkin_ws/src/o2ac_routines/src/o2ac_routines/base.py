#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, OMRON SINIC X
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
#  * Neither the name of OMRON SINIC X nor the names of its
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

import sys
import copy
import rospy
import rospkg
import tf_conversions
import tf 
import actionlib

from math import *
from math import pi
tau = 2.0*pi  # Part of math from Python 3.6

import yaml
import pickle

import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import std_msgs.msg
import robotiq_msgs.msg
import ur_dashboard_msgs.msg
import ur_dashboard_msgs.srv
import std_srvs.srv
import controller_manager_msgs.srv
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

import o2ac_msgs.msg
import o2ac_msgs.srv
import o2ac_task_planning_msgs.msg

from std_msgs.msg import String, Bool

from o2ac_assembly_database.assembly_reader import AssemblyReader

import ur_msgs.msg
import ur_msgs.srv
from o2ac_routines.helpers import *

from o2ac_routines.skill_server_client import SkillServerClient
from o2ac_routines.vision_client import VisionClient
from o2ac_routines.ur_robot import URRobot
from o2ac_routines.tools import Tools
from o2ac_routines.ur_force_control import URForceController

class O2ACBase(object):
  """
  This class contains the basic helper and convenience functions used in the routines.
  The basic functions include the initialization of the services and actions,
  and shorthand functions for the most common actions.
  """
  def __init__(self):
    rospy.init_node('o2ac_routines', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    self.listener = tf.TransformListener()

    # Status variables and settings
    self.use_real_robot = rospy.get_param("use_real_robot", False)
    self.force_ur_script_linear_motion = False
    self.force_moveit_linear_motion = True

    self.competition_mode = False   # Setting this to True disables confirmation dialogs etc., thus enabling uninterrupted automatic motion

    self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
    self.pause_mode_ = False
    self.test_mode_ = False


    self.speed_fast = 0.2
    self.speed_fastest = 0.3
    self.acc_fast = 0.2
    self.acc_fastest = 0.3

    self.reduced_mode_speed_limit = .25

    # Miscellaneous helpers
    self.robots = moveit_commander.RobotCommander()
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface()
    
    self.assembly_database = AssemblyReader()

    # Action clients and movegroups
    self.a_bot = URRobot("a_bot", self.use_real_robot)
    self.b_bot = URRobot("b_bot", self.use_real_robot)
    # For compatibility let's wrap the robots
    self.active_robots = {'a_bot': self.a_bot, 'b_bot': self.b_bot}
    
    for robot in ["a_bot", "b_bot"]:
      self.active_robots[robot].get_status_from_param_server()
    
    self.a_bot_compliant_arm = URForceController(robot_name='a_bot')
    self.b_bot_compliant_arm = URForceController(robot_name='b_bot')

    self.skill_server = SkillServerClient()
    self.vision = VisionClient()
    self.tools = Tools(self.use_real_robot)

    self.pick_planning_client = actionlib.SimpleActionClient('/pick_planning', o2ac_task_planning_msgs.msg.PickObjectAction)
    self.place_planning_client = actionlib.SimpleActionClient('/place_planning', o2ac_task_planning_msgs.msg.PlaceObjectAction)
    self.release_planning_client = actionlib.SimpleActionClient('/release_planning', o2ac_task_planning_msgs.msg.ReleaseObjectAction)
    self.pickplace_planning_client = actionlib.SimpleActionClient('/pick_place_planning', o2ac_task_planning_msgs.msg.PickPlaceWithRegraspAction)
    self.fastening_planning_client = actionlib.SimpleActionClient('/fastening_planning', o2ac_task_planning_msgs.msg.PlaceObjectAction)
    self.wrs_subtask_b_planning_client = actionlib.SimpleActionClient('/wrs_subtask_b_planning', o2ac_task_planning_msgs.msg.PickPlaceWithRegraspAction)
    
    # Subscribers
    # "robot_program_running" refers only to the ROS external control UR script, not just any program
    self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
    self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
    self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)
    
    # self.my_mutex = threading.Lock()

    self.resetTimerForDebugMonitor_client = rospy.ServiceProxy('/o2ac_debug_monitor/reset_timer', std_srvs.srv.Trigger)
    self.debugmonitor_publishers = dict() # used in log_to_debug_monitor()

    self.screw_tools = {}
    self.define_tool_collision_objects()

    self.objects_in_tray = dict()  # key: object ID. value: False or object pose

    rospy.sleep(.5)
    rospy.loginfo("Finished initializing class")
    
  ############## ------ Internal functions (and convenience functions)

  def confirm_to_proceed(self, next_task_name):
    if self.competition_mode:
      return True
    rospy.loginfo("Press enter to proceed to: " + next_task_name)
    i = raw_input()
    if i == "":
      if not rospy.is_shutdown():
        return True
    raise Exception("User caused exit!")
    return False

  def run_mode_callback(self, msg):
    self.run_mode_ = msg.data
  def pause_mode_callback(self, msg):
    self.pause_mode_ = msg.data
  def test_mode_callback(self, msg):
    self.test_mode_ = msg.data

  def get_robot_status_from_param_server(self):
    robot_status = dict()
    for robot in ["a_bot", "b_bot"]:
      robot_status[robot] = self.active_robots[robot].get_status_from_param_server()
    return robot_status
  
  def publish_robot_status(self):
    for robot in ["a_bot", "b_bot"]:
      self.active_robots[robot].publish_robot_status()

  def reset_scene_and_robots(self):
    self.a_bot.robot_status = o2ac_msgs.msg.RobotStatus()
    self.b_bot.robot_status = o2ac_msgs.msg.RobotStatus()
    self.planning_scene_interface.remove_attached_object()  # Detach objects
    self.planning_scene_interface.remove_world_object()  # Clear all objects
    self.publish_robot_status()

  def activate_led(self, LED_name="b_bot", on=True):
    req = ur_msgs.srv.SetIORequest()
    if LED_name == "b_bot":
      req.fun = ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT
      req.pin = 4
      if on:
        req.state = ur_msgs.srv.SetIORequest.STATE_ON
      else:
        req.state = ur_msgs.srv.SetIORequest.STATE_OFF
      return self.b_bot.set_io.call(req)
    elif LED_name == "a_bot":
      return self.a_bot.set_io.call(req)
    else:
      rospy.logerr("Invalid LED name")
  
  def unlock_base_plate(self):
    self.set_base_lock(closed=False)
  def lock_base_plate(self):
    self.set_base_lock(closed=True)
  def set_base_lock(self, closed=True):
    req_1 = ur_msgs.srv.SetIORequest()
    req_1.fun = ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT
    req_2 = copy.deepcopy(req_1)
    req_1.state = ur_msgs.srv.SetIORequest.STATE_OFF
    req_2.state = ur_msgs.srv.SetIORequest.STATE_ON
    if closed:
      req_1.pin = 3   # base_retract
      req_2.pin = 2   # base_extend
    else:
      req_1.pin = 2   
      req_2.pin = 3
    self.b_bot.set_io.call(req_1)
    self.b_bot.set_io.call(req_2)
    return
  
  def define_tool_collision_objects(self):
    PRIMITIVES = {"BOX": SolidPrimitive.BOX, "CYLINDER": SolidPrimitive.CYLINDER}

    path = rospkg.RosPack().get_path("o2ac_assembly_database") + "/config/tool_collisions.yaml"
    with open(path, 'r') as f:
      tools = yaml.load(f)

    for tool_key, tool in tools.items():
      
      tool_co = moveit_msgs.msg.CollisionObject()
      tool_co.header.frame_id = tool["frame_id"]
      tool_co.id = tool["id"]

      primitive_num = len(tool['primitives'])

      tool_co.primitives = [SolidPrimitive() for _ in range(primitive_num)] # instead of resize()
      tool_co.primitive_poses = [Pose() for _ in range(primitive_num)] 

      for i, primitive in enumerate(tool["primitives"]):
        try:
          tool_co.primitives[i].type = PRIMITIVES[(primitive['type'])]
        except KeyError as e:
          rospy.logerr("Invalid Collition Object Primitive type: %s " % primitive['type'])
          raise
        tool_co.primitives[i].dimensions = primitive['dimensions']
        tool_co.primitive_poses[i] = conversions.to_pose(conversions.to_float(primitive['pose']))
      
      tool_co.operation = tool_co.ADD

      # TODO(Cambel): - Fix warning of empty quaternion
      tool_co.subframe_poses = [conversions.to_pose(conversions.to_float(tool["subframe"]["pose"]))]
      tool_co.subframe_names = [tool["subframe"]["name"]]

      self.screw_tools[tool["id"]] = tool_co

    # # TODO(felixvd): Add the set screw and nut tool objects from the C++ file

  ############# ------ Robot motion functions

  def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 0.5, acceleration = 0.25,
                      end_effector_link = "", move_lin = True):
    robot = self.active_robots[group_name]
    if rospy.is_shutdown():
      return False
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit
    if move_lin:
      return self.move_lin(group_name, pose_goal_stamped, speed, acceleration, end_effector_link)
    self.skill_server.publish_marker(pose_goal_stamped, "pose")
    self.activate_ros_control_on_ur(group_name)
    group = robot.groups[group_name]
    
    if acceleration > speed:
      rospy.logwarn("Setting acceleration to " + str(speed) + " instead of " + str(acceleration) + " to avoid jerky motion.")
      acceleration = speed

    if not end_effector_link:
      if group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_robotiq_85_tip_link"
    group.set_end_effector_link(end_effector_link)
    
    group.set_pose_target(pose_goal_stamped)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    group.set_max_acceleration_scaling_factor(acceleration)
    group.set_planning_pipeline_id("ompl")
    group.set_planner_id("RRTConnect")

    move_success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01), move_success

  def move_lin(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.5, end_effector_link = ""):
    if rospy.is_shutdown():
      return False
    self.skill_server.publish_marker(pose_goal_stamped, "pose")
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit

    if not end_effector_link:
      if group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_robotiq_85_tip_link"

    if self.force_ur_script_linear_motion and self.use_real_robot and group_name == "b_bot":
      if not self.force_moveit_linear_motion:
        return self.skill_server.move_lin(group_name, pose_goal_stamped, end_effector_link, speed, acceleration, self.listener)

    if speed > 1.0:
      speed = 1.0
    if acceleration > speed:
      rospy.logwarn("Setting acceleration to " + str(speed) + " instead of " + str(acceleration) + " to avoid jerky motion.")
      acceleration = speed
    
    self.active_robots[group_name].activate_ros_control_on_ur()
    group = self.active_robots[group_name].groups[group_name]

    group.set_end_effector_link(end_effector_link)
    pose_goal_world = self.listener.transformPose("world", pose_goal_stamped)
    group.set_pose_target(pose_goal_world)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    group.set_max_acceleration_scaling_factor(acceleration)
    
    group.set_planning_pipeline_id("pilz_industrial_motion_planner")
    group.set_planner_id("LIN")
    
    success = group.go(wait=True)  # Bool
    group.stop()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    return success

  def move_lin_rel(self, robot_name, relative_translation = [0,0,0], relative_rotation = [0,0,0], acceleration = 0.5, velocity = .03, use_robot_base_csys=False, wait = True, max_wait=30.0):
    '''
    Does a lin_move relative to the current position of the robot.
    Uses the robot's TCP if using UR script, or the frame of the 

    robot_name = "b_bot" for example
    relative_translation: translatory movement relative to current tcp position, expressed in robot's own base frame
    relative_rotation: rotatory movement relative to current tcp position, expressed in robot's own base frame
    use_robot_base_csys: If true, uses the robot_base coordinates for the relative motion (not workspace_center!)
    '''
    if rospy.is_shutdown():
      return False
    # Uses UR coordinates
    if not self.use_real_robot:
      return True
    if self.force_ur_script_linear_motion:
      # Directly calls the UR service
      req = o2ac_msgs.srv.sendScriptToURRequest()
      req.robot_name = robot_name
      req.relative_translation.x = relative_translation[0]
      req.relative_translation.y = relative_translation[1]
      req.relative_translation.z = relative_translation[2]
      req.relative_rotation.x = relative_rotation[0]
      req.relative_rotation.y = relative_rotation[1]
      req.relative_rotation.z = relative_rotation[2]
      req.acceleration = acceleration
      req.velocity = velocity
      req.lin_move_rel_in_base_csys = use_robot_base_csys
      req.program_id = "lin_move_rel"
      res = self.urscript_client.call(req)
      if wait:
        rospy.sleep(1.0)
        wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(max_wait))
      return res.success
    
    group = self.active_robots[robot_name].groups[robot_name]
    # TODO: use use_robot_base_csys parameter
    new_pose1 = group.get_current_pose()
    new_pose2 = group.get_current_pose()
    if pose_dist(new_pose1.pose, new_pose2.pose) > 0.002:
      # This is guarding against a weird error that seems to occur with get_current_pose sometimes
      rospy.logerr("get_current_pose gave two different results!!")
      rospy.logwarn("pose1: ")
      rospy.logwarn(new_pose1.pose)
      rospy.logwarn("pose2: ")
      rospy.logwarn(new_pose2.pose)
    
    new_pose2.pose.position.x += relative_translation[0]
    new_pose2.pose.position.y += relative_translation[1]
    new_pose2.pose.position.z += relative_translation[2]
    new_pose2.pose.orientation = rotateQuaternionByRPY(relative_rotation[0], relative_rotation[1], 
                                                        relative_rotation[2], new_pose2.pose.orientation)
    return self.move_lin(robot_name, new_pose2, speed = velocity, acceleration = acceleration)

  def move_joints(self, group_name, joint_pose_goal, speed = 1.0, acceleration = 0.5, force_ur_script=False, force_moveit=False):
    if rospy.is_shutdown():
      return False
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit
    if force_ur_script and self.use_real_robot:
      if not force_moveit:
        return self.skill_server.move_joints(group_name, joint_pose_goal, speed, acceleration)
    
    if speed > 1.0:
      speed = 1.0
    self.active_robots[group_name].activate_ros_control_on_ur()
    self.active_robots[group_name].groups[group_name].set_joint_value_target(joint_pose_goal)
    self.active_robots[group_name].groups[group_name].set_max_velocity_scaling_factor(speed)
    self.active_robots[group_name].groups[group_name].set_planning_pipeline_id("ompl")
    self.active_robots[group_name].groups[group_name].set_planner_id("RRTConnect")
    return self.active_robots[group_name].groups[group_name].go(wait=True)

  def move_both_robots(self, pose_goal_a_bot, pose_goal_b_bot, speed = 0.05):
    if rospy.is_shutdown():
      return False
    if self.pause_mode_ or self.test_mode_:
      if speed > .25:
        rospy.loginfo("Reducing speed from " + str(speed) + " to .25 because robot is in test or pause mode")
        speed = .25
    rospy.logwarn("CAUTION: Moving front bots together, but MoveIt does not do continuous collision checking.")
    group = self.groups["front_bots"]
    group.set_pose_target(pose_goal_a_bot, end_effector_link="a_bot_robotiq_85_tip_link")
    group.set_pose_target(pose_goal_b_bot, end_effector_link="b_bot_robotiq_85_tip_link")
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    group.set_max_acceleration_scaling_factor(acceleration)
    group.set_planning_pipeline_id("ompl")
    group.set_planner_id("RRTConnect")

    success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    rospy.loginfo("Received:")
    rospy.loginfo(success)
    return success

    # =====

  def go_to_named_pose(self, pose_name, robot_name, speed = 0.5, acceleration = 0.5, force_ur_script=False):
    """
    pose_name should be a named pose in the moveit_config, such as "home", "back" etc.
    """
    if rospy.is_shutdown():
      return False
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit
    if force_ur_script and self.use_real_robot:
      d = self.groups[robot_name].get_named_target_values(pose_name)
      if not "panda" in d.keys()[0]:  # This avoids the panda being activated  
        joint_pose = [d[robot_name+"_shoulder_pan_joint"], 
                      d[robot_name+"_shoulder_lift_joint"],
                      d[robot_name+"_elbow_joint"],
                      d[robot_name+"_wrist_1_joint"],
                      d[robot_name+"_wrist_2_joint"],
                      d[robot_name+"_wrist_3_joint"]]
        return self.move_joints(robot_name, joint_pose, speed, acceleration, force_ur_script=force_ur_script)
    if speed > 1.0:
      speed = 1.0
    robot = self.active_robots[robot_name]
    robot.activate_ros_control_on_ur()
    robot.groups[robot_name].set_named_target(pose_name)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    robot.groups[robot_name].set_max_velocity_scaling_factor(speed)
    robot.groups[robot_name].set_planning_pipeline_id("ompl")
    robot.groups[robot_name].set_planner_id("RRTConnect")
    move_success = robot.groups[robot_name].go(wait=True)
    # self.active_robots[group_name].groups[robot_name].stop()
    robot.groups[robot_name].clear_pose_targets()
    return move_success

  ######
  def pick_screw_from_feeder(self, robot_name, screw_size, realign_tool_upon_failure=False):
    return self.skill_server.pick_screw_from_feeder(robot_name, screw_size, realign_tool_upon_failure)

  def do_insert_action(self, active_robot_name, passive_robot_name = "", 
                        starting_offset = 0.05, max_insertion_distance=0.01, 
                        max_approach_distance = .1, max_force = 5,
                        max_radius = .001, radius_increment = .0001):
    rospy.logerr("This is probably not implemented. Aborting")
    return
    goal = o2ac_msgs.msg.insertGoal()
    goal.active_robot_name = active_robot_name
    goal.passive_robot_name = passive_robot_name
    goal.starting_offset = starting_offset
    goal.max_insertion_distance = max_insertion_distance
    goal.max_approach_distance = max_approach_distance
    goal.max_force = max_force
    goal.max_radius = max_radius
    goal.radius_increment = radius_increment
    rospy.loginfo("Sending insert action goal.")    
    self.insert_client.send_goal(goal)
    self.insert_client.wait_for_result()
    return self.insert_client.get_result()

  def do_change_tool_action(self, robot_name, equip=True, screw_size = 4):
    if screw_size == 2:
      tool_name = "set_screw_tool"
    else:
      tool_name = "screw_tool_m" + str(screw_size)

    if equip:
      res = self.equip_tool(robot_name, tool_name)
    else:
      res = self.unequip_tool(robot_name, tool_name)
    return res
  
  def upload_tool_grasps_to_param_server(self, tool_id):
    transformer = tf.Transformer(True, rospy.Duration(10.0))

    rospy.sleep(0.2)
    (trans,rot) = self.listener.lookupTransform('/screw_tool_' + tool_id + '_pickup_link', '/move_group/screw_tool_' + tool_id, rospy.Time())
    collision_object_to_pickup_link = geometry_msgs.msg.TransformStamped()
    collision_object_to_pickup_link.header.frame_id = 'screw_tool_' + tool_id + '_pickup_link'
    collision_object_to_pickup_link.child_frame_id = 'screw_tool_' + tool_id
    collision_object_to_pickup_link.transform.translation = geometry_msgs.msg.Vector3(*trans)
    collision_object_to_pickup_link.transform.rotation = geometry_msgs.msg.Quaternion(*rot)
    transformer.setTransform(collision_object_to_pickup_link)

    grasp_pose_to_pickup_link = geometry_msgs.msg.TransformStamped()
    grasp_pose_to_pickup_link.header.frame_id = 'screw_tool_' + tool_id + '_pickup_link'
    grasp_pose_to_pickup_link.child_frame_id = 'grasp_1'
    grasp_pose_to_pickup_link.transform.translation = geometry_msgs.msg.Vector3(0.015,0.0,-0.03)
    grasp_pose_to_pickup_link.transform.rotation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))
    transformer.setTransform(grasp_pose_to_pickup_link)

    (trans,rot) = transformer.lookupTransform('screw_tool_' + tool_id, 'grasp_1', rospy.Time(0))
    rospy.set_param('tools/screw_tool_' + tool_id + '/grasp_1/position', trans)
    rospy.set_param('tools/screw_tool_' + tool_id + '/grasp_1/orientation', rot)

  def spawn_multiple_objects(self, assembly_name, objects, poses, reference_frame):
    # Init params
    # TODO(Cambel): Fix redundant calls to this definitions params and tool object collitions
    upload_mtc_modules_initial_params()
    self.assembly_database.change_assembly(assembly_name)

    # Spawn tools and objects
    self.define_tool_collision_objects()
    screw_ids = ['m3', 'm4']
    for screw_id in screw_ids:
      self.spawn_tool('screw_tool_' + screw_id)
      self.upload_tool_grasps_to_param_server(screw_id)
    # spawn_objects(self.assembly_database, objects, poses, reference_frame)
  
  def get_3d_poses_from_ssd(self):
    """
    Returns object poses as estimated by the SSD neural network and reprojection.
    Also updates self.objects_in_tray
    """
    # Read result and return
    try:
      rospy.logwarn("Clearing all object poses in memory")
      self.objects_in_tray = dict()
      res = self.vision.read_from_sdd()
      for idx, pose in zip(res.class_ids, res.poses):
        self.objects_in_tray[idx] = pose
      return res
    except:
      pass
    return False
  
  def get_bearing_angle(self, camera="b_bot_inside_camera"):
    """
    When looking at the bearing from the front, returns the rotation angle 
    to align the screw holes.
    """
    return self.vision.get_angle_from_vision(camera, item_name="bearing")
  
  def get_motor_angle(self, camera="b_bot_outside_camera"):
    """
    When looking at the motor in the vgroove, this returns the rotation angle.
    """
    return self.vision.get_angle_from_vision(camera, item_name="motor")

  def detect_object_in_camera_view(self, item_name):
    """
    Returns object pose if object was detected in current camera view and published to planning scene,
    False otherwise.
    """
    # TODO: merge with "look_and_get_grasp_points"

    # Send goal, wait for result
    object_type = self.assembly_database.name_to_type(item_name)
    if not object_type:
      rospy.logerr("Could not find the object " + item_name + " in database, or its type field is empty.")
      return False
    self.localization_client.send_goal(o2ac_msgs.msg.localizeObjectGoal(item_id=object_type))
    if (not self.localization_client.wait_for_result(rospy.Duration(15.0))):
      self.localization_client.cancel_goal()  # Cancel goal if timeout expired
      rospy.logerr("Localization node returned no result.")
      
    # Read result and publish to planning scene as collision_object if found
    success = False
    try:
      res = self.localization_client.get_result()
      success = res.succeeded
    except:
      pass
    
    # Publish to planning scene
    if success:
      rospy.loginfo("Detected " + item_name + " with confidence " + str(res.confidences[0]))
      co = self.assembly_database.get_collision_object(object_name=item_name)
      # TODO: Update MoveIt to use the object_pose
      # p_1 = copy.deepcopy(res.detected_poses[0].pose)
      # p_2 = copy.deepcopy(res.detected_poses[0].pose)
      # p_3 = copy.deepcopy(res.detected_poses[0].pose)
      # p_2 = 
      print(res.detected_poses[0])
      co.pose = res.detected_poses[0].pose
      co.header.frame_id = res.detected_poses[0].header.frame_id

      self.planning_scene_interface.apply_collision_object(co)
      return res.detected_poses[0]
    else:
      rospy.loginfo("Did not detect " + item_name)
      return False

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

  @save_task_plan
  def do_plan_pick_action(self, object_name, grasp_parameter_location = '', lift_direction_reference_frame = '', lift_direction = [], robot_name = ''):
    '''
    Function for calling the action for pick planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = o2ac_task_planning_msgs.msg.PickObjectGoal()
    goal.object_name = object_name
    goal.grasp_parameter_location = grasp_parameter_location
    goal.lift_direction_reference_frame = lift_direction_reference_frame
    goal.lift_direction = lift_direction
    goal.robot_name = robot_name
    rospy.loginfo("Sending pick planning goal.")
    self.pick_planning_client.send_goal(goal)
    self.pick_planning_client.wait_for_result()
    return self.pick_planning_client.get_result()

  @save_task_plan
  def do_plan_place_action(self, object_name, object_target_pose, release_object_after_place = True, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = []):
    '''
    Function for calling the action for place planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.PlaceObjectGoal()
    goal.object_name = object_name
    goal.release_object_after_place = release_object_after_place
    goal.object_target_pose = object_target_pose
    goal.object_subframe_to_place = object_subframe_to_place
    goal.approach_place_direction_reference_frame = approach_place_direction_reference_frame
    goal.approach_place_direction = approach_place_direction
    rospy.loginfo("Sending place planning goal.")
    self.place_planning_client.send_goal(goal)
    self.place_planning_client.wait_for_result()
    return self.place_planning_client.get_result()

  @save_task_plan
  def do_plan_release_action(self, object_name, pose_to_retreat_to = ''):
    '''
    Function for calling the action for release planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.ReleaseObjectGoal()
    goal.object_name = object_name
    goal.pose_to_retreat_to = pose_to_retreat_to
    rospy.loginfo("Sending release planning goal.")
    self.release_planning_client.send_goal(goal)
    self.release_planning_client.wait_for_result()
    return self.release_planning_client.get_result()

  @save_task_plan
  def do_plan_pickplace_action(self, object_name, object_target_pose, grasp_parameter_location = '', release_object_after_place = True, object_subframe_to_place = '',
    lift_direction_reference_frame = '', lift_direction = [], approach_place_direction_reference_frame = '', approach_place_direction = [], robot_names = '', force_robot_order = False):
    '''
    Function for calling the action for pick-place (potentially with regrasp) planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.PickPlaceWithRegraspGoal()
    goal.object_name = object_name
    goal.object_target_pose = object_target_pose
    goal.grasp_parameter_location = grasp_parameter_location
    goal.release_object_after_place = release_object_after_place
    goal.object_subframe_to_place = object_subframe_to_place
    goal.lift_direction_reference_frame = lift_direction_reference_frame
    goal.lift_direction = lift_direction
    goal.approach_place_direction_reference_frame = approach_place_direction_reference_frame
    goal.approach_place_direction = approach_place_direction
    goal.robot_names = robot_names
    goal.force_robot_order = force_robot_order
    rospy.loginfo("Sending pickplace planning goal.")
    self.pickplace_planning_client.send_goal(goal)
    self.pickplace_planning_client.wait_for_result()
    return self.pickplace_planning_client.get_result()


  @save_task_plan
  def do_plan_fastening_action(self, object_name, object_target_pose, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = []):
    '''
    Function for calling the action for fastening planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.PlaceObjectGoal()
    goal.object_name = object_name
    goal.object_target_pose = object_target_pose
    goal.object_subframe_to_place = object_subframe_to_place
    goal.approach_place_direction_reference_frame = approach_place_direction_reference_frame
    goal.approach_place_direction = approach_place_direction
    rospy.loginfo("Sending fastening planning goal.")
    self.fastening_planning_client.send_goal(goal)
    self.fastening_planning_client.wait_for_result()
    return self.fastening_planning_client.get_result()

  @save_task_plan
  def do_plan_wrs_subtask_b_action(self, object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame = '', approach_place_direction = []):
    '''
    Function for calling the action for subassembly (fixing the motor L plate on the base plate) planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.PickPlaceWithRegraspGoal()
    goal.object_name = object_name
    goal.object_target_pose = object_target_pose
    goal.object_subframe_to_place = object_subframe_to_place
    goal.approach_place_direction_reference_frame = approach_place_direction_reference_frame
    goal.approach_place_direction = approach_place_direction
    rospy.loginfo("Sending wrs subtask B planning goal.")
    self.wrs_subtask_b_planning_client.send_goal(goal)
    self.wrs_subtask_b_planning_client.wait_for_result()
    return self.wrs_subtask_b_planning_client.get_result()

  def spawn_tool(self, tool_name):
    if tool_name == "screw_tool_m3" or tool_name == "screw_tool_m4": 
      rospy.loginfo("Spawn: " + tool_name)
      self.planning_scene_interface.add_object(self.screw_tools[tool_name])
      return True
    else:
      rospy.logerr("Cannot spawn tool: " + tool_name)
      return False

  def despawn_tool(self, tool_name):
    if tool_name == "screw_tool_m3" or tool_name == "screw_tool_m4": 
      rospy.loginfo("Despawn: " + tool_name)
      self.planning_scene_interface.remove_world_object(self.screw_tools[tool_name].id)
      return True
    else:
      rospy.logerr("Cannot despawn tool: " + tool_name)
      return False

  def attach_tool(self, robot_name, toolname):
    try:
      self.active_robots[robot_name].robot_group.attach_object(toolname, robot_name + "_ee_link", touch_links= 
      [robot_name + "_robotiq_85_tip_link", 
      robot_name + "_robotiq_85_left_finger_tip_link", 
      robot_name + "_robotiq_85_left_inner_knuckle_link", 
      robot_name + "_robotiq_85_right_finger_tip_link", 
      robot_name + "_robotiq_85_right_inner_knuckle_link"])
    except:
      rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

  def detach_tool(self, robot_name, toolname):
    try:
      self.active_robots[robot_name].robot_group.detach_object(toolname)
    except:
      rospy.logerr(item_id_to_attach + " could not be detached! robot_name = " + robot_name)

  def equip_tool(self, robot_name, tool_name):
    return self.equip_unequip_realign_tool(robot_name, tool_name, "equip")
  
  def unequip_tool(self, robot_name, tool_name=""):
    if tool_name == "":
      tool_name = self.robot_status[robot_name].held_tool_id
    return self.equip_unequip_realign_tool(robot_name, tool_name, "unequip")
  
  def realign_tool(self, robot_name, screw_tool_id):
    """
    Goes to the tool holder and regrasps the tool to fix its orientation.
    This can be called when an operation fails and the tool has probably rotated in the gripper.
    """
    return self.equip_unequip_realign_tool(robot_name, screw_tool_id, "realign")

  def equip_unequip_realign_tool(self, robot_name, tool_name, operation):
    """
    operation can be "equip", "unequip" or "realign".
    """
    robot = self.active_robots[robot_name]
    if tool_name == "":
      tool_name = self.robot_status[robot_name].held_tool_id


    # Sanity check on the input instruction
    equip = (operation == "equip")
    unequip = (operation == "unequip")
    # if equip or unequip:
    #   raise NotImplementedError("Equip/unequip needs to be called via the C++ skill server instead")
    realign = (operation == "realign")

    ###
    lin_speed = 0.5
    # The second comparison is not always necessary, but readability comes first.
    if ((not equip) and (not unequip) and (not realign)):
      rospy.logerr("Cannot read the instruction " + operation + ". Returning False.")
      return False

    if ((robot.robot_status.carrying_object == True)):
      rospy.logerr("Robot holds an object. Cannot " + operation + " tool.")
      return False
    if ((robot.robot_status.carrying_tool == True) and equip):
      rospy.logerr("Robot already holds a tool. Cannot equip another.")
      return False
    if ((robot.robot_status.carrying_tool == False) and unequip):
      rospy.logerr("Robot is not holding a tool. Cannot unequip any.")
      return False
    
    rospy.loginfo("Going to before_tool_pickup pose.")
    
    tool_holder_used = "back"
    if tool_name in ["belt_tool", "plunger_tool"]:
      tool_holder_used = "front"
    
    if tool_holder_used == "back":
      if not self.go_to_named_pose("tool_pick_ready", robot_name):
        rospy.logerr("Could not plan to before_tool_pickup joint state. Abort!")
        return False
    elif tool_holder_used == "back":
      if not self.go_to_named_pose("tool_pick_ready", robot_name):
        rospy.logerr("Could not plan to before_tool_pickup joint state. Abort!")
        return False

    # Set up poses
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_move_away = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = tool_name + "_pickup_link"

    # Define approach pose
    # z = 0 is at the holder surface, and z-axis of pickup_link points downwards!
    rospy.loginfo("tool_name: " + tool_name)
    if tool_name == "screw_tool_m3" or tool_name == "screw_tool_m4":
      ps_approach.pose.position.x = -.05
      ps_approach.pose.position.z = -.008
    elif tool_name == "nut_tool_m6":
      ps_approach.pose.position.z = -.025
    elif tool_name == "set_screw_tool":
      ps_approach.pose.position.z = -.01
    elif tool_name == "belt_tool":
      ps_approach.pose.position.z = -.025
    elif tool_name == "plunger_tool":
      ps_approach.pose.position.z = -.025
    else:
      rospy.logerr(tool_name, " is not implemented!")
      return False

    # Go to named pose, then approach
    self.go_to_named_pose("tool_pick_ready", robot_name)

    ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/6, 0))
    ps_move_away = copy.deepcopy(ps_approach)

    # Define pickup pose
    ps_in_holder = copy.deepcopy(ps_approach)
    ps_in_holder.pose.position.x = .017
    if tool_name == "nut_tool_m6":
      ps_in_holder.pose.position.x = 0.01
    elif tool_name == "set_screw_tool":
      ps_in_holder.pose.position.x = 0.02

    if unequip or realign:
      ps_in_holder.pose.position.x -= 0.001   # Don't move all the way into the magnet to place
      ps_approach.pose.position.z -= 0.01 # Approach diagonally so nothing gets stuck

    if equip:
      robot.gripper.open()
      rospy.loginfo("Spawning tool.")
      if not self.spawn_tool(tool_name):
        rospy.logwarn("Could not spawn the tool. Continuing.")
      held_screw_tool_ = tool_name

    rospy.loginfo("Moving to screw tool approach pose LIN.")
    self.go_to_pose_goal(robot_name, ps_approach, speed=lin_speed, acceleration=lin_speed/2, move_lin=True)
  
    # Plan & execute linear motion to the tool change position
    rospy.loginfo("Moving to pose in tool holder LIN.")
    if equip:
      lin_speed = 0.5
    elif unequip:
      lin_speed = 0.5
      self.planning_scene_interface.allow_collisions("screw_tool_holder_long", tool_name)
    elif realign:
      lin_speed = 0.5

    self.go_to_pose_goal(robot_name, ps_in_holder, speed=lin_speed, acceleration=lin_speed/2, move_lin=True)
  
    # Close gripper, attach the tool object to the gripper in the Planning Scene.
    # Its collision with the parent link is set to allowed in the original planning scene.
    if equip:
      robot.gripper.close()
      self.attach_tool(robot_name, tool_name)
      self.allow_collisions_with_robot_hand(tool_name, robot_name)
      self.allow_collisions_with_robot_hand('screw_tool_holder', robot_name)  # TODO(felixvd): Is this required?
      robot.robot_status.carrying_tool = True
      robot.robot_status.held_tool_id = tool_name
      self.publish_robot_status()
    elif unequip:
      robot.gripper.open()
      self.detach_tool(robot_name, tool_name)
      self.allow_collisions_with_robot_hand(tool_name, robot_name, allow=False)
      held_screw_tool_ = ""
      robot.robot_status.carrying_tool = False
      robot.robot_status.held_tool_id = ""
      self.publish_robot_status()
    elif realign: # 
      robot.gripper.open()
      robot.gripper.close()
      pull_back_slightly = copy.deepcopy(ps_in_holder)
      pull_back_slightly.pose.position.x -= 0.003
      ps_in_holder.pose.position.x += 0.001  # To remove the offset for placing applied earlier
      lin_speed = 0.02
      self.go_to_pose_goal(robot_name, pull_back_slightly, speed=lin_speed, move_lin=True)
      robot.gripper.open()
      self.go_to_pose_goal(robot_name, ps_in_holder, speed=lin_speed, move_lin=True)
      robot.gripper.close()
    
    # Plan & execute linear motion away from the tool change position
    rospy.loginfo("Moving back to screw tool approach pose LIN.")
    
    lin_speed = 0.8

    self.go_to_pose_goal(robot_name, ps_move_away, speed=lin_speed, acceleration=lin_speed/2, move_lin=True)

    
    # Reactivate the collisions, with the updated entry about the tool
    # planning_scene_interface_.applyPlanningScene(planning_scene_)
    self.go_to_named_pose("tool_pick_ready", robot_name)
    
    return True

  def allow_collisions_with_robot_hand(self, link_name, robot_name, allow=True):
      """Allow collisions of a link with the robot hand"""
      hand_links = [
        robot_name + "_tip_link",
        robot_name + "_left_inner_finger_pad",
        robot_name + "_right_inner_finger_pad",
        robot_name + "_left_inner_finger",
        robot_name + "_right_inner_finger"
      ]
      for hand_link in hand_links:
        if allow:
          self.planning_scene_interface.allow_collisions(link_name, hand_link)
        else:
          self.planning_scene_interface.disallow_collisions(link_name, hand_link)
      return

  def playback_sequence(self, routine_filename):
    path = rospkg.RosPack().get_path("o2ac_routines") + ("/config/playback_sequences/%s.yaml" % routine_filename)
    with open(path, 'r') as f:
      routine = yaml.load(f)
    robot_name = routine["robot_name"]
    waypoints = routine["waypoints"]

    for i, point in enumerate(waypoints):
      print("point:", i+1)
      # raw_input()
      pose = point['pose']
      pose_type = point['type']
      gripper_action = point.get('gripper-action')
      duration = point['duration']
      self.move_to_sequence_waypoint(robot_name, pose, pose_type, gripper_action, duration)

  def move_to_sequence_waypoint(self, robot_name, pose, pose_type, gripper_action, duration):
    robot = self.active_robots[robot_name]
    group = robot.robot_group
    if robot_name == 'a_bot':
      arm = self.a_bot_compliant_arm
    elif robot_name == 'b_bot':
      arm = self.b_bot_compliant_arm
    else:
      raise Exception('Unsupported arm %s' % robot_name)


    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot_name + "_base_link"

    if pose_type == 'joint-space':
      arm.set_joint_positions(pose, wait=True, t=duration)
    elif pose_type == 'joint-space-goal-cartesian-lin-motion':
      target_pose = arm.end_effector(pose)  # Forward kinematics
      p.pose = conversions.to_pose(target_pose)
      self.move_lin(robot_name, p, speed=0.8)
    elif pose_type == 'task-space':
      p.pose = conversions.to_pose(pose)
      self.move_lin(robot_name, pose, speed=0.8)
    elif pose_type == 'relative-tcp':
      pass
      # self.move_lin_rel(robot_name, relative_translation=pose[:3], relative_rotation=pose[3:])
    elif pose_type == 'relative-base':
      pass
      # self.move_lin_rel(robot_name, relative_translation=pose[:3], relative_rotation=pose[3:], use_robot_base_csys=True)
    else:
      raise ValueError("Invalid pose_type: %s" % pose_type)

    if gripper_action:
      if gripper_action == 'open':
        robot.gripper.open()
      elif gripper_action == 'close':
        robot.gripper.close(force=100., velocity=0.01)
      elif gripper_action == 'close-open':
        robot.gripper.close(velocity=0.01)
        robot.gripper.open()
######

  def start_task_timer(self):
    """Reset timer in debug monitor"""
    try:
      _ = self.resetTimerForDebugMonitor_client.call()
    except:
      pass
  
  def log_to_debug_monitor(self, text, category):
    """Send message to rospy.loginfo and debug monitor.

    The topic name should be included in the parameter server. See test.launch in o2ac_debug_monitor.
    """
    rospy.loginfo(category + ": " + text)

    topic_name = "/o2ac_state/{}".format(category)
    if topic_name not in self.debugmonitor_publishers:
      pub = rospy.Publisher(topic_name, String, Bool, queue_size=1)
      rospy.sleep(0.5)
      self.debugmonitor_publishers[topic_name] = pub
    else:
      pub = self.debugmonitor_publishers[topic_name]

    msg = String, Bool()
    msg.data = text
    pub.publish(msg)


