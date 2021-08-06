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

from math import radians
import os
import sys
import copy

import numpy
from moveit_commander import robot
from o2ac_routines import helpers
from o2ac_routines.helpers import save_task_plan
from o2ac_routines.dual_arm import DualArm
import rospy
import rospkg
import tf_conversions
import tf 
import actionlib
from actionlib_msgs.msg import GoalStatus

import yaml
import pickle

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_srvs.srv
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

import o2ac_msgs.msg
import o2ac_task_planning_msgs.msg

from std_msgs.msg import String, Bool

from o2ac_assembly_database.assembly_reader import AssemblyReader
from ur_control.traj_utils import compute_trajectory

import ur_msgs.srv
from o2ac_routines.helpers import *

from o2ac_routines.skill_server_client import SkillServerClient
from o2ac_routines.vision_client import VisionClient
from o2ac_routines.ur_robot import URRobot
from o2ac_routines.tools import Tools


class AssemblyStatus(object):
  """ A helper class containing booleans describing the state of the assembly.
      May as well be a dictionary, but this makes auto-completion easier.

      This should help when restarting.
  """
  def __init__(self):
    # rospy.get_param("/last_assembly_status", False)
    self.tray_placed_on_table = False

    self.belt_placed_outside_of_tray = False
    self.motor_placed_outside_of_tray = False
    self.bearing_placed_outside_of_tray = False

    self.completed_subtask_zero = False  # Base
    self.completed_subtask_a = False  # Motor
    self.completed_subtask_b = False  # Motor pulley
    self.completed_subtask_c1 = False  # Bearing
    self.completed_subtask_c2 = False  # Shaft
    self.completed_subtask_d = False  # Fasten output pulley
    self.completed_subtask_e = False  # Output pulley
    self.completed_subtask_f = False  # Motor plate
    self.completed_subtask_g = False  # Bearing plate
    self.completed_subtask_h = False  # Belt
    self.completed_subtask_i1 = False  # Cable 1
    self.completed_subtask_i2 = False  # Cable 2
  


class O2ACBase(object):
  """
  This class contains the basic helper and convenience functions used in the routines.
  The basic functions include the initialization of the services and actions,
  and shorthand functions for the most common actions.
  """
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)

    self.listener = tf.TransformListener()

    # Status variables and settings
    self.use_real_robot = rospy.get_param("use_real_robot", False)
    self.force_ur_script_linear_motion = False
    self.force_moveit_linear_motion = True
    self.use_dummy_vision = False  # If True, avoids using the cameras and returns dummy values

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
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface(synchronous=True)
    
    self.assembly_database = AssemblyReader()
    self.assembly_status = AssemblyStatus()
    # TODO: Get current AssemblyStatus from param server

    # Action clients and movegroups
    self.a_bot = URRobot("a_bot", self.listener)
    self.b_bot = URRobot("b_bot", self.listener)
    self.ab_bot = DualArm("ab_bot", self.a_bot, self.b_bot, self.listener)
    # For compatibility let's wrap the robots
    self.active_robots = {'a_bot': self.a_bot, 'b_bot': self.b_bot, 'ab_bot': self.ab_bot}
    
    for robot in ["a_bot", "b_bot"]:
      self.active_robots[robot].get_status_from_param_server()

    self.skill_server = SkillServerClient()
    self.vision = VisionClient()
    self.tools = Tools()

    self.pick_place_planning_client = actionlib.SimpleActionClient('plan_pick_place', moveit_task_constructor_msgs.msg.PlanPickPlaceAction)
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

    # Publisher for status text
    self.pub_status_text = rospy.Publisher("/o2ac_text_to_image", String, queue_size=1)
    
    self.screw_tools = {}
    self.define_tool_collision_objects()

    self.objects_in_tray = dict()  # key: object ID. value: False or object pose

    rospy.sleep(.5)
    rospy.loginfo("Finished initializing class")
    
  ############## ------ Internal functions (and convenience functions)

  def spawn_object(self, object_name, object_pose, object_reference_frame=""):
 
    collision_object = self.assembly_database.get_collision_object(object_name)
    if isinstance(object_pose, geometry_msgs.msg._PoseStamped.PoseStamped):
      co_pose = object_pose.pose
      collision_object.header.frame_id = object_pose.header.frame_id
    elif isinstance(object_pose, geometry_msgs.msg._Pose.Pose):
      co_pose = object_pose
    elif isinstance(object_pose, list) or isinstance(object_pose, numpy.ndarray):
      co_pose = conversions.to_pose(object_pose)
    else:
      raise ValueError("Unsupported pose type: %s" % type(object_pose))
    
    if not isinstance(object_pose, geometry_msgs.msg._PoseStamped.PoseStamped):
      if not object_reference_frame:
        raise ValueError("object_reference_frame is required when providing a pose type Pose or List")
      else:
        collision_object.header.frame_id = object_reference_frame

    collision_object.pose = co_pose
        
    self.planning_scene_interface.add_object(collision_object)

  def despawn_object(self, object_name):
    self.planning_scene_interface.remove_attached_object(name=object_name)
    rospy.sleep(0.5) # Wait half a second for the detach to finish so that we can remove the object
    self.planning_scene_interface.remove_world_object(object_name)

  def confirm_to_proceed(self, next_task_name):
    if self.competition_mode:
      return True
    rospy.loginfo("Press enter to proceed to: " + next_task_name)
    i = raw_input()
    if i == "":
      if not rospy.is_shutdown():
        return True
    raise Exception("User caused exit!")

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
    """ Also see reset_assembly_visualization in common.py """
    self.a_bot.robot_status = o2ac_msgs.msg.RobotStatus()
    self.b_bot.robot_status = o2ac_msgs.msg.RobotStatus()
    self.planning_scene_interface.remove_attached_object()  # Detach objects
    rospy.sleep(0.5) # Wait half a second for the detach to finish so that we can remove the object
    self.planning_scene_interface.remove_world_object()  # Clear all objects
    self.publish_robot_status()

  @check_for_real_robot
  def activate_led(self, LED_name="b_bot", on=True):
    request = ur_msgs.srv.SetIORequest()
    request.fun = ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT
    request.pin = 4
    if on:
      request.state = ur_msgs.srv.SetIORequest.STATE_ON
    else:
      request.state = ur_msgs.srv.SetIORequest.STATE_OFF

    if LED_name == "b_bot":
      return self.b_bot.set_io.call(request)
    elif LED_name == "a_bot":
      return self.a_bot.set_io.call(request)
    else:
      rospy.logerr("Invalid LED name")
  
  def unlock_base_plate(self):
    self.set_base_lock(closed=False)
  def lock_base_plate(self):
    self.set_base_lock(closed=True)

  @check_for_real_robot
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

    path = rospkg.RosPack().get_path("o2ac_assembly_database") + "/config/tool_collision_objects.yaml"
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

  ######
  def pick_screw_from_feeder_python(self, robot_name, screw_size, realign_tool_upon_failure=True):
    if self.tools.screw_is_suctioned.get("m" + str(screw_size), False): 
      rospy.loginfo("(pick_screw_from_feeder) But a screw was already detected in the tool. Returning true without doing anything.")
      return True

    assert robot_name in ("a_bot", "b_bot"), "unsupported operation for robot %s" % robot_name
    offset = 1 if robot_name == "a_bot" else -1
    pose_feeder = conversions.to_pose_stamped("m" + str(screw_size) + "_feeder_outlet_link", [0.005, 0, 0, np.deg2rad(offset*60), 0, 0])
    
    screw_tool_id = "screw_tool_m" + str(screw_size)
    screw_tool_link = robot_name + "_screw_tool_m" + str(screw_size) + "_tip_link"
    fastening_tool_name = "screw_tool_m" + str(screw_size)
    
    if not self.active_robots[robot_name].robot_status.held_tool_id == fastening_tool_name:
      if not self.equip_tool(robot_name, fastening_tool_name):
        rospy.logerr("Robot is not carrying the correct tool (" + fastening_tool_name + ") and it failed to be equipped. Abort.")
        return False

    screw_picked = self.suck_screw(robot_name, pose_feeder, screw_tool_id, screw_tool_link, fastening_tool_name, do_spiral_search_at_bottom=False)
    
    if screw_picked:
      return True
    elif realign_tool_upon_failure:
        self.active_robots[robot_name].move_lin_rel(relative_translation=[0,0,0.05])
        self.active_robots[robot_name].go_to_named_pose("tool_pick_ready")
        rospy.loginfo("pickScrewFromFeeder failed. Realigning tool and retrying.")
        screw_tool_id = "screw_tool_m" + str(screw_size)
        self.realign_tool(robot_name, screw_tool_id)
        return self.pick_screw_from_feeder_python(robot_name, screw_size, realign_tool_upon_failure=False)
    else:
        self.active_robots[robot_name].go_to_named_pose("tool_pick_ready")
        return False

  def suck_screw(self, robot_name, screw_head_pose, screw_tool_id, screw_tool_link, fastening_tool_name, do_spiral_search_at_bottom=True):
    """ Strategy: 
         - Move 1 cm above the screw head pose
         - Go down real slow for 2 cm while turning the motor in the direction that would loosen the screw
         - If the suction reports success, return true
         - If screw is not there, spiral a bit 2mm around
         - If screw is still not there, move up again slowly
         - Then, try the same a few more times in nearby locations (square-like)"""
    rospy.loginfo("Suck screw command")

    if robot_name == "a_bot":
      rotation = [radians(80), 0, 0] if screw_tool_id == "screw_tool_m4" else [-tau/6, 0, 0]
    elif robot_name == "b_bot":
      rotation = [-tau/6, 0, 0]

    screw_picked = self.tools.screw_is_suctioned.get(screw_tool_id[-2:], False)
    if screw_picked:
      rospy.loginfo("Asked to pick screw, but one was already in the tool. Returning True.")
      return True
    
    above_screw_head_pose = copy.deepcopy(screw_head_pose)
    above_screw_head_pose.pose.orientation = conversions.to_quaternion(tf.transformations.quaternion_from_euler(*rotation))
    above_screw_head_pose.pose.position.x -= 0.01

    if (screw_tool_id == "screw_tool_m3"):
      self.planning_scene_interface.allow_collisions(screw_tool_id, "m3_feeder_link")
      screw_size = 3
    elif (screw_tool_id == "screw_tool_m4"):
      self.planning_scene_interface.allow_collisions(screw_tool_id, "m4_feeder_link")
      screw_size = 4

    initial_offset_y = rospy.get_param("screw_picking/" + robot_name + "/last_successful_offset_m" + str(screw_size) + "_y", 0.0)
    initial_offset_z = rospy.get_param("screw_picking/" + robot_name + "/last_successful_offset_m" + str(screw_size) + "_z", 0.0)
    if initial_offset_y or initial_offset_z:
      rospy.loginfo("Starting search with offsets: " + str(initial_offset_y) + ", " + str(initial_offset_z))
    adjusted_pose = copy.deepcopy(above_screw_head_pose)
    search_start_pose = copy.deepcopy(above_screw_head_pose)
    search_start_pose.pose.position.y += initial_offset_y
    search_start_pose.pose.position.z += initial_offset_z
    screw_picked = False

    self.tools.set_suction(screw_tool_id, suction_on=True, eject=False, wait=False)

    approach_height = 0.018

    max_radius = .0025
    theta_incr = tau/6
    r=0.0002
    radius_increment = .001
    radius_inc_set = radius_increment / (tau / theta_incr)
    theta, RealRadius = 0.0, 0.0
    
    first_approach = True
    while not screw_picked:
      self.tools.set_motor(fastening_tool_name, direction="loosen", wait=False, duration=15.0, skip_final_loosen_and_retighten=True)
      assert not rospy.is_shutdown(), "Did ros die?"
      
      # TODO(felixvd): Cancel the previous goal before sending this, or the start/stop commands from different actions overlap!
      rospy.loginfo("Moving into screw to pick it up.")
      adjusted_pose.pose.position.x += approach_height
      success = False
      while not success and not rospy.is_shutdown(): # TODO(cambel, felix): infinite loop?
        if not first_approach:
          success = self.active_robots[robot_name].go_to_pose_goal(adjusted_pose, speed=0.05, end_effector_link=screw_tool_link)
        else:  # Include initial motion from feeder_pick_ready
          waypoints = []
          waypoints.append(["feeder_pick_ready", 0.0, 1.0])  # pose, blend_radius, speed
          j1 = self.active_robots[robot_name].compute_ik(above_screw_head_pose, timeout=0.02, end_effector_link=screw_tool_link)
          j2 = self.active_robots[robot_name].compute_ik(adjusted_pose, timeout=0.02, end_effector_link=screw_tool_link)
          waypoints.append([j1, 0.0, 0.8])  # pose, blend_radius, speed
          waypoints.append([j2, 0.0, 0.05])
          success = self.active_robots[robot_name].move_joints_trajectory(waypoints, timeout=1.0)
          if not success:
            seq = []
            seq.append(helpers.to_sequence_item("feeder_pick_ready", speed=1.0))
            seq.append(helpers.to_sequence_trajectory([above_screw_head_pose,adjusted_pose], blend_radiuses=[0.001,0], speed=[0.4,0.05]))
            success = self.execute_sequence(robot_name, seq, "move into screw pick", end_effector_link=screw_tool_link)
          if success:
            first_approach = False

      # Break out of loop if screw suctioned or max search radius exceeded
      screw_picked = self.tools.screw_is_suctioned.get(screw_tool_id[-2:], False) or (not self.use_real_robot)
      if screw_picked:
        rospy.loginfo("Detected successful pick.")
        break

      if not do_spiral_search_at_bottom:
        tc = lambda a, b: self.tools.screw_is_suctioned.get(screw_tool_id[-2:], False)
        self.active_robots[robot_name].execute_spiral_trajectory("YZ", max_radius=0.0015, radius_direction="+Y", steps=25,
                                                          revolutions=1, target_force=0, check_displacement_time=10,
                                                          termination_criteria=tc, timeout=2, end_effector_link=screw_tool_link)

      rospy.loginfo("Moving back a bit slowly.")
      rospy.sleep(0.5)
      adjusted_pose.pose.position.x -= approach_height
      success = False
      while not success: # TODO(cambel, felix): infinite loop?
        success = self.active_robots[robot_name].go_to_pose_goal(adjusted_pose, speed=0.05, end_effector_link=screw_tool_link)
        
      # Break out of loop if screw suctioned or max search radius exceeded
      screw_picked = self.tools.screw_is_suctioned.get(screw_tool_id[-2:], False)
      if screw_picked:
        rospy.loginfo("Detected successful pick.")
        break

      if RealRadius > max_radius:
        break

      # Adjust the position (spiral search)
      theta = theta + theta_incr
      y = cos(theta) * r
      z = sin(theta) * r
      adjusted_pose = search_start_pose
      adjusted_pose.pose.position.y += y
      adjusted_pose.pose.position.z += z
      r = r + radius_inc_set
      RealRadius = sqrt(pow(y, 2) + pow(z, 2))
    
    # Record the offset at which the screw was successfully picked, so the next try can start at the same location
    if screw_picked:
      new_y_offset = adjusted_pose.pose.position.y - above_screw_head_pose.pose.position.y
      new_z_offset = adjusted_pose.pose.position.z - above_screw_head_pose.pose.position.z
    else:
      new_y_offset = 0.0
      new_z_offset = 0.0
    rospy.set_param("screw_picking/" + robot_name + "/last_successful_offset_m" + str(screw_size) + "_y", new_y_offset)
    rospy.set_param("screw_picking/" + robot_name + "/last_successful_offset_m" + str(screw_size) + "_z", new_z_offset)

    self.tools.set_motor(fastening_tool_name, direction="loosen", wait=False, duration=0.1, skip_final_loosen_and_retighten=True)

    # Move away from feeder
    waypoints = []
    through_gate = None
    if screw_picked:
      rospy.loginfo("Moving back through sensor gate.")
      rel_pose = self.active_robots[robot_name].move_lin_rel([0.02,0,0.0], pose_only=True)
      through_gate = self.active_robots[robot_name].compute_ik(rel_pose, timeout=0.02, retry=True)
      waypoints.append((through_gate,0,0.1))
    rel_pose = self.active_robots[robot_name].move_lin_rel([0.0,0,0.05], pose_only=True, initial_joints=through_gate)
    waypoints.append((self.active_robots[robot_name].compute_ik(rel_pose, timeout=0.02, retry=True), 0, 0.6))
    waypoints.append(("feeder_pick_ready",0,0.6))
    if not self.active_robots[robot_name].move_joints_trajectory(waypoints):
      rospy.logerr("Go to feeder_pick_ready failed. abort.")
      screw_picked = False

    if (screw_tool_id == "screw_tool_m3"):
      self.planning_scene_interface.disallow_collisions(screw_tool_id, "m3_feeder_link")
    elif (screw_tool_id == "screw_tool_m4"):
      self.planning_scene_interface.disallow_collisions(screw_tool_id, "m4_feeder_link")

    if screw_picked:
      rospy.loginfo("Finished picking up screw successfully.")
    else:
      rospy.logerr("Failed to pick screw.")
      self.tools.set_suction(screw_tool_id, suction_on=False, eject=False, wait=False)
      self.active_robots[robot_name].move_lin_rel([0,0,0.1], speed=0.2)
    
    return screw_picked

  def screw(self, robot_name, screw_hole_pose, screw_size, screw_height=0.02,
            stay_put_after_screwing=False, duration=20.0, skip_final_loosen_and_retighten=False,
            spiral_radius=0.0015, attempts=1):
    screw_tool_link = robot_name + "_screw_tool_" + "m" + str(screw_size) + "_tip_link"
    screw_tool_id = "screw_tool_m" + str(screw_size)
    fastening_tool_name = "screw_tool_m" + str(screw_size)
    rospy.loginfo("screw tool link: %s " % screw_tool_link)

    approach_height = .01
    insertion_amount = .015

    screw_tip_at_hole = copy.deepcopy(screw_hole_pose)
    screw_tip_at_hole.pose.position.x -= screw_height
    away_from_hole = copy.deepcopy(screw_tip_at_hole)
    away_from_hole.pose.position.x -= approach_height
    pushed_into_hole = copy.deepcopy(screw_tip_at_hole)
    pushed_into_hole.pose.position.x += insertion_amount

    self.active_robots[robot_name].go_to_pose_goal(away_from_hole, end_effector_link=screw_tool_link, speed=0.2)
    
    self.tools.set_motor(fastening_tool_name, direction="tighten", wait=False, duration=duration, 
                         skip_final_loosen_and_retighten=skip_final_loosen_and_retighten)

    self.planning_scene_interface.allow_collisions(screw_tool_id)

    self.active_robots[robot_name].go_to_pose_goal(pushed_into_hole, end_effector_link=screw_tool_link, speed=0.02)

    # Stop spiral motion if the tool action finished, regardless of success/failure
    tc = lambda a, b: self.tools.fastening_tool_client.get_state() != GoalStatus.ACTIVE
    self.active_robots[robot_name].execute_spiral_trajectory("YZ", max_radius=spiral_radius, radius_direction="+Y", steps=50,
                                                          revolutions=2, target_force=0, check_displacement_time=10,
                                                          termination_criteria=tc, timeout=10, end_effector_link=screw_tool_link)

    if not self.use_real_robot:
      return True
    
    finished_before_timeout = self.tools.fastening_tool_client.wait_for_result(rospy.Duration(15))
    result = self.tools.fastening_tool_client.get_result()
    rospy.loginfo("Screw tool motor command. Finish before timeout: %s" % finished_before_timeout)
    rospy.loginfo("Result: %s" % result)
    motor_stalled = False
    if result is not None:
      motor_stalled = result.motor_stalled

    if not stay_put_after_screwing:
      self.active_robots[robot_name].go_to_pose_goal(away_from_hole, end_effector_link=screw_tool_link, speed=0.02)

    self.planning_scene_interface.disallow_collisions(screw_tool_id)

    screw_picked = self.tools.screw_is_suctioned.get(screw_tool_id[-2:], False)
    if screw_picked and not stay_put_after_screwing:
      rospy.logerr("screw did not succeed: screw is still suctioned.")
      if attempts > 0:
        return self.screw(robot_name=robot_name, screw_hole_pose=screw_hole_pose, screw_size=screw_size, screw_height=screw_height,
              stay_put_after_screwing=stay_put_after_screwing, duration=duration, skip_final_loosen_and_retighten=skip_final_loosen_and_retighten,
              spiral_radius=spiral_radius, attempts=attempts-1)
      return False
        
    self.tools.set_suction(screw_tool_id, suction_on=False, eject=False, wait=False)
    return motor_stalled

  def pick_screw_from_feeder(self, robot_name, screw_size, realign_tool_upon_failure=True):
    return self.pick_screw_from_feeder_python(robot_name, screw_size, realign_tool_upon_failure)  # Python-only version
    res = self.skill_server.pick_screw_from_feeder(robot_name, screw_size, realign_tool_upon_failure)
    try:
      if res.success:
        return True
    except:
      pass
    if realign_tool_upon_failure:
        self.active_robots[robot_name].move_lin_rel(relative_translation=[0,0,0.05])
        self.active_robots[robot_name].go_to_named_pose("tool_pick_ready")
        rospy.loginfo("pickScrewFromFeeder failed. Realigning tool and retrying.")
        screw_tool_id = "screw_tool_m" + str(screw_size)
        self.realign_tool(robot_name, screw_tool_id)
        return self.pick_screw_from_feeder(robot_name, screw_size, realign_tool_upon_failure=False)
    else:
        self.active_robots[robot_name].go_to_named_pose("tool_pick_ready")
        return False

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
    spawn_objects(self.assembly_database, objects, poses, reference_frame)
  
  def get_3d_poses_from_ssd(self):
    """
    Returns object poses as estimated by the SSD neural network and reprojection.
    Also updates self.objects_in_tray
    """
    # Read result and return
    try:
      rospy.logwarn("Clearing all object poses in memory")
      self.objects_in_tray = dict()
      self.object_in_tray_is_upside_down = dict()
      res = self.vision.read_from_ssd()
      for idx, pose, upside_down in zip(res.class_ids, res.poses, res.upside_down):
        self.objects_in_tray[idx] = pose
        self.object_in_tray_is_upside_down[idx] = upside_down
      return res
    except:
      pass
    return False
  
  def get_bearing_angle(self, camera="b_bot_inside_camera"):
    """
    When looking at the bearing from the front, returns the rotation angle 
    to align the screw holes.
    """
    if self.use_dummy_vision or not self.use_real_robot:
      return 0.0001  # 0.0 would be recognized as vision failure
    return self.vision.get_angle_from_vision(camera, item_name="bearing")
  
  def get_motor_angle(self, camera="b_bot_outside_camera"):
    """
    When looking at the motor in the vgroove, this returns the rotation angle.
    """
    return self.vision.get_angle_from_vision(camera, item_name="motor")

  def detect_object_in_camera_view(self, item_name, publish_to_scene=True):
    """
    Returns object pose if object was detected in current camera view and published to planning scene,
    False otherwise.
    """
    # TODO: merge with "look_and_get_grasp_points"
    object_type = self.assembly_database.name_to_type(item_name)
    if not object_type:
      rospy.logerr("Could not find the object " + item_name + " in database, or its type field is empty.")
      return False
    object_pose = self.vision.localize_object(object_type)
    if object_pose:
      rospy.loginfo("Localized " + item_name + " via CAD matching")
      if publish_to_scene:
        co = self.assembly_database.get_collision_object(object_name=item_name)
        co.header.frame_id = object_pose.header.frame_id
        co.pose = object_pose.pose

        self.planning_scene_interface.apply_collision_object(co)
        self.planning_scene_interface.allow_collisions("tray_center", co.id)  # tray_center is the tray surface
      return object_pose
    else:
      rospy.logerr("Did not detect " + item_name)
      return False

  @save_task_plan
  def plan_pick_place(self, robot_name, object_name, grasp_poses, pick_only=True, place_only=False):
    '''
    Function for calling the plan_pick_place MTC action
    The function returns the MTC solution containing the trajectories

    grasp_pose is a geometry_msgs.msg.PoseStamped (usually in the frame "object_name") for the robot's gripper_tip_link frame.
    '''
    goal = moveit_task_constructor_msgs.msg.PlanPickPlaceGoal()
    if pick_only:
      goal.task_type = moveit_task_constructor_msgs.msg.PlanPickPlaceGoal.PICK_ONLY
    elif place_only:
      goal.task_type = moveit_task_constructor_msgs.msg.PlanPickPlaceGoal.PLACE_ONLY
    else:
      goal.task_type = moveit_task_constructor_msgs.msg.PlanPickPlaceGoal.PICK_AND_PLACE
    goal.arm_group_name = robot_name
    goal.hand_group_name = robot_name + '_robotiq_85'
    goal.eef_name = robot_name + '_tip'
    goal.hand_frame = robot_name + '_gripper_tip_link'
    goal.object_id = object_name
    goal.support_surfaces = ["tray_center"]

    for grasp_pose in grasp_poses:
      grasp = moveit_msgs.msg.Grasp()

      grasp.grasp_pose = grasp_pose

      approach_direction = geometry_msgs.msg.Vector3Stamped()
      approach_direction.header.frame_id = 'world'
      approach_direction.vector.z = -1
      grasp.pre_grasp_approach.direction = approach_direction
      grasp.pre_grasp_approach.min_distance = 0.05
      grasp.pre_grasp_approach.desired_distance = 0.1

      lift_direction = geometry_msgs.msg.Vector3Stamped()
      lift_direction.header.frame_id = 'world'
      lift_direction.vector.z = 1
      grasp.post_grasp_retreat.direction = lift_direction
      grasp.post_grasp_retreat.min_distance = 0.05
      grasp.post_grasp_retreat.desired_distance = 0.1

      hand = self.active_robots[robot_name].gripper_group
      hand_open = hand.get_named_target_values("open")
      hand_closed = hand.get_named_target_values("close")

      for (joint, value) in hand_open.items():
          joint_traj_point = JointTrajectoryPoint()
          joint_traj_point.positions.append(value)
          grasp.pre_grasp_posture.joint_names.append(joint)
          grasp.pre_grasp_posture.points.append(joint_traj_point)
      
      for (joint, value) in hand_closed.items():
          joint_traj_point = JointTrajectoryPoint()
          joint_traj_point.positions.append(value)
          grasp.grasp_posture.joint_names.append(joint)
          grasp.grasp_posture.points.append(joint_traj_point)
      
      goal.grasps.append(grasp)
    
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = 'tray_center'
    place_pose.pose.orientation.w = 1
    place_pose.pose.position.z = 0.3
    goal.place_locations = [moveit_msgs.msg.PlaceLocation()]
    goal.place_locations[0].place_pose = place_pose

    place_direction = geometry_msgs.msg.Vector3Stamped()
    place_direction.header.frame_id = 'world'
    place_direction.vector.z = -1
    goal.place_locations[0].pre_place_approach.direction = place_direction
    goal.place_locations[0].pre_place_approach.min_distance = 0.05
    goal.place_locations[0].pre_place_approach.desired_distance = 0.15

    retreat_direction = geometry_msgs.msg.Vector3Stamped()
    retreat_direction.header.frame_id = 'world'
    retreat_direction.vector.z = -1
    goal.place_locations[0].post_place_retreat.direction = retreat_direction
    goal.place_locations[0].post_place_retreat.min_distance = 0.05
    goal.place_locations[0].post_place_retreat.desired_distance = 0.15

    rospy.loginfo("Sending pick planning goal.")
    self.pick_place_planning_client.send_goal(goal)
    self.pick_place_planning_client.wait_for_result(rospy.Duration(15.0))
    return self.pick_place_planning_client.get_result()

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
    if tool_name in self.screw_tools: 
      rospy.loginfo("Spawn: " + tool_name)
      self.planning_scene_interface.add_object(self.screw_tools[tool_name])
      return True
    else:
      rospy.logerr("Cannot spawn tool: " + tool_name)
      return False

  def despawn_tool(self, tool_name):
    if tool_name in self.screw_tools: 
      rospy.loginfo("Despawn: " + tool_name)
      self.planning_scene_interface.remove_world_object(self.screw_tools[tool_name].id)
      return True
    else:
      rospy.logerr("Cannot despawn tool: " + tool_name)
      return False

  def equip_tool(self, robot_name, tool_name):
    return self.equip_unequip_realign_tool(robot_name, tool_name, "equip")
  
  def unequip_tool(self, robot_name, tool_name=""):
    if tool_name == "":
      tool_name = self.active_robots[robot_name].robot_status.held_tool_id
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
      tool_name = self.active_robots[robot_name].robot_status.held_tool_id

    # Sanity check on the input instruction
    equip = (operation == "equip")
    unequip = (operation == "unequip")
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
      if robot.robot_status.held_tool_id == tool_name:
        rospy.loginfo("Robot already holds the tool. Returning true.")
        return True
      rospy.logerr("Robot already holds a tool. Cannot equip another.")
      return False
    if ((robot.robot_status.carrying_tool == False) and unequip):
      rospy.logerr("Robot is not holding a tool. Cannot unequip any.")
      return False
    
    if equip:
      robot.gripper.open(opening_width=0.06, wait=False)

    # Set up poses
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = tool_name + "_pickup_link"

    # Define approach pose
    # z = 0 is at the holder surface, and z-axis of pickup_link points downwards!
    rospy.loginfo("tool_name: " + tool_name)
    if tool_name == "screw_tool_m3" or tool_name == "screw_tool_m4" or tool_name == "padless_tool_m4":
      ps_approach.pose.position.x = -.04
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

    rospy.loginfo("Going to before_tool_pickup pose.")
    # Go to named pose, then approach
    sequence = []
    sequence.append(helpers.to_sequence_item("tool_pick_ready"))
    # self.active_robots[robot_name].go_to_named_pose("tool_pick_ready")

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
      robot.gripper.open(opening_width=0.06)
      rospy.loginfo("Spawning tool.")
      if not self.spawn_tool(tool_name):
        rospy.logwarn("Could not spawn the tool. Continuing.")
      held_screw_tool_ = tool_name

    rospy.loginfo("Moving to screw tool approach pose LIN.")
  
    # Plan & execute linear motion to the tool change position
    rospy.loginfo("Moving to pose in tool holder LIN.")
    if equip:
      lin_speed = 0.5
    elif unequip:
      lin_speed = 0.5
      self.planning_scene_interface.allow_collisions("screw_tool_holder_long", tool_name)
    elif realign:
      lin_speed = 0.5

    sequence.append(helpers.to_sequence_trajectory([ps_approach,ps_in_holder], [0.005,0.0], speed=[0.5, 0.2]))
    if not self.execute_sequence(robot_name, sequence, "approach sequence equip/unequip tool", plan_while_moving=True):
      rospy.logerr("Fail to complete the equip/unequip tool sequence")
      return False

    # Close gripper, attach the tool object to the gripper in the Planning Scene.
    # Its collision with the parent link is set to allowed in the original planning scene.
    if equip:
      robot.gripper.close()
      self.active_robots[robot_name].gripper.attach_object(tool_name)
      self.allow_collisions_with_robot_hand(tool_name, robot_name)
      robot.robot_status.carrying_tool = True
      robot.robot_status.held_tool_id = tool_name
      self.publish_robot_status()
    elif unequip:
      robot.gripper.open(opening_width=0.06)
      self.active_robots[robot_name].gripper.detach_object(tool_name)
      self.planning_scene_interface.remove_attached_object(name=tool_name)
      self.planning_scene_interface.remove_world_object(name=tool_name)
      self.allow_collisions_with_robot_hand(tool_name, robot_name, allow=False)
      robot.robot_status.carrying_tool = False
      robot.robot_status.held_tool_id = ""
      self.publish_robot_status()
    elif realign:  # Drop the tool into the holder once
      robot.gripper.open(opening_width=0.06)
      robot.gripper.close()
    
    sequence = []

    if equip or realign:  # Pull back and let go once to align the tool with the magnet properly 
      pull_back_slightly = copy.deepcopy(ps_in_holder)
      pull_back_slightly.pose.position.x -= 0.003
      ps_in_holder.pose.position.x += 0.001  # To remove the offset for placing applied earlier
      lin_speed = 0.2
      sequence.append(helpers.to_sequence_item(pull_back_slightly, speed=lin_speed))
      sequence.append(helpers.to_sequence_gripper(action='open', gripper_opening_width=0.06, gripper_velocity=0.1))
      sequence.append(helpers.to_sequence_item(ps_in_holder, speed=lin_speed))
      sequence.append(helpers.to_sequence_gripper(action='close', gripper_force=100, gripper_velocity=0.1))
    
    # Plan & execute linear motion away from the tool change position
    rospy.loginfo("Moving back to screw tool approach pose LIN.")
    
    lin_speed = 1.0

    sequence.append(helpers.to_sequence_item(ps_move_away, speed=lin_speed))
    sequence.append(helpers.to_sequence_item("tool_pick_ready"))
    
    success = self.execute_sequence(robot_name, sequence, "equip/unequip tool", plan_while_moving=True)
    if not success:
      rospy.logerr("Failed to equip/unequip tool: %s" % tool_name)
      return False

    return True

  def allow_collisions_with_robot_hand(self, link_name, robot_name, allow=True):
      """Allow collisions of a link with the robot hand"""
      hand_links = [
        robot_name + "_tip_link",
        robot_name + "_left_inner_finger_pad",
        robot_name + "_right_inner_finger_pad",
        robot_name + "_left_inner_finger",
        robot_name + "_right_inner_finger",
        robot_name + "_left_inner_knuckle",
        robot_name + "_right_inner_knuckle",
        robot_name + "_left_outer_finger",
        robot_name + "_right_outer_finger",
      ]
      if allow:
        self.planning_scene_interface.allow_collisions(hand_links, link_name)
      else:
        self.planning_scene_interface.disallow_collisions(hand_links, link_name)
      return

  def disable_scene_object_collisions(self):
    """ Disables collisions between all world objects (except tools) and everything else.
        Used because our meshes are so heavy that they impact performance too much.
    """
    object_names = self.planning_scene_interface.get_known_object_names()
    rospy.loginfo("Disabling collisions for all scene objects (except tools).")
    objects_without_tools = []
    for n in object_names:
      if not "tool" in n:
        objects_without_tools.append(n)
    print(objects_without_tools)
    self.planning_scene_interface.allow_collisions(objects_without_tools, "")

  def read_playback_sequence(self, routine_filename, default_frame="world"):
    """
    default_frame: used for task-space-in-frame waypoints with undefined frame_id or where the value is specified as default, then the default_frame is used.
    """
    path = rospkg.RosPack().get_path("o2ac_routines") + ("/config/playback_sequences/%s.yaml" % routine_filename)
    with open(path, 'r') as f:
      routine = yaml.load(f)
    robot_name = routine["robot_name"]
    waypoints = routine["waypoints"]

    trajectory = []

    playback_trajectories = []

    for waypoint in waypoints:
      is_trajectory = waypoint.get('is_trajectory', False)
      
      if waypoint.get("frame_id", "default") == "default":
        waypoint.update({"frame_id": default_frame})

      if is_trajectory:
        pose = waypoint.get("pose", None)
        if waypoint["pose_type"] == "joint-space-goal-cartesian-lin-motion":
          p = self.active_robots[robot_name].compute_fk(pose) # Forward Kinematics
        elif waypoint["pose_type"] == "task-space-in-frame":
          # Convert orientation to radians!
          frame_id = waypoint.get("frame_id", "default")
          p = conversions.to_pose_stamped(frame_id, np.concatenate([pose[:3],np.deg2rad(pose[3:])]))
        elif waypoint["pose_type"] == "master-slave":
          p = waypoint # forward all the info to construct the master-slave trajectory
        else:
          raise ValueError("Unsupported trajectory for pose type: %s" % waypoint["pose_type"])
        blend = waypoint.get("blend", 0.0)
        trajectory_speed = waypoint.get("speed", 0.5)
        trajectory.append([p, blend, trajectory_speed])
      else:
        # Save any on-going trajectory
        if trajectory:
          playback_trajectories.append(["trajectory", trajectory])
          trajectory = []

        playback_trajectories.append(["waypoint", waypoint])
      
    if trajectory:
      playback_trajectories.append(["trajectory", trajectory])

    return robot_name, playback_trajectories

  def execute_sequence(self, robot_name, sequence, sequence_name, end_effector_link="", plan_while_moving=True, save_on_success=False, use_saved_plans=False):
    if use_saved_plans:
      # TODO(cambel): check that the original plan's file has not been updated. if so, try to do the online planning
      bagfile = helpers.get_plan_full_path(sequence_name)
      if not os.path.exists(bagfile):
        rospy.logwarn("Attempted to execute saved sequence, but file not found: %s" % bagfile)
        rospy.logwarn("Try to execute sequence with online planning")
      else:
        rospy.logwarn("Executing saved sequence")
        return self.execute_saved_sequence(sequence_name)

    robot = self.active_robots[robot_name]
    all_plans = []
    if not plan_while_moving:
      for i, point in enumerate(sequence):
        rospy.loginfo("Sequence point: %i - %s" % (i+1, point[0]))
        # self.confirm_to_proceed("playback_sequence")
        if point[0] == "waypoint":
          waypoint_params = point[1]
          res = self.move_to_sequence_waypoint(robot_name, waypoint_params)
        elif point[0] == "trajectory":
          trajectory = point[1]
          res = robot.move_lin_trajectory(trajectory)
        elif point[0] == "joint_trajectory":
          trajectory = point[1]
          res = robot.move_joints_trajectory(trajectory)
        if not res:
          rospy.logerr("Fail to complete playback sequence: %s" % sequence_name)
          return False
    else:
      rospy.loginfo("(plan_while_moving) Sequence name: %s" % sequence_name)
      all_plans.append(robot_name)
      active_plan = None
      active_plan_start_time = rospy.Time(0)
      active_plan_duration = 0.0
      previous_plan = None
      backlog = []
      previous_plan_type = ""
      for i, point in enumerate(sequence):
        gripper_action = None

        rospy.loginfo("(plan_while_moving) Sequence point: %i - %s" % (i+1, point[0]))
        # self.confirm_to_proceed("playback_sequence")

        res = self.plan_waypoint(robot_name, point, previous_plan, end_effector_link=end_effector_link) # res = plan, planning_time 

        if not res:
          rospy.logerr("Fail to complete playback sequence: %s" % sequence_name)
          return False
        
        if isinstance(res[0], dict):
          gripper_action = copy.copy(res[0])
          res = None, 0.0
        
        plan, _ = res

        # begin - For logging only
        if point[0] == "waypoint":
          previous_plan_type = point[1]["pose_type"]
        else:
          previous_plan_type = point[0]
        # end

        if save_on_success:
          if not gripper_action:
            if i == 0: # Just save the target joint configuration
              all_plans.append(helpers.get_trajectory_joint_goal(plan))
            else: # otherwise save the computed plan
              all_plans.append(plan)
          else: # or the gripper action
            all_plans.append(gripper_action)

        if gripper_action:
          backlog.append((gripper_action, (i+1), previous_plan_type))
          continue

        backlog.append((plan, (i+1), previous_plan_type))
        previous_plan = plan
        
        if active_plan:
          execution_time = (rospy.Time.now() - active_plan_start_time).secs
          remaining_time = execution_time - active_plan_duration
          if remaining_time < 0.1: # Not enough time for queue up another plan; wait for execution to complete
            if not robot.robot_group.wait_for_motion_result():
              rospy.logerr("MoveIt aborted the motion")
            rospy.loginfo("Waited for motion result")
          else:
            # Try planning another point
            continue
          
          if not self.check_plan_goal_reached(robot_name, active_plan):
            rospy.logerr("Failed to execute sequence plan: target pose not reached")
            return False

        # execute next plan 
        next_plan, index, plan_type = backlog.pop(0)
        print("Executing sequence plan: index,", index, "type", plan_type)
        wait = True if i == len(sequence) -1 else False
        self.execute_waypoint_plan(robot_name, next_plan, wait=wait)

        if isinstance(next_plan, dict):  # Gripper action
          continue
        elif isinstance(next_plan, moveit_msgs.msg.RobotTrajectory):
          active_plan = next_plan
          active_plan_start_time = rospy.Time.now()
          active_plan_duration = helpers.get_trajectory_duration(next_plan)

      # Finished preplanning the whole sequence: Execute remaining waypoints
      while backlog:
        robot.robot_group.wait_for_motion_result()
        if not self.check_plan_goal_reached(robot_name, active_plan):
          rospy.logerr("Fail to execute plan: target pose not reach")
          return False

        next_plan, index, plan_type = backlog.pop(0)
        print("Executing plan (backlog loop): index,", index, "type", plan_type)
        self.execute_waypoint_plan(robot_name, next_plan, True)
        if isinstance(next_plan, (moveit_msgs.msg.RobotTrajectory)):
          active_plan = next_plan

    if plan_while_moving and save_on_success:
      helpers.save_sequence_plans(name=sequence_name, plans=all_plans)

    return True

  def check_plan_goal_reached(self, robot_name, plan):
    current_joints = self.active_robots[robot_name].robot_group.get_current_joint_values()
    plan_goal = helpers.get_trajectory_joint_goal(plan, self.active_robots[robot_name].robot_group.get_active_joints())
    return helpers.all_close(plan_goal, current_joints, 0.01)

  def execute_waypoint_plan(self, robot_name, plan, wait):
    if isinstance(plan, dict): # gripper action
      if not self.execute_gripper_action(robot_name, plan):
        return False
    elif isinstance(plan, moveit_msgs.msg.RobotTrajectory):
      if not self.active_robots[robot_name].execute_plan(plan, wait=wait):
        rospy.logerr("plan execution failed")
        return False
    return True

  def plan_waypoint(self, robot_name, point, previous_plan, end_effector_link=""):
    """
      Plan a point defined in a dict
      a point can be of type `waypoint` or `trajectory`
      a `waypoint` can be a robot motion (joints, cartesian, linear, relative...) or a gripper action
      a `trajectory` can be for a single robot or for a master-slave relationship
      previous_plan: defines the initial state for the new `point`
      end_effector_link: defines the end effector for the plan. FIXME: so far, it is only used for move_lin_trajectory 
    """
    initial_joints = self.active_robots[robot_name].robot_group.get_current_joint_values() if not previous_plan else helpers.get_trajectory_joint_goal(previous_plan, self.active_robots[robot_name].robot_group.get_active_joints())
    if point[0] == "waypoint":
      rospy.loginfo("Sequence point type: %s > %s" % (point[1]["pose_type"], point[1].get("desc", '')))
      waypoint_params = point[1]
      gripper_action = waypoint_params["pose_type"] == 'gripper'
      if gripper_action:
        return waypoint_params["gripper"], 0.0
      else:
        if waypoint_params["pose_type"] == 'master-slave':
          joint_list = self.active_robots[waypoint_params["master_name"]].robot_group.get_active_joints() + self.active_robots[waypoint_params["slave_name"]].robot_group.get_active_joints()
          initial_joints_ = None if not previous_plan else helpers.get_trajectory_joint_goal(previous_plan, joint_list)
        else:
          initial_joints_ = initial_joints
        return self.move_to_sequence_waypoint(robot_name, waypoint_params, plan_only=True, initial_joints=initial_joints_)
    elif point[0] == "trajectory":
      trajectory = point[1]
      if isinstance(trajectory[0][0], geometry_msgs.msg.PoseStamped):
        return self.active_robots[robot_name].move_lin_trajectory(trajectory, plan_only=True, initial_joints=initial_joints, end_effector_link=end_effector_link)
      elif isinstance(trajectory[0][0], dict):
        joint_list = self.active_robots[trajectory[0][0]["master_name"]].robot_group.get_active_joints() + self.active_robots[trajectory[0][0]["slave_name"]].robot_group.get_active_joints()
        initial_joints_ = None if not previous_plan else helpers.get_trajectory_joint_goal(previous_plan, joint_list)
        return self.master_slave_trajectory(robot_name, trajectory, plan_only=True, initial_joints=initial_joints_)
      else:
        rospy.logerr("Trajectory: %s" % trajectory)
        raise ValueError("Invalid trajectory type %s" % type(trajectory[0][0]))
    elif point[0] == "joint_trajectory":
      # TODO(cambel): support master-slave trajectories
      return self.active_robots[robot_name].move_joints_trajectory(point[1], plan_only=True, initial_joints=initial_joints)
    else:
      raise ValueError("Invalid sequence type: %s" % point[0])

  def playback_sequence(self, routine_filename, default_frame="world", plan_while_moving=True, save_on_success=True, use_saved_plans=True):
    # TODO(felixvd): Remove this after the bearing procedure is fixed.
    if routine_filename == "bearing_orient_b_bot":
      rospy.logwarn("FIXME: Allow collision between b_bot_cam_cables_link and tray")
      self.planning_scene_interface.allow_collisions("b_bot_cam_cables_link", "tray")
      rospy.sleep(1.0)
    robot_name, playback_trajectories = self.read_playback_sequence(routine_filename, default_frame)

    return self.execute_sequence(robot_name, playback_trajectories, routine_filename, plan_while_moving=plan_while_moving, save_on_success=save_on_success, use_saved_plans=use_saved_plans)

  def execute_gripper_action(self, robot_name, gripper_params):
      gripper_action = gripper_params.get("action", None)
      opening_width = gripper_params.get("open_width", 0.140)
      force = gripper_params.get("force", 80.)
      velocity = gripper_params.get("velocity", 0.03)
      if robot_name == "ab_bot":
        if gripper_action == 'open':
          success = self.a_bot.gripper.open(opening_width=opening_width, velocity=velocity)
          success &= self.b_bot.gripper.open(opening_width=opening_width, velocity=velocity)
        elif gripper_action == 'close':
          success = self.a_bot.gripper.close(force=force, velocity=velocity)
          success &= self.b_bot.gripper.close(force=force, velocity=velocity)
        elif isinstance(gripper_action, float):
          success = self.a_bot.gripper.send_command(gripper_action, force=force, velocity=velocity)
          success &= self.b_bot.gripper.send_command(gripper_action, force=force, velocity=velocity)
        else:
          raise ValueError("Unsupported gripper action: %s of type %s" % (gripper_action, type(gripper_action)))
        return success
      else:
        robot = self.active_robots[robot_name]
        if gripper_action == 'open':
          return robot.gripper.open(opening_width=opening_width, velocity=velocity)
        elif gripper_action == 'close':
          return robot.gripper.close(force=force, velocity=velocity)
        elif gripper_action == 'close-open':
          robot.gripper.close(velocity=velocity)
          return robot.gripper.open(opening_width=opening_width, velocity=velocity)
        elif isinstance(gripper_action, float):
          return robot.gripper.send_command(gripper_action, force=force, velocity=velocity)
        else:
          raise ValueError("Unsupported gripper action: %s of type %s" % (gripper_action, type(gripper_action)))

  def move_to_sequence_waypoint(self, robot_name, params, plan_only=False, initial_joints=None):
    success = False
    robot = self.active_robots[robot_name]

    pose           = params.get("pose", None)
    pose_type      = params["pose_type"]
    speed          = params.get("speed", 0.5)
    acceleration   = params.get("acc", 0.25)
    gripper_params = params.get("gripper", None)

    if pose_type  == 'joint-space':
      success = robot.move_joints(pose, speed=speed, acceleration=acceleration, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'joint-space-goal-cartesian-lin-motion':
      p = robot.compute_fk(pose) # Forward Kinematics
      success = robot.move_lin(p, speed=speed, acceleration=acceleration, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'task-space-in-frame':
      frame_id = params.get("frame_id", "world")
      # Convert orientation to radians!
      if robot_name == "ab_bot":
        a_bot_pose = conversions.to_pose_stamped(frame_id, pose)
        b_bot_pose = conversions.to_pose_stamped(frame_id, params["pose2"])
        planner = params.get("planner", "LINEAR")
        success = self.ab_bot.go_to_goal_poses(a_bot_pose, b_bot_pose, plan_only=plan_only, initial_joints=initial_joints, planner=planner)
      else:
        p = conversions.to_pose_stamped(frame_id, np.concatenate([pose[:3],np.deg2rad(pose[3:])]))
        move_linear = params.get("move_linear", True)
        success = robot.go_to_pose_goal(p, speed=speed, acceleration=acceleration, move_lin=move_linear, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'relative-tcp':
      success = robot.move_lin_rel(relative_translation=pose[:3], relative_rotation=np.deg2rad(pose[3:]), speed=speed, acceleration=acceleration, relative_to_tcp=True, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'relative-world':
      success = robot.move_lin_rel(relative_translation=pose[:3], relative_rotation=np.deg2rad(pose[3:]), speed=speed, acceleration=acceleration, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'relative-base':
      success = robot.move_lin_rel(relative_translation=pose[:3], relative_rotation=np.deg2rad(pose[3:]), speed=speed, acceleration=acceleration, relative_to_robot_base=True, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'named-pose':
      success = robot.go_to_named_pose(pose, speed=speed, acceleration=acceleration, plan_only=plan_only, initial_joints=initial_joints)
    elif pose_type == 'master-slave':
      ps = conversions.to_pose_stamped(params["frame_id"], np.concatenate([pose[:3],np.deg2rad(pose[3:])]))
      success = robot.master_slave_control(params['master_name'], params['slave_name'], ps, params['slave_relation'], speed=speed, plan_only=plan_only, initial_joints=initial_joints)
    elif not plan_only and pose_type == 'gripper':
      success = self.execute_gripper_action(robot_name, gripper_params)
    else:
      raise ValueError("Invalid pose_type: %s" % pose_type)

    return success

  def execute_saved_sequence(self, name):
    sequence = helpers.load_sequence_plans(name)
    robot_name = sequence[0]
    robot = self.active_robots[robot_name]

    robot.move_joints(sequence[1])
    for seq in sequence[2:]:
      success = False
      if isinstance(seq, dict):
        success =  self.execute_gripper_action(robot_name, seq)
      else:
        # TODO(cambel): validate that the plan is still valid before execution
        success = robot.execute_plan(seq, wait=True)
      
      if not success:
        rospy.logerr("Fail to execute saved plan from sequence. Abort")
        return False
    
    return True

  def master_slave_trajectory(self, robot_name, trajectory, plan_only=False, initial_joints=None):
    master_trajectory = []
    waypoints = [wp for wp, _ , _ in trajectory] # Ignoring blend and speed as the whole waypoints dict is forwarded in the first value
    for waypoint in waypoints:
      pose = waypoint["pose"]
      frame_id = waypoint["frame_id"]
      ps = conversions.to_pose_stamped(frame_id, np.concatenate([pose[:3],np.deg2rad(pose[3:])]))
      master_trajectory.append((ps, waypoint.get("blend", 0), waypoints[0].get("speed", 0.5))) # poseStamped, blend, speed

    slave_initial_joints = initial_joints[6:] if initial_joints is not None else None
    master_initial_joints = initial_joints[:6] if initial_joints is not None else None

    master_plan, _ = self.active_robots[waypoints[0]["master_name"]].move_lin_trajectory(master_trajectory, speed=waypoints[0].get("speed", 0.5), plan_only=True, initial_joints=master_initial_joints)

    master_slave_plan, planning_time = self.active_robots[robot_name].compute_master_slave_plan(waypoints[0]["master_name"], 
                                                                                                waypoints[0]["slave_name"],
                                                                                                waypoints[0]["slave_relation"],
                                                                                                slave_initial_joints,
                                                                                                master_plan)

    if plan_only:
      return master_slave_plan, planning_time
    else:
      return self.active_robots[robot_name].execute_plan(master_slave_plan)

######

  def start_task_timer(self):
    """Reset timer in debug monitor"""
    try:
      _ = self.resetTimerForDebugMonitor_client.call()
    except:
      pass
  
  def publish_status_text(self, text):
    """ Publish a string to the status topic, which is then converted to an image and displayed in Rviz.
    """
    rospy.loginfo("Published status: " + text)
    self.pub_status_text.publish(String(text))
