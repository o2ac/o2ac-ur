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
import time
import threading
import copy
import rospy
import rospkg
import tf_conversions
import tf 
import actionlib
from math import *
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
from std_msgs.msg import Bool
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

import o2ac_msgs
import o2ac_msgs.msg
import o2ac_msgs.srv
import o2ac_task_planning_msgs.msg

from aist_camera_multiplexer import RealSenseMultiplexerClient

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from o2ac_assembly_database.assembly_reader import AssemblyReader

import ur_msgs.msg
import ur_msgs.srv
from o2ac_routines.helpers import *

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
    self.disable_markers = True

    self.competition_mode = False   # Setting this to True disables confirmation dialogs etc., thus enabling uninterrupted automatic motion

    self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
    self.pause_mode_ = False
    self.test_mode_ = False
    self.ur_ros_control_running_on_robot = {"a_bot": False, "b_bot": False}
    self.robot_status = { "a_bot":o2ac_msgs.msg.RobotStatus(), "b_bot":o2ac_msgs.msg.RobotStatus() }

    self.speed_fast = 0.1
    self.speed_fastest = 0.2
    self.acc_fast = 0.1
    self.acc_fastest = 0.2

    self.reduced_mode_speed_limit = .25

    # Miscellaneous helpers
    self.robots = moveit_commander.RobotCommander()
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface()
    self.apply_planning_scene_diff = rospy.ServiceProxy('/apply_planning_scene', moveit_msgs.srv.ApplyPlanningScene)
    self.apply_planning_scene_diff.wait_for_service(5.0)

    self.assembly_database = AssemblyReader()

    try:
      self.camera_multiplexer = RealSenseMultiplexerClient('camera_multiplexer')
    except:
      self.camera_multiplexer = []

    # Action clients and movegroups
    self.groups = {"a_bot":moveit_commander.MoveGroupCommander("a_bot"), "b_bot":moveit_commander.MoveGroupCommander("b_bot"),
      "a_bot_robotiq_85":moveit_commander.MoveGroupCommander("a_bot_robotiq_85"), "b_bot_robotiq_85":moveit_commander.MoveGroupCommander("b_bot_robotiq_85")}
    self.gripper_action_clients = {"a_bot":actionlib.SimpleActionClient('/a_bot/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction),
      "b_bot":actionlib.SimpleActionClient('/b_bot/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction)}
    
    self.pick_screw_from_feeder_client = actionlib.SimpleActionClient('/o2ac_skills/pick_screw_from_feeder', o2ac_msgs.msg.pickScrewFromFeederAction)
    self.place_client = actionlib.SimpleActionClient('/o2ac_skills/place', o2ac_msgs.msg.placeAction)
    self.regrasp_client = actionlib.SimpleActionClient('/o2ac_skills/regrasp', o2ac_msgs.msg.regraspAction)
    self.screw_client = actionlib.SimpleActionClient('/o2ac_skills/screw', o2ac_msgs.msg.screwAction)
    self.change_tool_client = actionlib.SimpleActionClient('/o2ac_skills/change_tool', o2ac_msgs.msg.changeToolAction)

    self.ssd_client = actionlib.SimpleActionClient('/o2ac_vision_server/get_3d_poses_from_ssd', o2ac_msgs.msg.get3DPosesFromSSDAction)
    self.detect_shaft_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_shaft_notch', o2ac_msgs.msg.shaftNotchDetectionAction)
    self.detect_angle_client = actionlib.SimpleActionClient('/o2ac_vision_server/detect_angle', o2ac_msgs.msg.detectAngleAction)
    self.recognition_client = actionlib.SimpleActionClient('/o2ac_vision_server/localize_object', o2ac_msgs.msg.localizeObjectAction)

    self.pick_planning_client = actionlib.SimpleActionClient('/pick_planning', o2ac_task_planning_msgs.msg.PickObjectAction)
    self.place_planning_client = actionlib.SimpleActionClient('/place_planning', o2ac_task_planning_msgs.msg.PlaceObjectAction)
    self.release_planning_client = actionlib.SimpleActionClient('/release_planning', o2ac_task_planning_msgs.msg.ReleaseObjectAction)
    self.pickplace_planning_client = actionlib.SimpleActionClient('/pick_place_planning', o2ac_task_planning_msgs.msg.PickPlaceWithRegraspAction)
    self.fastening_planning_client = actionlib.SimpleActionClient('/fastening_planning', o2ac_task_planning_msgs.msg.PlaceObjectAction)
    self.wrs_subtask_b_planning_client = actionlib.SimpleActionClient('/wrs_subtask_b_planning', o2ac_task_planning_msgs.msg.PickPlaceWithRegraspAction)
    
    self.suction_client = actionlib.SimpleActionClient('/suction_control', o2ac_msgs.msg.SuctionControlAction)
    self.fastening_tool_client = actionlib.SimpleActionClient('/screw_tool_control', o2ac_msgs.msg.FastenerGripperControlAction)

    # Service clients
    self.ur_dashboard_clients = {
      "a_bot_get_loaded_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
      "b_bot_get_loaded_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
      "a_bot_program_running":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
      "b_bot_program_running":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
      "a_bot_load_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
      "b_bot_load_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
      "a_bot_play":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger),
      "b_bot_play":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger),
      "a_bot_stop":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/stop', std_srvs.srv.Trigger),
      "b_bot_stop":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/stop', std_srvs.srv.Trigger),
      "a_bot_quit":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/quit', std_srvs.srv.Trigger),
      "b_bot_quit":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/quit', std_srvs.srv.Trigger),
      "a_bot_connect":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/connect', std_srvs.srv.Trigger),
      "b_bot_connect":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/connect', std_srvs.srv.Trigger),
      "a_bot_close_popup":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/close_popup', std_srvs.srv.Trigger),
      "b_bot_close_popup":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/close_popup', std_srvs.srv.Trigger),
      "a_bot_unlock_protective_stop":rospy.ServiceProxy("/a_bot/ur_hardware_interface/dashboard/unlock_protective_stop", std_srvs.srv.Trigger),
      "b_bot_unlock_protective_stop":rospy.ServiceProxy("/b_bot/ur_hardware_interface/dashboard/unlock_protective_stop", std_srvs.srv.Trigger)
    }

    self.a_bot_set_io = rospy.ServiceProxy('a_bot/ur_hardware_interface/set_io', ur_msgs.srv.SetIO)
    self.b_bot_set_io = rospy.ServiceProxy('b_bot/ur_hardware_interface/set_io', ur_msgs.srv.SetIO)

    self.urscript_client = rospy.ServiceProxy('/o2ac_skills/sendScriptToUR', o2ac_msgs.srv.sendScriptToUR)
    self.publishMarker_client = rospy.ServiceProxy('/o2ac_skills/publishMarker', o2ac_msgs.srv.publishMarker)
    self.toggleCollisions_client = rospy.ServiceProxy('/o2ac_skills/toggleCollisions', std_srvs.srv.SetBool)

    # Subscribers
    # "robot_program_running" refers only to the ROS external control UR script, not just any program
    self.sub_a_bot_status_ = rospy.Subscriber("/a_bot/ur_hardware_interface/robot_program_running", Bool, self.a_bot_ros_control_status_callback) 
    self.sub_b_bot_status_ = rospy.Subscriber("/b_bot/ur_hardware_interface/robot_program_running", Bool, self.b_bot_ros_control_status_callback)
    self.sub_a_bot_gripper_status_ = rospy.Subscriber("/a_bot/gripper_status", robotiq_msgs.msg.CModelCommandFeedback, self.a_bot_gripper_status_callback)
    self.sub_b_bot_gripper_status_ = rospy.Subscriber("/b_bot/gripper_status", robotiq_msgs.msg.CModelCommandFeedback, self.b_bot_gripper_status_callback)
    self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
    self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
    self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)
    self.sub_suction_m4_ = rospy.Subscriber("/screw_tool_m4/screw_suctioned", Bool, self.suction_m4_callback)
    self.sub_suction_m3_ = rospy.Subscriber("/screw_tool_m3/screw_suctioned", Bool, self.suction_m3_callback)
    self.sub_robot_safety_mode_b_bot = rospy.Subscriber("/b_bot/ur_hardware_interface/safety_mode", ur_dashboard_msgs.msg.SafetyMode, self.b_bot_safety_mode_callback)
    self.robot_safety_mode = dict() 
    self.screw_is_suctioned = dict()
    
    # self.my_mutex = threading.Lock()

    self.resetTimerForDebugMonitor_client = rospy.ServiceProxy('/o2ac_debug_monitor/reset_timer', std_srvs.srv.Trigger)
    self.debugmonitor_publishers = dict() # used in log_to_debug_monitor()

    self.screw_tools = {}

    self.define_tray_views()

    self.objects_in_tray = dict()  # key: object ID. value: False or object pose

    rospy.sleep(.5)
    rospy.loginfo("Finished initializing class")

  def define_tray_views(self):
    """
    Define the poses used to position the camera to look into the tray.

    Example usage: self.go_to_pose_goal("b_bot", self.tray_view_high, 
                                        1end_effector_link="b_bot_outside_camera_color_frame", 
                                        1speed=.1, acceleration=.04)
    """
    high_height = .37
    low_height = .22
    x_offset = .04  # At low_height
    y_offset = .06  # At low_height

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "tray_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    ps.pose.position.z = high_height

    # Centered views (high up and close)
    self.tray_view_high = copy.deepcopy(ps)    
    ps.pose.position.z = low_height
    self.tray_view_low = copy.deepcopy(ps)

    # Close views in corners
    ps.pose.position.x = x_offset
    ps.pose.position.y = y_offset
    self.tray_view_close_front_b = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = y_offset
    self.tray_view_close_back_b = copy.deepcopy(ps)
    ps.pose.position.x = x_offset
    ps.pose.position.y = -y_offset
    self.tray_view_close_front_a = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = -y_offset
    self.tray_view_close_back_a = copy.deepcopy(ps)
    
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
    # self.my_mutex.acquire()
    self.run_mode_ = msg.data
    # self.my_mutex.release()
  def pause_mode_callback(self, msg):
    # self.my_mutex.acquire()
    self.pause_mode_ = msg.data
    # self.my_mutex.release()
  def test_mode_callback(self, msg):
    # self.my_mutex.acquire()
    self.test_mode_ = msg.data
    # self.my_mutex.release()
  def suction_m4_callback(self, msg):
    self.screw_is_suctioned["m4"] = msg.data
  def suction_m3_callback(self, msg):
    self.screw_is_suctioned["m3"] = msg.data
  def b_bot_safety_mode_callback(self, msg):
    self.robot_safety_mode["b_bot"] = msg.mode
  def a_bot_ros_control_status_callback(self, msg):
    self.ur_ros_control_running_on_robot["a_bot"] = msg.data
  def b_bot_ros_control_status_callback(self, msg):
    self.ur_ros_control_running_on_robot["b_bot"] = msg.data
  def a_bot_gripper_status_callback(self, msg):
    self.a_bot_gripper_opening_width = msg.position  # [m]
  def b_bot_gripper_status_callback(self, msg):
    self.b_bot_gripper_opening_width = msg.position  # [m]
  
  def activate_camera(self, camera_name="b_bot_outside_camera"):
    if self.camera_multiplexer:
      self.camera_multiplexer.activate_camera(camera_name)
    else:
      rospy.logwarn("Camera multiplexer not functional! Returning true")
      return True
  
  def activate_led(self, LED_name="b_bot", on=True):
    req = ur_msgs.srv.SetIORequest()
    if LED_name == "b_bot":
      req.fun = ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT
      req.pin = 4
      if on:
        req.state = ur_msgs.srv.SetIORequest.STATE_ON
      else:
        req.state = ur_msgs.srv.SetIORequest.STATE_OFF
      return self.b_bot_set_io.call(req)
    elif LED_name == "a_bot":
      return self.a_bot_set_io.call(req)
    else:
      rospy.logerr("Invalid LED name")

  def is_robot_running_normally(self, robot_name):
    """
    Returns true if the robot is running (no protective stop, not turned off etc).
    """
    return self.robot_safety_mode[robot_name] == 1 or self.robot_safety_mode[robot_name] == 2 # Normal / Reduced
  
  def is_robot_protective_stopped(self, robot_name):
    """
    Returns true if the robot is in protective stop.
    """
    return self.robot_safety_mode[robot_name] == 3

  def unlock_protective_stop(self, robot="b_bot"):
    if not self.use_real_robot:
      return True
    if robot is not "b_bot" and robot is not "a_bot":
      rospy.logerr("Robot name was not found!")
    
    service_client = self.ur_dashboard_clients[robot + "_unlock_protective_stop"]
    request = std_srvs.srv.TriggerRequest()
    start_time = time.time()
    rospy.loginfo("Attempting to unlock protective stop of " + robot)
    while not rospy.is_shutdown():
      response = service_client.call(request)
      if time.time() - start_time > 5.0:
        break
      if response.success:
        break
      rospy.sleep(0.2)
    if not response.success:
      rospy.logwarn("Could not unlock protective stop of " + robot + "!")
    return response.success

  def activate_ros_control_on_ur(self, robot="b_bot", recursion_depth=0):
    if not self.use_real_robot:
      return True
    
    # Check if URCap is already running on UR
    try:
      if self.ur_ros_control_running_on_robot[robot]:
        return True
    except:
      rospy.logerr("Robot name '" + robot + "' was not found or the robot is not a UR!")
      return False
    
    if recursion_depth > 10:
      rospy.logerr("Tried too often. Breaking out.")
      rospy.logerr("Could not start UR ROS control.")
      return False
    
    if rospy.is_shutdown():
      return False

    load_success = False
    try:
      # Load program if it not loaded already
      rospy.loginfo("Activating ROS control on robot " + robot)
      response = self.ur_dashboard_clients[robot + "_get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
      if response.program_name != "Loaded program: /programs/ROS_external_control.urp":
        request = ur_dashboard_msgs.srv.LoadRequest()
        request.filename = "ROS_external_control.urp"
        response = self.ur_dashboard_clients[robot + "_load_program"].call(request)
        if response.success: # Try reconnecting to dashboard
          load_success = True
        else:
          rospy.logerr("Could not load the ROS_external_control.urp URCap. Is the UR in Remote Control mode and program installed with correct name?")
    except:
      rospy.logwarn("Dashboard service did not respond!")
    
    if not load_success:
      rospy.logwarn("Waiting and trying again.")
      try:
        if recursion_depth > 0:  # If connect alone failed, try quit and then connect
          response = self.ur_dashboard_clients[robot + "_quit"].call()
          rospy.sleep(.5)
        response = self.ur_dashboard_clients[robot + "_connect"].call()
      except:
        rospy.logwarn("Dashboard service did not respond! (2)")
        pass
      rospy.sleep(.5)
      return self.activate_ros_control_on_ur(robot, recursion_depth=recursion_depth+1)
    
    # Run the program
    response = self.ur_dashboard_clients[robot + "_play"].call(std_srvs.srv.TriggerRequest())
    rospy.sleep(2)
    if not response.success:
      rospy.logerr("Could not start UR control. Is the UR in Remote Control mode and program installed with correct name?")
      return False
    else:
      # Check if controller is running
      self.check_for_dead_controller_and_force_start("b_bot")
      rospy.loginfo("Successfully activated ROS control on robot " + robot)
      return True

  def check_for_dead_controller_and_force_start(self, robot="b_bot"):
    service_proxy_list = rospy.ServiceProxy('/b_bot/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
    service_proxy_switch = rospy.ServiceProxy('/b_bot/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)
    rospy.sleep(2)

    list_req = controller_manager_msgs.srv.ListControllersRequest()
    switch_req = controller_manager_msgs.srv.SwitchControllerRequest()

    list_res = service_proxy_list.call(list_req)
    print("======")
    print(list_res)
    for c in list_res.controller:
      if c.name == "scaled_pos_joint_traj_controller":
        if c.state == "stopped":
          # Force restart
          rospy.logerr("Force controller start")
          switch_req.start_controllers = ['scaled_pos_joint_traj_controller']
          switch_req.strictness = 1
          switch_res = service_proxy_switch.call(switch_req)
          return switch_res.ok
        else:
          return True

  def load_program(self, robot="b_bot", program_name="", recursion_depth=0):
    if not self.use_real_robot:
      return True

    if recursion_depth > 4:
      rospy.logerr("Tried too often. Breaking out.")
      rospy.logerr("Could not load " + program_name + ". Is the UR in Remote Control mode and program installed with correct name?")
      return False
    
    load_success = False
    try:
      # Try to stop running program
      self.ur_dashboard_clients[robot + "_stop"].call(std_srvs.srv.TriggerRequest())
      rospy.sleep(.5)

      # Load program if it not loaded already
      request = ur_dashboard_msgs.srv.LoadRequest()
      request.filename = program_name
      response = self.ur_dashboard_clients[robot + "_load_program"].call(request)
      if response.success: # Try reconnecting to dashboard
        load_success = True
        return True
      else:
        rospy.logerr("Could not load " + program_name + ". Is the UR in Remote Control mode and program installed with correct name?")
    except:
      rospy.logwarn("Dashboard service did not respond!")
    if not load_success:
      rospy.logwarn("Waiting and trying again")
      rospy.sleep(3)
      try:
        if recursion_depth > 0:  # If connect alone failed, try quit and then connect
          response = self.ur_dashboard_clients[robot + "_quit"].call()
          rospy.logerr("Program could not be loaded on UR: " + program_name)
          rospy.sleep(.5)
      except:
        rospy.logwarn("Dashboard service did not respond! (2)")
        pass
      response = self.ur_dashboard_clients[robot + "_connect"].call()
      rospy.sleep(.5)
      return self.load_program(robot, program_name=program_name, recursion_depth=recursion_depth+1)
  
  def execute_loaded_program(self, robot="b_bot"):
    # Run the program
    response = self.ur_dashboard_clients[robot + "_play"].call(std_srvs.srv.TriggerRequest())
    if not response.success:
      rospy.logerr("Could not start program. Is the UR in Remote Control mode and program installed with correct name?")
      return False
    else:
      rospy.loginfo("Successfully started program on robot " + robot)
      return True
  
  def close_ur_popup(self, robot="b_bot"):
    # Close a popup on the teach pendant to continue program execution
    response = self.ur_dashboard_clients[robot + "_close_popup"].call(std_srvs.srv.TriggerRequest())
    if not response.success:
      rospy.logerr("Could not close popup.")
      return False
    else:
      rospy.loginfo("Successfully closed popup on teach pendant of robot " + robot)
      return True
  
  def publish_marker(self, pose_stamped, marker_type):
    # Publishes a marker to Rviz for visualization
    if self.disable_markers:
      return True
    req = o2ac_msgs.srv.publishMarkerRequest()
    req.marker_pose = pose_stamped
    req.marker_type = marker_type
    self.publishMarker_client.call(req)
    return True

  def define_tool_collision_objects(self):
    # TODO: Use o2ac_assembly_database to load these instead
    screw_tool_m3 = moveit_msgs.msg.CollisionObject()
    screw_tool_m4 = moveit_msgs.msg.CollisionObject()
    
    #M4 tool
    screw_tool_m4.header.frame_id = "screw_tool_m4_link"
    screw_tool_m4.id = "screw_tool_m4"

    # The bit cushion and motor
    screw_tool_m4.primitives = [SolidPrimitive() for _ in range(3)] # instead of resize()
    screw_tool_m4.primitive_poses = [Pose() for _ in range(3)] 
    
    screw_tool_m4.primitives[0].type = SolidPrimitive.BOX
    screw_tool_m4.primitives[0].dimensions = [.026, .04, .055]
    screw_tool_m4.primitive_poses[0].position.x = 0
    screw_tool_m4.primitive_poses[0].position.y = -0.009
    screw_tool_m4.primitive_poses[0].position.z = 0.0275

    # The "shaft" + suction attachment
    screw_tool_m4.primitives[1].type = SolidPrimitive.BOX
    screw_tool_m4.primitives[1].dimensions = [.019, .03, .08]
    screw_tool_m4.primitive_poses[1].position.x = 0
    screw_tool_m4.primitive_poses[1].position.y = -0.0055  # 21 mm distance from axis
    screw_tool_m4.primitive_poses[1].position.z = -0.04

    # The cylinder representing the tip
    screw_tool_m4.primitives[2].type = SolidPrimitive.CYLINDER
    screw_tool_m4.primitives[2].dimensions = [.038, .0035] # Cylinder height, radius
    screw_tool_m4.primitive_poses[2].position.x = 0
    screw_tool_m4.primitive_poses[2].position.y = 0  # 21 mm distance from axis
    screw_tool_m4.primitive_poses[2].position.z = -0.099
    screw_tool_m4.operation = screw_tool_m4.ADD

    # The tool tip
    screw_tool_m4.subframe_poses = [Pose()]
    screw_tool_m4.subframe_names = [""]
    screw_tool_m4.subframe_poses[0].position.z = -.12
    screw_tool_m4.subframe_poses[0].orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 90.0/360.0*tau, -tau/4))
    screw_tool_m4.subframe_names[0] = "screw_tool_m4_tip"


    # M3 tool
    screw_tool_m3.header.frame_id = "screw_tool_m3_link"
    screw_tool_m3.id = "screw_tool_m3"

    # The bit cushion and motor
    screw_tool_m3.primitives = [SolidPrimitive() for _ in range(3)]
    screw_tool_m3.primitive_poses = [Pose() for _ in range(3)] 
    screw_tool_m3.primitives[0].type = SolidPrimitive.BOX
    screw_tool_m3.primitives[0].dimensions = [.026, .04, .055]
    screw_tool_m3.primitive_poses[0].position.x = 0
    screw_tool_m3.primitive_poses[0].position.y = -0.009
    screw_tool_m3.primitive_poses[0].position.z = 0.0275

    # The "shaft" + suction attachment
    screw_tool_m3.primitives[1].type = SolidPrimitive.BOX
    screw_tool_m3.primitives[1].dimensions = [.019, .03, .08]
    screw_tool_m3.primitive_poses[1].position.x = 0
    screw_tool_m3.primitive_poses[1].position.y = -0.0055  # 21 mm distance from axis
    screw_tool_m3.primitive_poses[1].position.z = -0.04

    # The cylinder representing the tip
    screw_tool_m3.primitives[2].type = SolidPrimitive.CYLINDER
    screw_tool_m3.primitives[2].dimensions = [.018, .0035] # Cylinder height, radius
    screw_tool_m3.primitive_poses[2].position.x = 0
    screw_tool_m3.primitive_poses[2].position.y = 0  # 21 mm distance from axis
    screw_tool_m3.primitive_poses[2].position.z = -0.089
    screw_tool_m3.operation = screw_tool_m3.ADD

    # The tool tip
    screw_tool_m3.subframe_poses = [Pose()]
    screw_tool_m3.subframe_names = [""]
    screw_tool_m3.subframe_poses[0].position.z = -.11
    screw_tool_m3.subframe_poses[0].orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 90.0/180.0*pi, -pi/2))
    screw_tool_m3.subframe_names[0] = "screw_tool_m3_tip"

    self.screw_tools["screw_tool_m3"] = screw_tool_m3
    self.screw_tools["screw_tool_m4"] = screw_tool_m4
    # TODO(felixvd): Add the set screw and nut tool objects from the C++ file

    # TODO: Write these to a YAML file
    return True

  ############# ------ Robot motion functions

  def get_current_pose_stamped(self, group_name):
    group = self.groups[group_name]
    return group.get_current_pose()

  def get_current_pose(self, group_name):
    group = self.groups[group_name]
    return group.get_current_pose().pose
  
  def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, high_precision = False, 
                      end_effector_link = "", move_lin = True):
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit
    if move_lin:
      return self.move_lin(group_name, pose_goal_stamped, speed, acceleration, end_effector_link)
    self.publish_marker(pose_goal_stamped, "pose")
    self.activate_ros_control_on_ur(group_name)
    group = self.groups[group_name]
    
    if not end_effector_link:
      if group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_robotiq_85_tip_link"
    group.set_end_effector_link(end_effector_link)
    
    group.set_pose_target(pose_goal_stamped)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    if high_precision:
      group.set_goal_tolerance(.000001)
      group.set_planning_time(10)

    move_success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    
    # Reset the precision
    if high_precision:
      group.set_goal_tolerance(.0001) 
      group.set_planning_time(3) 

    current_pose = group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01), move_success

  def transformTargetPoseFromTipLinkToURTCP(self, ps, robot_name, end_effector_link):
    # This transforms a pose from the end_effector_link set in MoveIt to the TCP used in the UR controller. 
    # It is used when sending commands to the UR controller directly, without MoveIt/ROS controllers.
    rospy.logdebug("Received pose to transform to TCP link:")
    rospy.logdebug(str(ps.pose.position.x) + ", " + str(ps.pose.position.y)  + ", " + str(ps.pose.position.z))
    rospy.logdebug(str(ps.pose.orientation.x) + ", " + str(ps.pose.orientation.y)  + ", " + str(ps.pose.orientation.z)  + ", " + str(ps.pose.orientation.w))

    t = self.listener.lookupTransform(end_effector_link, robot_name + "_tool0", rospy.Time())

    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = ps.header.frame_id
    m.child_frame_id = "temp_goal_pose__"
    m.transform.translation.x = ps.pose.position.x
    m.transform.translation.y = ps.pose.position.y
    m.transform.translation.z = ps.pose.position.z
    m.transform.rotation.x = ps.pose.orientation.x
    m.transform.rotation.y = ps.pose.orientation.y
    m.transform.rotation.z = ps.pose.orientation.z
    m.transform.rotation.w = ps.pose.orientation.w
    self.listener.setTransform(m)

    m.header.frame_id = "temp_goal_pose__"
    m.child_frame_id = "temp_wrist_pose__"
    m.transform.translation.x = t[0][0]
    m.transform.translation.y = t[0][1]
    m.transform.translation.z = t[0][2]
    m.transform.rotation.x = t[1][0]
    m.transform.rotation.y = t[1][1]
    m.transform.rotation.z = t[1][2]
    m.transform.rotation.w = t[1][3]
    self.listener.setTransform(m)

    ps_wrist = geometry_msgs.msg.PoseStamped()
    ps_wrist.header.frame_id = "temp_wrist_pose__"
    ps_wrist.pose.orientation.w = 1.0

    ps_new = self.listener.transformPose(ps.header.frame_id, ps_wrist)

    rospy.logdebug("New pose:")
    rospy.logdebug(str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y)  + ", " + str(ps_new.pose.position.z))
    rospy.logdebug(str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y)  + ", " + str(ps_new.pose.orientation.z)  + ", " + str(ps_new.pose.orientation.w))

    return ps_new

  def move_lin(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, end_effector_link = ""):
    self.publish_marker(pose_goal_stamped, "pose")
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
        rospy.logdebug("Real robot is being used. Send linear motion to robot controller directly via URScript.")
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move"
        req.robot_name = group_name
        req.target_pose = self.transformTargetPoseFromTipLinkToURTCP(pose_goal_stamped, group_name, end_effector_link)
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name, rospy.Duration.from_sec(30.0))
        return res.success

    # 
    if speed > 1.0:
      speed = 1.0
    
    self.activate_ros_control_on_ur(group_name)
    group = self.groups[group_name]

    group.set_end_effector_link(end_effector_link)
    group.set_pose_target(pose_goal_stamped)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    
    # FIXME: At the start of the program, get_current_pose() did not return the correct value. Should be a bug report.
    waypoints = []
    ### The current pose is not added anymore, because it causes a bug in Gazebo, and it is not necessary.
    # wpose1 = group.get_current_pose().pose
    # # rospy.loginfo("Wpose1:")
    # # rospy.loginfo(wpose1)
    # rospy.sleep(.05)
    # wpose2 = group.get_current_pose().pose
    # # rospy.loginfo("Wpose2:")
    # # rospy.loginfo(wpose2)
    # waypoints.append(wpose2)
    pose_goal_world = self.listener.transformPose("world", pose_goal_stamped).pose
    waypoints.append(pose_goal_world)
    (plan, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    rospy.loginfo("Compute cartesian path succeeded with " + str(fraction*100) + "%")
    plan = group.retime_trajectory(self.robots.get_current_state(), plan, speed)

    plan_success = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    return plan_success

  def move_lin_rel(self, robot_name, relative_translation = [0,0,0], relative_rotation = [0,0,0], acceleration = 0.5, velocity = .03, use_robot_base_csys=False, wait = True, max_wait=30.0):
    '''
    Does a lin_move relative to the current position of the robot. Uses the robot's TCP.

    robot_name = "b_bot" for example
    relative_translation: translatory movement relative to current tcp position, expressed in robot's own base frame
    relative_rotation: rotatory movement relative to current tcp position, expressed in robot's own base frame
    use_robot_base_csys: If true, uses the robot_base coordinates for the relative motion (not workspace_center!)
    '''
    # Uses UR coordinates
    if not self.use_real_robot:
      return True
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

  def move_joints(self, group_name, joint_pose_goal, speed = 1.0, acceleration = 0.0, force_ur_script=False, force_moveit=False):
    if self.pause_mode_ or self.test_mode_:
      if speed > self.reduced_mode_speed_limit:
        rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
        speed = self.reduced_mode_speed_limit
    if force_ur_script and self.use_real_robot:
      if not force_moveit:
        rospy.logdebug("Real robot is being used. Send joint command to robot controller directly via URScript.") 
        req = o2ac_msgs.srv.sendScriptToURRequest()
        req.program_id = "move_j"
        req.robot_name = group_name
        req.joint_positions = joint_pose_goal
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name, rospy.Duration.from_sec(20.0))
        return res.success
    
    if speed > 1.0:
      speed = 1.0
    self.activate_ros_control_on_ur(group_name)
    self.groups[group_name].set_joint_value_target(joint_pose_goal)
    self.groups[group_name].set_max_velocity_scaling_factor(speed)
    return self.groups[group_name].go(wait=True)

  def move_both_robots(self, pose_goal_a_bot, pose_goal_b_bot, speed = 0.05):
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

    success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    rospy.loginfo("Received:")
    rospy.loginfo(success)
    return success

  def horizontal_spiral_motion(self, robot_name, max_radius = .01, radius_increment = .001, speed = 0.02, spiral_axis="Z"):
    rospy.loginfo("Performing horizontal spiral motion at speed " + str(speed) + " and radius " + str(max_radius))
    if not self.use_real_robot:
      return True
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.program_id = "spiral_motion"
    req.robot_name = robot_name
    req.max_radius = max_radius
    req.radius_increment = radius_increment    
    req.velocity = speed
    req.spiral_axis = spiral_axis
    res = self.urscript_client.call(req)
    wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
    return res.success
    # =====

  def go_to_named_pose(self, pose_name, robot_name, speed = 0.5, acceleration = 0.0, force_ur_script=False):
    """
    pose_name should be a named pose in the moveit_config, such as "home", "back" etc.
    """
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
        self.move_joints(robot_name, joint_pose, speed, acceleration, force_ur_script=force_ur_script)
        return True
    if speed > 1.0:
      speed = 1.0
    self.activate_ros_control_on_ur(robot_name)
    self.groups[robot_name].set_named_target(pose_name)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    self.groups[robot_name].set_max_velocity_scaling_factor(speed)
    self.groups[robot_name].go(wait=True)
    # self.groups[robot_name].stop()
    self.groups[robot_name].clear_pose_targets()
    return True

  ######

  def pick_screw_from_feeder(self, robot_name, screw_size):
    """
    Picks a screw from one of the feeders. The screw tool already has to be equipped!
    Use this command to equip the screw tool: do_change_tool_action(self, "b_bot", equip=True, screw_size = 4)
    """
    goal = o2ac_msgs.msg.pickScrewFromFeederGoal()
    goal.robot_name = robot_name
    goal.screw_size = screw_size
    rospy.loginfo("Sending pickScrewFromFeeder action goal")
    rospy.logdebug(goal)

    self.pick_screw_from_feeder_client.send_goal(goal)
    rospy.logdebug("Waiting for result")
    self.pick_screw_from_feeder_client.wait_for_result()
    rospy.logdebug("Getting result")
    return self.pick_screw_from_feeder_client.get_result()

  def do_place_action(self, robot_name, pose_stamped, tool_name = "", screw_size=0):
    # Call the place action
    goal = o2ac_msgs.msg.placeGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    goal.screw_size = screw_size
    rospy.loginfo("Sending place action goal")
    rospy.logdebug(goal)

    self.place_client.send_goal(goal)
    rospy.logdebug("Waiting for result")
    self.place_client.wait_for_result()
    rospy.logdebug("Getting result")
    return self.place_client.get_result()

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

  def do_change_tool_action(self, robot_name, equip=True, 
                        screw_size = 4):
    # self.equip_unequip_tool(robot_name, screw_tool_id, angle=0.0, equip_or_unequip=equip)
    # return
    ### DEPRECATED
    self.log_to_debug_monitor("Change tool", "operation")
    goal = o2ac_msgs.msg.changeToolGoal()
    goal.robot_name = robot_name
    goal.equip_the_tool = equip
    goal.screw_size = screw_size
    rospy.loginfo("Sending changeTool action goal.")    
    self.change_tool_client.send_goal(goal)
    self.change_tool_client.wait_for_result()
    return self.change_tool_client.get_result()
  
  def do_screw_action(self, robot_name, target_hole, screw_height = 0.02, 
                        screw_size = 4, stay_put_after_screwing=False):
    goal = o2ac_msgs.msg.screwGoal()
    goal.target_hole = target_hole
    goal.screw_height = screw_height
    goal.screw_size = screw_size
    goal.robot_name = robot_name
    goal.stay_put_after_screwing = stay_put_after_screwing
    rospy.loginfo("Sending screw action goal.")
    self.screw_client.send_goal(goal)
    self.screw_client.wait_for_result()
    return self.screw_client.get_result()

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
    upload_mtc_modules_initial_params()

    # Spawn tools and objects
    self.define_tool_collision_objects()
    screw_ids = ['m3', 'm4']
    for screw_id in screw_ids:
      self.spawn_tool('screw_tool_' + screw_id)
      self.upload_tool_grasps_to_param_server(screw_id)
    spawn_objects(assembly_name, objects, poses, reference_frame)
  
  def get_3d_poses_from_ssd(self):
    """
    Returns object poses as estimated by the SSD neural network and reprojection.
    Also updates self.objects_in_tray
    """
    # Send goal, wait for result
    self.ssd_client.send_goal(o2ac_msgs.msg.get3DPosesFromSSDGoal())
    if (not self.ssd_client.wait_for_result(rospy.Duration(2.0))):
      self.ssd_client.cancel_goal()  # Cancel goal if timeout expired
      rospy.logerr("Call for SSD result returned no result. Is o2ac_vision running?")
      return False

    # Read result and return
    try:
      res = self.ssd_client.get_result()
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
    # Send goal, wait for result
    goal = o2ac_msgs.msg.detectAngleGoal()
    goal.item_id = "bearing"
    goal.camera_id = camera
    self.detect_angle_client.send_goal(goal)
    if (not self.detect_angle_client.wait_for_result(rospy.Duration(3.0))):
      self.detect_angle_client.cancel_goal()  # Cancel goal if timeout expired
      rospy.logerr("Call to detect angle returned no result. Is o2ac_vision running?")
      return False

    # Read result and return
    try:
      res = self.detect_angle_client.get_result()
      if res.succeeded:
        return res.rotation_angle
    except:
      pass
    return False
  
  def call_shaft_notch_detection(self):
    """
    Calls the action and returns the result as is
    """
    goal = o2ac_msgs.msg.shaftNotchDetectionGoal()
    self.detect_shaft_client.send_goal(goal)
    if (not self.detect_shaft_client.wait_for_result(rospy.Duration(3.0))):
      self.detect_shaft_client.cancel_goal()  # Cancel goal if timeout expired
      rospy.logerr("Call to shaft detection returned no result. Is o2ac_vision running?")
      return False
    res = self.detect_shaft_client.get_result()
    return res

  # def publish_ssd_results_to_scene(self):
  #   """
  #   As a first start. Use SSD results to place collision objects in 
  #   """
  #   rospy.loginfo("Detected " + item_name + " with confidence " + str(res.confidences[0]))
  #   co = self.assembly_reader._reader.get_collision_object("bearing")
  #   co.mesh_poses[0] = res.detected_poses[0].pose
  #   co.header.frame_id = "tray_center"
  #   print(co)
  #   self.planning_scene_interface.apply_collision_object(co)
  
  def detect_object_in_camera_view(self, item_name):
    """
    Returns object pose if object was detected in current camera view and published to planning scene,
    False otherwise.
    """
    # TODO: merge with "look_and_get_grasp_points"

    # Send goal, wait for result
    object_type = self.assembly_database.name_to_type(item_name)
    if not object_type:
      rospy.logerr("Could not find the object " + name + " in database, or its type field is empty.")
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
      co = self.assembly_database.get_collision_object(object_name=name)
      # TODO: Update MoveIt to use the object_pose
      co.mesh_poses[0] = res.detected_poses[0].pose
      co.header.frame_id = "tray_center"
      self.planning_scene_interface.apply_collision_object(co)
      return res.detected_poses[0]
    else:
      rospy.loginfo("Did not detect " + item_name)
      return False

  def do_plan_pick_action(self, object_name, grasp_parameter_location = '', lift_direction_reference_frame = '', lift_direction = [], robot_name = ''):
    '''
    Function for calling the action for pick planning
    The function returns the action result that contains the trajectories for the motion plan
    '''
    goal = moveit_task_constructor_msgs.msg.PickObjectGoal()
    goal.object_name = object_name
    goal.grasp_parameter_location = grasp_parameter_location
    goal.lift_direction_reference_frame = lift_direction_reference_frame
    goal.lift_direction = lift_direction
    goal.robot_name = robot_name
    rospy.loginfo("Sending pick planning goal.")
    self.pick_planning_client.send_goal(goal)
    self.pick_planning_client.wait_for_result()
    return self.pick_planning_client.get_result()

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

  def set_motor(self, motor_name, direction = "tighten", wait=False, speed = 0, duration = 0):
    if not self.use_real_robot:
      return True
    goal = o2ac_msgs.msg.FastenerGripperControlGoal()
    goal.fastening_tool_name = motor_name
    goal.direction = direction
    goal.speed = speed
    goal.duration = duration
    rospy.loginfo("Sending fastening_tool action goal.")
    self.fastening_tool_client.send_goal(goal)
    if wait:
      self.fastening_tool_client.wait_for_result()
    return self.fastening_tool_client.get_result()

  def set_suction(self, tool_name, suction_on=False, eject=False, wait=True):
    if not self.use_real_robot:
      return True
    goal = o2ac_msgs.msg.SuctionControlGoal()
    goal.fastening_tool_name = tool_name
    goal.turn_suction_on = suction_on
    goal.eject_screw = eject
    rospy.loginfo("Sending suction action goal.")
    self.suction_client.send_goal(goal)
    if wait:
      self.suction_client.wait_for_result(rospy.Duration(2.0))
    return self.suction_client.get_result()

  def do_insertion(self, robot_name, max_insertion_distance= 0.0, 
                        max_approach_distance = 0.0, max_force = .0,
                        max_radius = 0.0, radius_increment = .0,
                        peck_mode=False,
                        wait = True, horizontal=False):
    if not self.use_real_robot:
      return True
    # Directly calls the UR service rather than the action of the skill_server
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "insert"
    if horizontal:
      req.program_id = "horizontal_insertion"

    #  Original defaults:
    # max_approach_distance = .1, max_force = 5,
    #                     max_radius = .001, radius_increment = .0001,
    req.max_insertion_distance = max_insertion_distance
    req.max_approach_distance = max_approach_distance
    req.max_force = max_force
    req.peck_mode = peck_mode
    req.max_radius = max_radius
    req.radius_increment = radius_increment
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(2.0)
      wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
    return res.success

  def do_spiral_search(self, robot_name, max_insertion_distance= 0.0, 
                        max_approach_distance = 0.0, max_force = .0,
                        max_radius = 0.0, radius_increment = .0,
                        peck_mode=False, wait = True):
    if not self.use_real_robot:
      return True
    # Directly calls the UR service rather than the action of the skill_server
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "spiral"

    #  Original defaults:
    # max_approach_distance = .1, max_force = 5,
    #                     max_radius = .001, radius_increment = .0001,
    req.max_insertion_distance = max_insertion_distance
    req.max_approach_distance = max_approach_distance
    req.max_force = max_force
    req.peck_mode = peck_mode
    req.max_radius = max_radius
    req.radius_increment = radius_increment
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(2.0)
      wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
    return res.success
  
  def do_helix_motion(self, robot_name, max_force = 50,
                        helix_forward_axis = "Z+",
                        helix_forward_increment = 0.01, helix_forward_limit = 0.1,
                        max_radius = 0.005, radius_increment = .005,
                        wait = True):
    if not self.use_real_robot:
      return True
    rospy.loginfo("Performing helix motion with radius " + str(max_radius) + " and forward limit " + str(helix_forward_limit))
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "helix_motion"
    req.helix_forward_axis = helix_forward_axis
    req.helix_forward_increment = helix_forward_increment
    req.helix_forward_limit = helix_forward_limit
    req.max_force = max_force
    req.max_radius = max_radius
    req.radius_increment = radius_increment
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(2.0)
      wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(60.0))
    return res.success

  def movelin_around_shifted_tcp(self, robot_name, wait = True, desired_twist = [0,0,0,0,0,0], tcp_position = [0.0,0.0,0.0,0.0,0.0,0.0],
                        velocity = 0.1, acceleration = 0.02):
    """
    Shifts the TCP to tcp_position (in the robot wrist frame) and moves by desired_twist.

    For the UR, this sets the TCP inside the UR controller and uses the movel command. The TCP is reset afterwards.
    
    The desired twist is the desired relative motion and should be in the coordinates of the shifted TCP. This method is used to 
    rotate around the tip of the workpiece during the alignment for adaptive insertion, when the robot touches the hole with the peg to find its position precisely.
    Using the position calculated from the robot link lengths and reported joint angles would introduce too much of an error.
    """
    if not robot_name == "b_bot":
      rospy.logerr("This is not yet implemented for non-UR robots!")
      return False
    if not self.use_real_robot:
      return True
    # Directly calls the UR service rather than the action of the skill_server
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "tcp_movement"
    req.desired_twist = desired_twist
    req.tcp_pose = tcp_position
    req.velocity = velocity
    req.acceleration = acceleration
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(2.0)
      wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
    return res.success
  
  def set_tcp_in_ur(self, robot_name, tcp_pose = [0.0,0.0,0.0,0.0,0.0,0.0]):
    """
    Change the TCP inside the UR controller (use with caution!)
    """
    if not self.use_real_robot:
      return True
    # Directly calls the UR service rather than the action of the skill_server
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "set_tcp"
    req.tcp_pose = tcp_pose
    res = self.urscript_client.call(req)
    return res.success

  def do_linear_push(self, robot_name, force, wait = True, direction = "Z+", max_approach_distance=0.1, forward_speed=0.0, acceleration = 0.05, direction_vector=[0, 0, 0], use_base_coords=False):
    if not self.use_real_robot:
      return True
    # Directly calls the UR service rather than the action of the skill_server
    req = o2ac_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.max_force = force
    req.force_direction = direction
    req.direction_vector = direction_vector
    if sum(direction_vector) != 0:
      req.force_direction = "using_direction_vector"  # This overwrites the default argument
    req.max_approach_distance = max_approach_distance
    req.forward_speed = forward_speed
    req.use_base_coords = use_base_coords
    req.program_id = "linear_push"
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(1.0)
      wait_for_UR_program("/" + robot_name, rospy.Duration.from_sec(30.0))
    return res.success

  def do_regrasp(self, giver_robot_name, receiver_robot_name, grasp_distance = .02):
    """The item goes from giver to receiver."""
    goal = o2ac_msgs.msg.regraspGoal()
    goal.giver_robot_name = giver_robot_name
    goal.receiver_robot_name = receiver_robot_name
    goal.grasp_distance = grasp_distance

    self.regrasp_client.send_goal(goal)
    rospy.loginfo("Performing regrasp with grippers " + giver_robot_name + " and " + receiver_robot_name)
    self.regrasp_client.wait_for_result(rospy.Duration(90.0))
    result = self.regrasp_client.get_result()
    return result

  def toggle_collisions(self, collisions_on):
    """Turns collisions in MoveIt on and off. Use with caution!"""
    req = std_srvs.srv.SetBoolRequest()
    req.data = collisions_on
    res = self.toggleCollisions_client.call(req)
    return res.success

  def close_gripper(self, robot, force=40.0, wait=True):
    return self.send_gripper_command(robot, "close", force=force, wait=wait)
  def open_gripper(self, robot, wait=True):
    return self.send_gripper_command(robot, "open", wait=wait)

  def send_gripper_command(self, gripper, command, this_action_grasps_an_object = False, force = 40.0, velocity = .1, wait=True):
    """
    force: Gripper force in N. From 40 to 100
    velocity: Gripper speed. From 0.013 to 0.1

    Use a slow closing speed when using a low gripper force, or the force might be unexpectedly high.
    """
    if not self.use_real_robot:
      # TODO: Set the gripper width
      return True
    if gripper == "b_bot" or gripper == "a_bot":
      goal = robotiq_msgs.msg.CModelCommandGoal()
      action_client = self.gripper_action_clients[gripper]
      goal.velocity = velocity   
      goal.force = force         
      if command == "close":
        goal.position = 0.0
      elif command == "open":
        goal.position = 0.140
      else:
        goal.position = command     # This sets the opening width directly
        rospy.loginfo(command)
    else:
      try:
        rospy.logerr("Could not parse gripper command: " + command + " for gripper " + gripper)
      except:
        pass

    action_client.send_goal(goal)
    rospy.sleep(.5)
    rospy.loginfo("Sending command " + str(command) + " to gripper: " + gripper)
    if wait:
      action_client.wait_for_result(rospy.Duration(6.0))  # Default wait time: 6 s
    result = action_client.get_result()
    rospy.loginfo(result)
    return 

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
      self.groups[robot_name].attach_object(toolname, robot_name + "_ee_link", touch_links= 
      [robot_name + "_robotiq_85_tip_link", 
      robot_name + "_robotiq_85_left_finger_tip_link", 
      robot_name + "_robotiq_85_left_inner_knuckle_link", 
      robot_name + "_robotiq_85_right_finger_tip_link", 
      robot_name + "_robotiq_85_right_inner_knuckle_link"])
    except:
      rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

  def detach_tool(self, robot_name, toolname):
    try:
      self.groups[robot_name].detach_object(toolname)
    except:
      rospy.logerr(item_id_to_attach + " could not be detached! robot_name = " + robot_name)

  def equip_tool(self, robot_name, tool_name):
    return self.equip_unequip_tool(robot_name, tool_name, "equip")
  
  def unequip_tool(self, robot_name, tool_name):
    return self.equip_unequip_tool(robot_name, tool_name, "unequip")

  def equip_unequip_tool(self, robot_name, tool_name, equip_or_unequip):
    # TODO(felixvd): Finish this function. It is duplicated in C++.

    # Sanity check on the input instruction
    equip = (equip_or_unequip == "equip")
    unequip = (equip_or_unequip == "unequip")

    return self.do_change_tool_action(self, robot_name, equip=equip, screw_size = 4)
    ###
    lin_speed = 0.01
    # The second comparison is not always necessary, but readability comes first.
    if ((not equip) and (not unequip)):
      rospy.logerr("Cannot read the instruction " + equip_or_unequip + ". Returning False.")
      return False

    if ((self.robot_status[robot_name].carrying_object == True)):
      rospy.logerr("Robot holds an object. Cannot " + equip_or_unequip + " tool.")
      return False
    if ( (self.robot_status[robot_name].carrying_tool == True) and equip):
      rospy.logerr("Robot already holds a tool. Cannot equip another.")
      return False
    if ( (self.robot_status[robot_name].carrying_tool == False) and unequip):
      rospy.logerr("Robot is not holding a tool. Cannot unequip any.")
      return False
    
    rospy.loginfo("Going to before_tool_pickup pose.")
    
    if tool_name == "screw_tool_m3" or tool_name == "screw_tool_m4" or tool_name == "nut_tool_m6" or tool_name == "set_screw_tool":
      tool_holder_used = "back"
    elif tool_name == "belt_tool" or tool_name == "plunger_tool":
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
      ps_approach.pose.position.z = -.025
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

    if unequip: 
      ps_in_holder.pose.position.x -= 0.001   # Don't move all the way into the magnet
      ps_approach.pose.position.z -= 0.01 # Approach diagonally so nothing gets stuck

    if equip:
      self.open_gripper(robot_name)
      rospy.loginfo("Spawning tool.")
      if not self.spawn_tool(tool_name):
        rospy.logerr("Could not spawn the tool. Abort.")
        return False
      held_screw_tool_ = tool_name

    rospy.loginfo("Moving to screw tool approach pose LIN.")
    self.go_to_pose_goal(robot_name, ps_approach, move_lin=True)
  
    # Plan & execute linear motion to the tool change position
    rospy.loginfo("Moving to pose in tool holder LIN.")
    if equip:
      lin_speed = 0.5
    elif unequip:
      lin_speed = 0.08 

    self.go_to_pose_goal(robot_name, ps_in_holder, speed=lin_speed, move_lin=True)
  
    # Close gripper, attach the tool object to the gripper in the Planning Scene.
    # Its collision with the parent link is set to allowed in the original planning scene.
    if equip:
      rospy.loginfo("Closing the gripper.")
      self.close_gripper(robot_name)
      self.attach_tool(robot_name, tool_name)
      self.allow_collisions_with_robot_hand(tool_name, robot_name)
      self.allow_collisions_with_robot_hand('screw_tool_holder', robot_name)  # TODO(felixvd): Is this required?
      self.robot_status[robot_name].carrying_tool = True
      self.robot_status[robot_name].held_tool_id = tool_name
    elif unequip:
      self.open_gripper(robot_name)
      self.detach_tool(robot_name, tool_name)
      held_screw_tool_ = ""
      self.robot_status[robot_name].carrying_tool = False
      self.robot_status[robot_name].held_tool_id = ""

    rospy.sleep(.5)
    
    # Plan & execute linear motion away from the tool change position
    rospy.loginfo("Moving back to screw tool approach pose LIN.")
    
    if equip:
      lin_speed = 1.0
    elif unequip:
      lin_speed = 1.0

    ### This block is probably not needed anymore?
    # if self.use_real_robot:
    #   rospy.sleep(.3)
    #   UR_srv.request.velocity = .05
    #   t_rel.z = -(ps_tool_holder.pose.position.x - ps_approach.pose.position.x)
    #   UR_srv.request.relative_translation = t_rel
    #   sendScriptToURClient_.call(UR_srv)
    #   if (UR_srv.response.success == True):
    #     rospy.loginfo("Successfully called the URScript client to perform a linear movement backward.")
    #     waitForURProgram("/" + robot_name + "_controller")
    #   else:
    #     ROS_WARN("Could not call the URScript client to perform a linear movement backward.")
    # else:
    #   self.go_to_pose_goal(robot_name, ps_move_away, speed=lin_speed, move_lin=True)
    self.go_to_pose_goal(robot_name, ps_move_away, speed=lin_speed, move_lin=True)

    
    # Reactivate the collisions, with the updated entry about the tool
    # planning_scene_interface_.applyPlanningScene(planning_scene_)
    self.go_to_named_pose("tool_pick_ready", robot_name)
    
    # Delete tool collision object only after collision reinitialization to avoid errors
    if unequip:
      self.despawn_tool(tool_name)
    
    return True

  def allow_collisions_with_robot_hand(self, link_name, robot_name):
      """Allow collisions of a link with the robot hand"""
      self.planning_scene_interface.allow_collisions(link_name, robot_name + "_robotiq_85_tip_link")
      self.planning_scene_interface.allow_collisions(link_name, robot_name + "_robotiq_85_left_finger_tip_link")
      self.planning_scene_interface.allow_collisions(link_name, robot_name + "_robotiq_85_left_inner_knuckle_link")
      self.planning_scene_interface.allow_collisions(link_name, robot_name + "_robotiq_85_right_finger_tip_link")
      self.planning_scene_interface.allow_collisions(link_name, robot_name + "_robotiq_85_right_inner_knuckle_link")    
      return

######

  def start_task_timer(self):
    """Reset timer in debug monitor"""
    try:
      _ = self.resetTimerForDebugMonitor_client.call()
    except:
      pass
  
  def log_to_debug_monitor(self, text, category):
    """Send message to rospy.loginfo and debug monitor.

    This method create publisher on the fly. It's name is defined as "/o2ac_state/{}".format(category).
    The topic name should be included in the parameter server. See test.launch in o2ac_debug_monitor.
    """
    rospy.loginfo(category + ": " + text)

    topic_name = "/o2ac_state/{}".format(category)

    if topic_name not in self.debugmonitor_publishers:
      pub = rospy.Publisher(topic_name, String, queue_size=1)
      rospy.sleep(0.5)
      self.debugmonitor_publishers[topic_name] = pub
    else:
      pub = self.debugmonitor_publishers[topic_name]

    msg = String()
    msg.data = text
    pub.publish(msg)


