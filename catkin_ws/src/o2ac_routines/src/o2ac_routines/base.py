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
import threading
import copy
import rospy
import tf_conversions
import tf 
import actionlib
from math import *

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import robotiq_msgs.msg
import ur_dashboard_msgs.msg
import ur_dashboard_msgs.srv
import std_srvs.srv
from std_msgs.msg import Bool

import o2ac_msgs
import o2ac_msgs.msg
import o2ac_msgs.srv

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import ur_msgs.msg
from o2ac_routines.helpers import *

class O2ACCommonBase(object):
  """
  This class contains the common helper and convenience functions used in the routines.
  The common functions include the initialization of the services and actions,
  and shorthand functions for the most common actions.
  """
  def __init__(self):
    # super(O2ACCommonBase, self).__init__()
    rospy.init_node('o2ac_routines', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    self.listener = tf.TransformListener()
    self.use_real_robot = rospy.get_param("use_real_robot")
    self.force_ur_script_linear_motion = True
    self.force_moveit_linear_motion = False
    self.disable_markers = True

    self.competition_mode = False   # Setting this to True disables confirmation dialogs etc., thus enabling uninterrupted automatic motion

    self.speed_fast = 1.5
    self.speed_fastest = 3.0
    self.acc_fast = 1.0
    self.acc_fastest = 2.0

    self.robots = moveit_commander.RobotCommander()
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface()

    self.groups = {"a_bot":moveit_commander.MoveGroupCommander("a_bot"), "b_bot":moveit_commander.MoveGroupCommander("b_bot")}
    self.gripper_action_clients = {"b_bot":actionlib.SimpleActionClient('/b_bot/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction)}
    
    self.pick_client = actionlib.SimpleActionClient('/o2ac_skills/pick', o2ac_msgs.msg.pickAction)
    self.place_client = actionlib.SimpleActionClient('/o2ac_skills/place', o2ac_msgs.msg.placeAction)
    self.regrasp_client = actionlib.SimpleActionClient('/o2ac_skills/regrasp', o2ac_msgs.msg.regraspAction)
    self.screw_client = actionlib.SimpleActionClient('/o2ac_skills/screw', o2ac_msgs.msg.screwAction)
    self.change_tool_client = actionlib.SimpleActionClient('/o2ac_skills/changeTool', o2ac_msgs.msg.changeToolAction)
    
    self._suction_client = actionlib.SimpleActionClient('/suction_control', o2ac_msgs.msg.SuctionControlAction)
    self._fastening_tool_client = actionlib.SimpleActionClient('/screw_tool_control', o2ac_msgs.msg.FastenerGripperControlAction)
    self._nut_peg_tool_client = actionlib.SimpleActionClient('/nut_tools_action', o2ac_msgs.msg.ToolsCommandAction)
    self._flex_wrist_client = actionlib.SimpleActionClient('flex_wrist_control', o2ac_msgs.msg.FlexWristControlAction)

    self.ur_dashboard_clients = {
      "a_bot_get_loaded_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
      "b_bot_get_loaded_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
      "a_bot_program_running":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
      "b_bot_program_running":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
      "a_bot_load_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
      "b_bot_load_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
      "a_bot_play":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger),
      "b_bot_play":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger)
    }

    self.inner_pick_detection_client = actionlib.SimpleActionClient('inner_pick_detection_action', o2ac_msgs.msg.innerPickDetectionAction)

    self.urscript_client = rospy.ServiceProxy('/o2ac_skills/sendScriptToUR', o2ac_msgs.srv.sendScriptToUR)
    self.goToNamedPose_client = rospy.ServiceProxy('/o2ac_skills/goToNamedPose', o2ac_msgs.srv.goToNamedPose)
    self.publishMarker_client = rospy.ServiceProxy('/o2ac_skills/publishMarker', o2ac_msgs.srv.publishMarker)
    self.toggleCollisions_client = rospy.ServiceProxy('/o2ac_skills/toggleCollisions', std_srvs.srv.SetBool)

    self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
    self.pause_mode_ = False
    self.test_mode_ = False
    self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
    self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
    self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)
    self.sub_suction_m4_ = rospy.Subscriber("/screw_tool_m4/screw_suctioned", Bool, self.suction_m4_callback)
    self.sub_suction_m3_ = rospy.Subscriber("/screw_tool_m3/screw_suctioned", Bool, self.suction_m3_callback)
    self.sub_robot_safety_mode_b_bot = rospy.Subscriber("/b_bot/ur_hardware_interface/safety_mode", ur_dashboard_msgs.msg.SafetyMode, self.b_bot_safety_mode_callback)
    self.robot_safety_mode = dict() 
    self.screw_is_suctioned = dict()
    self.reduced_mode_speed_limit = .25
    
    # self.my_mutex = threading.Lock()

    self.resetTimerForDebugMonitor_client = rospy.ServiceProxy('/o2ac_debug_monitor/reset_timer', std_srvs.srv.Trigger)
    self.debugmonitor_publishers = dict() # used in log_to_debug_monitor()

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
  
  def is_robot_running_normally(self, robot_name):
    return self.robot_safety_mode[robot_name] == 1

  def unlock_protective_stop(self, robot="b_bot"):
    if robot is not "b_bot" and robot is not "a_bot":
      rospy.logerr("Robot name was not found!")
    service_client = rospy.ServiceProxy("/" + robot + "/ur_hardware_interface/dashboard/unlock_protective_stop", std_srvs.srv.Trigger)
    # rospy.wait_for_service(service_client, 5.0)
    request = std_srvs.srv.TriggerRequest()
    rospy.loginfo("Attempting to unlock protective stop of " + robot)
    response = service_client.call(request)
    rospy.loginfo("Response for unlocked protective stop of " + robot + ": " + response.message)
    return response.success

  def activate_ros_control_on_ur(self, robot="b_bot"):
    if robot is not "b_bot":
    # if robot is not "b_bot" and robot is not "a_bot":
      rospy.logerr("Robot name was not found or the robot is not a UR!")
      return False
    
    # Check if URCap is already running on UR
    response = self.ur_dashboard_clients[robot + "_program_running"].call(ur_dashboard_msgs.srv.IsProgramRunningRequest())
    if response.program_running:
      response = self.ur_dashboard_clients[robot + "_get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
      if response.program_name == "/programs/ROS_external_control.urp":
        return True
    
    # Load program
    request = ur_dashboard_msgs.srv.LoadRequest()
    request.filename = "ROS_external_control.urp"
    response = self.ur_dashboard_clients[robot + "_load_program"].call(request)
    if not response.success:
      rospy.logerr("Could not load the ROS_external_control.urp URCap. Is the UR robot set up correctly and the program installed with the correct name?")
      return False
    
    # Run the program
    response = self.ur_dashboard_clients[robot + "_play"].call(std_srvs.srv.TriggerRequest())
    rospy.sleep(2)
    return response.success
    

  def publish_marker(self, pose_stamped, marker_type):
    # Publishes a marker to Rviz for visualization
    if self.disable_markers:
      return True
    req = o2ac_msgs.srv.publishMarkerRequest()
    req.marker_pose = pose_stamped
    req.marker_type = marker_type
    self.publishMarker_client.call(req)
    return True

  def get_current_pose_stamped(self, group_name):
    group = self.groups[group_name]
    return group.get_current_pose()

  def get_current_pose(self, group_name):
    group = self.groups[group_name]
    return group.get_current_pose().pose
  
  def lookup_transform(self, robot_name, end_effector_link):
    return self.listener.lookupTransform(end_effector_link, robot_name, rospy.Time())

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
      if group_name == "c_bot":
        end_effector_link = "c_bot_robotiq_85_tip_link"
      elif group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_gripper_tip_link"
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
      if group_name == "c_bot":
        end_effector_link = "c_bot_robotiq_85_tip_link"
      elif group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_gripper_tip_link"

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
    group.set_pose_target(pose_goal_a_bot, end_effector_link="a_bot_gripper_tip_link")
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

  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, 
          special_pick = False, lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.5, gripper_force=40.0,
          gripper_velocity = .1,):
    # TODO: 
    
    self.log_to_debug_monitor("Pick", "operation")
    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    rospy.logdebug("Approach height 0: " + str(approach_height))
    object_pose.pose.position.z += approach_height
    rospy.logdebug("Height 1: " + str(object_pose.pose.position.z))
    if special_pick == True:
      object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi*45/180, pi/2))
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    object_pose.pose.position.z -= approach_height
    rospy.logdebug("Height 2: " + str(object_pose.pose.position.z))
    # self.publish_marker(object_pose, "place_pose")

    if gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    rospy.loginfo("Moving down to object")
    object_pose.pose.position.z += grasp_height
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, acceleration=acc_slow, high_precision=True, move_lin=True)
    object_pose.pose.position.z -= grasp_height

    #gripper close
    if gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="close", force = gripper_force, velocity = gripper_velocity,)

    if lift_up_after_pick:
      rospy.sleep(1.0)
      rospy.loginfo("Going back up")

      object_pose.pose.position.z += approach_height
      rospy.loginfo("Going to height " + str(object_pose.pose.position.z))
      self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
      object_pose.pose.position.z -= approach_height
    return True

  ######

  def place(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, lift_up_after_place = True, acc_fast=1.0, acc_slow=.5):
    #self.publish_marker(object_pose, "place_pose")
    self.log_to_debug_monitor("Place", "operation")
    rospy.loginfo("Going above place target")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    object_pose.pose.position.z -= approach_height

    rospy.loginfo("Moving to place target")
    object_pose.pose.position.z += place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    object_pose.pose.position.z -= place_height

    #gripper open
    if gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    
    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      object_pose.pose.position.z += approach_height
      self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, move_lin=True)  
      object_pose.pose.position.z -= approach_height
    return True

  ######

  def do_pick_action(self, robot_name, pose_stamped, screw_size = 0, z_axis_rotation = 0.0, use_complex_planning = False, tool_name = ""):
    # Call the pick action. It is useful for picking screws with the tool.
    goal = o2ac_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    goal.screw_size = screw_size
    goal.use_complex_planning = use_complex_planning
    goal.z_axis_rotation = z_axis_rotation
    rospy.loginfo("Sending pick action goal")
    rospy.logdebug(goal)

    self.pick_client.send_goal(goal)
    rospy.logdebug("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.logdebug("Getting result")
    return self.pick_client.get_result()

  def do_place_action(self, robot_name, pose_stamped, tool_name = "", screw_size=0):
    # Call the pick action
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

  def set_motor(self, motor_name, direction = "tighten", wait=False, speed = 0, duration = 0):
    if not self.use_real_robot:
      return True
    goal = o2ac_msgs.msg.FastenerGripperControlGoal()
    goal.fastening_tool_name = motor_name
    goal.direction = direction
    goal.speed = speed
    goal.duration = duration
    rospy.loginfo("Sending fastening_tool action goal.")
    self._fastening_tool_client.send_goal(goal)
    if wait:
      self._fastening_tool_client.wait_for_result()
    return self._fastening_tool_client.get_result()

  def set_suction(self, tool_name, suction_on=False, eject=False, wait=True):
    if not self.use_real_robot:
      return True
    goal = o2ac_msgs.msg.SuctionControlGoal()
    goal.fastening_tool_name = tool_name
    goal.turn_suction_on = suction_on
    goal.eject_screw = eject
    rospy.loginfo("Sending suction action goal.")
    self._suction_client.send_goal(goal)
    if wait:
      self._suction_client.wait_for_result(rospy.Duration(2.0))
    return self._suction_client.get_result()

  def do_nut_fasten_action(self, item_name, wait = True):
    if not self.use_real_robot:
      return True
    goal = o2ac_msgs.msg.ToolsCommandGoal()
    # goal.stop = stop
    goal.peg_fasten = (item_name == "peg" or item_name == "m10_nut")
    goal.setScrew_fasten = (item_name == "set_screw")
    # goal.big_nut_fasten = (item_name == "m10_nut")
    # goal.big_nut_fasten = True
    goal.small_nut_fasten = (item_name == "m6_nut")
    rospy.loginfo("Sending nut_tool action goal.")
    self._nut_peg_tool_client.send_goal(goal)
    if wait:
      self._nut_peg_tool_client.wait_for_result(rospy.Duration.from_sec(30.0))
    return self._nut_peg_tool_client.get_result()

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
  
  def move_lin_rel(self, robot_name, relative_translation = [0,0,0], relative_rotation = [0,0,0], acceleration = 0.5, velocity = .03, wait = True):
    '''
    Does a lin_move relative to the current position of the robot. Uses the robot's TCP.

    robot_name = "b_bot" for example
    relative_translation: translatory movement relative to current tcp position, expressed in robot's own base frame
    relative_rotation: rotatory movement relative to current tcp position, expressed in robot's own base frame
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
    req.program_id = "lin_move_rel"
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


  ################ ----- Gripper interfaces
  
  def send_gripper_command(self, gripper, command, this_action_grasps_an_object = False, force = 40.0, velocity = .1, wait=True):
    """
    force: Gripper force in N. From 40 to 100
    velocity: Gripper speed. From 0.013 to 0.1

    Use a slow closing speed when using a low gripper force, or the force might be unexpectedly high.
    """
    if not self.use_real_robot:
      # TODO: Set the gripper width
      return True
    if gripper == "precision_gripper_outer" or gripper == "precision_gripper_inner" or gripper == "a_bot":
      goal = o2ac_msgs.msg.PrecisionGripperCommandGoal()
      if command == "stop":
        goal.stop = True
      elif command == "close":
        if gripper == "precision_gripper_inner" or gripper == "a_bot":
          goal.close_inner_gripper_fully = True
        else:
          goal.close_outer_gripper_fully = True
      elif command == "open":
        if gripper == "precision_gripper_inner" or gripper == "a_bot":
          goal.open_inner_gripper_fully = True
        else:
          goal.open_outer_gripper_fully = True
      goal.this_action_grasps_an_object = this_action_grasps_an_object
      action_client = self.gripper_action_clients["a_bot"]
    elif gripper == "b_bot" or gripper == "c_bot":
      goal = robotiq_msgs.msg.CModelCommandGoal()
      action_client = self.gripper_action_clients[gripper]
      goal.velocity = velocity   
      goal.force = force         
      if command == "close":
        goal.position = 0.0
      elif command == "open":
        goal.position = 0.085
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
    if wait or gripper == "b_bot":  # b_bot uses the UR to close the gripper; it has to wait.
      action_client.wait_for_result(rospy.Duration(6.0))  # Default wait time: 6 s
    result = action_client.get_result()
    rospy.loginfo(result)
    return 
  
  def put_screw_in_feeder(self, screw_size):
    if not screw_size in [3,4]:
      rospy.logerr("There are no feeders of size " + str(screw_size) + ". Aborting.")
      return False

    self.log_to_debug_monitor("Put screw in feeder", "operation")

    self.go_to_named_pose("above_feeder", "a_bot", speed=1.5, acceleration=1.0, force_ur_script=self.use_real_robot)
    drop_pose = geometry_msgs.msg.PoseStamped()
    drop_pose.pose.position.z = .015
    drop_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    drop_pose.header.frame_id = "m" + str(screw_size) + "_feeder_inlet_link"

    self.set_feeder_power(False)
    self.move_lin("a_bot", drop_pose, speed = 1.0, acceleration = 1.0, end_effector_link = "a_bot_gripper_tip_link")
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    self.set_feeder_power(True)
    return True
        
  def pick_screw_from_feeder(self, screw_size, attempts = 1):
    """
    Picks a screw from one of the feeders. The screw tool already has to be equipped!
    """
    # Use this command to equip the screw tool: do_change_tool_action(self, "b_bot", equip=True, screw_size = 4)
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False

    self.log_to_debug_monitor("Pick screw from feeder", "operation")

    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)

    pose_feeder = geometry_msgs.msg.PoseStamped()
    pose_feeder.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    pose_feeder.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose_feeder.pose.position.x = 0.0

    attempt = 0
    screw_picked = False
    while attempt < attempts:
      self.set_feeder_power(False)
      self.do_pick_action("b_bot", pose_feeder, screw_size = screw_size, use_complex_planning = True, tool_name = "screw_tool")
      self.set_feeder_power(True)
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if screw_picked:
        rospy.loginfo("Successfully picked the screw")
        return True
      if not self.use_real_robot:
        rospy.loginfo("Pretending the screw is picked, because this is simulation.")
        return True
      attempt += 1

    return False

  def pick_nut_from_table(self,robot_name, object_pose, max_radius=0.005, end_effector_link="c_bot_nut_tool_m6_tip_link"):
    self.log_to_debug_monitor("Pick nut from table", "operation")
    approach_pose = copy.deepcopy(object_pose)
    approach_pose.pose.position.z += .02  # Assumes that z points upward
    self.go_to_pose_goal(robot_name, approach_pose, speed=self.speed_fast, move_lin = True, end_effector_link=end_effector_link)
    if robot_name == "c_bot":
      spiral_axis = "YZ"
      push_direction = "c_bot_diagonal"
    else:
      spiral_axis = "Y"
      push_direction = "Z+"
    self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.go_to_pose_goal(robot_name, approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def pick_nut_using_spiral_search(self,object_pose, max_radius=0.005, end_effector_link="c_bot_nut_tool_m6_tip_link"):
    self.log_to_debug_monitor("Pick nut using spiral search", "operation")

    # This spiral search does not work as well as we hoped. When the nut is not perfectly picked, it falls to a completely different place
    max_radius = .005
    theta_incr = pi/3
    r=0.00015
    radius_increment = .0008
    radius_inc_set = radius_increment / (2*pi / theta_incr)
    theta=0
    RealRadius=0
    
    # Try to pick the nut multiple times
    adjusted_pose = copy.deepcopy(object_pose)
    while RealRadius < max_radius and not rospy.is_shutdown():
      rospy.loginfo("Moving into screw to pick it up.")
      # adjusted_pose.pose.position.x += .02
      # self.go_to_pose_goal("c_bot", adjusted_pose, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.pick(robotname="c_bot",object_pose=object_pose,grasp_height=-0.002,
                                  speed_fast = .3, speed_slow = .03, gripper_command="none",
                                  approach_height = .05,end_effector_link=end_effector_link)

      # TODO: Test if pushing linear works better

      # Adjust the position (spiral search)
      rospy.loginfo("Retrying pickup with adjusted position")
      theta=theta+theta_incr
      y=math.cos(theta)*r
      z=math.sin(theta)*r
      adjusted_pose = copy.deepcopy(object_pose)
      adjusted_pose.pose.position.y += y
      adjusted_pose.pose.position.z += z
      r = r + radius_inc_set
      RealRadius = math.sqrt(math.pow(y,2)+math.pow(z,2))

  def adjust_tool_centering(self, tool_end_effector_link="b_bot_screw_tool_m4_tip_link", go_fast=False):
    rospy.logwarn("THIS IS UNTESTED!!")
    rospy.loginfo("============ Adjusting the position of the pin/shaft ============")
    self.go_to_named_pose("screw_ready", "b_bot")
    self.send_gripper_command(gripper="c_bot",command = "open")

    speed = .3
    acceleration = .5
    force_ur_script = False
    if go_fast:
      speed = 1.5
      acceleration = 1.5
      force_ur_script = True

    b_pose = geometry_msgs.msg.PoseStamped()
    b_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    b_pose.header.frame_id = "workspace_center"
    b_pose.pose.position.z = 0.4
    self.go_to_pose_goal("b_bot", b_pose, end_effector_link=tool_end_effector_link, speed=speed, acceleration=acceleration, move_lin = True)

    rospy.sleep(1)

    c_pose = geometry_msgs.msg.PoseStamped()
    c_pose.header.frame_id = "b_bot_robotiq_85_tip_link"
    c_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2,0,pi/2))
    c_pose.pose.position.z = 0.0  # MAGIC NUMBER!
    c_pose.pose.position.y = 0.025
    c_pose.pose.position.x = 0.015
    self.go_to_pose_goal("c_bot", c_pose, speed=speed, acceleration=acceleration, move_lin = True)

    # self.send_gripper_command(gripper="b_bot",command = "close", velocity = .015, force = 1.0)
    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(.5)
    self.send_gripper_command(gripper="b_bot",command = .01)
    rospy.sleep(.5)

    c_wiggle_1 = copy.deepcopy(c_pose)
    c_wiggle_1.pose.position.z += 0.002
    c_wiggle_2 = copy.deepcopy(c_pose)
    c_wiggle_2.pose.position.z -= 0.002
    self.go_to_pose_goal("c_bot", c_wiggle_1, speed=.03, acceleration=acceleration, move_lin = True)
    self.go_to_pose_goal("c_bot", c_wiggle_2, speed=.03, acceleration=acceleration, move_lin = True)
    self.go_to_pose_goal("c_bot", c_pose, speed=.03, acceleration=acceleration, move_lin = True)
    self.send_gripper_command(gripper="c_bot",command = "open")

    self.go_to_named_pose("home", "c_bot", speed=speed, acceleration=acceleration, force_ur_script=force_ur_script)
    return

  def adjust_centering(self, go_fast=False, handover_motor_to_c_bot=False):

    #rospy.loginfo("============ Adjusting the position of the pin/shaft
    # ============")
    speed = .3
    acceleration = .5
    force_ur_script = False
    if go_fast:
      speed = 3.0
      acceleration = 2.0
      force_ur_script = True
    
    self.log_to_debug_monitor("Adjust centering", "operation")
    self.go_to_named_pose("home", "b_bot", speed=speed, acceleration=acceleration, force_ur_script=force_ur_script)
    self.go_to_named_pose("home", "c_bot", speed=speed, acceleration=acceleration, force_ur_script=force_ur_script)
    self.send_gripper_command(gripper="c_bot",command = "open")

    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose1.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose1.pose.position.y = -0.15
    pose1.pose.position.z = 0.15
    self.go_to_pose_goal("b_bot", pose1,speed=speed, acceleration=acceleration, move_lin = True)

    rospy.sleep(1)

    pose2 = geometry_msgs.msg.PoseStamped()
    pose2.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2,0,pi/2))
    pose2.pose.position.z = -0.0015   # MAGIC NUMBER (positive moves c_bot towards ??)
    pose2.pose.position.y = 0.025 # (positive moves c_bot forward)
    if handover_motor_to_c_bot:
      pose2.pose.position.y = 0.015 
    pose2.pose.position.x = 0.015 # (positive moves c_bot down)
    self.go_to_pose_goal("c_bot", pose2, speed=speed, acceleration=acceleration, move_lin = True)

    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(.5)
    self.send_gripper_command(gripper="b_bot",command = .03)
    rospy.sleep(.5)
    self.send_gripper_command(gripper="b_bot",command = "open")
    rospy.sleep(1.0)
    self.send_gripper_command(gripper="b_bot",command = "close", velocity = .05, force = 1.0)
    rospy.sleep(.5)
    self.send_gripper_command(gripper="c_bot",command = "open")
    rospy.sleep(.5)

    pose3 = geometry_msgs.msg.PoseStamped()
    pose3.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    pose3.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose3.pose.position.y = 0
    pose3.pose.position.z = 0
    self.go_to_pose_goal("b_bot", pose3, speed=speed, acceleration=acceleration, move_lin = True)

    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(1)
    self.send_gripper_command(gripper="b_bot",command = "open")
    rospy.sleep(2)
    if not handover_motor_to_c_bot:
      self.send_gripper_command(gripper="b_bot",command = "close")
      rospy.sleep(2)
      self.send_gripper_command(gripper="c_bot",command = "open")
      rospy.sleep(1)

    ### This can be used to adjust the rotation during the handover
    # if handover_motor_to_c_bot:
    #   pose4 = geometry_msgs.msg.PoseStamped()
    #   pose4.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi*(90+5)/180, 0, 0))
    #   pose4.header.frame_id = "b_bot_robotiq_85_tip_link"
    #   pose4.pose.position.y = 0
    #   pose4.pose.position.z = 0
    #   self.confirm_to_proceed("How much extra rotation from here?")
    #   self.go_to_pose_goal("b_bot", pose4, speed=speed, acceleration=acceleration, move_lin = True)
    #   self.send_gripper_command(gripper="c_bot",command = "close")
    #   rospy.sleep(2)
    #   self.send_gripper_command(gripper="b_bot",command = "open")
    #   rospy.sleep(1)

    self.go_to_named_pose("home", "c_bot", speed=speed, acceleration=acceleration, force_ur_script=force_ur_script)
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
