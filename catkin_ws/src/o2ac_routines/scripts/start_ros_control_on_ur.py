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
import actionlib
from math import *
import yaml

import ur_dashboard_msgs.msg
import ur_dashboard_msgs.srv
import std_srvs.srv

from std_msgs.msg import String

import ur_msgs.msg
from o2ac_routines.helpers import *

if __name__ == '__main__':
  rospy.init_node('helper', anonymous=True)

  ur_dashboard_clients = {
    "a_bot_get_loaded_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
    "b_bot_get_loaded_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/get_loaded_program', ur_dashboard_msgs.srv.GetLoadedProgram),
    "a_bot_program_running":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
    "b_bot_program_running":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/program_running', ur_dashboard_msgs.srv.IsProgramRunning),
    "a_bot_load_program":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
    "b_bot_load_program":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/load_program', ur_dashboard_msgs.srv.Load),
    "a_bot_play":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger),
    "b_bot_play":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/play', std_srvs.srv.Trigger),
    "a_bot_connect":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/connect', std_srvs.srv.Trigger),
    "b_bot_connect":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/connect', std_srvs.srv.Trigger),
    "a_bot_quit":rospy.ServiceProxy('/a_bot/ur_hardware_interface/dashboard/quit', std_srvs.srv.Trigger),
    "b_bot_quit":rospy.ServiceProxy('/b_bot/ur_hardware_interface/dashboard/quit', std_srvs.srv.Trigger)
  }

  def activate_ros_control_on_ur(robot, recursion_depth=0):
    # Check if URCap is already running on UR
    try:
      response = ur_dashboard_clients[robot + "_program_running"].call(ur_dashboard_msgs.srv.IsProgramRunningRequest())
      if response.program_running:
        response = ur_dashboard_clients[robot + "_get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
        if response.program_name == "/programs/ROS_external_control.urp":
          return True
    except:  # Try reconnecting and restart the function
      response = ur_dashboard_clients[robot + "_quit"].call()
      rospy.sleep(.5)
      response = ur_dashboard_clients[robot + "_connect"].call()
      return activate_ros_control_on_ur(robot, recursion_depth=recursion_depth+1)
    
    # Load program
    rospy.logwarn("Activating ROS control on robot " + robot)
    request = ur_dashboard_msgs.srv.LoadRequest()
    request.filename = "ROS_external_control.urp"
    response = ur_dashboard_clients[robot + "_load_program"].call(request)
    if not response.success:
      rospy.logwarn("Could not load URCap. Trying to reconnect to dashboard server.")
      response = ur_dashboard_clients[robot + "_quit"].call()
      rospy.sleep(.5)
      response = ur_dashboard_clients[robot + "_connect"].call()
      if not response.success:
        rospy.logerr("Could not start UR control. Is the UR in Remote Control mode and program installed with correct name?")
        return False
      rospy.loginfo("Successfully connected to " + robot + " dashboard server.")
      # Try loading again
      request = ur_dashboard_msgs.srv.LoadRequest()
      request.filename = "ROS_external_control.urp"
      response = ur_dashboard_clients[robot + "_load_program"].call(request)
    
    # Run the program
    response = ur_dashboard_clients[robot + "_play"].call(std_srvs.srv.TriggerRequest())
    rospy.sleep(2)
    if not response.success:
      rospy.logerr("Could not start UR control. Is the UR in Remote Control mode and program installed with correct name?")
      return False
    else:
      rospy.logwarn("Successfully activated ROS control on robot " + robot)
      return True
  
  activate_ros_control_on_ur("a_bot")
  activate_ros_control_on_ur("b_bot")
