#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski

import actionlib
import o2ac_msgs.msg
import rospy
from std_msgs.msg import Bool

from o2ac_routines.helpers import check_for_real_robot


class Tools():
    def __init__(self):
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.suction_client = actionlib.SimpleActionClient('/suction_control', o2ac_msgs.msg.SuctionControlAction)
        self.fastening_tool_client = actionlib.SimpleActionClient('/screw_tool_control', o2ac_msgs.msg.ScrewToolControlAction)

        self.sub_suction_m4_ = rospy.Subscriber("/screw_tool_m4/screw_suctioned", Bool, self.suction_m4_callback)
        self.sub_suction_m3_ = rospy.Subscriber("/screw_tool_m3/screw_suctioned", Bool, self.suction_m3_callback)

        self.screw_is_suctioned = dict()

    def suction_m4_callback(self, msg):
        self.screw_is_suctioned["m4"] = msg.data

    def suction_m3_callback(self, msg):
        self.screw_is_suctioned["m3"] = msg.data

    @check_for_real_robot
    def set_suction(self, tool_name, suction_on=False, eject=False, wait=True):
        goal = o2ac_msgs.msg.SuctionControlGoal()
        goal.fastening_tool_name = tool_name
        goal.turn_suction_on = suction_on
        goal.eject_screw = eject
        rospy.loginfo("Sending suction action goal.")
        self.suction_client.send_goal(goal)
        if wait:
            self.suction_client.wait_for_result(rospy.Duration(2.0))
        return self.suction_client.get_result()

    @check_for_real_robot
    def set_motor(self, motor_name, direction="tighten", wait=False, speed=0, duration=0, skip_final_loosen_and_retighten=False):
        goal = o2ac_msgs.msg.ScrewToolControlGoal()
        goal.fastening_tool_name = motor_name
        goal.direction = direction
        goal.speed = speed
        goal.duration = duration
        goal.skip_final_loosen_and_retighten = skip_final_loosen_and_retighten
        rospy.loginfo("Sending fastening_tool action goal.")
        self.fastening_tool_client.send_goal(goal)
        if wait:
            self.fastening_tool_client.wait_for_result()
        return self.fastening_tool_client.get_result()
