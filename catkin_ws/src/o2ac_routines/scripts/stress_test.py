#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X
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
# Author: Felix von Drigalski, Cristian C. Beltran-Hernandez
#
# Simple stress test for simultaneous motions with both robots

from o2ac_routines.common import O2ACCommon
import rospy
from ur_control.constants import TERMINATION_CRITERIA
import o2ac_routines.helpers as helpers

# Allow easier kill of this process with ctrl+c on Terminal
import sys
import signal


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def execute_sequences():
    """ Continuously and simultaneously execute a simple `sequence` trajectory in two different poses """

    def a_task():
        controller.a_bot.go_to_named_pose("home")
        seq = []
        seq.append(helpers.to_sequence_item_relative([0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([-0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, 0.01, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, -0.01, 0, 0, 0, 0]))
        for i in range(8):
            controller.execute_sequence("a_bot", seq, "a"+str(i))
            if i % 2 == 0:
                controller.a_bot.go_to_named_pose("home")
            else:
                controller.a_bot.go_to_named_pose("feeder_pick_ready")

    def b_task():
        controller.b_bot.go_to_named_pose("home")
        seq = []
        seq.append(helpers.to_sequence_item_relative([0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([-0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, 0.01, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, -0.01, 0, 0, 0, 0]))
        for i in range(8):
            controller.execute_sequence("b_bot", seq, "b"+str(i))
            if i % 2 == 0:
                controller.b_bot.go_to_named_pose("home")
            else:
                controller.b_bot.go_to_named_pose("feeder_pick_ready")

    controller.do_tasks_simultaneous(a_task, b_task, timeout=500)


def equip_unequip():
    """ Continuously and simultaneously equip and unequip tool """

    controller.reset_scene_and_robots()

    def a_task():
        controller.a_bot.go_to_named_pose("home")
        for i in range(8):
            print("//// A_BOT equip ++++")
            controller.do_change_tool_action("a_bot", equip=True, screw_size=3)
            print("//// A_BOT equip done")
            print("//// A_BOT unequip ++++")
            controller.do_change_tool_action(
                "a_bot", equip=False, screw_size=3)
            print("//// A_BOT unequip done")

    def b_task():
        controller.b_bot.go_to_named_pose("home")
        for i in range(8):
            print(">>>> B_BOT equip ****")
            controller.do_change_tool_action("b_bot", equip=True, screw_size=4)
            print(">>>> B_BOT equip done")
            print(">>>> B_BOT unequip ****")
            controller.do_change_tool_action(
                "b_bot", equip=False, screw_size=4)
            print(">>>> B_BOT unequip done")

    controller.do_tasks_simultaneous(a_task, b_task, timeout=500)


def main():
    rospy.init_node("testscript")
    global controller
    controller = O2ACCommon()

    equip_unequip()

    execute_sequences()


if __name__ == "__main__":
    main()
