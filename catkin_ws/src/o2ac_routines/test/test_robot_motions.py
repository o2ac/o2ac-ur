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
# Author: Felix von Drigalski, Cristian Beltran

import rostest
import rospy
import unittest

from o2ac_routines import helpers
from o2ac_routines.common import O2ACCommon


class TestSimpleMoves(unittest.TestCase):

    def setUp(self):
        """
        Sets up the test once. Afterwards, all functions starting with test_ are executed.
        """
        self.base = O2ACCommon()

    def test_a_simple_motions(self):
        results = []
        results.append(self.base.a_bot.go_to_named_pose("home"))
        results.append(self.base.b_bot.go_to_named_pose("home"))
        results.append(self.base.a_bot.go_to_named_pose("tool_pick_ready"))
        results.append(self.base.b_bot.go_to_named_pose("tool_pick_ready"))
        results.append(self.base.ab_bot.go_to_named_pose("home"))
        all_motions_successful = all(result == True for result in results)
        self.assertTrue(all_motions_successful)

        joint_values = helpers.ordered_joint_values_from_dict(self.base.b_bot.robot_group.get_named_target_values("screw_ready"), self.base.b_bot.robot_group.get_active_joints())
        self.assertTrue(self.base.b_bot.move_joints(joint_values, speed=1.0))
        self.assertTrue(self.base.a_bot.go_to_pose_goal(self.base.tray_view_high, speed=1.0))
        self.assertTrue(self.base.b_bot.move_lin_rel(relative_translation=[0, 0, 0.1], speed=1.0))

        print("Finished simple motions")

    def test_b_tool_motions(self):
        self.assertTrue(self.base.do_change_tool_action("a_bot", equip=True, screw_size=3), "Fail to equip M3 tool with a_bot")
        self.assertTrue(self.base.do_change_tool_action("b_bot", equip=True, screw_size=4), "Fail to equip M4 tool with b_bot")
        self.assertTrue(self.base.do_change_tool_action("a_bot", equip=False, screw_size=3), "Fail to unequip M3 tool with a_bot")
        self.assertTrue(self.base.do_change_tool_action("b_bot", equip=False, screw_size=4), "Fail to unequip M4 tool with b_bot")
        print("Finished tool motions")

    def test_c_master_slave(self):
        self.base.ab_bot.go_to_named_pose("home")
        self.assertTrue(self.base.take_tray_from_agv_preplanned(use_saved_plans=False), "failed to do master-slave sequence")

    def test_d_simultaneous_motions(self):
        def a_bot_task():
            self.base.do_change_tool_action("a_bot", equip=True, screw_size=3)
            self.base.do_change_tool_action("a_bot", equip=False, screw_size=3)

        def b_bot_task():
            self.base.do_change_tool_action("b_bot", equip=True, screw_size=4)
            self.base.do_change_tool_action("b_bot", equip=False, screw_size=4)
        self.assertTrue(self.base.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120))
        print("Finished simultaneous motions")


if __name__ == '__main__':
    rospy.init_node('test_robot_motions')
    rostest.rosrun('o2ac_routines', 'test_robot_motions', TestSimpleMoves)
