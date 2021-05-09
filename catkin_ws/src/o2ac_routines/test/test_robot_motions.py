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
# Author: Felix von Drigalski

import sys
import rospy
import unittest

from o2ac_routines.helpers import *
from o2ac_routines.common import O2ACCommon

class TestSimpleMoves(unittest.TestCase):

    def setUp(self):
        """
        Sets up the test. Afterwards, all functions starting with test_ are executed.
        """
        self.base = O2ACCommon()

    def test_simple_motions(self):
        results = []
        results.append( self.base.active_robots["a_bot"].go_to_named_pose("home") )
        results.append( self.base.active_robots["b_bot"].go_to_named_pose("home") )
        results.append( self.base.active_robots["a_bot"].go_to_named_pose("tool_pick_ready") )
        results.append( self.base.active_robots["b_bot"].go_to_named_pose("tool_pick_ready") )
        all_motions_successful = all(result == True for result in results)
        print("Finished simple motions")
        self.assertTrue(all_motions_successful)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_robot_motions')
    rostest.rosrun('o2ac_routines', 'test_robot_motions', TestSimpleMoves)
