#!/usr/bin/env python3
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
# Author: Karoly Istvan Artur

import rospy
import os
import rospkg

from fast_downward_msgs.srv import CallFastDownward
from symbolic_plan_request import SymbolicPlanRequest

class DownwardClient():
    '''
    Ros service client for the fast downward planner service
    '''
    def __init__(self):
        rospy.wait_for_service('fast_downward')
        self.client = rospy.ServiceProxy('fast_downward', CallFastDownward)

    def make_request(self, request):
        '''
        Send a planning request to the Fast Downward planner

        Inputs:
            request (SymbolicPlanRequest): planning request

        Returns:
            CallFastDownwardResponse or None in case of error
        '''
        try:
            return self.client(request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            return None

if __name__ == "__main__":
    client = DownwardClient()
    rospack = rospkg.RosPack()
    file_names = ['domain.pddl', 'problem.pddl', 'sas_plan', 'failed_plans']  # PDDL domain file name, PDDL problem file name, output (trace) file name, failed plans file name
    file_paths = []
    for filename in file_names:
        file_paths.append(os.path.join(rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', filename))  # Files are looked for in the 'symbolic' folder

    request = SymbolicPlanRequest(file_paths[0], file_paths[1], search_output_file = file_paths[2], failed_plans_file = file_paths[3])
    client.make_request(request)