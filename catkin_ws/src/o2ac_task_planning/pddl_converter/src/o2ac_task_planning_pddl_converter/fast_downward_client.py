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

import argparse

import rospy
import os
import rospkg

from fast_downward_msgs.srv import CallFastDownward
from symbolic_plan_request import SymbolicPlanRequest

def parse_args():
    parser = argparse.ArgumentParser(
        description = 'Fast Downward client module, intended to be imported and used as a member of a class, main is for testing')
    parser.add_argument('pddl_domain_file')
    parser.add_argument('pddl_problem_file')
    parser.add_argument(
        '--translate_output_file', dest='translate_output_file',
        help='File name for the output of the translate module (the input for the planner). Default is: output.sas',
        default='output.sas')
    parser.add_argument(
        '--search_output_file', dest='search_output_file',
        help='File name for the output of the planner (the result of the symbolic search). Default is: sas_plan',
        default='sas_plan')
    parser.add_argument(
        '--failed_plans_file', dest='failed_plans_file',
        help='Name of the file containing the previous plans for the problem that were marked as failed plans based on the motion planning check. Default is empty string meaning no previous plans were checked',
        default='')

    args = parser.parse_args()

    return args

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
    args = parse_args()
    client = DownwardClient()
    rospack = rospkg.RosPack()
    file_names = [args.pddl_domain_file, args.pddl_problem_file, args.translate_output_file, args.search_output_file]
    if not args.failed_plans_file == '':
        file_names.append(args.failed_plans_file)
    file_paths = []
    for filename in file_names:
        file_paths.append(os.path.join(rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', filename))  # Files are looked for in the 'symbolic' folder

    request = SymbolicPlanRequest(*file_paths)
    client.make_request(request)