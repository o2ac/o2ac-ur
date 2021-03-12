#!/usr/bin/env python
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
import rospkg
import os
from fast_downward_client import DownwardClient
from symbolic_plan_request import SymbolicPlanRequest
from pddl_converter import PDDL_Converter


def parse_args():
    parser = argparse.ArgumentParser(
        description = 'PDDL converter module, bridging PDDL trace and MTC motion planning')
    parser.add_argument('pddl_domain_file')
    parser.add_argument('pddl_problem_file')
    parser.add_argument(
        '--translate_output_file', dest='translate_output_file',
        help='File name for the output of the translate module (the input for the planner). Default is: output.sas',
        default='output.sas')
    parser.add_argument(
        '--search_output_file', dest='search_output_file',
        help='File name for the output of the planner (the result of the symbolic search). Default is: result_plan',
        default='result_plan')
    parser.add_argument(
        '--failed_plans_file', dest='failed_plans_file',
        help='Name of the file containing the previous plans for the problem that were marked as failed plans based on the motion planning check. Default is empty string meaning no previous plans were checked',
        default='')
    parser.add_argument(
        '--disable_regrasp', dest='allow_regrasp',
        help='Boolean swith to allow, disable regrasp during picking. Default is: true (allow)',
        default=True, action='store_false')

    args = parser.parse_args()

    return args

class STAMP():
    def __init__(self, assembly_name):
        self.rospack = rospkg.RosPack()
        self.downward_client = DownwardClient()
        self.pddl_converter = PDDL_Converter(assembly_name)

    def do_step(self, pddl_domain_file, pddl_problem_file, translate_output_file = 'output.sas', search_output_file = 'result_plan', failed_plans_file = '', allow_regrasp = True):
        file_names = [pddl_domain_file, pddl_problem_file, translate_output_file, search_output_file]
        if not failed_plans_file == '':
            file_names.append(failed_plans_file)
        file_paths=[]
        i = 0
        for filename in file_names:
            if i < 2:
                file_paths.append(os.path.join(self.rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', filename))   # Input files are looked for in the 'symbolic' folder
            else:
                file_paths.append(os.path.join(self.rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', 'generated', filename))   # Generated files are created/looked for in the 'symbolic/generated' folder
            i += 1
        request = SymbolicPlanRequest(*file_paths)
        fast_downward_response = self.downward_client.make_request(request)
        if fast_downward_response:
            if fast_downward_response.exitcode == 0:
                rospy.loginfo('Symbolic planning succeeded')
                motion_planning_result = self.pddl_converter.construct_task_and_plan_motion(file_paths[3], allow_regrasp)
                if not motion_planning_result.success:
                    if not failed_plans_file == '':
                        self.pddl_converter.update_failed_plans_file(motion_planning_result.failing_stage_id, file_paths[3], file_paths[4])
                    else:
                        failed_plans_file_path = os.path.join(self.rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', 'generated', 'failed_plans')
                        self.pddl_converter.update_failed_plans_file(motion_planning_result.failing_stage_id, file_paths[3], failed_plans_file_path)
            else:
                rospy.logerr('Symbolic planning failed')
                

if __name__ == "__main__":
    rospy.init_node('stamp')
    args = parse_args()
    stamp = STAMP('wrs_assembly_1')
    stamp.do_step(**args.__dict__)
    