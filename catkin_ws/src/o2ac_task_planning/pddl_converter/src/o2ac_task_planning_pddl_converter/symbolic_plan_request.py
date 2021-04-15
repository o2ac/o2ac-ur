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

from fast_downward_msgs.srv import CallFastDownwardRequest

class SymbolicPlanRequest(CallFastDownwardRequest):
    '''Wrapper class for the CallFastDownwardRequest message, that sets default values for ease of use'''

    def __init__(self, pddl_domain_file, pddl_problem_file, translate_output_file = 'output.sas', search_output_file = 'sas_plan', failed_plans_file = ''):
        '''
        Construct a CallFastDownwardRequest with properly initialized fields

        Inputs:
            pddl_domain_file (string): the name of the file that contains the domain definition in PDDL format
            pddl_problem_file (string): the name of the file that contains the problem definition in PDDL format
            translate_output_file (string): the name of the file in which to save the output of the Fast Downward translate module (this contains the input for the planner)
            search_output_file (string): the name of the file in which to store the result of the search (the planned trace)
            failed_plans_file (string): the name of the file in which the previously failed plans are stored (for required format see README)
        '''
        self.log_level = self.LOGLEVEL_INFO
        self.run_all = True
        self.build = 'release'
        self.domain_file = pddl_domain_file
        self.task_file = pddl_problem_file
        self.sas_file = translate_output_file
        self.plan_file = search_output_file
        self.failed_plans_file = failed_plans_file

        self.translate_inputs = [pddl_domain_file, pddl_problem_file]
        self.translate_options = ['--sas-file', translate_output_file]

        self.search_input = translate_output_file
        self.search_options = ["--search", 'astar(blind(),lazy_evaluator=blind())']

        self.components = []
        self.translate_time_limit = 0
        self.translate_memory_limit = 0
        self.search_time_limit = 0
        self.search_memory_limit = 0
        self.overall_time_limit = 0
        self.overall_memory_limit = 0

        self.portfolio_bound = 0
        self.portfolio_single_plan = False
        self.portfolio = ''

