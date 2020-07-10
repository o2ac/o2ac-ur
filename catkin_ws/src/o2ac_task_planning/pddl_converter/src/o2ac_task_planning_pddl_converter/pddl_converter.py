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
import actionlib
import o2ac_task_planning_msgs.msg
import geometry_msgs.msg
import tf
import math
from o2ac_assembly_handler.assy import AssyHandler
from fast_downward_client import DownwardClient
from symbolic_plan_request import SymbolicPlanRequest


def parse_args():
    parser = argparse.ArgumentParser(
        description = 'PDDL converter module, bridging PDDL trace and MTC motion planning')
    parser.add_argument('tracefile')
    parser.add_argument(
        '--disable_regrasp', dest='allow_regrasp',
        help='Boolean swith to allow, disable regrasp during picking. Default is: true (allow)',
        default=True, action='store_false')

    args = parser.parse_args()

    return args

rospy.init_node('pddl_converter')


class PDDL_Converter():

    def __init__(self, assembly_name):
        # Init assembly Handler
        self.assy_name = assembly_name
        self.assy_handler = AssyHandler(assembly_name)

        # Action clients for pick, place retreat and controlling the task
        self.add_pick_client = actionlib.SimpleActionClient('add_pick', o2ac_task_planning_msgs.msg.AddPickAction)
        self.add_place_client = actionlib.SimpleActionClient('add_place', o2ac_task_planning_msgs.msg.AddPlaceAction)
        self.add_retreat_client = actionlib.SimpleActionClient('add_retreat', o2ac_task_planning_msgs.msg.AddRetreatAction)
        self.control_task_client = actionlib.SimpleActionClient('control_task', o2ac_task_planning_msgs.msg.ControlTaskAction)
        self.add_pick_client.wait_for_server()
        self.add_place_client.wait_for_server()
        self.add_retreat_client.wait_for_server()
        self.control_task_client.wait_for_server()
        rospy.loginfo('Initialized action clients')

        self.downward_client = DownwardClient()

    def call_symbolic_planner(self, request):
        return self.downward_client.make_request(request)

    def construct_task_and_plan_motion(self, trace_file_path, allow_regrasp=True):

        goal = o2ac_task_planning_msgs.msg.ControlTaskGoal()
        goal.operation = goal.CLEAR
        self.control_task_client.send_goal(goal)
        self.control_task_client.wait_for_result()
        result = self.control_task_client.get_result()

        with open(trace_file_path) as f:
            # Parse file line by line
            for line in f:
                if not line[0] == ';':  # Exclude comment lines
                    line = line[1:-2]  # Strip brackets
                    line_as_array = line.split(' ')  # Split the line into an array

                    # The command (pddl action) is allways the first element
                    command = line_as_array[0]
                    if not command == 'finished':
                        # Not yet finished, keep constructing the task

                        # The robot name and object name allways follow the command
                        robot_name = line_as_array[1]
                        object_name = line_as_array[2]


                        if command == 'pick':
                            # Create a goal for the add_pick action client
                            goal = o2ac_task_planning_msgs.msg.AddPickGoal()
                            goal.disable_regrasp = not allow_regrasp

                        if command == 'place':
                            # Create a goal for the add_place action client
                            base_object_name = line_as_array[3]
                            frame_mating = self.assy_handler.get_frame_mating(base_object_name, object_name)
                            goal = o2ac_task_planning_msgs.msg.AddPlaceGoal()
                            goal.object_target_pose = geometry_msgs.msg.PoseStamped()
                            goal.object_target_pose.header.frame_id = frame_mating.header.frame_id
                            goal.object_target_pose.pose.position.x = frame_mating.transform.translation.x
                            goal.object_target_pose.pose.position.y = frame_mating.transform.translation.y
                            goal.object_target_pose.pose.position.z = frame_mating.transform.translation.z
                            goal.object_target_pose.pose.orientation.x = frame_mating.transform.rotation.x
                            goal.object_target_pose.pose.orientation.y = frame_mating.transform.rotation.y
                            goal.object_target_pose.pose.orientation.z = frame_mating.transform.rotation.z
                            goal.object_target_pose.pose.orientation.w = frame_mating.transform.rotation.w
                            goal.object_subframe_to_place = frame_mating.child_frame_id


                        if command == 'release':
                            # Create a goal for the add_retreat action client and do release
                            goal = o2ac_task_planning_msgs.msg.AddRetreatGoal()
                            goal.include_release = True

                        if command == 'equip':
                            # Create a goal for the add_pick action client and fill in the extra information
                            goal = o2ac_task_planning_msgs.msg.AddPickGoal()
                            goal.grasp_parameter_location = 'tools'
                            lift_direction = geometry_msgs.msg.Vector3Stamped()
                            lift_direction.header.frame_id = object_name + '_pickup_link'
                            lift_direction.vector = geometry_msgs.msg.Vector3(-1,0,0)
                            goal.lift_direction = lift_direction
                            goal.disable_regrasp = True

                        if command == 'unequip':
                            # Create a goal for the add_place action client and fill in the extra information
                            goal = o2ac_task_planning_msgs.msg.AddPlaceGoal()
                            goal.object_target_pose = geometry_msgs.msg.PoseStamped()
                            goal.object_target_pose.header.frame_id = object_name + '_link'
                            goal.object_target_pose.pose.position.x = 0
                            goal.object_target_pose.pose.position.y = -0.009
                            goal.object_target_pose.pose.position.z = 0.0275
                            goal.object_target_pose.pose.orientation.w = 1

                            approach_place_direction = geometry_msgs.msg.Vector3Stamped()
                            approach_place_direction.header.frame_id = object_name + '_pickup_link'
                            approach_place_direction.vector = geometry_msgs.msg.Vector3(1,0,0)
                            goal.approach_place_direction = approach_place_direction

                            goal.robot_name = robot_name
                            goal.object_name = object_name

                            # Add the place container to the task by calling the action client
                            self.add_place_client.send_goal(goal)
                            self.add_place_client.wait_for_result()
                            result = self.add_place_client.get_result()

                            # Create a goal for the add_retreat action client and do release
                            if result.success:
                                goal = o2ac_task_planning_msgs.msg.AddRetreatGoal()
                                goal.include_release = True

                        if command == 'pick-screw':
                            # Create a goal for the add_place action client and fill in the extra information
                            screw_type = object_name.split('_')[-1]
                            goal = o2ac_task_planning_msgs.msg.AddPlaceGoal()
                            goal.object_target_pose = geometry_msgs.msg.PoseStamped()
                            goal.object_target_pose.header.frame_id = screw_type + '_feeder_outlet_link'
                            goal.object_target_pose.pose.position.x = -0.01
                            quaternion = tf.transformations.quaternion_from_euler(5*math.pi/3, 0, 0)
                            goal.object_target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion.tolist())

                            goal.object_subframe_to_place = object_name + '/' + object_name + '_tip'

                            goal.robot_name = robot_name
                            goal.object_name = object_name

                            # Add the place container to the task by calling the action client
                            self.add_place_client.send_goal(goal)
                            self.add_place_client.wait_for_result()
                            result = self.add_place_client.get_result()
                            
                            # Create a goal for the add_retreat action client and do not include release
                            if result.success:
                                goal = o2ac_task_planning_msgs.msg.AddRetreatGoal()
                                goal.include_release = False
                                retreat_direction = geometry_msgs.msg.Vector3Stamped()
                                retreat_direction.header.frame_id = 'world'
                                retreat_direction.vector = geometry_msgs.msg.Vector3(0,0,1)
                                goal.retreat_direction = retreat_direction

                        if command == 'fasten-with-screw':
                            # Create a goal for the add_place action client and fill in the extra information
                            screwing_subframe = line_as_array[4]
                            goal = o2ac_task_planning_msgs.msg.AddPlaceGoal()
                            goal.object_target_pose = geometry_msgs.msg.PoseStamped()
                            goal.object_target_pose.header.frame_id = screwing_subframe
                            goal.object_target_pose.pose.position.x = -0.01
                            goal.object_target_pose.pose.orientation.w = 1

                            approach_place_direction = geometry_msgs.msg.Vector3Stamped()
                            approach_place_direction.header.frame_id = screwing_subframe
                            approach_place_direction.vector = geometry_msgs.msg.Vector3(1, 1, 1)
                            goal.approach_place_direction = approach_place_direction

                            goal.object_subframe_to_place = object_name + '/' + object_name + '_tip'

                            goal.robot_name = robot_name
                            goal.object_name = object_name

                            # Add the place container to the task by calling the action client
                            self.add_place_client.send_goal(goal)
                            self.add_place_client.wait_for_result()
                            result = self.add_place_client.get_result()

                            # Create a goal for the add_retreat action client and do not include release
                            if result.success:
                                goal = o2ac_task_planning_msgs.msg.AddRetreatGoal()
                                goal.include_release = False
                                retreat_direction = geometry_msgs.msg.Vector3Stamped()
                                retreat_direction.header.frame_id = screwing_subframe
                                retreat_direction.vector = geometry_msgs.msg.Vector3(-1,-1,-1)
                                goal.retreat_direction = retreat_direction

                        # Set the common fields of the goals
                        goal.robot_name = robot_name
                        goal.object_name = object_name

                        # Send the goal with the appropriate action client
                        if command == 'pick':
                            self.add_pick_client.send_goal(goal)
                            self.add_pick_client.wait_for_result()
                            result = self.add_pick_client.get_result()
                        if command == 'place':
                            self.add_place_client.send_goal(goal)
                            self.add_place_client.wait_for_result()
                            result = self.add_place_client.get_result()
                        if command == 'release':
                            self.add_retreat_client.send_goal(goal)
                            self.add_retreat_client.wait_for_result()
                            result = self.add_retreat_client.get_result()
                        if command == 'equip':
                            self.add_pick_client.send_goal(goal)
                            self.add_pick_client.wait_for_result()
                            result = self.add_pick_client.get_result()
                        if command == 'unequip':
                            self.add_retreat_client.send_goal(goal)
                            self.add_retreat_client.wait_for_result()
                            result = self.add_retreat_client.get_result()
                        if command == 'pick-screw' or command == 'fasten-with-screw':
                            self.add_retreat_client.send_goal(goal)
                            self.add_retreat_client.wait_for_result()
                            result = self.add_retreat_client.get_result()
                    else:
                        # Finished, call the planning
                        goal = o2ac_task_planning_msgs.msg.ControlTaskGoal()
                        goal.operation = goal.PLAN
                        goal.number_of_required_solutions = 1  # Finish planning as soon as a solution is found
                        self.control_task_client.send_goal(goal)
                        self.control_task_client.wait_for_result()
                        result = self.control_task_client.get_result()
                        rospy.loginfo(result.task_state)
                        if not result.success:
                            rospy.loginfo('Failing stage ID: ' + str(result.failing_stage_id))
                        return result

    def update_failed_plans_file(self, failing_stage_id, trace_file_path, failed_plans_file_path):
        i = 0
        try:
            failed_plans_file_existed =  os.path.isfile(failed_plans_file_path)
            with open(trace_file_path) as trace, open(failed_plans_file_path, 'a') as failed_plans:
                if failed_plans_file_existed:
                    failed_plans.write('=====\n')
                for line in trace:
                    if i == failing_stage_id:
                        line = line[0:-2] + '%%%' + ')\n'
                    failed_plans.write(line)
                    i += 1
        except:
            rospy.logerr('Error opening files')




def main():
    args = parse_args()
    pddl_converter = PDDL_Converter('wrs_assembly_1') #(TODO: karolyartur - Include assembly name in the pddl plan)

    # Get path of PDDL trace file
    rospack = rospkg.RosPack()
    trace_file_for_motion_planning = os.path.join(rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic',args.tracefile)
    failed_plans_file_for_motion_planning = os.path.join(rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic','failed_plans_test')
    files = ['domain.pddl', 'problem.pddl', 'sas_plan', 'failed_plans']
    file_paths=[]
    for filename in files:
        file_paths.append(os.path.join(rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', filename))

    # Construct the MTC task and do the planning in MTC
    request = SymbolicPlanRequest(file_paths[0], file_paths[1], search_output_file=file_paths[2], failed_plans_file=file_paths[3])
    fast_downward_response = pddl_converter.call_symbolic_planner(request)
    if fast_downward_response:
        if fast_downward_response.exitcode == 0:
            rospy.loginfo('Symbolic planning succeeded')
            motion_planning_result = pddl_converter.construct_task_and_plan_motion(trace_file_for_motion_planning, args.allow_regrasp)
            if not motion_planning_result.success:
                pddl_converter.update_failed_plans_file(motion_planning_result.failing_stage_id, trace_file_for_motion_planning, failed_plans_file_for_motion_planning)
        else:
            rospy.logerr('Symbolic planning failed')


if __name__ == "__main__":
    main()
            