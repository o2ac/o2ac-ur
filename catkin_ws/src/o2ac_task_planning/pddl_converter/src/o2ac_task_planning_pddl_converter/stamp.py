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

import sys
import tf
import moveit_commander
from math import pi
import geometry_msgs.msg


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
    parser.add_argument(
        '--keep_generated_files', dest='keep_generated_files',
        help='Boolean swith to set if the generated files should be deleted or not. Default is: false (do not keep)',
        default=False, action='store_true')

    args = parser.parse_args(rospy.myargv()[1:])

    return args

class STAMP():
    def __init__(self, assembly_name):
        self.assembly_name = assembly_name
        self.rospack = rospkg.RosPack()
        self.downward_client = DownwardClient()
        self.pddl_converter = PDDL_Converter(assembly_name)

    def get_objects_to_spawn(self):
        objects_to_spawn = rospy.get_param("/stamp/objects_to_spawn", None)
        objects_per_reference_frame = []
        reference_frames = []
        if objects_to_spawn:
            for (obj, properties) in objects_to_spawn.items():
                if properties['reference_frame'] not in reference_frames:
                    reference_frames.append(properties['reference_frame'])
            for reference_frame in reference_frames:
                objects = []
                poses = []
                for (obj, properties) in objects_to_spawn.items():
                    if properties['reference_frame'] == reference_frame:
                        objects.append(obj)
                        poses.append(properties['pose'])
                objects_per_reference_frame.append({'reference_frame': reference_frame, 'objects': objects, 'poses': poses})
            return objects_per_reference_frame
        else:
            return None






    def spawn_objects(self, object_names, object_poses, object_reference_frame):
        '''
        Spawn collision objects in the planning scene

        This function uses the o2ac_assembly_database module to spawn objects in the scene. The assembly, its objects and their metadata
        has to be set up inside the o2ac_assembly_database module.

        Given a list of object names from the set database (self.assembly_name), this functions spawns the listed objects in the corresponding poses in input 'object_poses'.
        The inputs 'object_names' and 'object_poses' must have the same lengths.
        The object poses are lists of floats in [x,y,z,r,p,y] format and are relative to the object_reference_frame
        '''
        moveit_commander.roscpp_initialize([])
        planning_scene_interface = moveit_commander.PlanningSceneInterface()
        transformer = tf.Transformer(True, rospy.Duration(10.0))

        for (object_name, object_pose) in zip(object_names, object_poses):
            co_pose = geometry_msgs.msg.Pose()
            co_pose.position.x = object_pose[0]
            co_pose.position.y = object_pose[1]
            co_pose.position.z = object_pose[2]
            quaternion = tf.transformations.quaternion_from_euler(eval(str(object_pose[3])),eval(str(object_pose[4])),eval(str(object_pose[5])))
            co_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)

            collision_object_transform = geometry_msgs.msg.TransformStamped()
            collision_object_transform.header.frame_id = 'WORLD'
            collision_object_transform.child_frame_id = object_name
            collision_object_transform.transform.translation.x = co_pose.position.x
            collision_object_transform.transform.translation.y = co_pose.position.y
            collision_object_transform.transform.translation.z = co_pose.position.z
            collision_object_transform.transform.rotation.x = co_pose.orientation.x
            collision_object_transform.transform.rotation.y = co_pose.orientation.y
            collision_object_transform.transform.rotation.z = co_pose.orientation.z
            collision_object_transform.transform.rotation.w = co_pose.orientation.w
            transformer.setTransform(collision_object_transform)

            collision_object = next(co for co in self.pddl_converter.assembly_handler.collision_objects if co.id == object_name)
            collision_object.header.frame_id = object_reference_frame
            collision_object.mesh_poses[0] = co_pose

            subframe_poses = []

            for (subframe_name, subframe_pose) in zip(collision_object.subframe_names, collision_object.subframe_poses):
                subframe_transform = geometry_msgs.msg.TransformStamped()
                subframe_transform.header.frame_id = object_name
                subframe_transform.child_frame_id = subframe_name
                subframe_transform.transform.translation.x = subframe_pose.position.x
                subframe_transform.transform.translation.y = subframe_pose.position.y
                subframe_transform.transform.translation.z = subframe_pose.position.z
                subframe_transform.transform.rotation.x = subframe_pose.orientation.x
                subframe_transform.transform.rotation.y = subframe_pose.orientation.y
                subframe_transform.transform.rotation.z = subframe_pose.orientation.z
                subframe_transform.transform.rotation.w = subframe_pose.orientation.w

                transformer.setTransform(subframe_transform)

                (trans,rot) = transformer.lookupTransform('WORLD', subframe_name, rospy.Time(0))

                subframe_pose = geometry_msgs.msg.Pose()
                subframe_pose.position = geometry_msgs.msg.Point(*trans)
                subframe_pose.orientation = geometry_msgs.msg.Quaternion(*rot)

                subframe_poses.append(subframe_pose)

            collision_object.subframe_poses = subframe_poses

            rospy.sleep(1)

            planning_scene_interface.add_object(collision_object)


    def do_step(self, pddl_domain_file, pddl_problem_file, translate_output_file = 'output.sas', search_output_file = 'result_plan', failed_plans_file = '', allow_regrasp = True):
        success = False
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
                    success = True
            else:
                rospy.logerr('Symbolic planning failed')
                raise RuntimeError('Fast downward server returned with an error')

        return success
                

if __name__ == "__main__":
    rospy.init_node('stamp')
    args = parse_args()
    stamp = STAMP('wrs_assembly_1')
    no_failed_plan_yet = True
    successful_plan = False
    max_iterations = 20
    objects_to_spawn = stamp.get_objects_to_spawn()
    if objects_to_spawn:
        for object_group in objects_to_spawn:
            stamp.spawn_objects(object_group['objects'], object_group['poses'], object_group['reference_frame'])
    iteration_counter = 0
    input_filenames = dict(args.__dict__)
    del input_filenames['keep_generated_files']
    while (not successful_plan):
        try:
            if no_failed_plan_yet:
                input_filenames['failed_plans_file'] = ''
                successful_plan = stamp.do_step(**input_filenames)
            else:
                input_filenames['failed_plans_file'] = 'failed_plans'
                successful_plan = stamp.do_step(**input_filenames)
            if not successful_plan:
                no_failed_plan_yet = False

        except Exception as ex:
            rospy.logerr(ex.__str__())
            break
        if iteration_counter == max_iterations:
            rospy.loginfo('Max number of iterations reached, exiting')
            break
        iteration_counter += 1
    if not args.keep_generated_files:
        directory = os.path.join(stamp.rospack.get_path('o2ac_task_planning_pddl_converter'),'symbolic', 'generated')
        for element in os.listdir(directory):
            if os.path.isfile(os.path.join(directory, element)):
                os.remove(os.path.join(directory, element))

    