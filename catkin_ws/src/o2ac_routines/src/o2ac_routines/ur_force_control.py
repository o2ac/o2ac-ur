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
# Author: Cristian C. Beltran-Hernandez

from o2ac_routines import helpers
import rospy
import rospkg
import yaml

import numpy as np

from ur_control import utils, spalg, transformations, traj_utils, conversions
from ur_control.hybrid_controller import ForcePositionController
from ur_control.compliant_controller import CompliantController
from ur_control.constants import STOP_ON_TARGET_FORCE, TERMINATION_CRITERIA, DONE

from o2ac_routines.helpers import get_target_force, get_direction_index, get_orthogonal_plane, get_random_valid_direction


def create_pid(pid, default_ki=0.0, default_kd=0.0):
    kp = np.array(pid['p'])
    if pid['d'] == 'default':
        kd = kp * default_kd
    else:
        kd = np.array(pid['d'])
    if pid['i'] == 'default':
        ki = kp * default_ki
    else:
        ki = np.array(pid['i'])
    dynamic = pid['dynamic']
    return utils.PID(Kp=kp, Ki=ki, Kd=kd, dynamic_pid=dynamic, max_gain_multiplier=100.0)


class URForceController(CompliantController):
    def __init__(self, robot_name, listener, tcp_link='gripper_tip_link', **kwargs):
        self.listener = listener
        self.default_tcp_link = robot_name + '_' + tcp_link
        self.robot_name = robot_name

        CompliantController.__init__(self, ft_sensor=True, namespace=robot_name,
                                     joint_names_prefix=robot_name+'_', robot_urdf=robot_name,
                                     robot_urdf_package='o2ac_scene_description',
                                     ee_link=tcp_link, **kwargs)

    def _init_force_controller(self, config_file):
        path = rospkg.RosPack().get_path("o2ac_routines") + "/config/" + config_file + ".yaml"
        with open(path, 'r') as f:
            config = yaml.load(f)

        position_pd = create_pid(config['position'], default_kd=0.01, default_ki=0.01)
        force_pd = create_pid(config['force'],    default_kd=0.03,  default_ki=0.01)

        dt = config['dt']
        selection_matrix = config['selection_matrix']

        self.max_force_torque = config['max_force_torque']

        self.force_model = ForcePositionController(position_pd=position_pd, force_pd=force_pd, alpha=np.diag(selection_matrix), dt=dt)

    def force_control(self, target_force=None, target_positions=None,
                      selection_matrix=None, relative_to_ee=False,
                      timeout=10.0, stop_on_target_force=False, termination_criteria=None,
                      displacement_epsilon=0.002, check_displacement_time=2.0,
                      config_file=None, time_compensation=True, end_effector_link=None):
        """ 
            Use with caution!! 
            target_force: list[6], target force for each direction x,y,z,ax,ay,az
            target_positions: array[array[7]] or array[7], can define a single target pose or a trajectory of multiple poses.
            selection_matrix: list[6], define which direction is controlled by position(1.0) or force(0.0)
            relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
            timeout: float, duration in seconds of the force control
        """
        if end_effector_link and end_effector_link != self.default_tcp_link:
            # Init IK and FK solvers with new end effector link
            self._init_ik_solver(self.base_link, end_effector_link)

        if config_file is None:
            config_file = "force_control_" + self.robot_name
        self._init_force_controller(config_file)

        rospy.sleep(1.0)  # give it some time to set up before moving
        self.set_wrench_offset(True)  # offset the force sensor
        self.relative_to_ee = relative_to_ee if relative_to_ee is not None else self.relative_to_ee

        target_positions = self.end_effector() if target_positions is None else np.array(target_positions)
        target_force = np.array([0., 0., 0., 0., 0., 0.]) if target_force is None else np.array(target_force)

        self.force_model.set_goals(force=target_force)
        self.force_model.alpha = np.diag(selection_matrix) if selection_matrix is not None else self.force_model.alpha  # alpha is the selection_matrix

        result = self.set_hybrid_control_trajectory(target_positions, self.force_model, max_force_torque=self.max_force_torque, timeout=timeout,
                                                    stop_on_target_force=stop_on_target_force, termination_criteria=termination_criteria,
                                                    displacement_epsilon=displacement_epsilon, check_displacement_time=check_displacement_time, debug=False,
                                                    time_compensation=time_compensation)
        self.force_model.reset()  # reset pid errors

        if end_effector_link and end_effector_link != self.default_tcp_link:
            # Init IK and FK solvers with default end effector link
            self._init_ik_solver(self.base_link, self.default_tcp_link)

        return result

    def execute_circular_trajectory(self, *args, **kwargs):
        """
            Execute a circular trajectory on a given plane, with respect to the base of the robot, with a given radius
            Note: we assume that the robot is in its initial position 
        """
        kwargs.update({"trajectory_type": "circular"})
        return self.execute_trajectory(*args, **kwargs)

    def execute_spiral_trajectory(self, *args, **kwargs):
        kwargs.update({"trajectory_type": "spiral"})
        return self.execute_trajectory(*args, **kwargs)

    def execute_trajectory(self, plane, max_radius, radius_direction=None,
                           steps=100, revolutions=5,
                           wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                           target_force=None, selection_matrix=None, timeout=10.,
                           displacement_epsilon=0.002, check_displacement_time=2.0,
                           termination_criteria=None, config_file=None,
                           end_effector_link=None, trajectory_type="spiral"):
        from_center = True if trajectory_type == "spiral" else False
        eff = self.default_tcp_link if not end_effector_link else end_effector_link
        direction = radius_direction if radius_direction else helpers.get_random_valid_direction(plane)
        dummy_trajectory = traj_utils.compute_trajectory([0, 0, 0, 0, 0, 0, 1.],
                                                         plane, max_radius, direction, steps, revolutions, from_center=from_center, trajectory_type=trajectory_type,
                                                         wiggle_direction=wiggle_direction, wiggle_angle=wiggle_angle, wiggle_revolutions=wiggle_revolutions)
        # convert dummy_trajectory (initial pose frame id) to robot's base frame
        now = rospy.Time.now()
        try:
            self.listener.waitForTransform(self.base_link, eff, now, rospy.Duration(1))
            transform2target = self.listener.fromTranslationRotation(*self.listener.lookupTransform(self.base_link, eff, now))
        except:
            return False

        trajectory = []
        for p in dummy_trajectory:
            ps = conversions.to_pose_stamped(self.base_link, p)
            trajectory.append(conversions.from_pose_to_list(conversions.transform_pose(self.base_link, transform2target, ps).pose))

        sm = selection_matrix if selection_matrix else [1., 1., 1., 1., 1., 1.]  # no force control by default
        return self.force_control(target_force=target_force, target_positions=trajectory, selection_matrix=sm,
                                  timeout=timeout, relative_to_ee=False, termination_criteria=termination_criteria,
                                  displacement_epsilon=displacement_epsilon, check_displacement_time=check_displacement_time, config_file=config_file,
                                  time_compensation=False, end_effector_link=end_effector_link)

    def linear_push(self, force, direction, max_translation=None, relative_to_ee=False, timeout=10.0, slow=False, selection_matrix=None):
        """
        Apply force control in one direction until contact with `force`
        robot_name: string, name of the robot
        force: float, desired force
        direction: string, direction for linear_push +- X,Y,Z relative to base or end-effector, see next argument
        relative_to_ee: bool, whether to use the base_link of the robot as frame or the ee_link (+ ee_transform)
        """
        config_file = "force_control_" + self.robot_name if slow else "force_control_linear_push_" + self.robot_name
        if "Z" in direction:
            target_force = get_target_force(direction, force)
        else:
            offset = 1 if self.robot_name == "b_bot" else -1  # account for robot's mirror position
            target_force = offset * get_target_force(direction, force)

        if selection_matrix is None:
            selection_matrix = np.array(target_force == 0.0) * 1.0  # define the selection matrix based on the target force

        initial_pose = self.end_effector()[get_direction_index(direction[1])]

        if max_translation is not None:
            def termination_criteria(cpose, standby): return abs(initial_pose - cpose[get_direction_index(direction[1])]) >= max_translation
        else:
            termination_criteria = None

        result = self.force_control(target_force=target_force, selection_matrix=selection_matrix,
                                    relative_to_ee=relative_to_ee, timeout=timeout, stop_on_target_force=True,
                                    termination_criteria=termination_criteria,
                                    config_file=config_file)

        if result in (TERMINATION_CRITERIA, DONE, STOP_ON_TARGET_FORCE):
            rospy.loginfo("Completed linear_push: %s" % result)
            return True
        rospy.logerr("Fail to complete linear_push %s" % result)
        return False

    def do_insertion(self, target_pose_in_target_frame, insertion_direction, timeout,
                     radius=0.0, radius_direction=None, revolutions=3, force=1.0, relaxed_target_by=0.0,
                     wiggle_direction=None, wiggle_angle=0.0, wiggle_revolutions=0.0,
                     selection_matrix=None, displacement_epsilon=0.002, check_displacement_time=2.0,
                     config_file=None):
        """
            target_pose_in_target_frame: PoseStamp, target in target frame
            insertion_direction: string, [+/-] "X", "Y", or "Z" in robot's base frame! Note: limited to one direction TODO: convert to target frame? or compute from target frame?
            relaxed_target_by: float, distance to allow the insertion to succeed if there is no motion in 2 seconds
            TODO: try to make the insertion direction define-able as an axis with respect to the object-to-be-inserted and the target-frame
        """
        axis = get_direction_index(insertion_direction[1])
        plane = get_orthogonal_plane(insertion_direction[1])
        radius_direction = get_random_valid_direction(plane) if radius_direction is None else radius_direction

        if "Z" in insertion_direction:
            target_force = get_target_force(insertion_direction, force)
        else:
            offset = 1 if self.robot_name == "b_bot" else -1  # account for robot's mirror position
            target_force = offset * get_target_force(insertion_direction, force)

        if selection_matrix is None:
            selection_matrix = np.array(target_force == 0.0) * 0.8  # define the selection matrix based on the target force

        translation, rotation = self.listener.lookupTransform(target_pose_in_target_frame.header.frame_id, self.ns + "_base_link", rospy.Time.now())
        transform2target = self.listener.fromTranslationRotation(translation, rotation)

        start_pose_robot_base = conversions.to_pose_stamped(self.ns + "_base_link", self.end_effector())
        start_pose_in_target_frame = conversions.transform_pose(target_pose_in_target_frame.header.frame_id, transform2target, start_pose_robot_base)
        start_pose_of = conversions.from_pose_to_list(start_pose_in_target_frame.pose)
        target_pose_of = conversions.from_pose_to_list(target_pose_in_target_frame.pose)
        more_than = start_pose_of[axis] < target_pose_of[axis]

        def termination_criteria(current_pose, standby):
            current_pose_robot_base = conversions.to_pose_stamped(self.ns + "_base_link", current_pose)
            current_pose_in_target_frame = conversions.transform_pose(target_pose_in_target_frame.header.frame_id, transform2target, current_pose_robot_base)
            current_pose_of = conversions.from_pose_to_list(current_pose_in_target_frame.pose)
            # print("check cp,tp", current_pose_of[axis], target_pose_of[axis])
            if more_than:
                return current_pose_of[axis] >= target_pose_of[axis] or \
                    (standby and current_pose_of[axis] >= target_pose_of[axis] - relaxed_target_by)
            return current_pose_of[axis] <= target_pose_of[axis] or \
                (standby and current_pose_of[axis] <= target_pose_of[axis] + relaxed_target_by)

        result = self.execute_spiral_trajectory(plane, max_radius=radius, radius_direction=radius_direction, steps=100, revolutions=revolutions,
                                                wiggle_direction=wiggle_direction, wiggle_angle=wiggle_angle, wiggle_revolutions=wiggle_revolutions,
                                                target_force=target_force, selection_matrix=selection_matrix, timeout=timeout,
                                                displacement_epsilon=displacement_epsilon, check_displacement_time=check_displacement_time,
                                                termination_criteria=termination_criteria, config_file=config_file)

        if result in (TERMINATION_CRITERIA, DONE, STOP_ON_TARGET_FORCE):
            rospy.loginfo("Completed insertion with state: %s" % result)
        else:
            rospy.logerr("Fail to complete insertion with state %s" % result)
        return result
