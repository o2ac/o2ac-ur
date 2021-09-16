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
# Author: Cristian C. Beltran-Hernandez, Felix von Drigalski

import copy

import numpy as np
import moveit_commander

import moveit_msgs.msg
import moveit_msgs.srv
from o2ac_routines import helpers

from o2ac_routines.robot_base import RobotBase
import rospy

from ur_control import conversions, transformations


class DualArm(RobotBase):
    def __init__(self, group_name, robot1, robot2, tf_listener):
        RobotBase.__init__(self, group_name=group_name, tf_listener=tf_listener)

        self.robot_group = moveit_commander.MoveGroupCommander(group_name)
        self.robot1 = robot1
        self.robot2 = robot2
        self.active_robots = {self.robot1.ns: self.robot1, self.robot2.ns: self.robot2}

        rospy.wait_for_service('/check_state_validity')
        self.moveit_state_validity_srv = rospy.ServiceProxy('/check_state_validity', moveit_msgs.srv.GetStateValidity)

    def check_state_validity(self, robot_state=None, robot_name=None):
        """
            Check with the PlanningScene that the robot state for the move_group is valid
            robot_state: if pass as `list`: assumes that the joint values are in the same order as defined for that group
                         if not defined, return the validity of the current pose of the robot
            robot_name: if defined, that robot is used to create the RobotState
        """
        if robot_state:
            if isinstance(robot_state, moveit_msgs.msg.RobotState):
                robot_state_ = robot_state
            elif isinstance(robot_state, list):
                robot_state_ = moveit_msgs.msg.RobotState()
                robot_state_.joint_state.name = self.active_robots[robot_name].robot_group.get_active_joints() if robot_name else self.robot_group.get_active_joints()
                robot_state_.joint_state.position = robot_state
            else:
                rospy.logerr("Unsupported type of robot_state %s" % type(robot_state))
                raise
        else:
            return self.check_state_validity(robot_state=self.robot_group.get_current_joint_values())

        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.robot_group.get_name()
        req.robot_state = robot_state_

        response = self.moveit_state_validity_srv.call(req)
        return response.valid

    def compute_ik(self, robot1_pose, robot2_pose, timeout, end_effector_link1="", end_effector_link2="", joints_seed1=None, joints_seed2=None, retry=False, allow_collisions=False):
        joints1 = self.robot1.compute_ik(robot1_pose, joints_seed=joints_seed1, timeout=timeout, end_effector_link=end_effector_link1, retry=retry, allow_collisions=allow_collisions)
        joints2 = self.robot2.compute_ik(robot2_pose, joints_seed=joints_seed2, timeout=timeout, end_effector_link=end_effector_link2, retry=retry, allow_collisions=allow_collisions)
        return joints1 + joints2  # concat results

    # Dual Arm manipulation

    def go_to_goal_poses(self, robot1_pose, robot2_pose, plan_only=False, speed=0.5, acceleration=0.25,
                         planner="OMPL", robot1_ee_link=None, robot2_ee_link=None, initial_joints=None, timeout=10.0):
        speed_, accel_ = self.set_up_move_group(speed, acceleration, planner)

        ee_link1 = self.robot1.ns + "_gripper_tip_link" if robot1_ee_link is None else robot1_ee_link
        ee_link2 = self.robot2.ns + "_gripper_tip_link" if robot2_ee_link is None else robot2_ee_link

        self.robot_group.set_pose_target(robot1_pose, end_effector_link=ee_link1)
        self.robot_group.set_pose_target(robot2_pose, end_effector_link=ee_link2)

        if initial_joints:
            self.robot_group.set_start_state(helpers.to_robot_state(self.robot_group, initial_joints))

        success = False
        start_time = rospy.get_time()
        while not success and (rospy.get_time()-start_time < timeout) and not rospy.is_shutdown():
            success, plan, planning_time, error = self.robot_group.plan()
            if success:
                if plan_only:
                    plan = self.robot_group.retime_trajectory(self.robot_group.get_current_state(), plan, algorithm="time_optimal_trajectory_generation",
                                                              velocity_scaling_factor=speed_, acceleration_scaling_factor=accel_)
                    self.robot_group.clear_pose_targets()
                    return plan, planning_time
                else:
                    self.robot_group.go(wait=True)
                    success = self.robot1.check_goal_pose_reached(robot1_pose) and self.robot2.check_goal_pose_reached(robot2_pose)
            if not success:
                rospy.logwarn("ab_go_to_poses attempt failed")

        self.robot_group.clear_pose_targets()
        return success

    def get_relative_pose_of_slave(self, master_name, slave_name):
        """ Return the relative pose """
        master = self.active_robots[master_name]
        slave = self.active_robots[slave_name]
        master_tcp = conversions.from_pose_to_list(master.get_current_pose())
        slave_tcp = conversions.from_pose_to_list(slave.get_current_pose())
        return np.concatenate([slave_tcp[:3]-master_tcp[:3], transformations.diff_quaternion(slave_tcp[3:], master_tcp[3:])])

    def master_slave_control(self, master_name, slave_name, target_pose, slave_relation, speed=0.3, plan_only=False, initial_joints=None):
        """
        Moves b_bot and forces a_bot to follow.
        slave_relation is the slave TCP's pose (TODO: which coordinate system?), as a list in the form [xyz,xyzw].
        Obtain it from get_relative_pose_of_slave before calling this function.
        """
        self.robot1.activate_ros_control_on_ur()
        self.robot2.activate_ros_control_on_ur()

        master = self.active_robots[master_name]
        slave_initial_joints = initial_joints[6:] if initial_joints is not None else None
        master_initial_joints = initial_joints[:6] if initial_joints is not None else None
        result = master.go_to_pose_goal(target_pose, speed=speed, plan_only=True, initial_joints=master_initial_joints, move_lin=True)
        if not result:
            rospy.logerr("Failed to plan `master` trajectory. Abort.")
            return False

        master_plan, m_planning_time = result

        result = self.compute_master_slave_plan(master_name, slave_name, slave_relation, slave_initial_joints, master_plan)
        if not result:
            return False

        master_slave_plan, ms_planning_time = result

        if plan_only:
            return master_slave_plan, (m_planning_time + ms_planning_time)
        else:
            return self.execute_plan(master_slave_plan, wait=True)

    def compute_master_slave_plan(self, master_name, slave_name, slave_relation, slave_initial_joints, master_plan):
        init_time = rospy.Time.now()
        slave = self.active_robots[slave_name]
        master = self.active_robots[master_name]
        master_slave_plan = copy.deepcopy(master_plan)
        master_slave_plan.joint_trajectory.joint_names += slave.robot_group.get_active_joints()

        last_ik_solution = None
        last_velocities = None
        master_initial_pose = master.compute_fk(master_plan.joint_trajectory.points[0].positions)
        master_initial_pose = conversions.from_pose_to_list(self.listener.transformPose("world", master_initial_pose).pose)
        for i, point in enumerate(master_slave_plan.joint_trajectory.points):
            master_tcp = master.compute_fk(point.positions)
            master_tcp = conversions.from_pose_to_list(self.listener.transformPose("world", master_tcp).pose)
            master_tcp_rotation = transformations.diff_quaternion(master_initial_pose[3:], master_tcp[3:])
            slave_rotation = transformations.quaternion_multiply(slave_relation[3:], master_tcp[3:])
            slave_translation = master_tcp[:3]+transformations.vector_to_pyquaternion(master_tcp_rotation).rotate(slave_relation[:3])
            slave_tcp = np.concatenate([slave_translation, slave_rotation])
            slave_tcp = conversions.to_pose_stamped("world", slave_tcp)
            slave_tcp = self.listener.transformPose(slave.ns + "_base_link", slave_tcp)

            tries = 10.0
            ik_solver_timeout = 0.001
            ik_solution = None

            if last_ik_solution is None:
                ik_solution = slave.robot_group.get_current_joint_values() if slave_initial_joints is None else slave_initial_joints

            while ik_solution is None and tries > 0:
                tries -= 1
                # after every failure, incrementally give more time to the IK solver to compute a better solution
                ik_solver_timeout += ik_solver_timeout
                ik_solution = slave.compute_ik(target_pose=slave_tcp, joints_seed=last_ik_solution, timeout=ik_solver_timeout, allow_collisions=True)

                if not ik_solution:
                    continue

                # Sanity check
                # 1. there shouldn't be any configuration flips
                if self.joint_configuration_changes(last_ik_solution, ik_solution):
                    continue

                # 2. Compare the slave largest joint displacement for this IK solution vs the master largest joint displacement
                # if the displacement is more than 5 deg, check that the displacement is not larger than 2x the master joint displacement
                if i > 0:
                    slave_joint_displacement = np.max(np.abs(ik_solution-last_ik_solution))
                    master_joint_displacement = np.max(np.abs(np.array(master_plan.joint_trajectory.points[i].positions)-master_plan.joint_trajectory.points[i-1].positions))
                    if slave_joint_displacement > np.deg2rad(10):  # arbitrary
                        if slave_joint_displacement > master_joint_displacement * 2.0:  # arbitrary
                            ik_solution = None  # reject solution
                            continue

                if ik_solution and self.check_state_validity(list(point.positions) + list(ik_solution)):
                    break

            if ik_solution is None:
                rospy.logerr("Could not find a valid IK solution for the slave-robot at point: %s" % (i+1))
                helpers.publish_marker(slave_tcp, "pose", "slave_move_failed_IK_pose")
                return False
            # else:
            #     helpers.publish_marker(slave_tcp, "pose", "slave_move", "o2ac_success_markers")

            # Compute slave velocities/accelerations
            if i == 0 or i == (len(master_plan.joint_trajectory.points) - 1):
                slave_velocities = np.zeros_like(point.velocities)
                slave_accelerations = np.zeros_like(point.accelerations)
            else:
                previous_time = 0 if i == 0 else master_plan.joint_trajectory.points[i-1].time_from_start.to_sec()
                duration = point.time_from_start.to_sec()-previous_time

                slave_joint_displacement = np.array(ik_solution)-last_ik_solution if i > 0 else np.zeros_like(ik_solution)
                # v = x/t
                slave_velocities = slave_joint_displacement/duration if duration > 0 else np.zeros_like(slave_joint_displacement)
                # a = 2*(x/t^2 - v/t). Here is halved, I supposed it is the enforced a = v/2 that we set for the robots
                slave_accelerations = slave_joint_displacement/pow(duration, 2) - last_velocities/duration if duration > 0 else np.zeros_like(slave_joint_displacement)

            point.positions = list(point.positions) + list(ik_solution)
            point.velocities = list(point.velocities) + slave_velocities.tolist()
            point.accelerations = list(point.accelerations) + slave_accelerations.tolist()
            last_ik_solution = np.copy(ik_solution)
            last_velocities = np.copy(slave_velocities)

        return master_slave_plan, (rospy.Time.now() - init_time).secs

    def set_up_move_group(self, speed, acceleration, planner="OMPL"):
        self.robot1.activate_ros_control_on_ur()
        self.robot2.activate_ros_control_on_ur()
        return RobotBase.set_up_move_group(self, speed, acceleration, planner)
