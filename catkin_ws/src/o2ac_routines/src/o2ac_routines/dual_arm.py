import copy
from math import degrees

import numpy as np
import moveit_commander
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

    # Dual Arm manipulation

    def go_to_goal_poses(self, robot1_pose, robot2_pose, plan_only=False, speed=0.5, acceleration=0.25, planner="OMPL", robot1_ee_link=None, robot2_ee_link=None):
        self.set_up_move_group(speed, acceleration, planner)

        ee_link1 = self.robot1.ns + "_gripper_tip_link" if robot1_ee_link is None else robot1_ee_link
        ee_link2 = self.robot2.ns + "_gripper_tip_link" if robot2_ee_link is None else robot2_ee_link

        self.robot_group.set_pose_target(robot1_pose, end_effector_link=ee_link1)
        self.robot_group.set_pose_target(robot2_pose, end_effector_link=ee_link2)

        success = False
        tries = 10
        while not success and tries > 0 and not rospy.is_shutdown():
            tries -= 1
            if plan_only:
                success, plan, planning_time, error = self.robot_group.plan()

                self.robot_group.clear_pose_targets()
                return plan, planning_time
            else:
                self.robot_group.go(wait=True)
                success = self.robot1.check_goal_pose_reached(robot1_pose) and self.robot2.check_goal_pose_reached(robot2_pose)

            if not success:
                rospy.logwarn("ab_go_to_poses attempt failed")

        self.robot_group.clear_pose_targets()
        return True

    def get_relative_pose_of_slave(self, master_name, slave_name):
        """ Return the relative pose """
        master = self.active_robots[master_name]
        slave = self.active_robots[slave_name]
        master_tcp = conversions.from_pose_to_list(master.get_current_pose())
        slave_tcp = conversions.from_pose_to_list(slave.get_current_pose())
        return np.concatenate([slave_tcp[:3]-master_tcp[:3], transformations.diff_quaternion(slave_tcp[3:], master_tcp[3:])])

    def master_slave_control(self, master_name, slave_name, target_pose, slave_relation, speed=0.3):
        """
        Moves b_bot and forces a_bot to follow.
        slave_relation is the slave TCP's pose (TODO: which coordinate system?), as a list in the form [xyz,xyzw].
        Obtain it from get_relative_pose_of_slave before calling this function.
        """
        self.robot1.activate_ros_control_on_ur()
        self.robot2.activate_ros_control_on_ur()
        
        master = self.active_robots[master_name]
        slave = self.active_robots[slave_name]

        master_plan, _ = master.go_to_pose_goal(target_pose, speed=speed, plan_only=True)

        master_slave_plan = copy.deepcopy(master_plan)
        master_slave_plan.joint_trajectory.joint_names += slave.robot_group.get_active_joints()

        last_ik_solution = None
        last_velocities = None
        for i, point in enumerate(master_slave_plan.joint_trajectory.points):
            master_tcp = master.get_tcp_pose(point.positions)
            master_tcp = conversions.from_pose_to_list(self.listener.transformPose("world", master_tcp).pose)
            slave_tcp = np.concatenate([master_tcp[:3]+slave_relation[:3], transformations.quaternion_multiply(slave_relation[3:], master_tcp[3:])])
            slave_tcp = conversions.to_pose_stamped("world", slave_tcp)
            slave_tcp = self.listener.transformPose(slave.ns + "_base_link", slave_tcp)
            ok = False
            tries = 10.0
            while not ok and tries > 0:
                tries -= 1
                if last_ik_solution is None:
                    ik_solution = slave.robot_group.get_current_joint_values()
                else:
                    ik_solution = slave.solve_ik(conversions.from_pose_to_list(slave_tcp.pose), q_guess=last_ik_solution, attempts=20, verbose=True)
                # Sanity check
                # Compare the slave largest joint displacement for this IK solution vs the master largest joint displacement
                # if the displacement is more than 5 deg, check that the displacement is not larger than 2x the master joint displacement
                if i > 0:
                    slave_joint_displacement = np.max(np.abs(ik_solution-last_ik_solution))
                    master_joint_displacement = np.max(np.abs(np.array(master_plan.joint_trajectory.points[i].positions)-master_plan.joint_trajectory.points[i-1].positions))
                    if slave_joint_displacement > degrees(5):  # arbitrary
                        ok = slave_joint_displacement < master_joint_displacement * 2.0  # arbitrary
                    else:
                        ok = True
                else:
                    ok = True

            if not ok:
                rospy.logerr("Could not find a valid IK solution for the slave-robot")
                return False
            
            # Compute slave velocities/accelerations
            previous_time = 0 if i == 0 else master_plan.joint_trajectory.points[i-1].time_from_start.to_sec()
            duration = point.time_from_start.to_sec()-previous_time

            slave_joint_displacement = np.array(ik_solution)-last_ik_solution if i > 0 else np.zeros_like(ik_solution)
            # v = x/t
            slave_velocities = slave_joint_displacement/duration  if duration > 0 else np.zeros_like(slave_joint_displacement)
            # a = 2*(x/t^2 - v/t). Here is halved, I supposed it is the enforced a = v/2 that we set for the robots
            slave_accelerations = slave_joint_displacement/pow(duration,2) - last_velocities/duration  if duration > 0 else np.zeros_like(slave_joint_displacement)
            
            point.positions = list(point.positions) + list(ik_solution)
            point.velocities = list(point.velocities) + slave_velocities.tolist()
            point.accelerations = list(point.accelerations) + slave_accelerations.tolist()
            last_ik_solution = np.copy(ik_solution)
            last_velocities = np.copy(slave_velocities)

        return self.robot_group.execute(master_slave_plan)

    def set_up_move_group(self, speed, acceleration, planner="OMPL"):
        self.robot1.activate_ros_control_on_ur()
        self.robot2.activate_ros_control_on_ur()
        return RobotBase.set_up_move_group(self, speed, acceleration, planner)
