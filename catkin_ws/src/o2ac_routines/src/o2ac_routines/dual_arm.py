import copy
import moveit_commander
from o2ac_routines.robot_base import RobotBase
import rospy
from std_msgs.msg import Bool
from o2ac_routines import helpers

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
        p = conversions.to_pose_stamped(slave.ns + "_gripper_tip_link", [0,0,0,0,0,0])
        slave_relation = self.listener.transformPose(master.ns + "_gripper_tip_link", p)
        return conversions.from_pose_to_list(slave_relation.pose)

    def master_slave_control(self, master_name, slave_name, target_pose, slave_relation, speed=0.3):
        """
        Moves b_bot and forces a_bot to follow.
        slave_relation is the slave TCP's pose (TODO: which coordinate system?), as a list in the form [xyz,xyzw].
        Obtain it from get_relative_pose_of_slave before calling this function.
        """

        master = self.active_robots[master_name]
        slave = self.active_robots[slave_name]

        master_plan, _ = master.go_to_pose_goal(target_pose, speed=speed, plan_only=True)

        master_slave_plan = copy.deepcopy(master_plan)
        master_slave_plan.joint_trajectory.joint_names += slave.robot_group.get_active_joints()

        last_ik_solution = None
        for point in master_slave_plan.joint_trajectory.points:
            master_tcp = master.get_tcp_pose(point.positions)
            master_tcp = self.listener.transformPose("world", master_tcp)
            slave_tcp = conversions.transform_pose("world", transformations.pose_to_transform(slave_relation), master_tcp)
            slave_tcp = self.listener.transformPose(slave.ns + "_base_link", slave_tcp)
            ik_solution = slave.force_controller._solve_ik(conversions.from_pose_to_list(slave_tcp.pose), q_guess=last_ik_solution, attempts=20, verbose=True)
            point.positions = list(point.positions) + list(ik_solution)
            point.velocities = list(point.velocities)*2
            point.accelerations = list(point.accelerations)*2

        return self.robot_group.execute(master_slave_plan)
