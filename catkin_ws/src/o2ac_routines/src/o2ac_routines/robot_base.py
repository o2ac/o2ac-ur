import actionlib
from actionlib_msgs.msg import GoalStatus

import moveit_commander
import rospy
import copy

from std_msgs.msg import Bool
import moveit_msgs.msg
import moveit_msgs.srv

from o2ac_routines import helpers
from ur_control import conversions, transformations


class RobotBase():
    """ Base methods for any robot arm controlled via MoveIt """

    def __init__(self, group_name, tf_listener):
        self.robot_group = moveit_commander.MoveGroupCommander(group_name)
        self.listener = tf_listener

        self.sequence_move_group = actionlib.SimpleActionClient("/sequence_move_group", moveit_msgs.msg.MoveGroupSequenceAction)
        self.plan_sequence_path = rospy.ServiceProxy('plan_sequence_path', moveit_msgs.srv.GetMotionSequence)

        self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
        self.pause_mode_ = False
        self.test_mode_ = False

        self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
        self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
        self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)

    def run_mode_callback(self, msg):
        self.run_mode_ = msg.data

    def pause_mode_callback(self, msg):
        self.pause_mode_ = msg.data

    def test_mode_callback(self, msg):
        self.test_mode_ = msg.data

    def set_up_move_group(self, speed, acceleration, planner="OMPL"):
        assert not rospy.is_shutdown()
        (speed_, accel_) = self.limit_speed_and_acc(speed, acceleration)
        group = self.robot_group
        rospy.logdebug("Setting velocity scaling to " + str(speed_))
        rospy.logdebug("Setting acceleration scaling to " + str(accel_))
        group.set_max_velocity_scaling_factor(speed_)
        group.set_max_acceleration_scaling_factor(accel_)
        self.set_planner(planner)
        return True

    def set_planner(self, planner="OMPL"):
        group = self.robot_group
        if planner == "OMPL":
            group.set_planning_pipeline_id("ompl")
            group.set_planner_id("RRTConnect")
            group.set_goal_joint_tolerance(1e-3)
        elif planner == "LINEAR":
            group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            group.set_planner_id("LIN")
        else:
            raise ValueError("Unsupported planner: %s" % planner)

    def limit_speed_and_acc(self, speed, acceleration):
        if self.pause_mode_ or self.test_mode_:
            if speed > self.reduced_mode_speed_limit:
                rospy.loginfo("Reducing speed from " + str(speed) + " to " + str(self.reduced_mode_speed_limit) + " because robot is in test or pause mode")
                speed = self.reduced_mode_speed_limit
        sp = copy.copy(speed)
        acc = copy.copy(acceleration)
        if sp > 1.0:
            sp = 1.0
        if acc > sp/2.0:
            # if acc > (sp/2.0 + .00001):  # This seems to trigger because of rounding errors unless we add this small value
            rospy.logdebug("Setting acceleration to " + str(sp/2.0) + " instead of " + str(acceleration) + " to avoid jerky motion.")
            acc = sp/2.0
        return (sp, acc)

    def check_goal_pose_reached(self, goal_pose):
        current_pose = self.robot_group.get_current_pose()
        if current_pose.header.frame_id != goal_pose.header.frame_id:
            gp = self.listener.transformPose(current_pose.header.frame_id, goal_pose)
        else:
            gp = goal_pose
        return helpers.all_close(gp.pose, current_pose.pose, 0.01)

    def get_current_pose_stamped(self):
        return self.robot_group.get_current_pose()

    def get_current_pose(self):
        return self.robot_group.get_current_pose().pose

    # ------ Robot motion functions

    def execute_plan(self, plan, wait=True):
        if self.robot_group.execute(plan, wait=wait):
            self.robot_group.clear_pose_targets()
            if wait:
                current_joints = self.robot_group.get_current_joint_values()
                goal_joints = plan.joint_trajectory.points[-1].positions
                return helpers.all_close(goal_joints, current_joints, 0.01)
            return True
        return False

    def go_to_pose_goal(self, pose_goal_stamped, speed=0.5, acceleration=0.25,
                        end_effector_link="", move_lin=True, wait=True, plan_only=False, initial_joints=None):
        planner = "LINEAR" if move_lin else "OMPL"
        if not self.set_up_move_group(speed, acceleration, planner):
            return False

        group = self.robot_group
        group.clear_pose_targets()

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"
        group.set_end_effector_link(end_effector_link)

        if initial_joints:
            group.set_start_state(helpers.to_robot_state(group, initial_joints))

        if move_lin:  # is this necessary??
            pose_goal_world = self.listener.transformPose("world", pose_goal_stamped)
            group.set_pose_target(pose_goal_world)
        else:
            group.set_pose_target(pose_goal_stamped)

        success = False
        tries = 0
        while not success and tries < 5 and not rospy.is_shutdown():
            if plan_only:
                success, plan, planning_time, error = group.plan()
                if success:
                    group.clear_pose_targets()
                    group.set_start_state_to_current_state()
                    return plan, planning_time
            else:
                res = group.go(wait=wait)  # Bool
                success = res & self.check_goal_pose_reached(pose_goal_stamped)

            if not success:
                rospy.sleep(0.2)
                rospy.logwarn("go_to_pose_goal(move_lin=%s) attempt failed. Retrying." % str(move_lin))
                tries += 1

        if not success:
            rospy.logerr("go_to_pose_goal failed " + str(tries) + " times! Broke out, published failed pose.")
            helpers.publish_marker(pose_goal_stamped, "pose", self.ns + "_move_lin_failed_pose_" + str(self.marker_counter))
            self.marker_counter += 1
        else:
            helpers.publish_marker(pose_goal_stamped, "pose", self.ns + "_go_to_pose_goal_failed_pose_" + str(self.marker_counter), marker_topic="o2ac_success_markers")
            self.marker_counter += 1

        group.clear_pose_targets()
        return success

    def move_lin_trajectory(self, trajectory, speed=1.0, acceleration=0.5, end_effector_link="", plan_only=False, initial_joints=None, retries=10):
        if not self.set_up_move_group(speed, acceleration, planner="LINEAR"):
            return False

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"

        group = self.robot_group

        group.set_end_effector_link(end_effector_link)
        waypoints = [(self.listener.transformPose("world", ps).pose, blend_radius) for ps, blend_radius in trajectory]

        motion_plan_requests = []

        # Start from current pose
        if initial_joints:
            initial_pose = self.get_tcp_pose(initial_joints, end_effector_link)
            group.set_pose_target(initial_pose)
        else:
            group.set_pose_target(group.get_current_pose(end_effector_link))
        msi = moveit_msgs.msg.MotionSequenceItem()
        msi.req = group.construct_motion_plan_request()
        msi.blend_radius = 0.0
        motion_plan_requests.append(msi)

        if initial_joints:
            msi.req.start_state = helpers.to_robot_state(self.robot_group, initial_joints)

        for wp, blend_radius in waypoints:
            group.clear_pose_targets()
            group.set_pose_target(wp)
            msi = moveit_msgs.msg.MotionSequenceItem()
            msi.req = group.construct_motion_plan_request()
            msi.req.start_state = moveit_msgs.msg.RobotState()
            # FIXME(cambel): blend radius does not seem to work for plan only
            msi.blend_radius = blend_radius if not plan_only else 0.0
            motion_plan_requests.append(msi)

        # Force last point to be 0.0 to avoid raising an error in the planner
        motion_plan_requests[-1].blend_radius = 0.0

        # Make MotionSequence
        goal = moveit_msgs.msg.MoveGroupSequenceGoal()
        goal.request = moveit_msgs.msg.MotionSequenceRequest()
        goal.request.items = motion_plan_requests

        for i in range(retries):
            if plan_only:
                # Make MotionSequence
                response = self.plan_sequence_path(goal.request)
                group.clear_pose_targets()

                if response.response.error_code.val == 1:
                    plan = response.response.planned_trajectories[0]  # support only one plan?
                    planning_time = response.response.planning_time
                    return plan, planning_time
                else:
                    rospy.logwarn("(move_lin_trajectory) Planning failed, retry: %s of %s" % (i+1, retries))
            else:
                result = self.sequence_move_group.send_goal_and_wait(goal)
                if result == GoalStatus.SUCCEEDED:
                    group.clear_pose_targets()
                    return True
                else:
                    rospy.logwarn("(move_lin_trajectory) failed, retry: %s of %s" % (i+1, retries))
        if plan_only:
            rospy.logerr("Failed to plan linear trajectory. error code: %s" % response.response.error_code.val)
        else:
            rospy.logerr("Fail move_lin_trajectory with status %s" % result)
        return False

    def move_lin(self, pose_goal_stamped, speed=0.5, acceleration=0.5, end_effector_link="", wait=True, plan_only=False, initial_joints=None):
        return self.go_to_pose_goal(pose_goal_stamped, speed, acceleration, end_effector_link, move_lin=True, wait=wait, plan_only=plan_only, initial_joints=initial_joints)

    def move_lin_rel(self, relative_translation=[0, 0, 0], relative_rotation=[0, 0, 0], speed=.5,
                     acceleration=0.2, relative_to_robot_base=False, relative_to_tcp=False,
                     wait=True, end_effector_link="", plan_only=False, initial_joints=None):
        '''
        Does a lin_move relative to the current position of the robot.

        relative_translation: translation relative to current tcp position, expressed in robot's own base frame
        relative_rotation: rotation relative to current tcp position, expressed in robot's own base frame
        relative_to_robot_base: If true, uses the robot_base coordinates for the relative motion (not workspace_center!)
        '''
        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"

        group = self.robot_group
        group.set_end_effector_link(end_effector_link)

        if initial_joints:
            w2b = self.listener.lookupTransform("world", self.ns + "_base_link", rospy.Time.now())  # static transform
            t_w2b = transformations.pose_to_transform(list(w2b[0]) + list(w2b[1]))  # transform robot's base to world frame
            b2tcp = self.kdl.forward(initial_joints, end_effector_link)  # forward kinematics
            t_b2tcp = transformations.pose_to_transform(b2tcp)  # transform tcp to robot's base
            if relative_to_tcp:
                new_pose = conversions.to_pose_stamped(end_effector_link, [0, 0, 0, 0, 0, 0.])
            elif relative_to_robot_base:
                new_pose = self.get_tcp_pose(initial_joints, end_effector_link)
            else:
                t_w2tcp = transformations.concatenate_matrices(t_w2b, t_b2tcp)
                new_pose = conversions.to_pose_stamped("world", transformations.pose_quaternion_from_matrix(t_w2tcp))
        else:
            new_pose = group.get_current_pose()

            if relative_to_robot_base:
                new_pose = self.listener.transformPose(self.ns + "_base_link", new_pose)
            elif relative_to_tcp:
                new_pose.header.stamp = rospy.Time.now() - rospy.Time(0.5)  # Workaround for TF lookup into the future error
                new_pose = self.listener.transformPose(self.ns + "_gripper_tip_link", new_pose)

        new_position = conversions.from_point(new_pose.pose.position) + relative_translation
        new_pose.pose.position = conversions.to_point(new_position)
        new_pose.pose.orientation = helpers.rotateQuaternionByRPYInUnrotatedFrame(relative_rotation[0], relative_rotation[1],
                                                                                  relative_rotation[2], new_pose.pose.orientation)

        if initial_joints:
            newpose = conversions.from_pose_to_list(new_pose.pose)  # new relative transformation
            t_newpose = transformations.pose_to_transform(newpose)
            if relative_to_tcp:
                # manually compute the transform from TCP to world since we are doing offline planning
                t_w2tcp = transformations.concatenate_matrices(t_w2b, t_b2tcp, t_newpose)
                new_pose = conversions.to_pose_stamped("world", transformations.pose_quaternion_from_matrix(t_w2tcp))
            if relative_to_robot_base:
                # manually compute the transform from base to world since we are doing offline planning
                t_w2tcp = transformations.concatenate_matrices(t_w2b, t_newpose)
                new_pose = conversions.to_pose_stamped("world", transformations.pose_quaternion_from_matrix(t_w2tcp))

        return self.go_to_pose_goal(new_pose, speed=speed, acceleration=acceleration,
                                    end_effector_link=end_effector_link,  wait=wait,
                                    move_lin=True, plan_only=plan_only, initial_joints=initial_joints)

    def go_to_named_pose(self, pose_name, speed=0.5, acceleration=0.5, wait=True, plan_only=False, initial_joints=None):
        """
        pose_name should be a named pose in the moveit_config, such as "home", "back" etc.
        """
        if not self.set_up_move_group(speed, acceleration, planner="OMPL"):
            return False
        group = self.robot_group

        if initial_joints:
            group.set_start_state(helpers.to_robot_state(group, initial_joints))

        group.set_named_target(pose_name)

        if plan_only:
            success, plan, planning_time, error = group.plan()
            if success:
                group.clear_pose_targets()
                group.set_start_state_to_current_state()
                return plan, planning_time
            else:
                rospy.logerr("Failed planning with error: %s" % error)
        else:
            group.go(wait=wait)
            group.clear_pose_targets()
            goal = helpers.ordered_joint_values_from_dict(group.get_named_target_values(pose_name), group.get_active_joints())
            current_joint_values = group.get_current_joint_values()
            move_success = helpers.all_close(goal, current_joint_values, 0.01)
            return move_success
        return False

    def move_joints(self, joint_pose_goal, speed=0.6, acceleration=0.3, wait=True, plan_only=False, initial_joints=None):
        if not self.set_up_move_group(speed, acceleration, planner="OMPL"):
            return False
        group = self.robot_group

        if initial_joints:
            group.set_start_state(helpers.to_robot_state(group, initial_joints))

        group.set_joint_value_target(joint_pose_goal)

        if plan_only:
            success, plan, planning_time, error = group.plan()
            if success:
                group.set_start_state_to_current_state()
                return plan, planning_time
            else:
                rospy.logerr("Failed planning with error: %s" % error)
        else:
            group.go(wait=wait)
            current_joints = group.get_current_joint_values()
            return helpers.all_close(joint_pose_goal, current_joints, 0.01)
        return False
