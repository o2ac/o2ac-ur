import actionlib
from actionlib_msgs.msg import GoalStatus

import copy
import rosbag
import rospkg
import rospy
import moveit_commander
import numpy as np

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
from std_msgs.msg import Bool

from o2ac_routines import helpers
from ur_control import conversions, transformations


class RobotBase():
    """ Base methods for any robot arm controlled via MoveIt """

    def __init__(self, group_name, tf_listener):
        self.robot_group = moveit_commander.MoveGroupCommander(group_name)
        self.listener = tf_listener

        self.sequence_move_group = actionlib.SimpleActionClient("/sequence_move_group", moveit_msgs.msg.MoveGroupSequenceAction)

        self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
        self.pause_mode_ = False
        self.test_mode_ = False

        self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
        self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
        self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)

        rospy.wait_for_service('compute_ik')
        rospy.wait_for_service('compute_fk')

        self.moveit_ik_srv = rospy.ServiceProxy('/compute_ik', moveit_msgs.srv.GetPositionIK)
        self.moveit_fk_srv = rospy.ServiceProxy('/compute_fk', moveit_msgs.srv.GetPositionFK)

    def run_mode_callback(self, msg):
        self.run_mode_ = msg.data

    def pause_mode_callback(self, msg):
        self.pause_mode_ = msg.data

    def test_mode_callback(self, msg):
        self.test_mode_ = msg.data

    def compute_fk(self, robot_state=None, tcp_link=None, frame_id=None):
        """
            Compute the Forward kinematics for a move group using the moveit service
            robot_state: if pass as `list`: assumes that the joint values are in the same order as defined for that group
        """
        if robot_state:
            if isinstance(robot_state, moveit_msgs.msg.RobotState):
                robot_state_ = robot_state
            elif isinstance(robot_state, (list, tuple, np.ndarray)):
                robot_state_ = moveit_msgs.msg.RobotState()
                robot_state_.joint_state.name = self.robot_group.get_active_joints()
                robot_state_.joint_state.position = list(robot_state)
            else:
                rospy.logerr("Unsupported type of robot_state %s" % type(robot_state))
                raise
        else:
            return self.compute_fk(robot_state=self.robot_group.get_current_joint_values())
        req = moveit_msgs.srv.GetPositionFKRequest()
        req.fk_link_names = [tcp_link if tcp_link else self.robot_group.get_end_effector_link()]
        req.robot_state = robot_state_
        res = self.moveit_fk_srv.call(req)
        if res.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            rospy.logwarn("compute FK failed with code: %s" % res.error_code.val)
            return False
        else:
            if frame_id:
                return self.listener.transformPose(frame_id, res.pose_stamped[0])
            return res.pose_stamped[0]

    def compute_ik(self, target_pose, joints_seed=None, timeout=0.01, end_effector_link="", retry=False, allow_collisions=False):
        """
            Compute the inverse kinematics for a move group the moveit service
            return
            solution: `list`: the joint values are in the same order as defined for that group
        """
        if isinstance(target_pose, geometry_msgs.msg.PoseStamped):
            ik_request = moveit_msgs.msg.PositionIKRequest()
            ik_request.avoid_collisions = not allow_collisions
            ik_request.timeout = rospy.Duration(timeout)
            ik_request.pose_stamped = target_pose
            ik_request.group_name = self.robot_group.get_name()
            ik_request.ik_link_name = end_effector_link
            ik_request.robot_state.joint_state.name = self.robot_group.get_active_joints()
            ik_request.robot_state.joint_state.position = joints_seed if joints_seed is not None else self.robot_group.get_current_joint_values()
        else:
            rospy.logerr("Unsupported type of target_pose %s" % type(target_pose))
            raise

        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request = ik_request
        res = self.moveit_ik_srv.call(req)

        if retry:
            start_time = rospy.get_time()
            while res.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS \
                  and not rospy.is_shutdown() and (rospy.get_time() - start_time < 10):
                res = self.moveit_ik_srv.call(req)
        
        if res.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            rospy.logwarn("compute IK failed with code: %s" % res.error_code.val)
            return None

        solution = []
        for joint_name in self.robot_group.get_active_joints():
            solution.append(res.solution.joint_state.position[res.solution.joint_state.name.index(joint_name)])
        return solution

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
        elif planner == "PTP":
            group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            group.set_planner_id("PTP")
        elif planner == "CIRC":
            group.set_planning_pipeline_id("pilz_industrial_motion_planner")
            group.set_planner_id("CIRC")
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

    def joint_configuration_changes(self, start, end, tolerance=0.1):
        """ Returns True if the sign of any joint angle changes during the motion,
            and the joint angle is not near 0 (0.01 rad =~ 0.5 deg tolerance).
        """
        signs = np.sign(np.array(start)*np.array(end))

        if np.all(signs > 0):
            return False  # = all OK

        joint_changes_small = True
        for i in range(len(signs)):

            if signs[i] < 0:
                if abs(start[i] < tolerance) or abs(end[i] < tolerance):
                    rospy.logdebug("Joint changes sign, but the change is small. Ignoring.")
                    rospy.logdebug("start[i] = %d6, end[i] = %d6", (start[i], end[i]))
                    continue
                rospy.logerr("Joint angle " + str(i) + " would change sign!")
                print("start[i] = %d6, end[i] = %d6", (start[i], end[i]))
                joint_changes_small = False
        if joint_changes_small:
            return False  # = all OK
        else:
            return True  # Joints change

    def get_current_pose_stamped(self):
        return self.robot_group.get_current_pose()

    def get_current_pose(self):
        return self.robot_group.get_current_pose().pose

    def save_plan(self, filename, plan):
        rp = rospkg.RosPack()
        bagfile = rp.get_path("o2ac_routines") + "/config/saved_plans/" + filename
        with rosbag.Bag(bagfile, 'w') as bag:
            bag.write(topic="saved_plan", msg=plan)

    def load_saved_plan(self, filename):
        rp = rospkg.RosPack()
        bagfile = rp.get_path("o2ac_routines") + "/config/saved_plans/" + filename
        with rosbag.Bag(bagfile, 'r') as bag:
            for (topic, plan, ts) in bag.read_messages():
                return plan

    def execute_saved_plan(self, filename="", plan=[], wait=True):
        if filename and not plan:
            plan = self.load_saved_plan(filename)
        return self.execute_plan(plan, wait)

    # ------ Robot motion functions

    def execute_plan(self, plan, wait=True):
        self.robot_group.execute(plan, wait=wait)
        self.robot_group.clear_pose_targets()
        if wait:
            current_joints = self.robot_group.get_current_joint_values()
            goal_joints = helpers.get_trajectory_joint_goal(plan, self.robot_group.get_active_joints())
            return helpers.all_close(goal_joints, current_joints, 0.01)
        return True

    def go_to_pose_goal(self, pose_goal_stamped, speed=0.5, acceleration=0.25,
                        end_effector_link="", move_lin=True, wait=True, plan_only=False, initial_joints=None,
                        allow_joint_configuration_flip=False, move_ptp=False):
        planner = "LINEAR" if move_lin else ("PTP" if move_ptp else "OMPL")
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
        start_time = rospy.Time.now()
        tries = 0
        while not success and (rospy.Time.now() - start_time < rospy.Duration(5)) and not rospy.is_shutdown():
            success, plan, planning_time, error = group.plan()

            if success:
                if self.joint_configuration_changes(plan.joint_trajectory.points[0].positions,
                                                    plan.joint_trajectory.points[-1].positions) \
                        and not allow_joint_configuration_flip:
                    success = False
                    rospy.logwarn("Joint configuration would have flipped.")
                    continue
            if success:
                if plan_only:
                    group.clear_pose_targets()
                    group.set_start_state_to_current_state()
                    return plan, planning_time
                else:
                    self.execute_plan(plan, wait=wait)
            else:
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

    def move_lin_trajectory(self, trajectory, speed=1.0, acceleration=0.5, end_effector_link="",
                            plan_only=False, initial_joints=None, allow_joint_configuration_flip=False, timeout=10):
        # TODO: Add allow_joint_configuration_flip
        if not self.set_up_move_group(speed, acceleration, planner="LINEAR"):
            return False

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"

        group = self.robot_group

        group.set_end_effector_link(end_effector_link)
        if len(trajectory[0]) == 2:  # Speed per point was not defined
            waypoints = [(self.listener.transformPose("world", ps), blend_radius, speed) for ps, blend_radius in trajectory]
        elif len(trajectory[0]) == 3:
            waypoints = [(self.listener.transformPose("world", ps), blend_radius, speed) for ps, blend_radius, speed in trajectory]

        motion_plan_requests = []

        # Start from current pose
        if initial_joints:
            initial_pose = self.compute_fk(initial_joints, end_effector_link)
            group.set_pose_target(initial_pose)
        else:
            group.set_pose_target(group.get_current_pose(end_effector_link))
        msi = moveit_msgs.msg.MotionSequenceItem()
        msi.req = group.construct_motion_plan_request()
        msi.blend_radius = 0.0

        if initial_joints:
            msi.req.start_state = helpers.to_robot_state(self.robot_group, initial_joints)
        else:
            msi.req.start_state = helpers.to_robot_state(self.robot_group, self.robot_group.get_current_joint_values())
        
        motion_plan_requests.append(msi)

        for wp, blend_radius, spd in waypoints:
            self.set_up_move_group(spd, spd/2.0, planner="LINEAR")
            group.clear_pose_targets()
            group.set_pose_target(wp)
            msi = moveit_msgs.msg.MotionSequenceItem()
            msi.req = group.construct_motion_plan_request()
            msi.req.start_state = moveit_msgs.msg.RobotState()
            msi.blend_radius = blend_radius
            motion_plan_requests.append(msi)

        # Force last point to be 0.0 to avoid raising an error in the planner
        motion_plan_requests[-1].blend_radius = 0.0

        # Make MotionSequence
        goal = moveit_msgs.msg.MoveGroupSequenceGoal()
        goal.request = moveit_msgs.msg.MotionSequenceRequest()
        goal.request.items = motion_plan_requests
        # Plan only always for compatibility with simultaneous motions
        goal.planning_options.plan_only = True

        start_time = rospy.Time.now()
        success = False
        while not success and (rospy.Time.now() - start_time < rospy.Duration(timeout)) and not rospy.is_shutdown():

            self.sequence_move_group.send_goal_and_wait(goal)
            response = self.sequence_move_group.get_result()
            
            group.clear_pose_targets()

            if response.response.error_code.val == 1:
                plan = response.response.planned_trajectories[0]  # support only one plan?
                planning_time = response.response.planning_time
                if plan_only:
                    return plan, planning_time
                else:
                    return self.execute_plan(plan, wait=True)
            else:
                rospy.sleep(0.2)
        rospy.logerr("Failed to plan linear trajectory. error code: %s" % response.response.error_code.val)
        return False

    def move_lin(self, pose_goal_stamped, speed=0.5, acceleration=0.5, end_effector_link="", wait=True,
                 plan_only=False, initial_joints=None, allow_joint_configuration_flip=False):
        return self.go_to_pose_goal(pose_goal_stamped, speed, acceleration, end_effector_link, move_lin=True,
                                    wait=wait, plan_only=plan_only, initial_joints=initial_joints,
                                    allow_joint_configuration_flip=allow_joint_configuration_flip)

    def move_lin_rel(self, relative_translation=[0, 0, 0], relative_rotation=[0, 0, 0], speed=.5,
                     acceleration=0.2, relative_to_robot_base=False, relative_to_tcp=False,
                     wait=True, end_effector_link="", plan_only=False, initial_joints=None,
                     allow_joint_configuration_flip=False, pose_only=False):
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
            b2tcp = self.compute_fk(initial_joints, tcp_link=end_effector_link, frame_id=self.ns + "_base_link")  # forward kinematics
            t_b2tcp = conversions.from_pose(b2tcp.pose)  # transform tcp to robot's base
            if relative_to_tcp:
                new_pose = conversions.to_pose_stamped(end_effector_link, [0, 0, 0, 0, 0, 0.])
            elif relative_to_robot_base:
                new_pose = self.compute_fk(initial_joints, tcp_link=end_effector_link, frame_id=self.ns + "_base_link")
            else:
                t_w2tcp = transformations.concatenate_matrices(t_w2b, t_b2tcp)
                new_pose = conversions.to_pose_stamped("world", transformations.pose_quaternion_from_matrix(t_w2tcp))
        else:
            new_pose = group.get_current_pose()

            if relative_to_robot_base:
                new_pose = self.listener.transformPose(self.ns + "_base_link", new_pose)
            elif relative_to_tcp:
                new_pose.header.stamp = rospy.Time.now()
                # Workaround for TF lookup into the future error
                self.listener.waitForTransform(self.ns + "_gripper_tip_link", new_pose.header.frame_id, new_pose.header.stamp, rospy.Duration(1))
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

        if pose_only:
            return new_pose
        else:
            return self.go_to_pose_goal(new_pose, speed=speed, acceleration=acceleration,
                                        end_effector_link=end_effector_link,  wait=wait,
                                        move_lin=True, plan_only=plan_only, initial_joints=initial_joints,
                                        allow_joint_configuration_flip=allow_joint_configuration_flip)

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

        success, plan, planning_time, error = group.plan()
        if success:
            group.clear_pose_targets()
            group.set_start_state_to_current_state()
            if plan_only:
                return plan, planning_time
            else:
                return self.execute_plan(plan, wait=wait)
        else:
            rospy.logerr("Failed planning with error: %s" % error)
            return False

    def move_joints(self, joint_pose_goal, speed=0.6, acceleration=0.3, wait=True, plan_only=False, initial_joints=None):
        if not self.set_up_move_group(speed, acceleration, planner="OMPL"):
            return False
        group = self.robot_group

        if initial_joints:
            group.set_start_state(helpers.to_robot_state(group, initial_joints))

        group.set_joint_value_target(joint_pose_goal)

        success, plan, planning_time, error = group.plan()
        if success:
            group.set_start_state_to_current_state()
            if plan_only:
                return plan, planning_time
            else:
                return self.execute_plan(plan, wait=wait)
        else:
            rospy.logerr("Failed planning with error: %s" % error)
            return False

    def move_joints_trajectory(self, trajectory, speed=1.0, acceleration=0.5, plan_only=False, initial_joints=None, timeout=5.0):
        if not self.set_up_move_group(speed, acceleration, planner="PTP"):
            return False

        group = self.robot_group

        waypoints = []
        for point, blend_radius, speed in trajectory:
            if isinstance(point, str):
                joint_values = helpers.ordered_joint_values_from_dict(group.get_named_target_values(point), group.get_active_joints())
            else:
                joint_values = point
            waypoints.append((joint_values, blend_radius, speed))

        group.set_joint_value_target(initial_joints if initial_joints else group.get_current_joint_values())
        # Start from current pose
        msi = moveit_msgs.msg.MotionSequenceItem()
        msi.req = group.construct_motion_plan_request()
        msi.blend_radius = 0.0
        msi.req.start_state = helpers.to_robot_state(group, initial_joints if initial_joints else group.get_current_joint_values())
        
        motion_plan_requests = []
        motion_plan_requests.append(msi)

        for wp, blend_radius, spd in waypoints:
            self.set_up_move_group(spd, spd/2.0, planner="PTP")
            group.clear_pose_targets()
            group.set_joint_value_target(wp)
            msi = moveit_msgs.msg.MotionSequenceItem()
            msi.req = group.construct_motion_plan_request()
            msi.req.start_state = moveit_msgs.msg.RobotState()
            msi.blend_radius = blend_radius
            motion_plan_requests.append(msi)

        # Force last point to be 0.0 to avoid raising an error in the planner
        motion_plan_requests[-1].blend_radius = 0.0

        # Make MotionSequence
        goal = moveit_msgs.msg.MoveGroupSequenceGoal()
        goal.request = moveit_msgs.msg.MotionSequenceRequest()
        goal.request.items = motion_plan_requests
        # Plan only always for compatibility with simultaneous motions
        goal.planning_options.plan_only = True

        start_time = rospy.Time.now()
        success = False
        while not success and (rospy.Time.now() - start_time < rospy.Duration(timeout)) and not rospy.is_shutdown():

            self.sequence_move_group.send_goal_and_wait(goal)
            response = self.sequence_move_group.get_result()
            
            group.clear_pose_targets()

            if response.response.error_code.val == 1:
                plan = response.response.planned_trajectories[0]  # support only one plan?
                planning_time = response.response.planning_time
                if plan_only:
                    return plan, planning_time
                else:
                    return self.execute_plan(plan, wait=True)
        rospy.logerr("Failed to plan joint trajectory. error code: %s" % response.response.error_code.val)
        return False

    def move_circ(self, pose_goal_stamped, constraint_point, constraint_type="center", speed=0.5, acceleration=0.25, wait=True, end_effector_link="",
                    plan_only=False, initial_joints=None, timeout=5.0):
        if not self.set_up_move_group(speed, acceleration, "CIRC"):
            return False

        group = self.robot_group
        group.clear_pose_targets()

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"
        group.set_end_effector_link(end_effector_link)

        if initial_joints:
            group.set_start_state(helpers.to_robot_state(group, initial_joints))

        pose_goal_world = self.listener.transformPose("world", pose_goal_stamped)
        group.set_pose_target(pose_goal_world)

        constraint = moveit_msgs.msg.Constraints()
        if constraint_type not in ("center", "interim"):
            rospy.logerr("Invalid parameter: %s" % constraint_type)
            return False
        constraint.name = constraint_type
        pc = moveit_msgs.msg.PositionConstraint()
        if constraint_type == "center":
            constraint_pose = conversions.from_pose_to_list(self.get_current_pose())[:3] - constraint_point
            constraint_pose = conversions.to_pose(constraint_pose.tolist()+[0,0,0])
        else:
            constraint_pose = conversions.to_pose(constraint_point+[0,0,0]) # Pose
        pc.constraint_region.primitive_poses = [constraint_pose]
        constraint.position_constraints = [pc] 
        group.set_path_constraints(constraint)

        success = False
        start_time = rospy.Time.now()
        while not success and (rospy.Time.now() - start_time < rospy.Duration(timeout)) and not rospy.is_shutdown():
            success, plan, planning_time, error = group.plan()

            if success:
                if plan_only:
                    group.clear_pose_targets()
                    group.set_start_state_to_current_state()
                    return plan, planning_time
                else:
                    self.execute_plan(plan, wait=wait)
            else:
                rospy.sleep(0.2)
                rospy.logwarn("go_to_pose_goal attempt failed. Retrying.")

        group.clear_pose_targets()
        return success
