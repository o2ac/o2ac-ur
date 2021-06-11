import actionlib
from actionlib_msgs.msg import GoalStatus

import moveit_commander
import rospy
import time
import copy

import o2ac_msgs.msg
import controller_manager_msgs.msg
import std_srvs.srv
import ur_dashboard_msgs.srv
import ur_msgs.srv
import moveit_msgs.msg
import moveit_msgs.srv

from std_msgs.msg import Bool

from o2ac_routines.ur_force_control import URForceController
from o2ac_routines.robotiq_gripper import RobotiqGripper
from o2ac_routines import helpers

from ur_control import conversions, transformations
from ur_pykdl import ur_kinematics


class URRobot():
    def __init__(self, namespace, tf_listener):
        """
        namespace should be "a_bot" or "b_bot".
        use_real_robot is a boolean
        """
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.ns = namespace
        self.listener = tf_listener
        self.marker_counter = 0

        # forward kinematics helper
        self.kdl = ur_kinematics(namespace, base_link=self.ns+"_base_link", ee_link=self.ns+"_gripper_tip_link", prefix=namespace+"_", rospackage='o2ac_scene_description',)
        try:
            self.force_controller = URForceController(robot_name=namespace, listener=tf_listener)
        except rospy.ROSException as e:
            rospy.logwarn("No force control capabilities since controller could not be instantiated" + str(e))

        self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
        self.pause_mode_ = False
        self.test_mode_ = False

        self.robot_group = moveit_commander.MoveGroupCommander(self.ns)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.ns+"_robotiq_85")
        self.sequence_move_group = actionlib.SimpleActionClient("/sequence_move_group", moveit_msgs.msg.MoveGroupSequenceAction)
        self.plan_sequence_path = rospy.ServiceProxy('plan_sequence_path', moveit_msgs.srv.GetMotionSequence)

        self.ur_dashboard_clients = {
            "get_loaded_program": rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/get_loaded_program' % self.ns, ur_dashboard_msgs.srv.GetLoadedProgram),
            "program_running":    rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/program_running' % self.ns, ur_dashboard_msgs.srv.IsProgramRunning),
            "load_program":       rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/load_program' % self.ns, ur_dashboard_msgs.srv.Load),
            "play":               rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/play' % self.ns, std_srvs.srv.Trigger),
            "stop":               rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/stop' % self.ns, std_srvs.srv.Trigger),
            "quit":               rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/quit' % self.ns, std_srvs.srv.Trigger),
            "connect":            rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/connect' % self.ns, std_srvs.srv.Trigger),
            "close_popup":        rospy.ServiceProxy('/%s/ur_hardware_interface/dashboard/close_popup' % self.ns, std_srvs.srv.Trigger),
            "unlock_protective_stop": rospy.ServiceProxy("/%s/ur_hardware_interface/dashboard/unlock_protective_stop" % self.ns, std_srvs.srv.Trigger),
        }

        self.set_io = rospy.ServiceProxy('/%s/ur_hardware_interface/set_io' % self.ns, ur_msgs.srv.SetIO)

        self.sub_status_ = rospy.Subscriber("/%s/ur_hardware_interface/robot_program_running" % self.ns, Bool, self.ros_control_status_callback)
        self.service_proxy_list = rospy.ServiceProxy("/" + self.ns + "/controller_manager/list_controllers", controller_manager_msgs.srv.ListControllers)
        self.service_proxy_switch = rospy.ServiceProxy("/" + self.ns + "/controller_manager/switch_controller", controller_manager_msgs.srv.SwitchController)

        self.sub_robot_safety_mode = rospy.Subscriber("/%s/ur_hardware_interface/safety_mode" % self.ns, ur_dashboard_msgs.msg.SafetyMode, self.safety_mode_callback)
        self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
        self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
        self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)

        self.ur_ros_control_running_on_robot = False
        self.robot_safety_mode = None
        self.robot_status = dict()

        self.gripper = RobotiqGripper(namespace=self.ns, gripper_group=self.gripper_group)

    def run_mode_callback(self, msg):
        self.run_mode_ = msg.data

    def pause_mode_callback(self, msg):
        self.pause_mode_ = msg.data

    def test_mode_callback(self, msg):
        self.test_mode_ = msg.data

    def safety_mode_callback(self, msg):
        self.robot_safety_mode = msg.mode

    def ros_control_status_callback(self, msg):
        self.ur_ros_control_running_on_robot = msg.data

    def is_running_normally(self):
        """
        Returns true if the robot is running (no protective stop, not turned off etc).
        """
        return self.robot_safety_mode == 1 or self.robot_safety_mode == 2  # Normal / Reduced

    def is_protective_stopped(self):
        """
        Returns true if the robot is in protective stop.
        """
        return self.robot_safety_mode == 3

    def unlock_protective_stop(self):
        if not self.use_real_robot:
            return True

        service_client = self.ur_dashboard_clients["unlock_protective_stop"]
        request = std_srvs.srv.TriggerRequest()
        start_time = time.time()
        rospy.loginfo("Attempting to unlock protective stop of " + self.ns)
        while not rospy.is_shutdown():
            response = service_client.call(request)
            if time.time() - start_time > 5.0:
                break
            if response.success:
                break
            rospy.sleep(0.2)
        if not response.success:
            rospy.logwarn("Could not unlock protective stop of " + self.ns + "!")
        return response.success

    def get_status_from_param_server(self):
        self.robot_status = o2ac_msgs.msg.RobotStatus()
        self.robot_status.carrying_object = rospy.get_param(self.ns + "/carrying_object", False)
        self.robot_status.carrying_tool = rospy.get_param(self.ns + "/carrying_tool", False)
        self.robot_status.held_tool_id = rospy.get_param(self.ns + "/held_tool_id", "")
        return self.robot_status

    def publish_robot_status(self):
        rospy.set_param(self.ns + "/carrying_object", self.robot_status.carrying_object)
        rospy.set_param(self.ns + "/carrying_tool",   self.robot_status.carrying_tool)
        rospy.set_param(self.ns + "/held_tool_id",    self.robot_status.held_tool_id)

    @helpers.check_for_real_robot
    def wait_for_control_status_to_turn_on(self, waittime):
        start = rospy.Time.now()
        elapsed = rospy.Time.now() - start
        while not self.ur_ros_control_running_on_robot and elapsed < rospy.Duration(waittime) and not rospy.is_shutdown():
            rospy.sleep(.1)
            elapsed = rospy.Time.now() - start
            if self.ur_ros_control_running_on_robot:
                return True
        return False

    @helpers.check_for_real_robot
    def activate_ros_control_on_ur(self, recursion_depth=0):
        if not self.use_real_robot:
            return True

        # Check if URCap is already running on UR
        try:
            if self.ur_ros_control_running_on_robot:
                return True
            else:
                rospy.loginfo("robot_program_running not true for " + self.ns)
        except:
            rospy.logerr("Robot name '" + self.ns + "' was not found or the robot is not a UR!")
            return False

        if recursion_depth > 10:
            rospy.logerr("Tried too often. Breaking out.")
            rospy.logerr("Could not start UR ROS control.")
            raise Exception("Could not activate ROS control on robot " + self.ns + ". Breaking out. Is the UR in Remote Control mode and program installed with correct name?")

        if rospy.is_shutdown():
            return False

        program_loaded = False
        try:
            # Load program if it not loaded already
            response = self.ur_dashboard_clients["get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
            # print("response:")
            # print(response)
            if response.program_name == "/programs/ROS_external_control.urp":
                program_loaded = True
            else:
                rospy.loginfo("Currently loaded program was:  " + response.program_name)
                rospy.loginfo("Loading ROS control on robot " + self.ns)
                request = ur_dashboard_msgs.srv.LoadRequest()
                request.filename = "ROS_external_control.urp"
                response = self.ur_dashboard_clients["load_program"].call(request)
                if response.success:  # Try reconnecting to dashboard
                    program_loaded = True
                else:
                    rospy.logerr("Could not load the ROS_external_control.urp URCap. Is the UR in Remote Control mode and program installed with correct name?")
                for i in range(10):
                    rospy.sleep(0.2)
                    # rospy.loginfo("After-load check nr. " + str(i))
                    response = self.ur_dashboard_clients["get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
                    # rospy.loginfo("Received response: " + response.program_name)
                    if response.program_name == "/programs/ROS_external_control.urp":
                        break

        except:
            rospy.logwarn("Dashboard service did not respond!")

        if not program_loaded:
            rospy.logwarn("Could not load.")
            try:
                if recursion_depth > 3:  # If connect alone failed, try quit and then connect
                    rospy.logwarn("Try to quit before connecting.")
                    response = self.ur_dashboard_clients["quit"].call()
                    rospy.sleep(3.0)
                rospy.logwarn("Try to connect to dashboard service.")
                response = self.ur_dashboard_clients["connect"].call()
            except:
                rospy.logwarn("Dashboard service did not respond! (2)")
                pass
            rospy.sleep(1.0)
            return self.activate_ros_control_on_ur(recursion_depth=recursion_depth+1)

        # Run the program
        rospy.loginfo("Running the program (play)")
        try:
            response = self.ur_dashboard_clients["play"].call(std_srvs.srv.TriggerRequest())
        except:
            pass
        rospy.loginfo("Enter wait_for_control_status_to_turn_on")
        self.wait_for_control_status_to_turn_on(2.0)
        rospy.loginfo("Exited wait_for_control_status_to_turn_on")

        if self.check_for_dead_controller_and_force_start():
            rospy.loginfo("Successfully activated ROS control on robot " + self.ns)
            return True
        else:
            # Try stopping and restarting the program to restart the controllers
            try:
                rospy.logwarn("Trying to restart URCap program on UR to restart controllers on ROS side")
                response = self.ur_dashboard_clients["stop"].call()
                rospy.sleep(2.0)
                response = self.ur_dashboard_clients["play"].call()
                if self.wait_for_control_status_to_turn_on(2.0):
                    return True
            except:
                rospy.logerr("Failed to quit/restart")
                pass
            return self.activate_ros_control_on_ur(recursion_depth=recursion_depth+1)

    @helpers.check_for_real_robot
    def check_for_dead_controller_and_force_start(self):
        list_req = controller_manager_msgs.srv.ListControllersRequest()
        switch_req = controller_manager_msgs.srv.SwitchControllerRequest()
        rospy.loginfo("Checking for dead controllers for robot " + self.ns)
        list_res = self.service_proxy_list.call(list_req)
        for c in list_res.controller:
            if c.name == "scaled_pos_joint_traj_controller":
                if c.state == "stopped":
                    # Force restart
                    rospy.logwarn("Force restart of controller")
                    switch_req.start_controllers = ['scaled_pos_joint_traj_controller']
                    switch_req.strictness = 1
                    switch_res = self.service_proxy_switch.call(switch_req)
                    rospy.sleep(1)
                    return switch_res.ok
                else:
                    rospy.loginfo("Controller state is " + c.state + ", returning True.")
                    return True

    @helpers.check_for_real_robot
    def load_and_execute_program(self, program_name="", recursion_depth=0):
        if not self.load_program(program_name, recursion_depth):
            return False
        return self.execute_loaded_program()

    @helpers.check_for_real_robot
    def load_program(self, program_name="", recursion_depth=0):
        if not self.use_real_robot:
            return True

        if recursion_depth > 6:
            rospy.logerr("Tried too often. Breaking out.")
            rospy.logerr("Could not load " + program_name + ". Is the UR in Remote Control mode and program installed with correct name?")
            return False

        load_success = False
        try:
            # Try to stop running program
            self.ur_dashboard_clients["stop"].call(std_srvs.srv.TriggerRequest())
            rospy.sleep(.5)

            # Load program if it not loaded already
            request = ur_dashboard_msgs.srv.LoadRequest()
            request.filename = program_name
            response = self.ur_dashboard_clients["load_program"].call(request)
            if response.success:  # Try reconnecting to dashboard
                load_success = True
                return True
            else:
                rospy.logerr("Could not load " + program_name + ". Is the UR in Remote Control mode and program installed with correct name?")
        except:
            rospy.logwarn("Dashboard service did not respond to load_program!")
        if not load_success:
            rospy.logwarn("Waiting and trying again")
            rospy.sleep(3)
            try:
                if recursion_depth > 0:  # If connect alone failed, try quit and then connect
                    response = self.ur_dashboard_clients["quit"].call()
                    rospy.logerr("Program could not be loaded on UR: " + program_name)
                    rospy.sleep(.5)
            except:
                rospy.logwarn("Dashboard service did not respond to quit! ")
                pass
            response = self.ur_dashboard_clients["connect"].call()
            rospy.sleep(.5)
            return self.load_program(program_name=program_name, recursion_depth=recursion_depth+1)

    @helpers.check_for_real_robot
    def execute_loaded_program(self):
        # Run the program
        response = self.ur_dashboard_clients["play"].call(std_srvs.srv.TriggerRequest())
        if not response.success:
            rospy.logerr("Could not start program. Is the UR in Remote Control mode and program installed with correct name?")
            return False
        else:
            rospy.loginfo("Successfully started program on robot " + self.ns)
            return True

    @helpers.check_for_real_robot
    def close_ur_popup(self):
        # Close a popup on the teach pendant to continue program execution
        response = self.ur_dashboard_clients["close_popup"].call(std_srvs.srv.TriggerRequest())
        if not response.success:
            rospy.logerr("Could not close popup.")
            return False
        else:
            rospy.loginfo("Successfully closed popup on teach pendant of robot " + self.ns)
            return True

    def get_tcp_pose(self, joints, end_effector_link=None):
        """ Get TCP position with respect to robot's base frame
        """
        return conversions.to_pose_stamped(self.ns + "_base_link", self.kdl.forward(joints, end_effector_link))

    # ------ Robot motion functions

    def get_current_pose_stamped(self, group_name=None):
        return self.robot_group.get_current_pose()

    def get_current_pose(self, group_name=None):
        return self.robot_group.get_current_pose().pose

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
                    plan = response.response.planned_trajectories[0] # support only one plan?
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

    def move_lin_rel(self, relative_translation=[0, 0, 0],relative_rotation=[0, 0, 0], speed=.5, 
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

    def set_up_move_group(self, speed, acceleration, planner="OMPL"):
        assert not rospy.is_shutdown()
        (speed_, accel_) = self.limit_speed_and_acc(speed, acceleration)
        self.activate_ros_control_on_ur()
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

    # ------ Force control functions

    def force_control(self, *args, **kwargs):
        self.activate_ros_control_on_ur()
        return self.force_controller.force_control(*args, **kwargs)

    def execute_circular_trajectory(self, *args, **kwargs):
        self.activate_ros_control_on_ur()
        return self.force_controller.execute_circular_trajectory(*args, **kwargs)

    def execute_spiral_trajectory(self, *args, **kwargs):
        self.activate_ros_control_on_ur()
        return self.force_controller.execute_spiral_trajectory(*args, **kwargs)

    def linear_push(self, *args, **kwargs):
        self.activate_ros_control_on_ur()
        return self.force_controller.linear_push(*args, **kwargs)
