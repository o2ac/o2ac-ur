import actionlib
import moveit_commander
import rospy
import time
import copy

import o2ac_msgs.msg
import controller_manager_msgs.msg
import std_srvs.srv
import ur_dashboard_msgs.srv
import ur_msgs.srv

from std_msgs.msg import Bool

from o2ac_routines.ur_force_control import URForceController
from o2ac_routines.robotiq_gripper import RobotiqGripper
from o2ac_routines import helpers


class URRobot():
    def __init__(self, namespace, use_real_robot, tf_listener):
        """
        namespace should be "a_bot" or "b_bot".
        use_real_robot is a boolean
        """
        self.use_real_robot = use_real_robot
        self.ns = namespace
        self.listener = tf_listener

        try:
            self.force_controller = URForceController(robot_name=namespace)
        except rospy.ROSException as e:
            rospy.logwarn("No force control capabilities since controller could not be instantiated" + str(e))

        self.run_mode_ = True     # The modes limit the maximum speed of motions. Used with the safety system @WRS2020
        self.pause_mode_ = False
        self.test_mode_ = False

        self.robot_group = moveit_commander.MoveGroupCommander(self.ns)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.ns+"_robotiq_85")

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

        self.sub_robot_safety_mode = rospy.Subscriber("/%s/ur_hardware_interface/safety_mode" % self.ns, ur_dashboard_msgs.msg.SafetyMode, self.safety_mode_callback)
        self.sub_run_mode_ = rospy.Subscriber("/run_mode", Bool, self.run_mode_callback)
        self.sub_pause_mode_ = rospy.Subscriber("/pause_mode", Bool, self.pause_mode_callback)
        self.sub_test_mode_ = rospy.Subscriber("/test_mode", Bool, self.test_mode_callback)

        self.ur_ros_control_running_on_robot = False
        self.robot_safety_mode = None
        self.robot_status = dict()

        self.gripper = RobotiqGripper(namespace=self.ns, use_real_robot=use_real_robot)

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
            raise Exception("Could not activate ROS control on robot " + self.ns + ". Breaking out.")

        if rospy.is_shutdown():
            return False

        load_success = False
        try:
            # Load program if it not loaded already
            rospy.loginfo("Activating ROS control on robot " + self.ns)
            response = self.ur_dashboard_clients["get_loaded_program"].call(ur_dashboard_msgs.srv.GetLoadedProgramRequest())
            if response.program_name != "Loaded program: /programs/ROS_external_control.urp":
                request = ur_dashboard_msgs.srv.LoadRequest()
                request.filename = "ROS_external_control.urp"
                response = self.ur_dashboard_clients["load_program"].call(request)
                rospy.sleep(2.0)
                if response.success:  # Try reconnecting to dashboard
                    load_success = True
                else:
                    rospy.logerr("Could not load the ROS_external_control.urp URCap. Is the UR in Remote Control mode and program installed with correct name?")
        except:
            rospy.logwarn("Dashboard service did not respond!")

        if not load_success:
            rospy.logwarn("Waiting and trying again.")
            try:
                if recursion_depth > 0:  # If connect alone failed, try quit and then connect
                    response = self.ur_dashboard_clients["quit"].call()
                    rospy.sleep(.5)
                response = self.ur_dashboard_clients["connect"].call()
            except:
                rospy.logwarn("Dashboard service did not respond! (2)")
                pass
            rospy.sleep(3.0)
            return self.activate_ros_control_on_ur(recursion_depth=recursion_depth+1)

        # Run the program
        response = self.ur_dashboard_clients["play"].call(std_srvs.srv.TriggerRequest())
        rospy.sleep(2)
        if not response.success:
            rospy.logerr("Could not start UR control. Is the UR in Remote Control mode and program installed with correct name?")
            return False
        else:
            # Check if controller is running
            self.check_for_dead_controller_and_force_start()
            rospy.loginfo("Successfully activated ROS control on robot " + self.ns)
            return True

    def check_for_dead_controller_and_force_start(self):
        service_proxy_list = rospy.ServiceProxy("/" + self.ns + "/controller_manager/list_controllers", controller_manager_msgs.srv.ListControllers)
        service_proxy_switch = rospy.ServiceProxy("/" + self.ns + "/controller_manager/switch_controller", controller_manager_msgs.srv.SwitchController)
        rospy.sleep(2)

        list_req = controller_manager_msgs.srv.ListControllersRequest()
        switch_req = controller_manager_msgs.srv.SwitchControllerRequest()

        list_res = service_proxy_list.call(list_req)
        for c in list_res.controller:
            if c.name == "scaled_pos_joint_traj_controller":
                if c.state == "stopped":
                    # Force restart
                    rospy.logerr("Force restart of controller")
                    switch_req.start_controllers = ['scaled_pos_joint_traj_controller']
                    switch_req.strictness = 1
                    switch_res = service_proxy_switch.call(switch_req)
                    rospy.sleep(1)
                    return switch_res.ok
                else:
                    return True

    def load_and_execute_program(self, program_name="", recursion_depth=0):
        if not self.load_program(program_name, recursion_depth):
            return False
        return self.execute_loaded_program()

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

    def execute_loaded_program(self):
        # Run the program
        response = self.ur_dashboard_clients["play"].call(std_srvs.srv.TriggerRequest())
        if not response.success:
            rospy.logerr("Could not start program. Is the UR in Remote Control mode and program installed with correct name?")
            return False
        else:
            rospy.loginfo("Successfully started program on robot " + self.ns)
            return True

    def close_ur_popup(self):
        # Close a popup on the teach pendant to continue program execution
        response = self.ur_dashboard_clients["close_popup"].call(std_srvs.srv.TriggerRequest())
        if not response.success:
            rospy.logerr("Could not close popup.")
            return False
        else:
            rospy.loginfo("Successfully closed popup on teach pendant of robot " + self.ns)
            return True

    # ------ Robot motion functions

    def get_current_pose_stamped(self, group_name=None):
        return self.robot_group.get_current_pose()

    def get_current_pose(self, group_name=None):
        return self.robot_group.get_current_pose().pose

    def go_to_pose_goal(self, pose_goal_stamped, speed=0.5, acceleration=0.25,
                        end_effector_link="", move_lin=True):
        if move_lin:
            return self.move_lin(pose_goal_stamped, speed, acceleration, end_effector_link)
        if not self.set_up_move_group(speed, acceleration):
            return False

        group = self.robot_group

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"
        group.set_end_effector_link(end_effector_link)

        group.set_pose_target(pose_goal_stamped)
        rospy.logdebug("Setting velocity scaling to " + str(speed))
        group.set_max_velocity_scaling_factor(speed)
        group.set_max_acceleration_scaling_factor(acceleration)
        group.set_planning_pipeline_id("ompl")
        group.set_planner_id("RRTConnect")

        move_success = group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        return helpers.all_close(pose_goal_stamped.pose, current_pose, 0.01), move_success

    def move_lin(self, pose_goal_stamped, speed=1.0, acceleration=0.5, end_effector_link=""):
        if not self.set_up_move_group(speed, acceleration):
            return False

        if not end_effector_link:
            end_effector_link = self.ns + "_gripper_tip_link"

        group = self.robot_group

        group.set_end_effector_link(end_effector_link)
        pose_goal_world = self.listener.transformPose("world", pose_goal_stamped)
        group.set_pose_target(pose_goal_world)

        group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        group.set_planner_id("LIN")

        move_success = group.go(wait=True)  # Bool
        group.stop()
        group.clear_pose_targets()
        return move_success

    def move_lin_rel(self, relative_translation=[0, 0, 0], relative_rotation=[0, 0, 0], speed=.03, acceleration=0.5, use_robot_base_csys=False, wait=True, max_wait=30.0):
        '''
        Does a lin_move relative to the current position of the robot.

        relative_translation: translatory movement relative to current tcp position, expressed in robot's own base frame
        relative_rotation: rotatory movement relative to current tcp position, expressed in robot's own base frame
        use_robot_base_csys: If true, uses the robot_base coordinates for the relative motion (not workspace_center!)
        '''
        if rospy.is_shutdown():
            return False

        group = self.robot_group
        # TODO: use use_robot_base_csys parameter
        new_pose1 = group.get_current_pose()
        new_pose2 = group.get_current_pose()
        if helpers.pose_dist(new_pose1.pose, new_pose2.pose) > 0.002:
            # This is guarding against a weird error that seems to occur with get_current_pose sometimes
            rospy.logerr("get_current_pose gave two different results!!")
            rospy.logwarn("pose1: ")
            rospy.logwarn(new_pose1.pose)
            rospy.logwarn("pose2: ")
            rospy.logwarn(new_pose2.pose)

        new_pose2.pose.position.x += relative_translation[0]
        new_pose2.pose.position.y += relative_translation[1]
        new_pose2.pose.position.z += relative_translation[2]
        new_pose2.pose.orientation = helpers.rotateQuaternionByRPY(relative_rotation[0], relative_rotation[1],
                                                           relative_rotation[2], new_pose2.pose.orientation)
        return self.move_lin(new_pose2, speed=speed, acceleration=acceleration)

    def move_joints(self, joint_pose_goal, speed=1.0, acceleration=0.5):
        if not self.set_up_move_group(speed, acceleration):
            return False
        group = self.robot_group
        group.set_joint_value_target(joint_pose_goal)
        group.set_planning_pipeline_id("ompl")
        group.set_planner_id("RRTConnect")
        return group.go(wait=True)

    def go_to_named_pose(self, pose_name, speed=0.5, acceleration=0.5):
        """
        pose_name should be a named pose in the moveit_config, such as "home", "back" etc.
        """
        if not self.set_up_move_group(speed, acceleration):
            return False
        group = self.robot_group
        group.set_named_target(pose_name)
        group.set_planning_pipeline_id("ompl")
        group.set_planner_id("RRTConnect")
        move_success = group.go(wait=True)
        group.clear_pose_targets()
        return move_success

    def set_up_move_group(self, speed, acceleration):
        if rospy.is_shutdown():
            return False
        (speed_, accel_) = self.limit_speed_and_acc(speed, acceleration)
        self.activate_ros_control_on_ur()
        group = self.robot_group
        rospy.logdebug("Setting velocity scaling to " + str(speed_))
        rospy.logdebug("Setting acceleration scaling to " + str(accel_))
        group.set_max_velocity_scaling_factor(speed_)
        group.set_max_acceleration_scaling_factor(accel_)
        return True

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
            rospy.logwarn("Setting acceleration to " + str(speed) + " instead of " + str(acceleration) + " to avoid jerky motion.")
            acc = sp/2.0
        return (sp, acc)

    # ------ Force control functions

    def force_control(self, *args, **kwargs):
        return self.force_controller.force_control(*args, **kwargs)

    def execute_circular_trajectory(self, *args, **kwargs):
        return self.force_controller.execute_circular_trajectory(*args, **kwargs)

    def execute_spiral_trajectory(self, *args, **kwargs):
        return self.force_controller.execute_spiral_trajectory(*args, **kwargs)

    def linear_push(self, *args, **kwargs):
        return self.force_controller(*args, **kwargs)
