import moveit_commander
from o2ac_routines.robot_base import RobotBase
import rospy
import time

import o2ac_msgs.msg
import controller_manager_msgs.msg
import std_srvs.srv
import ur_dashboard_msgs.srv
import ur_msgs.srv

from std_msgs.msg import Bool

from o2ac_routines.ur_force_control import URForceController
from o2ac_routines.robotiq_gripper import RobotiqGripper
from o2ac_routines import helpers

from ur_control import conversions
from ur_pykdl import ur_kinematics

from trac_ik_python.trac_ik import IK


class URRobot(RobotBase):
    def __init__(self, namespace, tf_listener):
        """
        namespace should be "a_bot" or "b_bot".
        use_real_robot is a boolean
        """
        RobotBase.__init__(self, group_name=namespace, tf_listener=tf_listener)

        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.ns = namespace
        self.marker_counter = 0

        # forward kinematics helper
        self.kdl = ur_kinematics(namespace, base_link=self.ns+"_base_link", ee_link=self.ns+"_gripper_tip_link", prefix=namespace+"_", rospackage='o2ac_scene_description',)
        # IK solver
        self.ik_solver = IK(base_link=self.ns+"_base_link", tip_link=self.ns+"_gripper_tip_link", solve_type="Distance", timeout=0.01)

        try:
            self.force_controller = URForceController(robot_name=namespace, listener=tf_listener)
        except rospy.ROSException as e:
            rospy.logwarn("No force control capabilities since controller could not be instantiated" + str(e))

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

        self.speed_slider = rospy.ServiceProxy("/%s/ur_hardware_interface/set_speed_slider" % self.ns, ur_msgs.srv.SetSpeedSliderFraction)

        self.set_io = rospy.ServiceProxy('/%s/ur_hardware_interface/set_io' % self.ns, ur_msgs.srv.SetIO)

        self.sub_status_ = rospy.Subscriber("/%s/ur_hardware_interface/robot_program_running" % self.ns, Bool, self.ros_control_status_callback)
        self.service_proxy_list = rospy.ServiceProxy("/" + self.ns + "/controller_manager/list_controllers", controller_manager_msgs.srv.ListControllers)
        self.service_proxy_switch = rospy.ServiceProxy("/" + self.ns + "/controller_manager/switch_controller", controller_manager_msgs.srv.SwitchController)

        self.sub_robot_safety_mode = rospy.Subscriber("/%s/ur_hardware_interface/safety_mode" % self.ns, ur_dashboard_msgs.msg.SafetyMode, self.safety_mode_callback)

        self.ur_ros_control_running_on_robot = False
        self.robot_safety_mode = None
        self.robot_status = dict()

        self.gripper = RobotiqGripper(namespace=self.ns, gripper_group=self.gripper_group)

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
                self.speed_slider(ur_msgs.srv.SetSpeedSliderFractionRequest(speed_slider_fraction=1.0))
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

        # Try to connect to dashboard if first try failed
        try:
            if recursion_depth > 2:
                if recursion_depth > 3:
                    rospy.logwarn("Try to quit before connecting.")
                    response = self.ur_dashboard_clients["quit"].call()
                    rospy.sleep(3.0)
                rospy.logwarn("Try to connect to dashboard service.")
                response = self.ur_dashboard_clients["connect"].call()
                rospy.sleep(1.0)
        except:
            rospy.logwarn("Dashboard service did not respond! (2)")
            pass

        if not program_loaded:
            rospy.logwarn("Could not load.")
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
            self.speed_slider(ur_msgs.srv.SetSpeedSliderFractionRequest(speed_slider_fraction=1.0))
            return True
        else:
            # Try stopping and restarting the program to restart the controllers
            try:
                rospy.logwarn("Trying to restart URCap program on UR to restart controllers on ROS side")
                response = self.ur_dashboard_clients["stop"].call()
                rospy.sleep(2.0)
                response = self.ur_dashboard_clients["play"].call()
                if self.wait_for_control_status_to_turn_on(2.0):
                    self.speed_slider(ur_msgs.srv.SetSpeedSliderFractionRequest(speed_slider_fraction=1.0))
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

    def set_up_move_group(self, speed, acceleration, planner="OMPL"):
        self.activate_ros_control_on_ur()
        return RobotBase.set_up_move_group(self, speed, acceleration, planner)

    def solve_ik(self, pose, q_guess=None, attempts=5, verbose=True):
        q_guess_ = q_guess if q_guess is not None else self.robot_group.get_current_joint_values()

        ik = self.ik_solver.get_ik(q_guess_, *pose)
        if ik is None:
            if attempts > 0:
                return self.solve_ik(pose, q_guess, attempts-1)
            if verbose:
                rospy.logwarn("TRACK-IK: solution not found!")

        return ik

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

    def do_insertion(self, *args, **kwargs):
        self.activate_ros_control_on_ur()
        return self.force_controller.do_insertion(*args, **kwargs)
