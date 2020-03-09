#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
import os.path
import yaml
from std_msgs.msg import String
from o2ac_fastening_tools.srv import *
from o2ac_msgs.msg import *
from util import *

class FasteningToolController(object):
    max_retry = 5
    _feedback = FastenerGripperControlFeedback()
    _result = FastenerGripperControlResult()

    def __init__(self):
        config_dir = rospy.get_param("~config_dir")
        config_file = rospy.get_param("~fastening_tools")

        # get data for .yaml
        conf_gripper_filename = config_dir + "/" + config_file + ".yaml"
        fastening_tools = read_object_yaml_config(conf_gripper_filename)

        # initialize motor id table
        self.fastening_tools = dict()
        data_list = fastening_tools['fastening_tools']
        for data in data_list:
            rospy.loginfo("Loaded " + data['name'] + " on motor id " + str(data['motor_id']))
            self.fastening_tools.update({data['name'] : data['motor_id']})
        
        self.dynamixel_command_write = rospy.ServiceProxy('o2ac_fastening_tools/dynamixel_write_command', DynamixelWriteCommand)
        self.dynamixel_read_state = rospy.ServiceProxy('o2ac_fastening_tools/dynamixel_read_state', DynamixelReadState)

        self._action_name = 'screw_tool_control'
        self._as = actionlib.SimpleActionServer(self._action_name, FastenerGripperControlAction, execute_cb=self.execute_control, auto_start = False)
        self._as.start()

    def set_torque_enable(self, motor_id, value):
        for i in range(1, self.max_retry + 1):
            try:
                res = self.dynamixel_command_write(motor_id, "Torque_Enable", value)
            except rospy.ServiceException as exc:
                rospy.logwarn('An exception occurred in the Torque_Enable set, but processing continues.')
            else:
                if not res.comm_result:
                    rospy.logerr('Can not set torque_enable to XL-320. (ID=%i)' %motor_id)
                return res.comm_result

        rospy.logwarn('Maxmum number of retries has been reached.')
        return True

    def set_moving_speed(self, motor_id, value):
        for i in range(1, self.max_retry + 1):
            try:
                res = self.dynamixel_command_write(motor_id, "Moving_Speed", value)
            except rospy.ServiceException as exc:
                rospy.logwarn('An exception occurred in the Moving_Speed set. Processing retry.')
            else:
                if not res.comm_result:
                    rospy.logerr('Can not set speed to XL-320. (ID=%i)' %motor_id)
                return res.comm_result

        rospy.logwarn('Maxmum number of retries has been reached.')
        return True

    def get_present_speed(self, motor_id):
        res = self.dynamixel_read_state(motor_id, "Present_Speed")
        if res.comm_result:
            return res.value
        else:
            rospy.logerr('Can not get speed for XL-320. (ID=%i)' %motor_id)
            return -1

    def execute_control(self, goal):
        t_duration = 1
        motor_id = self.fastening_tools[goal.fastening_tool_name]
        self._result.control_result = True
        if not goal.speed:
            goal.speed = 1023
        self._feedback.motor_speed = goal.speed

        rospy.wait_for_service('o2ac_fastening_tools/dynamixel_write_command')
        rospy.wait_for_service('o2ac_fastening_tools/dynamixel_read_state')

        if goal.direction == "loosen" :
            goal.speed = 1024 + goal.speed
            if goal.speed > 2047 :
                goal.speed = 2047
        elif goal.direction == "tighten" :
            if goal.speed > 1023 :
                goal.speed = 1023

        if (goal.fastening_tool_name in self.fastening_tools) == False :
            rospy.logerr("'%s' is not exist in %s." % (goal.fastening_tool_name, self.conf_gripper_filename))
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return

        if not self.set_moving_speed(motor_id, goal.speed) :
            self.set_moving_speed(motor_id, 0)
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = False
            self._as.set_aborted(self._result)
            return
        
        if goal.direction == "loosen" and not goal.duration:
            rospy.logwarn("Loosen command was sent, but without a duration. Setting to 2 seconds.")
            goal.duration = 2
        elif goal.direction == "tighten" and not goal.duration:
            rospy.logwarn("Tighten command was sent, but without a maximum duration. Setting to 10 seconds.")
            goal.duration = 10

        # Turn the motor for the specified number of seconds.
        # Rotate the motor until goal.duration is reached is loaded and stops.
        success_flag = True
        start_time = rospy.get_rostime()
        rospy.sleep(1)   # Wait for motor to start up (and avoid reading wrong speed values)
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success_flag = False
                break

            if (rospy.get_rostime().secs - start_time.secs) > goal.duration:
                rospy.loginfo("Stopping motor due to timeout")
                if goal.direction == "loosen":
                    success_flag = True
                elif goal.direction == "tighten": # If motor does not stall before timeout, tightening was not successful
                    success_flag = False
                break
            
            if self._feedback.motor_speed == 0:
                rospy.loginfo("Stopping motor because it has stalled (the screw is tightened)")
                success_flag = True
                break
            
            # Read motor speed. To avoid erroneous readouts, it is read twice. This is not an ideal workaround.
            first_speed = self.get_present_speed(motor_id)
            rospy.sleep(0.1)
            second_speed = self.get_present_speed(motor_id)
            if first_speed == -1 or second_speed == -1:
                success_flag = False
                rospy.logwarn("Error in motor readout. Stopping.")
                break
                
            # If both readings are below a threshold, the motor has stalled
            if first_speed <= 10 and second_speed <= 10:
                self._feedback.motor_speed = 0
            else:
                self._feedback.motor_speed = max(first_speed, second_speed)

            rospy.logdebug("first_speed, second_speed = " + str(first_speed) + ", " + str(second_speed))
            self._as.publish_feedback(self._feedback)
        
        motor_stopped = self.set_moving_speed(motor_id, 0)
        if motor_stopped and success_flag:
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = True
            self._as.set_succeeded(self._result)
        else:
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = False
            self._as.set_aborted(self._result)

        
if __name__ == '__main__':
    rospy.init_node('fastening_tool_controller')
    server = FasteningToolController()
    rospy.spin()
