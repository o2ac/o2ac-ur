#!/usr/bin/env python
#
# Copyright (c) 2020, OMRON SINIC X
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
# Author: Karoly Istvan Artur

import rospy
import rospkg
import actionlib

import time
import yaml

import dynamixel_workbench_msgs.msg
import dynamixel_workbench_msgs.srv

import o2ac_msgs.msg

class DynamixelControllerActionServer():
    '''
    This class provides an actionlib server for sending commands to grippers equipped with Dynamixel Robotis motors.
    The class uses the Dynamixel Workbench Controller to send commands to a single Dynamixel controller.
    For multiple controllers, multiple instances of this class should be created.
    '''
    def __init__(self):
        # Read the configuration for the motors (The motors are configured by the controller directly from this file,
        # this class does not configure the motors, it just uses the configuration data)
        rospack = rospkg.RosPack()
        path = rospack.get_path('o2ac_dynamixel') + '/config/motors_config.yaml'
        with open(path, 'r') as file_open:
            motors_config = yaml.load(file_open)
        
        self._motors_config = motors_config
        self._num_of_configured_motors = len(motors_config)
        # Store the current limit that is set in the config file, because the current limit in the motors config variable will be modified during operation
        # depending on the type of operation (grasping motion or non-grasping motion)
        self._static_current_limits = [{'id': value['ID'], 'current_limit': value['current_limit']} for (key, value) in self._motors_config.iteritems()]

        # Get ROS params and setup names and namespaces
        self._controller_namespace = rospy.get_param("~controller_ns")
        self._gripper_current_ratio = rospy.get_param("~gripper_current_ratio")
        self._service_name = '/'.join(['',self._controller_namespace, 'dynamixel_command'])
        self._dynamixel_state_topic = '/'.join(['',self._controller_namespace, 'dynamixel_state'])
        self._action_server_name = '/'.join(['',self._controller_namespace, 'action_server'])

        # Current state of the motors will be stored in this variable, it is used to check if the action is completed. It could also be used for feedback.
        self._dynamixel_current_state = []

        # Connect to the service provided by Dynamixel Workbench Controller to send commands to the motors
        rospy.wait_for_service(self._service_name)
        self._service_proxy = rospy.ServiceProxy(self._service_name, dynamixel_workbench_msgs.srv.DynamixelCommand)

        # Subscribe to the topic to update current state of the motors. First a test listener is subscibed to determine the message type used in the topic
        # The message type is dependent on the number of motors connected to the controller.
        self._test_listener = rospy.Subscriber(self._dynamixel_state_topic, rospy.AnyMsg, self._test_listener_callback)

        # Start action server
        self._action_server = actionlib.SimpleActionServer(self._action_server_name, o2ac_msgs.msg.DynamixelGripperCommandAction, execute_cb = self._action_server_callback, auto_start = False)
        self._action_result = o2ac_msgs.msg.DynamixelGripperCommandResult()
        self._action_server.start()

    def _send_dynamixel_command(self, command):
        '''
        This function is used to send commands to the motors via the service provided by Dynamixel Workbench Controller
        The input "command" is a ROS message type DynamixelCommand from "dynamixel_workbench_msgs.srv"
        The function returns a boolean value, which is True in case of a successful communication of the command
        '''
        rospy.loginfo('Sending command "' + command.addr_name + ' = ' + str(command.value) + '" to motor with ID: ' + str(command.motor_id))

        # Send the command to the specified motor
        response = self._service_proxy('', command.motor_id, command.addr_name, command.value)

        # Check if the command was sent successfully
        if not response.comm_result:
            rospy.logerr('Sending the dynamixel command was not succesful')
            return False
        else:
            # If the command was sent successfully, start waiting for the task to complete
            # First wait for the listener that provided the current state of the motors to connect and wait until it starts to update the current state
            while '_listener' not in dir(self):
                rospy.sleep(0.1)
            if self._motor_status_topic_is_list == True:
                while not self._dynamixel_current_state.dynamixel_state:
                    rospy.sleep(0.1)
            elif self._motor_status_topic_is_list == False:
                while self._dynamixel_current_state.name == '':
                    rospy.sleep(0.1)
            
            # If the current state of the motors is updating, read the states and wait for the completion of the task
            target_reached = False
            while not target_reached:
                # Get the up-to-date state of the motor and its config values
                if self._motor_status_topic_is_list == True:
                    addressed_motor_state = next((motor for motor in self._dynamixel_current_state.dynamixel_state if motor.id == command.motor_id))
                elif self._motor_status_topic_is_list == False:
                    addressed_motor_state = self._dynamixel_current_state
                addressed_motor_config = next((value for (key, value) in self._motors_config.iteritems() if value['ID'] == command.motor_id))

                # If the command was a position goal, wait until the motor stops moving
                if command.addr_name == 'Goal_Position' and addressed_motor_state.present_velocity == 0:
                    # If the motor is stopped at the requested position, the task is completed
                    if addressed_motor_state.present_position == command.value:
                        target_reached = True
                    # If the motor is stopped and the current is at the set limit, the task is completed
                    elif abs(addressed_motor_state.present_current) >= addressed_motor_config['current_limit']:
                        target_reached = True
                    
                # If the command was a current goal, check if the current is set to the requested value (in operating mode 1, which is current control)
                # or just update the current limit
                if command.addr_name == 'Goal_Current':
                    if addressed_motor_config['operating_mode'] == 1:
                        if addressed_motor_state.present_current == command.value:
                            target_reached = True
                    elif addressed_motor_config['operating_mode'] == 5:
                        addressed_motor_config['current_limit'] = command.value
                        target_reached = True

                # If the command was a velocity goal, check if the velocity is set to the requested value
                if command.addr_name == 'Goal_Velocity' and addressed_motor_state.present_velocity == command.value:
                    target_reached = True

                rospy.sleep(0.1)

            rospy.loginfo('Dynamixel controller: Target reached')
            return target_reached

    def _test_listener_callback(self, data):
        '''
        This function is executed when a messgae is received by _test_listener.
        It determines the actual ROS message type that is used in the topic and creates a subscriber with the
        given type, to read the status of the motors.
        '''
        message_type = data._connection_header['type']
        self._test_listener.unregister()
        if message_type == 'dynamixel_workbench_msgs/DynamixelState':
            self._motor_status_topic_is_list = False
            self._dynamixel_current_state = dynamixel_workbench_msgs.msg.DynamixelState()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic, dynamixel_workbench_msgs.msg.DynamixelState, self._listener_callback)
        elif message_type == 'dynamixel_workbench_msgs/DynamixelStateList':
            self._motor_status_topic_is_list = True
            self._dynamixel_current_state = dynamixel_workbench_msgs.msg.DynamixelStateList()
            self._listener = rospy.Subscriber(self._dynamixel_state_topic, dynamixel_workbench_msgs.msg.DynamixelStateList, self._listener_callback)
        else:
            rospy.logerr('Unexpected message type: ' + message_type)

    def _listener_callback(self, data):
        '''
        This function is executed when a messgae is received by _listener.
        The data received is the status of the motors connected to the controller. It is stored in the _dynamixel_current_state variable
        '''
        self._dynamixel_current_state = data

    def _action_server_callback(self, goal):
        '''
        This function is the callback of the action server. It is executed when a goal is submitted to the action server.
        The goal is defined in the o2ac_msgs/DinamixelGripperCommand action definition.
        '''
        addressed_motor_config = next((value for (key, value) in self._motors_config.iteritems() if value['ID'] == goal.motor_id))
        command = dynamixel_workbench_msgs.srv.DynamixelCommand()
        command.motor_id = goal.motor_id

        current_limit = next((entry['current_limit'] for entry in self._static_current_limits if entry['id'] == goal.motor_id))

        # If the action is to open the fingers and the motion is not a grasping operation, open it with full speed
        if goal.direction_open == goal.OPENING and goal.is_grasping_operation == goal.IS_NOT_GRASP:
            command.addr_name = 'Goal_Current'
            command.value = current_limit
            current_set_result = self._send_dynamixel_command(command)
            command.addr_name = 'Goal_Position'
            command.value = addressed_motor_config['max_position_limit']
            self._action_result.success = current_set_result and self._send_dynamixel_command(command)
            self._action_server.set_succeeded(self._action_result)

        # If the action is to close the fingers and the motion is a grasping operation, close it using the reduced current
        elif goal.direction_open == goal.CLOSING and goal.is_grasping_operation == goal.IS_GRASP:
            command.addr_name = 'Goal_Current'
            command.value = int(current_limit * self._gripper_current_ratio)
            current_set_result = self._send_dynamixel_command(command)
            command.addr_name = 'Goal_Position'
            command.value = addressed_motor_config['min_position_limit']
            self._action_result.success = current_set_result and self._send_dynamixel_command(command)
            self._action_server.set_succeeded(self._action_result)

        # If the action is to open the fingers and the motion is a grasping operation, open it using the reduced current
        elif goal.direction_open == goal.OPENING and goal.is_grasping_operation == goal.IS_GRASP:
            command.addr_name = 'Goal_Current'
            command.value = int(current_limit * self._gripper_current_ratio)
            current_set_result = self._send_dynamixel_command(command)
            command.addr_name = 'Goal_Position'
            command.value = addressed_motor_config['max_position_limit']
            self._action_result.success = current_set_result and self._send_dynamixel_command(command)
            self._action_server.set_succeeded(self._action_result)

        # If the action is to close the fingers and the motion is not a grasping operation, close it with full speed
        elif goal.direction_open == goal.CLOSING and goal.is_grasping_operation == goal.IS_NOT_GRASP:
            command.addr_name = 'Goal_Current'
            command.value = current_limit
            current_set_result = self._send_dynamixel_command(command)
            command.addr_name = 'Goal_Position'
            command.value = addressed_motor_config['min_position_limit']
            self._action_result.success = current_set_result and self._send_dynamixel_command(command)
            self._action_server.set_succeeded(self._action_result)


if __name__ == '__main__':
    rospy.init_node('dynamixel_controller_action_server')
    action_server = DynamixelControllerActionServer()
    rospy.spin()
