#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
from o2ac_fastening_tools.srv import *
from o2ac_msgs.msg import *
from ur_msgs.msg import *
from ur_msgs.srv import *
from util import *
from std_msgs.msg import String, Bool


class SuctionController(object):
    _feedback = SuctionControlFeedback()

    def __init__(self):
        self.comm_result = False
        self.pin_state = False
        config_dir = rospy.get_param("~config_dir")
        config_file = rospy.get_param("~suction_control")
        operation_mode_file = rospy.get_param("~operation_mode")

        rospy.Subscriber(
            "b_bot/ur_hardware_interface/io_states",
            IOStates,
            self.io_state_callback,
            queue_size=1)

        self.set_io = rospy.ServiceProxy(
            'b_bot/ur_hardware_interface/set_io', SetIO)

        # get data for .yaml
        conf_suction_filename = config_dir + "/" + config_file + ".yaml"
        conf_file_content = read_object_yaml_config(conf_suction_filename)

        conf_operation_mode_filename = config_dir + "/" + operation_mode_file + ".yaml"
        conf_operation_mode_file_content = read_object_yaml_config(
            conf_operation_mode_filename)

        # initialize ur_control table
        self.digital_in_port = dict()
        self.digital_out_port_vac = dict()
        self.in_state = dict()
        self.out_state = dict()
        self.tool_suction_publisher = dict()
        self.in_port_name = dict()
        self.operation_mode_in_port_name = dict()
        self.operation_mode_publishers = dict()

        # get data for .yaml
        self.suction_tool_list = conf_file_content['suction_control']
        for tool_data in self.suction_tool_list:
            self.in_port_name.update(
                {tool_data['digital_in_port']: tool_data['name']})
            self.digital_in_port.update(
                {tool_data['name']: tool_data['digital_in_port']})
            self.digital_out_port_vac.update(
                {tool_data['name']: tool_data['digital_out_port_vac']})
            self.tool_suction_publisher[tool_data['name']] = rospy.Publisher(
                tool_data['name'] + '/screw_suctioned', Bool, queue_size=1)
            # Goal: Publish a boolean for each tool under
            # '[tool_name]/screw_suctioned'

        # for operation_mode
        self.operation_mode_list = conf_operation_mode_file_content['operation_mode']
        for operation_mode_data in self.operation_mode_list:
            self.operation_mode_in_port_name.update(
                {operation_mode_data['digital_in_port']: operation_mode_data['name']})
            self.operation_mode_publishers[operation_mode_data['name']] = rospy.Publisher(
                operation_mode_data['name'], Bool, queue_size=1)

        self._action_name = 'suction_control'
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SuctionControlAction,
            execute_cb=self.suction_control,
            auto_start=False)
        self._as.start()

    def io_state_callback(self, data):
        for read_in_status in data.digital_in_states:
            self.in_state.update({read_in_status.pin: read_in_status.state})
            if read_in_status.pin in self.in_port_name:
                bool_msg = Bool()
                bool_msg.data = read_in_status.state
                self.tool_suction_publisher[self.in_port_name[read_in_status.pin]].publish(
                    bool_msg)
            if read_in_status.pin in self.operation_mode_in_port_name:
                mode_read = Bool()
                mode_read.data = read_in_status.state
                self.operation_mode_publishers[self.operation_mode_in_port_name[read_in_status.pin]].publish(
                    mode_read)

        for read_out_status in data.digital_out_states:
            self.out_state.update({read_out_status.pin: read_out_status.state})

    def set_out_pin_switch(self, port, state):
        success_flag = True

        if state > 0:
            b_state = True
        else:
            b_state = False

        start_time = rospy.get_rostime()

        while ((rospy.get_rostime().secs - start_time.secs) <= 3.0):
            res = self.set_io(1, port, state)
            if not res:
                success_flag = False
                break
            if (port in self.out_state):
                if self.out_state[port] == b_state:
                    break

        if success_flag:
            rospy.logdebug(
                "Success. Digital_out_port(%d) changed %r." %
                (port, b_state))
        else:
            rospy.logerr(
                "Error. Digital_out_port(%d) can't be changed." %
                (port))

        return success_flag

    def suction_control(self, goal):
        success_flag = True
        res = SuctionControlResult()

        # yaml file check
        if (goal.fastening_tool_name in self.digital_in_port) == False:
            rospy.logerr(
                "yaml file: '%s' does not exist in %s." %
                (goal.fastening_tool_name, self.digital_in_port))
            success_flag = False

        if (goal.fastening_tool_name in self.digital_out_port_vac) == False:
            rospy.logerr("yaml file: '%s' does not exist in %s." %
                         (goal.fastening_tool_name, self.digital_out_port_vac))
            success_flag = False

        if not success_flag:
            res.success = success_flag
            self._as.set_aborted(res)
            return

        vac_port = self.digital_out_port_vac[goal.fastening_tool_name]

        if goal.turn_suction_on:
            if not self.set_out_pin_switch(vac_port, 1):
                success_flag = False
        elif goal.eject_screw:
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False
        else:  # turn_suction_on and pick_screw == False
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False

        if not success_flag:
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False

        res.success = success_flag
        if not success_flag:
            self._as.set_aborted(res)
        self._as.set_succeeded(res)
        return


if __name__ == '__main__':
    rospy.init_node('suction_controller')
    server = SuctionController()
    rospy.spin()
