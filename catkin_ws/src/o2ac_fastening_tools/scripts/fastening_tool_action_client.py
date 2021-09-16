#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from o2ac_fastening_tools.srv import *
from o2ac_msgs.msg import *

import actionlib
import actionlib_tutorials.msg


class FasteningToolController:
    def fasten(self, name):
        client = actionlib.SimpleActionClient(
            'screw_tool_control', ScrewToolControlAction)
        client.wait_for_server()
        goal = ScrewToolControlGoal()

        goal.fastening_tool_name = name
        goal.speed = 100
        goal.direction = "tighten"

        client.send_goal_and_wait(goal, rospy.Duration(10), rospy.Duration(10))
        client.wait_for_result()

        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('fastening_tool_controller_test')

        controller = FasteningToolController()

        name_list = [
            "screw_tool_m4",
            "screw_tool_m3",
        ]

        for name in name_list:
            if not controller.fasten(name):
                break

        print('')

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
