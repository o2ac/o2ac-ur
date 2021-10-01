#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from o2ac_fastening_tools.srv import *
from o2ac_msgs.msg import *

import actionlib
import actionlib_tutorials.msg


class SuctionClient:
    def suction(self, name, turn_suction_on, eject_screw):
        client = actionlib.SimpleActionClient(
            'suction_control', SuctionControlAction)
        client.wait_for_server()
        goal = SuctionControlGoal()

        goal.fastening_tool_name = name
        goal.turn_suction_on = turn_suction_on
        goal.eject_screw = eject_screw

        client.send_goal_and_wait(goal, rospy.Duration(30), rospy.Duration(10))
        client.wait_for_result()

        return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('suction_tool_test')

        controller = SuctionClient()

        name_list = [
            "screw_tool_m4"
        ]

        for name in name_list:
            res = controller.suction(name, True, False)
            rospy.sleep(10)
            res = controller.suction(name, False, False)

            if not res.success:
                rospy.logerr("Could not pick screw")

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
