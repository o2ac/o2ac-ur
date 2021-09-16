#!/usr/bin/env python

import threading
from o2ac_routines.common import O2ACCommon
from o2ac_routines.assembly import O2ACAssembly
from o2ac_routines.taskboard import O2ACTaskboard
from o2ac_routines.thread_with_trace import ThreadTrace
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from ur_control import controllers, conversions
import tf
from o2ac_routines.helpers import get_target_force
import o2ac_routines.helpers as helpers
import numpy as np
import sys
import signal
import geometry_msgs.msg
from math import pi, radians
tau = 2.0*pi  # Part of math from Python 3.6


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def execute_sequences():
    """ Continously and simultaneously execute a simple `sequence` trajectory in two different poses """

    def a_task():
        controller.a_bot.go_to_named_pose("home")
        seq = []
        seq.append(helpers.to_sequence_item_relative([0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([-0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, 0.01, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, -0.01, 0, 0, 0, 0]))
        for i in range(8):
            controller.execute_sequence("a_bot", seq, "a"+str(i))
            if i % 2 == 0:
                controller.a_bot.go_to_named_pose("home")
            else:
                controller.a_bot.go_to_named_pose("feeder_pick_ready")

    def b_task():
        controller.b_bot.go_to_named_pose("home")
        seq = []
        seq.append(helpers.to_sequence_item_relative([0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([-0.01, 0, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, 0.01, 0, 0, 0, 0]))
        seq.append(helpers.to_sequence_item_relative([0, -0.01, 0, 0, 0, 0]))
        for i in range(8):
            controller.execute_sequence("b_bot", seq, "b"+str(i))
            if i % 2 == 0:
                controller.b_bot.go_to_named_pose("home")
            else:
                controller.b_bot.go_to_named_pose("feeder_pick_ready")

    controller.do_tasks_simultaneous(a_task, b_task, timeout=500)


def equip_unequip():
    """ Continously and simultaneously equip and unequip tool """
    controller.reset_scene_and_robots()

    def a_task():
        controller.a_bot.go_to_named_pose("home")
        for i in range(8):
            print("//// A_BOT equip ++++")
            controller.do_change_tool_action("a_bot", equip=True, screw_size=3)
            print("//// A_BOT equip done")
            print("//// A_BOT unequip ++++")
            controller.do_change_tool_action("a_bot", equip=False, screw_size=3)
            print("//// A_BOT unequip done")

    def b_task():
        controller.b_bot.go_to_named_pose("home")
        for i in range(8):
            print(">>>> B_BOT equip ****")
            controller.do_change_tool_action("b_bot", equip=True, screw_size=4)
            print(">>>> B_BOT equip done")
            print(">>>> B_BOT unequip ****")
            controller.do_change_tool_action("b_bot", equip=False, screw_size=4)
            print(">>>> B_BOT unequip done")

    controller.do_tasks_simultaneous(a_task, b_task, timeout=500)


def main():
    rospy.init_node("testscript")
    global controller
    controller = O2ACCommon()

    equip_unequip()

    execute_sequences()


if __name__ == "__main__":
    main()
