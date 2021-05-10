#!/usr/bin/env python

from o2ac_routines.common import O2ACCommon
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from o2ac_routines.helpers import get_target_force
import numpy as np
import sys, signal
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def main():
    rospy.init_node("testscript")
    global controller
    controller = O2ACCommon()
    # controller.b_bot.activate_ros_control_on_ur()
    # controller.playback_sequence("bearing_orient_totb")

    controller.b_bot.move_joints([1.5791, -1.257, 1.8261, -2.2056, -2.6119, -1.5235])
    controller.insert_bearing(task = "taskboard")


if __name__ == "__main__":
    main()
