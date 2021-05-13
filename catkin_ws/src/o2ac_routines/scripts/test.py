#!/usr/bin/env python

from o2ac_routines.common import O2ACCommon
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from ur_control import conversions
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

    controller.a_bot.move_lin_rel(relative_translation=[0, 0, 0.10])
    controller.pick_and_insert_idle_pulley("taskboard")
    # controller.orient_idle_pulley("taskboard")
    # controller.prepare_screw_tool_idler_pulley("taskboard")
    # controller.assembly_database.change_assembly('taskboard')
    # print("bearing_pose", controller.look_and_get_grasp_point("bearing"))
    # fake_vision_pose = conversions.to_pose_stamp("tray_center", [-0.03078, 0.06248, 0.02, 0.0,0.7071,0.0,0.7071])

    # controller.b_bot.activate_ros_control_on_ur()
    # controller.a_bot.go_to_named_pose("home")
    # controller.a_bot.gripper.open()
    # controller.playback_sequence("idler_pulley_prepare_nut")
    # controller.a_bot.move_joints([0.6958768367767334, -1.8462234936156214, 1.9580543676959437, -1.1763850015452881,
    # -0.6372688452350062, 3.2055156230926514], acceleration = 0.05, speed=.1)
    # controller.prepare_nut_tool("taskboard_long_hole_middle_link")

    # rospy.sleep(2)
    # controller.b_bot.move_joints([1.7609, -2.0808, 2.4036, -1.9124, -1.5491, 0.1639], acceleration = 0.05, speed=.1)

    # rospy.sleep(2)
    # controller.b_bot.move_joints([1.864789605140686, -1.8883010349669398, 2.4725635687457483, -2.175955434838766,
    # -1.5510090033160608, 1.1179262399673462], acceleration = 0.05, speed=.1)
    
    # controller.b_bot.move_joints([1.8636610507965088, -1.6897312603392542, 2.5860729853259485, -2.4881612263121546,
    # -1.5512121359454554, 1.1185405254364014], acceleration = 0.05, speed=.1)
    
    
    # controller.insert_bearing(task = "taskboard")


if __name__ == "__main__":
    main()
