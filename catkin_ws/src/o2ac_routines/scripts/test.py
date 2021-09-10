#!/usr/bin/env python

import threading
from o2ac_routines.common import O2ACCommon
from o2ac_routines.assembly import O2ACAssembly
from o2ac_routines.taskboard import O2ACTaskboard
from o2ac_routines.thread_with_trace import ThreadTrace
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from ur_control import conversions
import tf
from o2ac_routines.helpers import get_target_force
import o2ac_routines.helpers as helpers
import numpy as np
import sys, signal
import geometry_msgs.msg
from math import pi, radians

from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model

tau = 2.0*pi  # Part of math from Python 3.6

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def main():
    rospy.init_node("testscript")
    global c
    c = O2ACCommon()
    print(conversions.from_pose_to_list(c.listener.transformPose("assembled_part_02_back_hole", c.b_bot.get_current_pose_stamped()).pose))
    # c.reset_scene_and_robots()
    # c.center_panel("panel_bearing")
    # c.center_panel_on_base_plate("panel_motor")
    # c.ab_bot.go_to_named_pose("home")
    # c.place_panel("a_bot", "panel_bearing", pick_again=True, fake_position=True)
    # c.fasten_panel("panel_bearing", simultaneous=False)
    # c.place_panel("a_bot", "panel_motor", pick_again=True, fake_position=True)
    # c.fasten_panel("panel_motor", simultaneous=False)
    # c.hold_panel_for_fastening("panel_motor")
    # c = O2ACTaskboard()
    # c.is_bearing_in_storage = True
    # c.pick_bearing(robot_name="a_bot")
    # c.is_motor_pulley_in_storage = True
    # c.pick_motor_pulley(robot_name="b_bot")
    # c.fallback_recenter_bearing("taskboard", "a_bot")
    # c.insert_bearing("taskboard_bearing_target_link", robot_name="a_bot", task="taskboard")
    # c.equip_nut_tool()
    # c.fasten_idler_pulley_with_nut_tool("taskboard_long_hole_top_link")
    # c.publish_part_in_assembled_position("base", marker_only=True)
    # c.publish_part_in_assembled_position("panel_motor")
    # c.publish_part_in_assembled_position("panel_bearing")
    # c.publish_part_in_assembled_position("motor", marker_only=True)
    # c.insert_motor_cables_with_tool()
    # c.equip_cable_tool()
    # c.unequip_cable_tool()
    # c.spawn_tool("cable_tool")
    

if __name__ == "__main__":
    main()
