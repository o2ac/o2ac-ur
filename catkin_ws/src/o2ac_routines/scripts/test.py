#!/usr/bin/env python

import threading
from o2ac_routines.common import O2ACCommon
from o2ac_routines.cooking import Cooking
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
import sys
import signal
import geometry_msgs.msg
from math import pi, radians

from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model

import tf_conversions
import copy

tau = 2.0*pi  # Part of math from Python 3.6


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def test_gazebo():
    name = "panel_bearing"
    object_pose = conversions.to_pose_stamped("tray_center", [0,0,0,0,0,0])
    object_pose = c.listener.transformPose("world", object_pose)
    op = conversions.from_pose_to_list(object_pose.pose)
    objpose = [op[:3], op[3:]]
    models = [Model(name, objpose[0], orientation=objpose[1], reference_frame="world")]
    c.gazebo_scene.load_models(models,)
    c.b_bot.gripper.gripper.grab(link_name="panel_bearing_tmp::panel_bearing")
    c.b_bot.gripper.gripper.release(link_name="panel_bearing_tmp::panel_bearing")

def main():
    rospy.init_node("testscript")
    global c
    c = Cooking()
    c.a_bot.move_lin_rel(relative_translation=[0.1, 0, 0.0], relative_to_tcp=True)
    if not c.equip_knife():
        return
    c.unequip_knife()
       
    
    # c = O2ACAssembly()

    # a_bot_at_cable_end_joints = [1.5075, -1.7509, 2.4526, -2.9421, -1.1429, 4.6537]
    # c.a_bot.move_joints(a_bot_at_cable_end_joints)
    # c.a_bot.move_lin_rel([-0.002,0,0], relative_to_tcp=True)

    # print("a_bot",np.round(c.a_bot.robot_group.get_current_joint_values(), 4).tolist())
    # # print("a_bot ps",np.round(conversions.from_pose_to_list(c.listener.transformPose("assembled_part_07_inserted", c.a_bot.get_current_pose_stamped()).pose), 4).tolist())
    # print("b_bot",np.round(c.b_bot.robot_group.get_current_joint_values(), 4).tolist())

    # approach_pose = conversions.to_pose_stamped("assembled_part_03_pulley_ridge_top", [0.1, 0.0, 0.0,0,0,0])
    # c.a_bot.go_to_pose_goal(approach_pose, speed=0.1, move_lin=True, end_effector_link="a_bot_nut_tool_m4_hole_link")
    # approach_centering = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.1, 0, -0.150,  0, radians(-60), 0])
    # c.b_bot.go_to_pose_goal(approach_centering, speed=0.1, end_effector_link="b_bot_outside_camera_link", move_lin=True)
    # c.vision.activate_camera("b_bot_outside_camera")
    # c.vision.activate_pulley_screw_detection()
    # c.confirm_to_proceed("stop?")
    # c.vision.activate_pulley_screw_detection(False)
    # approach_pulley = conversions.to_pose_stamped("assembled_part_07_front_hole", [-0.03, 0.001, 0.001, tau/2, 0, 0])

    # # Grasp with a_bot
    # c.confirm_to_proceed("approach pulley a_bot")
    # c.a_bot.go_to_pose_goal(approach_pulley, speed=0.3)
    # # c.confirm_to_proceed("pulley_grasp_pose a_bot")
    # # c.a_bot.go_to_pose_goal(pulley_grasp_pose, speed=0.3)
    # # c.a_bot.gripper.close(force=100)
    # c.confirm_to_proceed("rotate 1 deg")
    # c.rotate_cylinder_by_angle(1, "a_bot", approach_pulley, grasp_width=0)
    # c.confirm_to_proceed("rotate more")
    # c.rotate_cylinder_by_angle(10, "a_bot", approach_pulley, grasp_width=0)
    # tp = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.0] + np.deg2rad([0, 90., 0]).tolist())
    # c.b_bot.go_to_pose_goal(tp)
    # c.confirm_to_proceed("go")
    # c.b_bot.go_to_named_pose("home")
    # c.a_bot.go_to_pose_goal(tp)
    # c.confirm_to_proceed("go")
    # c.a_bot.go_to_named_pose("home")
    # test_gazebo()
    # c.a_bot.gripper.open(opening_width=0.006)
    # c.b_bot.move_lin_rel(relative_rotation=[0, -radians(0.5), 0], speed=0.01, end_effector_link="b_bot_screw_tool_m4_tip_link")
    # print(conversions.from_pose_to_list(c.listener.transformPose("assembled_part_02_back_hole", c.b_bot.get_current_pose_stamped()).pose))
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
    # c.reset_scene_and_robots()
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

    # c = O2ACAssembly()
    # c.insert_motor_cables_without_tools_normal(cable_color="black", cable_straighten_distance=0.133)
    # print("done")
    # return

    # c.insert_motor_cables_with_tool(cable_color="black")
    # c.equip_cable_tool()
    # c.unequip_cable_tool()
    # c.spawn_tool("cable_tool")


if __name__ == "__main__":
    main()
