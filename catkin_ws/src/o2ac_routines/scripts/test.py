#!/usr/bin/env python

from o2ac_routines.common import O2ACCommon
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from ur_control import conversions
import tf
from o2ac_routines.helpers import get_target_force
import o2ac_routines.helpers as helpers
import numpy as np
import sys, signal
import geometry_msgs.msg

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def main():
    rospy.init_node("testscript")
    global controller
    controller = O2ACCommon()

    # controller.playback_sequence("idler_pulley_release_screw_tool")

    # controller.b_bot.go_to_named_pose("screw_ready")
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # controller.assembly_database.change_assembly('taskboard')
    # controller.pick_and_insert_shaft("taskboard")
    # controller.pick_and_insert_idler_pulley("taskboard")

    controller.playback_sequence("idler_pulley_equip_nut_tool")
    approach_pose = conversions.to_pose_stamp("taskboard_long_hole_middle_link", [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    controller.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.4)
    controller.playback_sequence("idler_pulley_unequip_nut_tool")
    

    # controller.playback_sequence("idler_pulley_release_screw_tool")
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # controller.prepare_screw_tool_idler_pulley("taskboard_long_hole_middle_link")

    # controller.unequip_tool("b_bot", "padless_tool_m4")
    # controller.b_bot.go_to_named_pose("screw_ready")
    # controller.playback_sequence("idler_pulley_push_with_screw_tool")

    # controller.b_bot.move_lin_rel(relative_translation=[0.1, 0, 0], speed=0.2)
    # controller.b_bot.move_lin_rel(relative_translation=[0.0, 0, 0.25], speed=0.2)
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # approach_pose = conversions.to_pose_stamp("taskboard_long_hole_middle_link", [0.06, 0.0, 0.0, 0.0, 0.0, 0.0])
    # controller.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
    # approach_pose = conversions.to_pose_stamp("taskboard_long_hole_middle_link", [0.012, 0.0, 0.0, 0.0, 0.0, 0.0])
    # controller.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
    # controller.b_bot.move_lin_rel(relative_translation=[0, 0, -0.015], speed=0.2)
    # controller.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.15], relative_to_robot_base=True)
    # controller.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.1], relative_rotation=[0,0,0], relative_to_robot_base=True)
    # controller.a_bot.move_lin_rel(relative_translation=[0.0, -0.1, 0.], relative_rotation=[0,0, np.deg2rad(45)], relative_to_robot_base=True)
    # controller.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.], relative_rotation=[0,np.deg2rad(-90), 0], relative_to_robot_base=True)
    # controller.fasten_idler_pulley_with_nut_tool("taskboard_long_hole_middle_link")
    # controller.playback_sequence("idler_pulley_unequip_nut_tool")
    # controller.unequip_tool("b_bot", "padless_tool_m4")
    # controller.prepare_nut_tool()
    # controller.hold_screw_tool_idler_pulley("taskboard_long_hole_middle_link")
    # controller.pick_and_insert_shaft("taskboard")

    # controller.b_bot.move_joints([1.6166805028915405, -1.666929384271139, 1.8822982946978968, -1.794750829736227, -1.5447457472430628, 1.592530369758606], acceleration = 0.05, speed=.1)
    # controller.test("taskboard_assy_part_07_inserted")
    # controller.spiral_search_with_nut_tool()
    # controller.insert_idler_pulley("taskboard_long_hole_middle_link")
    
    # controller.a_bot.move_joints([0.6765, -1.5287, 2.2274, -0.6668, -0.1085, 1.5388], acceleration = 0.05, speed=.1)
    # controller.playback_sequence("idler_pulley_prepare_nut_tool")
    # controller.orient_idle_pulley("taskboard")
    # controller.prepare_screw_tool_idler_pulley("taskboard")
    # print("bearing_pose", controller.look_and_get_grasp_point("bearing"))
    # fake_vision_pose = conversions.to_pose_stamp("tray_center", [-0.03078, 0.06248, 0.02, 0.0,0.7071,0.0,0.7071])

    # controller.b_bot.activate_ros_control_on_ur()
    # print(controller.a_bot.go_to_named_pose("home"))
    # controller.a_bot.gripper.open()
    # controller.prepare_nut_tool("taskboard_long_hole_middle_link")

    # rospy.sleep(2)
    # controller.b_bot.move_joints([1.7609, -2.0808, 2.4036, -1.9124, -1.5491, 0.1639], acceleration = 0.05, speed=.1)

    # rospy.sleep(2)
    # controller.b_bot.move_joints([1.864789605140686, -1.8883010349669398, 2.4725635687457483, -2.175955434838766,
    # -1.5510090033160608, 1.1179262399673462], acceleration = 0.05, speed=.1)
    
    # controller.b_bot.move_joints([1.8636610507965088, -1.6897312603392542, 2.5860729853259485, -2.4881612263121546,
    # -1.5512121359454554, 1.1185405254364014], acceleration = 0.05, speed=.1)
    
    
    # controller.insert_bearing(task = "taskboard")


    ### Calculate relative motions from UR script (bugged)
    # incline_from_p=[-.240053176805, -.418617584930, .068066729204, -3.141276531850, -.007684409336, .022148737280]
    # incline_to_p=[-.240066422408, -.413499926398, .066111609186, 2.466355790625, -.000195404290, -.019394393672]
    
    # xyz_from = incline_from_p[:3]
    # axisangle_from = incline_from_p[3:]
    # quat_from = helpers.ur_axis_angle_to_quat(axisangle_from)
    # quat_from_inv = tf.transformations.quaternion_inverse(quat_from)
    
    # xyz_to = incline_to_p[:3]
    # axisangle_to = incline_to_p[3:]
    # quat_to = helpers.ur_axis_angle_to_quat(axisangle_to)

    # quat_relative = tf.transformations.quaternion_multiply(quat_from_inv, quat_to)
    # xyz_relative = np.array(xyz_from) - np.array(xyz_to)

    # print("relative translation and rotation is (in robot TCP gripper-140_tip):")
    # print(xyz_relative)
    # print(quat_relative)
    # print(tf.transformations.euler_from_quaternion(quat_relative))

    # p = geometry_msgs.msg.PoseStamped()
    # p.header.frame_id = "a_bot_tool0"
    # p.pose.position = geometry_msgs.msg.Point(*xyz_relative)
    # p.pose.orientation = geometry_msgs.msg.Quaternion(*quat_relative)
    
    # relative_motion_in_tip_link = helpers.rotatePoseByRPY(1.571, -1.571, 0.000, p)  # From tool0 to gripper_tip_link
    # relative_motion_in_tip_link.pose.position = helpers.rotateTranslationByRPY(1.571, -1.571, 0.000, relative_motion_in_tip_link.pose.position)
    # print("relative translation and rotation is (in gripper_tip_link):")
    # print(relative_motion_in_tip_link.pose)
    


if __name__ == "__main__":
    main()
