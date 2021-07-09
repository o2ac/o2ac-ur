#!/usr/bin/env python

from numpy.lib.npyio import save
from o2ac_routines.common import O2ACCommon
from o2ac_routines.assembly import O2ACAssembly
import rospy
from ur_control.constants import TERMINATION_CRITERIA
from ur_control import conversions
import tf
from o2ac_routines.helpers import get_target_force
import o2ac_routines.helpers as helpers
import numpy as np
import sys, signal
import geometry_msgs.msg
from math import pi
tau = 2.0*pi  # Part of math from Python 3.6

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

def test_force_control(c):
    selection_matrix = [0., 1., 1., 1., 1., 1.]
    target_force = np.array([0., 0., 0., 0., 0., 0.])

    c.b_bot.go_to_pose_goal(controller.tray_view_high)
    c.b_bot.force_controller.force_control(target_force=target_force, selection_matrix=selection_matrix, timeout=30.0)

def main():
    rospy.init_node("testscript")
    global controller
    controller = O2ACCommon()
    test_force_control(controller)

    # controller.a_bot.go_to_named_pose("screw_ready")
    # controller.a_bot.force_controller.execute_spiral_trajectory2("YZ", max_radius=0.0015, radius_direction="+Y", steps=50,
    #                                                       revolutions=5, target_force=0, termination_criteria=None, timeout=10,
    #                                                       check_displacement_time=10, end_effector_link="a_bot_screw_tool_m3_tip_link")

    # print(":::::::::::::::")
    # controller.a_bot.force_controller.execute_spiral_trajectory2("YZ", max_radius=0.0015, radius_direction="+Y", steps=50,
    #                                                       revolutions=5, target_force=0, termination_criteria=None, timeout=10,
    #                                                       check_displacement_time=10)
    # controller.a_bot.go_to_named_pose("horizontal_screw_ready")
    # controller.a_bot.force_controller.execute_spiral_trajectory2("YZ", max_radius=0.0015, radius_direction="+Y", steps=50,
    #                                                       revolutions=5, target_force=0, termination_criteria=None, timeout=10,
    #                                                       check_displacement_time=10)
    # controller.b_bot.go_to_named_pose("screw_ready")
    # controller.playback_sequence("idler_pulley_ready_screw_tool")
    # controller.confirm_to_proceed("")
    # controller.playback_sequence("idler_pulley_return_screw_tool")
    # controller.b_bot.go_to_named_pose("screw_ready")

    # controller.b_bot.gripper.open()
    # controller.b_bot.go_to_named_pose("home")
    # at_object_pose = conversions.to_pose_stamped("right_centering_link", [-0.005, 0, 0.0, 0, 0, 0] )
    # controller.b_bot.go_to_pose_goal(at_object_pose, speed=0.5)
    # controller.center_with_gripper("b_bot", opening_width=0.085)
    
    # controller.vision.activate_camera("b_bot_inside_camera")
    # look_at_output_pulley = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.212, 0, 0.0, -tau/4, 0, -tau/4])
    # controller.b_bot.go_to_pose_goal(look_at_output_pulley, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=0.05)

    # controller.assembly_database.change_assembly("taskboard")
    # controller.pick_bearing()
    # controller.orient_bearing("taskboard")
    # controller = O2ACAssembly()
    # controller.playback_sequence(routine_filename="plunger_tool_equip", plan_while_moving=True, use_saved_plans=False)
    # controller.playback_sequence(routine_filename="plunger_tool_unequip", plan_while_moving=True, use_saved_plans=False)
    # controller.playback_sequence(routine_filename="idler_pulley_return_screw_tool", plan_while_moving=True)

    # controller.take_tray_from_agv(preplanned=False)
    # controller.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    # controller.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    # controller.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    # controller.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)
    # controller.take_tray_from_agv_preplanned()
    # controller.playback_sequence("tray_take_from_agv", plan_while_moving=True, save_on_success=True, use_saved_plans=True)
    # controller.test_ik()
    # controller.reset_scene_and_robots()
    # controller.fasten_bearing("assembly")
    # controller.tools.set_motor("padless_tool_m4", "tighten", duration=2)
    # controller.take_tray_from_agv()
    # controller.confirm_to_proceed("")
    # controller.publish_part_in_assembled_position("base")
    # controller.publish_part_in_assembled_position("panel_bearing")
    # controller.subtask_e()
    # b_plan, _ = controller.b_bot.go_to_named_pose("screw_ready", wait=False, plan_only=True)
    # controller.b_bot.execute_plan(b_plan, wait=False)
    # rospy.sleep(1)
    # a_plan, _ = controller.a_bot.go_to_named_pose("screw_ready", wait=False, plan_only=True)
    # controller.a_bot.execute_plan(a_plan, wait=False)
    # controller.a_bot.go_to_named_pose("screw_ready")

    # controller.assembly_database.change_assembly('taskboard')
    # controller.fallback_b_bot_outside_tray_centering()
    # controller.reset_scene_and_robots()

    # controller.fasten_bearing("taskboard")

    # print(controller.b_bot.robot_group.get_current_joint_values())

    # return
    # controller.b_bot.move_joints([1.8108925819396973, -2.119059225121969, 1.8984859625445765, -1.3372403693250199,
    # -1.780367676411764, 1.1874641180038452])
    # controller.b_bot.move_joints([1.444, -1.61 ,  2.504, -1.964, -1.2  , -2.578])
    

    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # st = rospy.get_time()
    # print("TIME old", rospy.get_time()-st)
    
    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # controller.b_bot.gripper.open()
    # st = rospy.get_time()
    # controller.playback_sequence(routine_filename="idler_pulley_equip_nut_tool")
    # controller.playback_sequence(routine_filename="return_screw_tool_horizontal")
    # print("TIME new", rospy.get_time()-st)

    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # controller.b_bot.gripper.open()
    # st = rospy.get_time()
    # controller.playback_sequence(routine_filename="ready_screw_tool_horizontal", wait=False)
    # print("TIME new", rospy.get_time()-st)
    
    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # # plan, _ = controller.b_bot.plan_relative_goal(relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = controller.b_bot.plan_relative_goal(relative_rotation=[0,np.deg2rad(-15),0], relative_to_tcp=True)
    # controller.b_bot.execute_plan(plan, wait=True)
    
    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # q = controller.b_bot.robot_group.get_current_joint_values()
    # # plan, _ = controller.b_bot.plan_relative_goal(initial_joints=q, relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = controller.b_bot.plan_relative_goal(relative_rotation=[0,np.deg2rad(-15),0], relative_to_tcp=True, initial_joints=q)
    # controller.b_bot.execute_plan(plan, wait=True)
    
    # controller.b_bot.go_to_named_pose("home", speed=1.0)
    # q = controller.b_bot.robot_group.get_current_joint_values()
    # # plan, _ = controller.b_bot.plan_relative_goal(initial_joints=q, relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = controller.b_bot.plan_relative_goal(relative_rotation=[np.deg2rad(-15),0,0], relative_to_tcp=True, initial_joints=q)
    # controller.b_bot.execute_plan(plan, wait=True)

    # controller = O2ACAssembly()

    # controller.reset_scene_and_robots()
    # controller.publish_part_in_assembled_position("base")
    # controller.publish_part_in_assembled_position("panel_bearing")
    # controller.publish_part_in_assembled_position("panel_motor")
    # controller.publish_part_in_assembled_position("bearing")
    # controller.subtask_c2()

    # rotation = np.deg2rad([-22.5+180, -88.5, -157.5]).tolist()  # Arbitrary
    # above_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.0, 0.002, -0.10] + rotation)
    # controller.confirm_to_proceed("boo")
    # controller.b_bot.go_to_pose_goal(above_pose, speed=0.2, move_lin=False)

    # controller.pick_and_insert_motor_pulley("taskboard")

    # controller.b_bot.move_lin_rel(relative_translation=[-0.03,0,0], relative_to_robot_base=True)

    # st = rospy.get_time()
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # controller.b_bot.go_to_pose_goal(approach_vgroove, speed=1.0)

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # controller.b_bot.go_to_pose_goal(approach_vgroove, speed=1.0)
   
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, -0.2, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # controller.a_bot.go_to_pose_goal(approach_vgroove, speed=1.0)    

    # rospy.logwarn("TOTAL TIME!! %f" % (st-rospy.get_time()))

    ############# with planning ##########3
    # st = rospy.get_time()
    # start_time = rospy.get_time()
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # plan, planning_time = controller.b_bot.plan_goal_pose(approach_vgroove, speed=1.0)
    # duration = helpers.get_trajectory_duration(plan)
    # print("b_bot duration", duration)
    # if not controller.b_bot.execute_plan(plan, wait=False):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False

    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))
    # rospy.loginfo("waiting for second motion")

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # plan, planning_time = controller.b_bot.plan_goal_pose(approach_vgroove, initial_pose=helpers.get_trajectory_joint_goal(plan),speed=1.0)
    # print("b_bot planning_time", planning_time)

    # waiting_time = duration - planning_time if duration - planning_time > 0 else 0.0
    # print("b_bot waiting time", waiting_time)
    # rospy.sleep(waiting_time)
    
    # start_time = rospy.get_time()
    # if not controller.b_bot.execute_plan(plan):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False
    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, -0.2, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # plan, planning_time = controller.a_bot.plan_goal_pose(approach_vgroove, speed=1.0)
    # print("a_bot planning_time", planning_time)

    # waiting_time = duration - planning_time if duration - planning_time > 0 else 0.0
    # print("a_bot waiting time", waiting_time)
    # rospy.sleep(waiting_time)
    
    # start_time = rospy.get_time()
    # if not controller.a_bot.execute_plan(plan):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False
    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))

    # rospy.logwarn("TOTAL TIME!! %f" % (st-rospy.get_time()))
    ###### traj ##
    # p1 = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # p2 = conversions.to_pose_stamped("tray_center", [0.05, 0.1, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # p3 = conversions.to_pose_stamped("tray_center", [0.1, 0.0, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # controller.b_bot.plan_linear_trajectory([[p1, 0.0],[p2, 0.0],[p3, 0.0]])
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # if not controller.a_bot.go_to_pose_goal(approach_vgroove, speed=0.2, move_lin=False):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False

    # controller.a_bot.linear_push(3, "-Z", max_translation=0.1, timeout=10.0)

    # controller.assembly_database.change_assembly('wrs_assembly_2020')
    # controller.reset_scene_and_robots()
    # controller.subtask_c2()
    # controller.orient_shaft()
    # controller.orient_shaft_end_cap()

    # controller.check_motor_pulley_angle()
    # controller.check_bearing_angle()

    # controller.turn_shaft_until_groove_found()
    
    # controller.panel_subtask2()


    # controller.insert_shaft("taskboard_assy_part_07_inserted")

    # controller.playback_sequence(routine_filename="motor_pulley_orient")
    # controller.insert_motor_pulley("taskboard_small_shaft")
    # controller.assembly_database.change_assembly('taskboard')
    # controller.pick_and_insert_motor_pulley("taskboard")

    # controller.pick_and_insert_motor_pulley("taskboard")
    # controller.insert_motor_pulley("taskboard_small_shaft")
    # target_force = get_target_force("+X", 2)
    # print(np.array(target_force == 0.0) )
    # controller.playback_sequence("idler_pulley_release_screw_tool")

    # controller.b_bot.go_to_named_pose("screw_ready")
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # controller.pick_and_insert_shaft("taskboard")

    # controller.playback_sequence("idler_pulley_equip_nut_tool")
    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    # controller.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.4)
    # controller.playback_sequence("idler_pulley_unequip_nut_tool")
    

    # controller.playback_sequence("idler_pulley_release_screw_tool")
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # controller.prepare_screw_tool_idler_pulley("taskboard_long_hole_top_link")

    # controller.unequip_tool("b_bot", "padless_tool_m4")
    # controller.b_bot.go_to_named_pose("screw_ready")
    # controller.playback_sequence("idler_pulley_push_with_screw_tool")

    # controller.b_bot.move_lin_rel(relative_translation=[0.1, 0, 0], speed=0.2)
    # controller.b_bot.move_lin_rel(relative_translation=[0.0, 0, 0.25], speed=0.2)
    # controller.unequip_tool("b_bot", "padless_tool_m4")

    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.06, 0.0, 0.0, 0.0, 0.0, 0.0])
    # controller.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.012, 0.0, 0.0, 0.0, 0.0, 0.0])
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
    # fake_vision_pose = conversions.to_pose_stamped("tray_center", [-0.03078, 0.06248, 0.02, 0.0,0.7071,0.0,0.7071])

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
