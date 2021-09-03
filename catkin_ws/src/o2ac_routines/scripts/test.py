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
    c = O2ACAssembly()
    c.reset_scene_and_robots()
    c.confirm_to_proceed("start?")
    c.subtask_c2(simultaneous_execution=True)
    # c.pick_end_cap()
    # c.orient_shaft_end_cap("a_bot")
    # c.set_assembly("wrs_assembly_2019_surprise")
    # c.flip_motor_in_aid()
    # c.publish_part_in_assembled_position("panel_motor")
    # c.publish_part_in_assembled_position("motor", marker_only=True)
    # c.confirm_to_proceed("ok?")
    # c.align_motor_pre_insertion()
    # c.insert_motor("assembled_part_02_back_hole")
    # c.set_assembly()
    # c.pick_panel_with_handover()
    # c.center_panel("panel_bearing")
    # c.place_panel("a_bot", "panel_bearing", fake_position=True)
    # c.fasten_panel("panel_bearing", unequip_tool_on_success=True)
    # c.publish_part_in_assembled_position("base")
    # c.pick_motor_pulley(robot_name="a_bot")
    # c.orient_motor_pulley(target_link="assembled_part_05_center", robot_name="a_bot")
    # c.insert_motor_pulley("assembled_part_05_center", robot_name="a_bot")
    # c.fasten_motor_pulley("assembled_part_05_center")
    # c.align_motor_pre_insertion()
    # c.insert_motor("assembled_part_02_back_hole")
    # c.place_panel("a_bot", "panel_motor", fake_position=True)
    # c.fasten_panel("panel_motor", unequip_tool_on_success=True)
    # c.prepare_screw_tool_idler_pulley("taskboard_long_hole_top_link")
    # c.equip_nut_tool()
    # c.confirm_to_proceed(">>>")
    # def a_bot_task2():
    #     rospy.sleep(2)
    #     c.a_success = c.unequip_nut_tool()
    #     return c.a_success
    # def b_bot_task2():
    #   # rospy.sleep(10)
    #   c.b_success = c.return_padless_tool_idler_pulley()
    #   return c.b_success
    # c.do_tasks_simultaneous(a_bot_task2, b_bot_task2, timeout=60)
    # c.return_padless_tool_idler_pulley()
    # c = O2ACAssembly()
    # c.confirm_motor_and_place_in_aid()
    # c.align_motor_pre_insertion()
    # c.b_bot.gripper.close()
    # c.b_bot.move_lin_rel([0,0,0.01], speed=0.1)
    # c.b_bot.move_lin_rel(relative_rotation=[tau/4, 0, 0.0], speed=0.1)
    # c.b_bot.move_lin_rel([0,0,0.01], speed=0.1)
    # c.b_bot.gripper.open()

    # c.reset_scene_and_robots()
    # c.pick_bearing("a_bot")
    # c.orient_bearing("taskboard", "a_bot", part1=True)
    # grasp_pose = conversions.to_pose_stamped("taskboard_bearing_target_link", [-0.155, 0.005, 0.0, -.5, .5, -.5, .5])
    # c.a_bot.go_to_pose_goal(grasp_pose, speed=0.1, end_effector_link="a_bot_outside_camera_color_optical_frame", move_lin=False)
    # approach_centering = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.1, 0, -0.15,  np.deg2rad(180), np.deg2rad(-60), 0])
    # c.b_bot.go_to_pose_goal(approach_centering, speed=0.05, end_effector_link="b_bot_outside_camera_link", move_lin=False)
    # c.pick_bearing("a_bot")
    # c.publish_part_in_assembled_position("panel_bearing")
    # st = rospy.get_time()
    # c.fasten_motor(part1=False, part2=True)
    # print("Duration:", rospy.get_time()-st)
    # # c.subtask_zero()
    # c.align_motor_pre_insertion()
    # c.insert_motor("assembled_part_02_back_hole")
    # c.orient_bearing("assembly", "a_bot", part1=True, part2=True)
    # c.insert_bearing("assembled_part_07_inserted", robot_name="a_bot")
    # c.b_bot.gripper.open(opening_width=0.0105)
    # c.align_bearing_spacer_pre_insertion("a_bot")
    # c.orient_bearing_spacer("a_bot")
    # c.set_assembly(assembly_name="wrs_assembly_2020")
    # c.subtask_b(simultaneous_execution=False)
    # c.fasten_bearing("assembly", robot_name="b_bot", with_extra_retighten="True")
    # c.pick_shaft()
    # c.orient_shaft()
    # c.subtask_c2()
    # c.orient_shaft_end_cap()
    # c.pick_shaft()
    # c.pick_shaft()
    # c.subtask_b(simultaneous_execution=True)
#     a = np.array([	
# 54380492,
# 2.566528,
# 0.802040,
# 2.566528,
# 73519448,
# 5042046,
# 0.802040,
# 5042046,
# 123943176 
#     ])
#     print(a/(1000.0)**5)
    # c = O2ACTaskboard()
    # c.b_bot.go_to_pose_goal(c.at_set_screw_hole, speed=0.01, end_effector_link="b_bot_set_screw_tool_tip_link")
    # c.b_bot.go_to_named_pose("home")
    # c.a_bot.go_to_named_pose("home")
    # c.equip_tool("b_bot", "set_screw_tool")
    # c.move_b_bot_to_setscrew_initial_pos()
    # c.do_task("M2 set screw")
    # c.do_task("M3 screw", fake_execution_for_calibration=False, simultaneous=False)
    # rospy.set_param("grasp_plugin", True)
    # c.reset_scene_and_robots()
    # # c.orient_bearing("assembly", "a_bot", part1=False, part2=True)
    # # motor_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.01, -0.02, -0.005, tau/2,0,tau/4])
    # # c.markers_scene.spawn_item("motor", motor_pose)
    # c.confirm_motor_and_place_in_aid()
    # c.align_motor_pre_insertion()

    # c = O2ACCommon()
    # c.a_bot.go_to_named_pose("home")
    # c.allow_collisions_with_robot_hand("front_bar", "a_bot")
    # c.allow_collisions_with_robot_hand("base_fixture_top", "a_bot")
    # c.nut_tool_used = True
    # c.equip_nut_tool()
    # c.unequip_nut_tool()
    # c.equip_nut_tool()
    # c.unequip_nut_tool()
    # c.a_bot.gripper.open()
    # c.a_bot.move_lin_rel([0,-0.005,0])
    # c.spawn_tool("nut_holder")
    # above_nut_aid = [1.7012, -1.9212, 1.788, -1.4422, -1.5809, 3.2784]
    # c.a_bot.move_joints(above_nut_aid)
    # motor_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [0.0, 0, 0, 0, 0, 0])
    # c.simple_place("b_bot", motor_pose, place_height=0.0, speed_fast=1.0, speed_slow=0.3, approach_height=0.1, axis="x", sign=-1)
    # c.reset_scene_and_robots()
    # c.pick_idler_spacer()
    # c.reset_scene_and_robots()
    # c.pick_idler_pin()
    # c.reset_scene_and_robots()
    # c.pick_idler_pulley_assembly()
    # c.playback_sequence("bearing_orient_a_bot", use_saved_plans=False)
    # c.reset_scene_and_robots()
    # # c.orient_bearing("assembly", "a_bot", part1=False, part2=True)
    # # motor_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.01, -0.02, -0.005, tau/2,0,tau/4])
    # # c.markers_scene.spawn_item("motor", motor_pose)
    # c.confirm_motor_and_place_in_aid()
    # c.align_motor_pre_insertion()

    # # c.b_bot.gripper.gripper.release(link_name="panel_bearing_tmp::panel_bearing")
    # c.ab_bot.go_to_named_pose("home")
    # c.get_large_item_position_from_top("panel_motor")
    # pose = conversions.to_pose_stamped("tray_center", [0.09, 0.037, 0.015, 0, tau/4, 0])
    # c.b_bot.go_to_pose_goal(pose)
    # pose = conversions.to_pose_stamped("tray_center", [-0.024, -0.108, 0.006, 0.707, -0.010, -0.707, 0.003])
    # c.a_bot.go_to_pose_goal(pose)
    # spawner = GazeboModels('o2ac_gazebo')]
    # name = "panel_bearing"
    # object_pose = conversions.to_pose_stamped("tray_center", [0,0,0,0,0,0])
    # object_pose = c.listener.transformPose("world", object_pose)
    # op = conversions.from_pose_to_list(object_pose.pose)
    # objpose = [op[:3], op[3:]] 
    # models = [Model(name, objpose[0], orientation=objpose[1], reference_frame="world")]
    # c.gazebo_scene.load_models(models,)
    # c.b_bot.gripper.gripper.grab(link_name="panel_bearing_tmp::panel_bearing")
    # c.orient_motor_pulley("taskboard_small_shaft")
    # c.insert_motor_pulley("taskboard_small_shaft")
    # c.prepare_screw_tool_idler_pulley("taskboard_long_hole_top_link", simultaneous=True)
    # c.reset_scene_and_robots()
    # c.b_bot.go_to_named_pose("home")
    # c.centering_shaft()
    # c.align_shaft("taskboard_assy_part_07_inserted")
    # c.b_bot.go_to_pose_goal(pre_insertion_pose, move_lin=False)
    
    # c.equip_tool("b_bot", "set_screw_tool")
    # c.b_bot.go_to_named_pose("horizontal_screw_ready")
    # c.move_b_bot_to_setscrew_initial_pos()
    # c.do_task("M2 set screw")
    # # c.a_bot.move_lin_rel(relative_translation = [0.01,0,0], acceleration = 0.015, speed=.03)
    # c.ab_bot.go_to_named_pose("home")
    # # c.playback_sequence("bearing_orient_a_bot", plan_while_moving=False, use_saved_plans=False)
    # c.pick_bearing("a_bot")
    # c.orient_bearing("taskboard", robot_name="a_bot", part1=True, part2=False)
    # c.orient_bearing("taskboard", robot_name="a_bot", part1=False, part2=True)
    # c.insert_bearing("taskboard_bearing_target_link", robot_name="a_bot")
    # c = O2ACCommon()
    # c.publish_status_text("O2AC Go! <3")
    # c.allow_collisions_with_robot_hand("agv_table", "a_bot")
    # c.planning_scene_interface.allow_collisions("agv_table")
    # c.playback_sequence("bearing_orient_a_bot", use_saved_plans=False`)
    print("a_bot", np.round(c.a_bot.robot_group.get_current_joint_values(), 5).tolist())
    print("b_bot", np.round(c.b_bot.robot_group.get_current_joint_values(), 5).tolist())
    # pose = conversions.to_pose_stamped("left_centering_link", [-0.02, 0, 0.02, radians(-30), 0, 0])
    # c.a_bot.go_to_pose_goal(pose)
    # c.a_bot.move_lin_rel(relative_translation=[0,0,0.015], relative_rotation=[0, radians(35), 0], relative_to_tcp=True)

    
    # c.do_change_tool_action("b_bot", equip=False, screw_size = 4)
    # c = O2ACAssembly()
    # c.reset_scene_and_robots()
    # c.get_large_item_position_from_top("motor")

    # c.publish_part_in_assembled_position("panel_bearing")
    # c.publish_part_in_assembled_position("panel_motor")
    # c.publish_part_in_assembled_position("base")
    # c.fasten_motor()
    # c.fasten_motor_fallback()
    # c = O2ACAssembly()
    # c.reset_scene_and_robots()
    # c.publish_part_in_assembled_position("panel_bearing")
    # c.publish_part_in_assembled_position("panel_motor")
    # c.publish_part_in_assembled_position("base")
    # c.confirm_motor_and_place_in_aid()
    # c.center_panel_on_base_plate("panel_motor")
    # inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.0, 0.002, -0.005, tau/2., radians(3.0), radians(30)])
    # c.b_bot.go_to_pose_goal(inside_vgroove)

    # c.confirm_to_proceed("al??")
    # c.align_motor_pre_insertion()
    # c.confirm_to_proceed("insert??")
    # c.insert_motor("assembled_part_02_back_hole")
    # c.confirm_to_proceed("fastem??")
    # c.fasten_motor()
    # c.confirm_to_proceed("fasten??")
    # c.b_bot.gripper.open()
    # c.b_bot.move_lin_rel(relative_translation=[-0.02,0.1,0.1])
    # c.b_bot.go_to_named_pose("home")
    # print("result", c.b_bot.load_and_execute_program(program_name="wrs2020/motor_orient.urp"))

    # pose = conversions.to_pose_stamped("assembled_part_02_back_hole", [-0.022, -0.004, -0.014, -0.098, 0.702, -0.086, 0.700])
    # c.b_bot.go_to_pose_goal(pose)
    # tc = None
    # selection_matrix = [1., 1., 1., 0.7, 1., 1.]
    # c.b_bot.execute_spiral_trajectory("XY", max_radius=0, radius_direction="+Y", steps=50,
    #                                      revolutions=1, target_force=0, check_displacement_time=10,
    #                     
    #                  wiggle_direction="Z", wiggle_angle=radians(10.0), wiggle_revolutions=1.0,
    #                                      termination_criteria=tc, timeout=20, selection_matrix=selection_matrix)
    # c.place_panel("a_bot", "panel_motor", pick_again=True, fake_position=True)
    # c.fasten_panel("panel_motor", True)
    # c.center_panel_on_base_plate("panel_bearing")
    # push_pose     = conversions.to_pose_stamped("assembled_part_03_bottom_corner_2", [-0.01, 0.0, 0.015, 0, 0, 0])
    # c.a_bot.go_to_pose_goal(push_pose)

    # c.set_assembly("wrs_assembly_2021")
    # c.get_large_item_position_from_top("panel_motor")
    # c.get_large_item_position_from_top("panel_bearing")
    
    # cp = c.listener.transformPose("tray_center", c.a_bot.get_current_pose_stamped())
    # print("a bot pose", cp.pose.position)
    # grasp_pose = c.get_transformed_grasp_pose("base", "terminal_grasp", target_frame="tray_center")
    # print("grasp pose", grasp_pose.pose.position)
    # current_base_pose = c.get_transformed_collision_object_pose("base", target_frame="tray_center")
    # current_base_pose.pose.position.x += cp.pose.position.x - grasp_pose.pose.position.x
    # current_base_pose.pose.position.y += cp.pose.position.y - grasp_pose.pose.position.y
    # c.update_collision_item_pose("base", current_base_pose)

    # c.reset_scene_and_robots()
    # c.ab_bot.go_to_named_pose("home")
    # c.place_panel("a_bot", "panel_bearing", fake_position=True)
    # c.despawn_object("panel_bearing")
    # c.fasten_panel("panel_bearing")
    # c.do_change_tool_action("b_bot", equip=False, screw_size = 4)
    # c.panels_assembly()
    # c.subtask_zero(use_b_bot_camera=True)
    # c.a_bot.gripper.open()
    # c.pick_panel("panel_bearing")
    # c.center_panel("panel_bearing")
    # c.confirm_to_proceed("1??")
    # c.ab_bot.go_to_named_pose("home")
    # c.pick_panel("panel_motor")
    # pose = c.center_panel("panel_bearing")
    # c.place_panel("a_bot", "panel_bearing", grasp_pose=pose)
    # c.b_bot.go_to_named_pose("feeder_pick_ready")
    # c.confirm_to_proceed("2??")
    
    # c.ab_bot.go_to_named_pose("home")
    # c.reset_scene_and_robots()
    # c.get_large_item_position_from_top("base", "b_bot")
    # c.confirm_to_proceed("1")
    # c.pick_base_panel(skip_initial_perception=True)

    # c = O2ACCommon()
    # handover_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.1, 0.2, -tau/4, tau/8, -tau/4])
    # c.b_bot.go_to_pose_goal(handover_pose)
    # c.a_bot.gripper.open()
    # c.a_bot.gripper.close()
    # c.confirm_to_proceed("")
    # c.a_bot.gripper.open(velocity=1.0)
    # c.a_bot.gripper.close(velocity=1.0)
    # print(c.a_bot.robot_group.get_current_joint_values())
    # pre_hand_over_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.25, -tau/4, 0, tau/4])
    # c.a_bot.go_to_pose_goal(pre_hand_over_pose, 0.5, move_lin=False)
    # c.update_collision_item_pose("panel_bearing", conversions.to_pose_stamped("left_centering_link", [0.000, 0.070, -0.080, -0.500, 0.500, -0.500, -0.500]))
    # c = O2ACAssembly()
    # c.a_bot.gripper.open()
    # c.ab_bot.go_to_named_pose("home")
    # c.reset_scene_and_robots()
    # # c.publish_part_in_assembled_position("base")
    # # c.place_panel("a_bot", "panel_bearing")
    # # c.fasten_panel("panel_bearing")
    # c.pick_panel("panel_bearing")
    # c.center_panel("panel_bearing")
    # c.confirm_to_proceed("")
    # c.subtask_zero()
    # gp = c.panel_subtask2("panel_bearing")
    # c.place_panel("a_bot", "panel_motor", grasp_pose=None)
    # TODO:
    # 1. add the incline hold pose for fastening
    # 2. create the simultaneous task for picking,centering the panels, then picking the base plate and then fastening
    # c.fasten_panel("panel_bearing")
    # c = O2ACCommon()
    # c.reset_scene_and_robots()
    # c.ab_bot.go_to_named_pose("home")
    # c.competition_mode = True
    # c.reset_scene_and_robots()
    # c.center_tray_stack(True, False)
    # c.spawn_tray_stack(stack_center=[-0.03, 0.0], tray_heights=[0.05, 0.0], orientation_parallel=True)
    # c.pick_tray_from_agv_stack_calibration_long_side("tray1")
    # c.return_tray_to_agv_stack_calibration_long_side("tray1")
    # c.pick_tray_from_agv_stack_calibration_long_side("tray2")
    # c.return_tray_to_agv_stack_calibration_long_side("tray2")
    # q = [0.19453047870641985, -1.414852203750135, 1.7189982978203278, -1.8730085962497756, -1.567615792398194, 1.7666671594586172, 2.453019687833446, -1.3449819279957274, 1.8068236138946376, -2.0324751409170405, -1.5648680786549045, -2.2588772450321226]
    # c.ab_bot.move_joints(q, speed=1.0)
    # q = [0.19502419962975553, -1.302128082437928, 1.886556773525712, -2.1543437146483777, -1.568541929253542, 1.766640025829011, 2.4522167306633054, -1.2014465832104217, 1.9290941675252034, -2.298411409316249, -1.5642835257198011, -2.2577476650719626]
    # c.ab_bot.move_joints(q, speed=1.0)
    # c.confirm_to_proceed("")
    # c.spawn_tray_stack(stack_center=[0.05, 0.14], tray_heights=[0.075,0.02], orientation_parallel=False)
    # c.pick_tray_from_agv_stack_calibration_short_side("tray1")
    # c.pick_tray_from_agv_stack_calibration_short_side()
    ###### insert motor ######
    # c.publish_part_in_assembled_position("base")
    # c.publish_part_in_assembled_position("panel_bearing")
    # c.publish_part_in_assembled_position("panel_motor")
    # c.b_bot.move_lin_rel(relative_translation=[0,0,0.1])
    # c.ab_bot.go_to_named_pose("home")
    # c.do_change_tool_action("a_bot", equip=True, screw_size=3)
    # c.a_bot.go_to_named_pose("screw_ready")

    # above_vgroove  =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.2, 0, 0, tau/2., 0, 0])
    # if not c.b_bot.go_to_pose_goal(above_vgroove, speed=0.4, move_lin=False):
    #   rospy.logerr("fail to go above vgroove")
    #   return False
    # c.b_bot.gripper.send_command(0.05)
    # if not c.align_motor_pre_insertion():
    #     return False
    # c.confirm_to_proceed("finetune")
    # c.insert_motor("assembled_part_02_back_hole")

    # c.confirm_to_proceed("move")
    # selection_matrix = [1., 1., 1., 0.8, 1., 1.]
    # c.b_bot.execute_spiral_trajectory("XY", max_radius=0.0, radius_direction="+Y", steps=50,
    #                                             revolutions=1, target_force=0, check_displacement_time=10,
    #                                             wiggle_direction="Z", wiggle_angle=radians(15.0), wiggle_revolutions=2.0,
    #                                             termination_criteria=None, timeout=10, selection_matrix=selection_matrix)
    
    ### fasten motor ####
    # c.confirm_to_proceed("fasten?")
    # c.do_change_tool_action("a_bot", equip=True, screw_size=3)
    # c.fasten_motor()


    ###################################

    # c = O2ACCommon()
    # c.a_bot.go_to_named_pose("screw_ready")
    # c.reset_scene_and_robots()
    # c.b_bot.go_to_named_pose("home")
    # c.take_tray_from_agv_preplanned(save_on_success=True, use_saved_plans=False)
    # c.fasten_bearing("assembly", only_retighten=True)
    # c.b_bot.go_to_named_pose("home")

    # c.allow_collisions_with_robot_hand("tray", "a_bot")
    # c.allow_collisions_with_robot_hand("tray_center", "a_bot")
    
    # c.playback_sequence("bearing_orient_down_b_bot")

    # robot_name = "b_bot"
    # task = "assembly"
    # bearing_target_link = "assembled_part_07_inserted"
    # prefix = "right" if robot_name == "b_bot" else "left"
    # at_tray_border_pose = conversions.to_pose_stamped(prefix + "_centering_link", [-0.15, 0, 0.10, np.radians(-100), 0, 0])
    # print(at_tray_border_pose)

    # rotation = [0, np.radians(-35.0), 0] if robot_name == "b_bot" else [tau/4, 0, np.radians(-35.0)]
    # approach_pose       = conversions.to_pose_stamped(bearing_target_link, [-0.050, -0.001, 0.005] + rotation)
    # if task == "taskboard":
    #   preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.017,  0.000, 0.002 ]+ rotation)
    # elif task == "assembly":
    #   preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.017, -0.000, 0.006] + rotation)

    # trajectory = helpers.to_sequence_trajectory([at_tray_border_pose, approach_pose, preinsertion_pose], blend_radiuses=[0.01,0.02,0], speed=0.4)
    # if not c.execute_sequence(robot_name, [trajectory], "go to preinsertion", plan_while_moving=False):
    #   rospy.logerr("Could not go to preinsertion")

    
    
    # # tool_pull_pose = conversions.to_pose_stamped("move_group/panel_motor", [0.03, 0.038, 0.0, 0, 0, 0])
    # tool_pull_pose = conversions.to_pose_stamped("move_group/panel_bearing/front_hole", [0.0, 0.0, 0.0, 0, 0, 0])
    # c.move_towards_center_with_tool("b_bot", target_pose=tool_pull_pose, distance=0.05, start_with_spiral=True)
    

    # # c = O2ACCommon()
    # # test_force_control(c)

    # a.start()
    # # rospy.sleep(10)
    # b.start()

    # a.join(30)
    # b.join(30)

    # if a.is_alive():
    #     a.kill()
    #     b.kill()
    
    # if b.is_alive():
    #     b.kill()

    # c = O2ACTaskboard()
    # c.reset_scene_and_robots()
    # c.ab_bot.go_to_named_pose("home")
    # c.equip_tool("a_bot", "screw_tool_m3")
    # c.equip_tool("b_bot", "screw_tool_m4")
    # a = threading.Thread(target=a_bot_m3, args=(c,))
    # a.daemon = True
    # b = threading.Thread(target=b_bot_m4, args=(c,))
    # b.daemon = True

    # a.start()
    # rospy.sleep(10)
    # b.start()

    # rospy.spin()

    # =====

    # c.b_bot.go_to_named_pose("screw_ready")
    # c.playback_sequence("idler_pulley_ready_screw_tool")
    # c.confirm_to_proceed("")
    # c.playback_sequence("idler_pulley_return_screw_tool")
    # c.b_bot.go_to_named_pose("screw_ready")

    # c.b_bot.gripper.open()
    # c.b_bot.go_to_named_pose("home")
    # at_object_pose = conversions.to_pose_stamped("right_centering_link", [-0.005, 0, 0.0, 0, 0, 0] )
    # c.b_bot.go_to_pose_goal(at_object_pose, speed=0.5)
    # c.center_with_gripper("b_bot", opening_width=0.085)
    
    # c.vision.activate_camera("b_bot_inside_camera")
    # look_at_output_pulley = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.212, 0, 0.0, -tau/4, 0, -tau/4])
    # c.b_bot.go_to_pose_goal(look_at_output_pulley, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=0.05)

    # c.assembly_database.change_assembly("taskboard")
    # c.pick_bearing()
    # c.orient_bearing("taskboard")
    # c = O2ACAssembly()
    # c.playback_sequence(routine_filename="plunger_tool_equip", plan_while_moving=True, use_saved_plans=False)
    # c.playback_sequence(routine_filename="plunger_tool_unequip", plan_while_moving=True, use_saved_plans=False)
    # c.playback_sequence(routine_filename="idler_pulley_return_screw_tool", plan_while_moving=True)

    # c.take_tray_from_agv(preplanned=False)
    # c.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    # c.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    # c.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    # c.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)
    # c.take_tray_from_agv_preplanned()
    # c.playback_sequence("tray_take_from_agv", plan_while_moving=True, save_on_success=True, use_saved_plans=True)
    # c.test_ik()
    # c.reset_scene_and_robots()
    # c.fasten_bearing("assembly")
    # c.tools.set_motor("padless_tool_m4", "tighten", duration=2)
    # c.take_tray_from_agv()
    # c.confirm_to_proceed("")
    # c.publish_part_in_assembled_position("base")
    # c.publish_part_in_assembled_position("panel_bearing")
    # c.subtask_e()
    # b_plan, _ = c.b_bot.go_to_named_pose("screw_ready", wait=False, plan_only=True)
    # c.b_bot.execute_plan(b_plan, wait=False)
    # rospy.sleep(1)
    # a_plan, _ = c.a_bot.go_to_named_pose("screw_ready", wait=False, plan_only=True)
    # c.a_bot.execute_plan(a_plan, wait=False)
    # c.a_bot.go_to_named_pose("screw_ready")

    # c.assembly_database.change_assembly('taskboard')
    # c.fallback_b_bot_outside_tray_centering()
    # c.reset_scene_and_robots()

    # c.fasten_bearing("taskboard")

    # print(c.b_bot.robot_group.get_current_joint_values())

    # return
    # c.b_bot.move_joints([1.8108925819396973, -2.119059225121969, 1.8984859625445765, -1.3372403693250199,
    # -1.780367676411764, 1.1874641180038452])
    # c.b_bot.move_joints([1.444, -1.61 ,  2.504, -1.964, -1.2  , -2.578])
    

    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # st = rospy.get_time()
    # print("TIME old", rospy.get_time()-st)
    
    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # c.b_bot.gripper.open()
    # st = rospy.get_time()
    # c.playback_sequence(routine_filename="idler_pulley_equip_nut_tool")
    # c.playback_sequence(routine_filename="return_screw_tool_horizontal")
    # print("TIME new", rospy.get_time()-st)

    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # c.b_bot.gripper.open()
    # st = rospy.get_time()
    # c.playback_sequence(routine_filename="ready_screw_tool_horizontal", wait=False)
    # print("TIME new", rospy.get_time()-st)
    
    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # # plan, _ = c.b_bot.plan_relative_goal(relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = c.b_bot.plan_relative_goal(relative_rotation=[0,np.deg2rad(-15),0], relative_to_tcp=True)
    # c.b_bot.execute_plan(plan, wait=True)
    
    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # q = c.b_bot.robot_group.get_current_joint_values()
    # # plan, _ = c.b_bot.plan_relative_goal(initial_joints=q, relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = c.b_bot.plan_relative_goal(relative_rotation=[0,np.deg2rad(-15),0], relative_to_tcp=True, initial_joints=q)
    # c.b_bot.execute_plan(plan, wait=True)
    
    # c.b_bot.go_to_named_pose("home", speed=1.0)
    # q = c.b_bot.robot_group.get_current_joint_values()
    # # plan, _ = c.b_bot.plan_relative_goal(initial_joints=q, relative_translation=[0.05,0.0,0.0], relative_to_tcp=True)
    # plan, _ = c.b_bot.plan_relative_goal(relative_rotation=[np.deg2rad(-15),0,0], relative_to_tcp=True, initial_joints=q)
    # c.b_bot.execute_plan(plan, wait=True)

    # c = O2ACAssembly()

    # c.reset_scene_and_robots()
    # c.publish_part_in_assembled_position("base")
    # c.publish_part_in_assembled_position("panel_bearing")
    # c.publish_part_in_assembled_position("panel_motor")
    # c.publish_part_in_assembled_position("bearing")
    # c.subtask_c2()

    # rotation = np.deg2rad([-22.5+180, -88.5, -157.5]).tolist()  # Arbitrary
    # above_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.0, 0.002, -0.10] + rotation)
    # c.confirm_to_proceed("boo")
    # c.b_bot.go_to_pose_goal(above_pose, speed=0.2, move_lin=False)

    # c.pick_and_insert_motor_pulley("taskboard")

    # c.b_bot.move_lin_rel(relative_translation=[-0.03,0,0], relative_to_robot_base=True)

    # st = rospy.get_time()
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # c.b_bot.go_to_pose_goal(approach_vgroove, speed=1.0)

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # c.b_bot.go_to_pose_goal(approach_vgroove, speed=1.0)
   
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, -0.2, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # c.a_bot.go_to_pose_goal(approach_vgroove, speed=1.0)    

    # rospy.logwarn("TOTAL TIME!! %f" % (st-rospy.get_time()))

    ############# with planning ##########3
    # st = rospy.get_time()
    # start_time = rospy.get_time()
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # plan, planning_time = c.b_bot.plan_goal_pose(approach_vgroove, speed=1.0)
    # duration = helpers.get_trajectory_duration(plan)
    # print("b_bot duration", duration)
    # if not c.b_bot.execute_plan(plan, wait=False):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False

    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))
    # rospy.loginfo("waiting for second motion")

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # plan, planning_time = c.b_bot.plan_goal_pose(approach_vgroove, initial_pose=helpers.get_trajectory_joint_goal(plan),speed=1.0)
    # print("b_bot planning_time", planning_time)

    # waiting_time = duration - planning_time if duration - planning_time > 0 else 0.0
    # print("b_bot waiting time", waiting_time)
    # rospy.sleep(waiting_time)
    
    # start_time = rospy.get_time()
    # if not c.b_bot.execute_plan(plan):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False
    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))

    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, -0.2, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # plan, planning_time = c.a_bot.plan_goal_pose(approach_vgroove, speed=1.0)
    # print("a_bot planning_time", planning_time)

    # waiting_time = duration - planning_time if duration - planning_time > 0 else 0.0
    # print("a_bot waiting time", waiting_time)
    # rospy.sleep(waiting_time)
    
    # start_time = rospy.get_time()
    # if not c.a_bot.execute_plan(plan):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False
    # rospy.logwarn("TIME!! %f" % (start_time-rospy.get_time()))

    # rospy.logwarn("TOTAL TIME!! %f" % (st-rospy.get_time()))
    ###### traj ##
    # p1 = conversions.to_pose_stamped("tray_center", [0.0, 0.2, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # p2 = conversions.to_pose_stamped("tray_center", [0.05, 0.1, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # p3 = conversions.to_pose_stamped("tray_center", [0.1, 0.0, 0.3]+np.deg2rad([-180, 90, 90]).tolist())
    # c.b_bot.plan_linear_trajectory([[p1, 0.0],[p2, 0.0],[p3, 0.0]])
    # approach_vgroove = conversions.to_pose_stamped("tray_center", [0.0, 0, 0.3]+np.deg2rad([-180, 90, -90]).tolist())
    # if not c.a_bot.go_to_pose_goal(approach_vgroove, speed=0.2, move_lin=False):
    #   rospy.logerr("Fail to go to approach_vgroove")
    #   return False

    # c.a_bot.linear_push(3, "-Z", max_translation=0.1, timeout=10.0)

    # c.assembly_database.change_assembly('wrs_assembly_2020')
    # c.reset_scene_and_robots()
    # c.subtask_c2()
    # c.orient_shaft()
    # c.orient_shaft_end_cap()

    # c.check_motor_pulley_angle()
    # c.check_bearing_angle()

    # c.turn_shaft_until_groove_found()
    
    # c.panel_subtask2()


    # c.insert_shaft("taskboard_assy_part_07_inserted")

    # c.playback_sequence(routine_filename="motor_pulley_orient")
    # c.insert_motor_pulley("taskboard_small_shaft")
    # c.assembly_database.change_assembly('taskboard')
    # c.pick_and_insert_motor_pulley("taskboard")

    # c.pick_and_insert_motor_pulley("taskboard")
    # c.insert_motor_pulley("taskboard_small_shaft")
    # target_force = get_target_force("+X", 2)
    # print(np.array(target_force == 0.0) )
    # c.playback_sequence("idler_pulley_release_screw_tool")

    # c.b_bot.go_to_named_pose("screw_ready")
    # c.unequip_tool("b_bot", "padless_tool_m4")

    # c.pick_and_insert_shaft("taskboard")

    # c.playback_sequence("idler_pulley_equip_nut_tool")
    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    # c.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.4)
    # c.playback_sequence("idler_pulley_unequip_nut_tool")
    

    # c.playback_sequence("idler_pulley_release_screw_tool")
    # c.unequip_tool("b_bot", "padless_tool_m4")

    # c.prepare_screw_tool_idler_pulley("taskboard_long_hole_top_link")

    # c.unequip_tool("b_bot", "padless_tool_m4")
    # c.b_bot.go_to_named_pose("screw_ready")
    # c.playback_sequence("idler_pulley_push_with_screw_tool")

    # c.b_bot.move_lin_rel(relative_translation=[0.1, 0, 0], speed=0.2)
    # c.b_bot.move_lin_rel(relative_translation=[0.0, 0, 0.25], speed=0.2)
    # c.unequip_tool("b_bot", "padless_tool_m4")

    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.06, 0.0, 0.0, 0.0, 0.0, 0.0])
    # c.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
    # approach_pose = conversions.to_pose_stamped("taskboard_long_hole_middle_link", [0.012, 0.0, 0.0, 0.0, 0.0, 0.0])
    # c.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
    # c.b_bot.move_lin_rel(relative_translation=[0, 0, -0.015], speed=0.2)
    # c.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.15], relative_to_robot_base=True)
    # c.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.1], relative_rotation=[0,0,0], relative_to_robot_base=True)
    # c.a_bot.move_lin_rel(relative_translation=[0.0, -0.1, 0.], relative_rotation=[0,0, np.deg2rad(45)], relative_to_robot_base=True)
    # c.a_bot.move_lin_rel(relative_translation=[0.0, 0, 0.], relative_rotation=[0,np.deg2rad(-90), 0], relative_to_robot_base=True)
    # c.fasten_idler_pulley_with_nut_tool("taskboard_long_hole_middle_link")
    # c.playback_sequence("idler_pulley_unequip_nut_tool")
    # c.unequip_tool("b_bot", "padless_tool_m4")
    # c.prepare_nut_tool()
    # c.hold_screw_tool_idler_pulley("taskboard_long_hole_middle_link")
    # c.pick_and_insert_shaft("taskboard")

    # c.b_bot.move_joints([1.6166805028915405, -1.666929384271139, 1.8822982946978968, -1.794750829736227, -1.5447457472430628, 1.592530369758606], acceleration = 0.05, speed=.1)
    # c.test("taskboard_assy_part_07_inserted")
    # c.spiral_search_with_nut_tool()
    # c.insert_idler_pulley("taskboard_long_hole_middle_link")
    
    # c.a_bot.move_joints([0.6765, -1.5287, 2.2274, -0.6668, -0.1085, 1.5388], acceleration = 0.05, speed=.1)
    # c.playback_sequence("idler_pulley_prepare_nut_tool")
    # c.orient_idle_pulley("taskboard")
    # c.prepare_screw_tool_idler_pulley("taskboard")
    # print("bearing_pose", c.look_and_get_grasp_point("bearing"))
    # fake_vision_pose = conversions.to_pose_stamped("tray_center", [-0.03078, 0.06248, 0.02, 0.0,0.7071,0.0,0.7071])

    # c.b_bot.activate_ros_control_on_ur()
    # print(c.a_bot.go_to_named_pose("home"))
    # c.a_bot.gripper.open()
    # c.prepare_nut_tool("taskboard_long_hole_middle_link")

    # rospy.sleep(2)
    # c.b_bot.move_joints([1.7609, -2.0808, 2.4036, -1.9124, -1.5491, 0.1639], acceleration = 0.05, speed=.1)

    # rospy.sleep(2)
    # c.b_bot.move_joints([1.864789605140686, -1.8883010349669398, 2.4725635687457483, -2.175955434838766,
    # -1.5510090033160608, 1.1179262399673462], acceleration = 0.05, speed=.1)
    
    # c.b_bot.move_joints([1.8636610507965088, -1.6897312603392542, 2.5860729853259485, -2.4881612263121546,
    # -1.5512121359454554, 1.1185405254364014], acceleration = 0.05, speed=.1)
    
    
    # c.insert_bearing(task = "taskboard")


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
