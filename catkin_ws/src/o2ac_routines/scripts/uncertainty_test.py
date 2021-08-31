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
    global controller
    
    controller = O2ACAssembly()
    controller.reset_scene_and_robots()
    controller.b_bot.go_to_named_pose("home")
    
    

    # abot motor plate 
    start_pose = conversions.to_pose_stamped("workspace_center", [-0.091, -0.045, 0.124, -0.003, 0.863, 1.567])  # start
    drop_pose = conversions.to_pose_stamped("workspace_center", [-0.091, -0.058, 0.124, -0.003, 0.863, 1.567])  # pull left
    push_start_pose = conversions.to_pose_stamped("workspace_center", [-0.087, -0.112, 0.098, -0.000, 1.57079632679, 1.57079632679])  # push start
    push_almost_over_pose = conversions.to_pose_stamped("workspace_center", [-0.087, -0.086, 0.098, -0.000, 1.57079632679, 1.57079632679])  # push almost over
    push_done_pose = conversions.to_pose_stamped("workspace_center", [-0.087, -0.086, 0.098, -0.000, 1.57079632679, 1.57079632679])  # push done
    at_plate_pose = conversions.to_pose_stamped("workspace_center", [-0.091, -0.045, 0.117, -0.003, 0.863, 1.567])  # at plate
    
    

    # # controller.orient_bearing("assembly", "a_bot", part1=False, part2=True)
    # controller.markers_scene.spawn_item("panel_motor", motor_pose)
    controller.set_assembly("wrs_assembly_2021")
    
    # controller.publish_part_in_assembled_position("panel_motor")
    controller.publish_part_in_assembled_position("base")

    controller.a_bot.go_to_pose_goal(start_pose)
    controller.a_bot.gripper.open(opening_width=0.003)
    rospy.sleep(1.0)
    
    # Spawn the motor plate with a given uncertainty in the gripper. 
    plate_pose = conversions.to_pose_stamped("assembled_part_02", [.008, 0.01, 0, 0, 0, 0.1])  # at plate
    plate_pose = controller.listener.transformPose("a_bot_gripper_tip_link", plate_pose)
    pose_with_uncertainty = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose_with_uncertainty.header = plate_pose.header
    pose_with_uncertainty.pose.pose = plate_pose.pose
    # Is the covariance defined in the header frame? 
    pose_with_uncertainty.pose.covariance= [0.0001, 0.00, 0.00, 0.00, 0.00, 0.00,
                                            0.00, 0.000, 0.00, 0.00, 0.00, 0.00,
                                            0.00, 0.00, 0.0001, 0.00, 0.00, 0.00,
                                            0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                            0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
                                            0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
    motor_panel_co = controller.assembly_database.get_collision_object("panel_motor")
    
    controller.visualize_object_with_distribution(motor_panel_co, pose_with_uncertainty, frame_locked=True)
    controller.confirm_to_proceed("Start?")
    
    controller.a_bot.go_to_pose_goal(drop_pose)
    controller.a_bot.gripper.open(opening_width=0.02)
    support_surface = controller.listener.transformPose("world", plate_pose)
    rospy.sleep(1.0)
    controller.place_object_with_uncertainty("panel_motor", pose_with_uncertainty, support_surface_height=0.856)
    controller.confirm_to_proceed("Dropped plate. Continue?")
    
    controller.a_bot.go_to_pose_goal(push_start_pose)
    controller.a_bot.gripper.close()
    controller.a_bot.go_to_pose_goal(push_almost_over_pose)
    controller.a_bot.go_to_pose_goal(push_done_pose)
    rospy.sleep(1.0)
    controller.push_object_with_uncertainty("panel_motor", push_done_pose, pose_with_uncertainty)
    controller.confirm_to_proceed("Pushed plate. Continue?")
    
    controller.a_bot.go_to_pose_goal(push_start_pose)
    controller.a_bot.gripper.open(opening_width=0.02)
    controller.a_bot.go_to_pose_goal(at_plate_pose)
    controller.a_bot.gripper.open(opening_width=0.005)


    # pose_with_uncertainty=geometry_msgs.msg.PoseWithCovarianceStamped()
    # pose_with_uncertainty.header = goal.header
    # pose_with_uncertainty.pose.pose = goal.pose
    # pose_with_uncertainty.pose.covariance=[0.0001, 0.00, 0.00, 0.00, 0.00, 0.00,
    #                                         0.00, 0.0001, 0.00, 0.00, 0.00, 0.00,
    #                                         0.00, 0.00, 0.0000, 0.00, 0.00, 0.00,
    #                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    #                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
    #                                         0.00, 0.00, 0.00, 0.00, 0.00, 0.01]

    
    # # controller.b_bot.gripper.gripper.release(link_name="panel_bearing_tmp::panel_bearing")
    # controller = O2ACAssembly()
    # controller.ab_bot.go_to_named_pose("home")
    # controller.get_large_item_position_from_top("panel_motor")
    # pose = conversions.to_pose_stamped("tray_center", [0.09, 0.037, 0.015, 0, tau/4, 0])
    # controller.b_bot.go_to_pose_goal(pose)
    # pose = conversions.to_pose_stamped("tray_center", [-0.024, -0.108, 0.006, 0.707, -0.010, -0.707, 0.003])
    # controller.a_bot.go_to_pose_goal(pose)
    # spawner = GazeboModels('o2ac_gazebo')]
    # name = "panel_bearing"
    # object_pose = conversions.to_pose_stamped("tray_center", [0,0,0,0,0,0])
    # object_pose = controller.listener.transformPose("world", object_pose)
    # op = conversions.from_pose_to_list(object_pose.pose)
    # objpose = [op[:3], op[3:]] 
    # models = [Model(name, objpose[0], orientation=objpose[1], reference_frame="world")]
    # controller.gazebo_scene.load_models(models,)
    # controller.b_bot.gripper.gripper.grab(link_name="panel_bearing_tmp::panel_bearing")
    # controller.orient_motor_pulley("taskboard_small_shaft")
    # controller.insert_motor_pulley("taskboard_small_shaft")
    # controller.prepare_screw_tool_idler_pulley("taskboard_long_hole_top_link", simultaneous=True)
    # controller.reset_scene_and_robots()
    # controller.b_bot.go_to_named_pose("home")
    # controller.centering_shaft()
    # controller.align_shaft("taskboard_assy_part_07_inserted")
    # controller.b_bot.go_to_pose_goal(pre_insertion_pose, move_lin=False)
    
    # controller.equip_tool("b_bot", "set_screw_tool")
    # controller.b_bot.go_to_named_pose("horizontal_screw_ready")
    # controller.move_b_bot_to_setscrew_initial_pos()
    # controller.do_task("M2 set screw")
    # # controller.a_bot.move_lin_rel(relative_translation = [0.01,0,0], acceleration = 0.015, speed=.03)
    # controller.ab_bot.go_to_named_pose("home")
    # # controller.playback_sequence("bearing_orient_a_bot", plan_while_moving=False, use_saved_plans=False)
    # controller.pick_bearing("a_bot")
    # controller.orient_bearing("taskboard", robot_name="a_bot", part1=True, part2=False)
    # controller.orient_bearing("taskboard", robot_name="a_bot", part1=False, part2=True)
    # controller.insert_bearing("taskboard_bearing_target_link", robot_name="a_bot")
    # controller = O2ACCommon()
    # controller.publish_status_text("O2AC Go! <3")
    # controller.allow_collisions_with_robot_hand("agv_table", "a_bot")
    # controller.planning_scene_interface.allow_collisions("agv_table")
    # controller.playback_sequence("bearing_orient_a_bot", use_saved_plans=False`)
    print("a_bot", np.round(controller.a_bot.robot_group.get_current_joint_values(), 5).tolist())
    # print("b_bot", controller.b_bot.robot_group.get_current_joint_values())
    # pose = conversions.to_pose_stamped("left_centering_link", [-0.02, 0, 0.02, radians(-30), 0, 0])
    # controller.a_bot.go_to_pose_goal(pose)
    # controller.a_bot.move_lin_rel(relative_translation=[0,0,0.015], relative_rotation=[0, radians(35), 0], relative_to_tcp=True)

    
    # controller.do_change_tool_action("b_bot", equip=False, screw_size = 4)
    # controller = O2ACAssembly()
    # controller.reset_scene_and_robots()
    # controller.get_large_item_position_from_top("motor")

    # controller.publish_part_in_assembled_position("panel_bearing")
    # controller.publish_part_in_assembled_position("panel_motor")
    # controller.publish_part_in_assembled_position("base")
    # controller.fasten_motor()
    # controller.fasten_motor_fallback()
    # controller = O2ACAssembly()
    # controller.reset_scene_and_robots()
    # controller.publish_part_in_assembled_position("panel_bearing")
    # controller.publish_part_in_assembled_position("panel_motor")
    # controller.publish_part_in_assembled_position("base")
    # controller.confirm_motor_and_place_in_aid()
    # controller.center_panel_on_base_plate("panel_motor")
    # inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.0, 0.002, -0.005, tau/2., radians(3.0), radians(30)])
    # controller.b_bot.go_to_pose_goal(inside_vgroove)

    # controller.confirm_to_proceed("al??")
    # controller.align_motor_pre_insertion()
    # controller.confirm_to_proceed("insert??")
    # controller.insert_motor("assembled_part_02_back_hole")
    # controller.confirm_to_proceed("fastem??")
    # controller.fasten_motor()
    # controller.confirm_to_proceed("fasten??")
    # controller.b_bot.gripper.open()
    # controller.b_bot.move_lin_rel(relative_translation=[-0.02,0.1,0.1])
    # controller.b_bot.go_to_named_pose("home")
    # print("result", controller.b_bot.load_and_execute_program(program_name="wrs2020/motor_orient.urp"))

    # pose = conversions.to_pose_stamped("assembled_part_02_back_hole", [-0.022, -0.004, -0.014, -0.098, 0.702, -0.086, 0.700])
    # controller.b_bot.go_to_pose_goal(pose)
    # tc = None
    # selection_matrix = [1., 1., 1., 0.7, 1., 1.]
    # controller.b_bot.execute_spiral_trajectory("XY", max_radius=0, radius_direction="+Y", steps=50,
    #                                      revolutions=1, target_force=0, check_displacement_time=10,
    #                     
    #                  wiggle_direction="Z", wiggle_angle=radians(10.0), wiggle_revolutions=1.0,
    #                                      termination_criteria=tc, timeout=20, selection_matrix=selection_matrix)
    # controller.place_panel("a_bot", "panel_motor", pick_again=True, fake_position=True)
    # controller.fasten_panel("panel_motor", True)
    # controller.center_panel_on_base_plate("panel_bearing")
    # push_pose     = conversions.to_pose_stamped("assembled_part_03_bottom_corner_2", [-0.01, 0.0, 0.015, 0, 0, 0])
    # controller.a_bot.go_to_pose_goal(push_pose)

    # controller.set_assembly("wrs_assembly_2021")
    # controller.get_large_item_position_from_top("panel_motor")
    # controller.get_large_item_position_from_top("panel_bearing")
    
    # cp = controller.listener.transformPose("tray_center", controller.a_bot.get_current_pose_stamped())
    # print("a bot pose", cp.pose.position)
    # grasp_pose = controller.get_transformed_grasp_pose("base", "terminal_grasp", target_frame="tray_center")
    # print("grasp pose", grasp_pose.pose.position)
    # current_base_pose = controller.get_transformed_collision_object_pose("base", target_frame="tray_center")
    # current_base_pose.pose.position.x += cp.pose.position.x - grasp_pose.pose.position.x
    # current_base_pose.pose.position.y += cp.pose.position.y - grasp_pose.pose.position.y
    # controller.update_collision_item_pose("base", current_base_pose)

    # controller.reset_scene_and_robots()
    # controller.ab_bot.go_to_named_pose("home")
    # controller.place_panel("a_bot", "panel_bearing", fake_position=True)
    # controller.despawn_object("panel_bearing")
    # controller.fasten_panel("panel_bearing")
    # controller.do_change_tool_action("b_bot", equip=False, screw_size = 4)
    # controller.panels_assembly()
    # controller.subtask_zero(use_b_bot_camera=True)
    # controller.a_bot.gripper.open()
    # controller.pick_panel("panel_bearing")
    # controller.center_panel("panel_bearing")
    # controller.confirm_to_proceed("1??")
    # controller.ab_bot.go_to_named_pose("home")
    # controller.pick_panel("panel_motor")
    # pose = controller.center_panel("panel_bearing")
    # controller.place_panel("a_bot", "panel_bearing", grasp_pose=pose)
    # controller.b_bot.go_to_named_pose("feeder_pick_ready")
    # controller.confirm_to_proceed("2??")
    
    # controller.ab_bot.go_to_named_pose("home")
    # controller.reset_scene_and_robots()
    # controller.get_large_item_position_from_top("base", "b_bot")
    # controller.confirm_to_proceed("1")
    # controller.pick_base_panel(skip_initial_perception=True)

    # controller = O2ACCommon()
    # handover_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.1, 0.2, -tau/4, tau/8, -tau/4])
    # controller.b_bot.go_to_pose_goal(handover_pose)
    # controller.a_bot.gripper.open()
    # controller.a_bot.gripper.close()
    # controller.confirm_to_proceed("")
    # controller.a_bot.gripper.open(velocity=1.0)
    # controller.a_bot.gripper.close(velocity=1.0)
    # print(controller.a_bot.robot_group.get_current_joint_values())
    # pre_hand_over_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.25, -tau/4, 0, tau/4])
    # controller.a_bot.go_to_pose_goal(pre_hand_over_pose, 0.5, move_lin=False)
    # controller.update_collision_item_pose("panel_bearing", conversions.to_pose_stamped("left_centering_link", [0.000, 0.070, -0.080, -0.500, 0.500, -0.500, -0.500]))
    # controller = O2ACAssembly()
    # controller.a_bot.gripper.open()
    # controller.ab_bot.go_to_named_pose("home")
    # controller.reset_scene_and_robots()
    # # controller.publish_part_in_assembled_position("base")
    # # controller.place_panel("a_bot", "panel_bearing")
    # # controller.fasten_panel("panel_bearing")
    # controller.pick_panel("panel_bearing")
    # controller.center_panel("panel_bearing")
    # controller.confirm_to_proceed("")
    # controller.subtask_zero()
    # gp = controller.panel_subtask2("panel_bearing")
    # controller.place_panel("a_bot", "panel_motor", grasp_pose=None)
    # TODO:
    # 1. add the incline hold pose for fastening
    # 2. create the simultaneous task for picking,centering the panels, then picking the base plate and then fastening
    # controller.fasten_panel("panel_bearing")
    # controller = O2ACCommon()
    # controller.reset_scene_and_robots()
    # controller.ab_bot.go_to_named_pose("home")
    # controller.competition_mode = True
    # controller.reset_scene_and_robots()
    # controller.center_tray_stack(True, False)
    # controller.spawn_tray_stack(stack_center=[-0.03, 0.0], tray_heights=[0.05, 0.0], orientation_parallel=True)
    # controller.pick_tray_from_agv_stack_calibration_long_side("tray1")
    # controller.return_tray_to_agv_stack_calibration_long_side("tray1")
    # controller.pick_tray_from_agv_stack_calibration_long_side("tray2")
    # controller.return_tray_to_agv_stack_calibration_long_side("tray2")
    # q = [0.19453047870641985, -1.414852203750135, 1.7189982978203278, -1.8730085962497756, -1.567615792398194, 1.7666671594586172, 2.453019687833446, -1.3449819279957274, 1.8068236138946376, -2.0324751409170405, -1.5648680786549045, -2.2588772450321226]
    # controller.ab_bot.move_joints(q, speed=1.0)
    # q = [0.19502419962975553, -1.302128082437928, 1.886556773525712, -2.1543437146483777, -1.568541929253542, 1.766640025829011, 2.4522167306633054, -1.2014465832104217, 1.9290941675252034, -2.298411409316249, -1.5642835257198011, -2.2577476650719626]
    # controller.ab_bot.move_joints(q, speed=1.0)
    # controller.confirm_to_proceed("")
    # controller.spawn_tray_stack(stack_center=[0.05, 0.14], tray_heights=[0.075,0.02], orientation_parallel=False)
    # controller.pick_tray_from_agv_stack_calibration_short_side("tray1")
    # controller.pick_tray_from_agv_stack_calibration_short_side()
    ###### insert motor ######
    # controller.publish_part_in_assembled_position("base")
    # controller.publish_part_in_assembled_position("panel_bearing")
    # controller.publish_part_in_assembled_position("panel_motor")
    # controller.b_bot.move_lin_rel(relative_translation=[0,0,0.1])
    # controller.ab_bot.go_to_named_pose("home")
    # controller.do_change_tool_action("a_bot", equip=True, screw_size=3)
    # controller.a_bot.go_to_named_pose("screw_ready")

    # above_vgroove  =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.2, 0, 0, tau/2., 0, 0])
    # if not controller.b_bot.go_to_pose_goal(above_vgroove, speed=0.4, move_lin=False):
    #   rospy.logerr("fail to go above vgroove")
    #   return False
    # controller.b_bot.gripper.send_command(0.05)
    # if not controller.align_motor_pre_insertion():
    #     return False
    # controller.confirm_to_proceed("finetune")
    # controller.insert_motor("assembled_part_02_back_hole")

    # controller.confirm_to_proceed("move")
    # selection_matrix = [1., 1., 1., 0.8, 1., 1.]
    # controller.b_bot.execute_spiral_trajectory("XY", max_radius=0.0, radius_direction="+Y", steps=50,
    #                                             revolutions=1, target_force=0, check_displacement_time=10,
    #                                             wiggle_direction="Z", wiggle_angle=radians(15.0), wiggle_revolutions=2.0,
    #                                             termination_criteria=None, timeout=10, selection_matrix=selection_matrix)
    
    ### fasten motor ####
    # controller.confirm_to_proceed("fasten?")
    # controller.do_change_tool_action("a_bot", equip=True, screw_size=3)
    # controller.fasten_motor()


    ###################################

    # controller = O2ACCommon()
    # controller.a_bot.go_to_named_pose("screw_ready")
    # controller.reset_scene_and_robots()
    # controller.b_bot.go_to_named_pose("home")
    # controller.take_tray_from_agv_preplanned(save_on_success=True, use_saved_plans=False)
    # controller.fasten_bearing("assembly", only_retighten=True)
    # controller.b_bot.go_to_named_pose("home")

    # controller.allow_collisions_with_robot_hand("tray", "a_bot")
    # controller.allow_collisions_with_robot_hand("tray_center", "a_bot")
    
    # controller.playback_sequence("bearing_orient_down_b_bot")

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
    # if not controller.execute_sequence(robot_name, [trajectory], "go to preinsertion", plan_while_moving=False):
    #   rospy.logerr("Could not go to preinsertion")

    
    
    # # tool_pull_pose = conversions.to_pose_stamped("move_group/panel_motor", [0.03, 0.038, 0.0, 0, 0, 0])
    # tool_pull_pose = conversions.to_pose_stamped("move_group/panel_bearing/front_hole", [0.0, 0.0, 0.0, 0, 0, 0])
    # controller.move_towards_center_with_tool("b_bot", target_pose=tool_pull_pose, distance=0.05, start_with_spiral=True)
    

    # # controller = O2ACCommon()
    # # test_force_control(controller)

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

    # controller = O2ACTaskboard()
    # controller.reset_scene_and_robots()
    # controller.ab_bot.go_to_named_pose("home")
    # controller.equip_tool("a_bot", "screw_tool_m3")
    # controller.equip_tool("b_bot", "screw_tool_m4")
    # a = threading.Thread(target=a_bot_m3, args=(controller,))
    # a.daemon = True
    # b = threading.Thread(target=b_bot_m4, args=(controller,))
    # b.daemon = True

    # a.start()
    # rospy.sleep(10)
    # b.start()

    # rospy.spin()

    # =====

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
