#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi, radians

from ur_control import conversions
tau = 2.0*pi  # Part of math from Python 3.6

from o2ac_routines.common import O2ACCommon
from o2ac_routines.helpers import wait_for_UR_program
from o2ac_routines.thread_with_trace import ThreadTrace

class O2ACTaskboard(O2ACCommon):
  """
  This class contains the taskboard routines.
  """
  def __init__(self):
    super(O2ACTaskboard, self).__init__()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # Initialize debug monitor
    self.start_task_timer()
    
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/4))
    self.at_set_screw_hole = conversions.to_pose_stamped("taskboard_set_screw_link", [-0.005, -0.001, 0.001, 0, 0, 0])  # MAGIC NUMBER (points downward) MIZUMI tool tip
    # self.at_set_screw_hole = conversions.to_pose_stamped("taskboard_set_screw_link", [-0.018, -0.001, 0.001, 0, 0, 0])  # MAGIC NUMBER (points downward)  PROXXON tool tip
    if not self.assembly_database.db_name == "taskboard":
      self.assembly_database.change_assembly("taskboard")
    
    self.subtask_completed = {
      "M2 set screw": True,
      "M3 screw": False,
      "M4 screw": False,
      "belt": False,
      "bearing": False,
      "screw_bearing": False,
      "motor pulley": False,
      "shaft": False,  
      "idler pulley": False,
    }

  def spawn_example_objects(self):
    # This function spawns the objects into the tray as if they had been recognized by the vision node
    names = ["taskboard_idler_pulley_small", "bearing", "shaft", "motor_pulley"]
    offsets = {"bearing": [-.04, -.02, .001],
    "taskboard_idler_pulley_small": [.07, .06, .03],
    "shaft": [.03, -.06, .005], 
    "motor_pulley": [-.01, .12, .005]}

    # We publish each object to its own frame.
    broadcaster = tf.TransformBroadcaster()
    rospy.sleep(.5)
    counter = 0
    for name in names:
      collision_object = self.assembly_database.get_collision_object(name)
      if collision_object:
        counter += 1
        collision_object.header.frame_id = "collision_object_spawn_helper_frame" + str(counter)
        if name == "bearing":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        if name == "taskboard_idler_pulley_small" or name == "motor_pulley":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0)
        elif name == "shaft":
          q_rotate = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
          
        offset = offsets[name]
        broadcaster.sendTransform((offset[0], offset[1], offset[2]), q_rotate, rospy.Time.now(), 
            "collision_object_spawn_helper_frame" + str(counter), "tray_center")
        rospy.sleep(1.0) # Wait for the transform to have propagated through the system

        self.planning_scene_interface.add_object(collision_object)
        # print("======== collision object: " + name)
        # print(collision_object.mesh_poses)
        # print(collision_object.subframe_names)
        # print(collision_object.subframe_poses)
      else:
        rospy.logerr("Could not retrieve collision object:" + name)

  #####
  def look_in_tray(self):
    #TODO (felixvd)
    # Look at tray
    # loop through all items
    # check if they were recognized and are graspable
    # otherwise check position again
    # if still not graspable either skip to next item or try to reposition
    pass
  
  def check_objects_in_tray(self, robot_name, order):
    """ Return the easier to find objects from a birdview """
    self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, 
                                                   end_effector_link=robot_name + "_outside_camera_color_frame",
                                                   speed=.5, acceleration=.3, wait=True, move_lin=False)
    
    success = False
    while not success:
      rospy.sleep(0.5)
      success = self.get_3d_poses_from_ssd()

    order_ids = [self.assembly_database.name_to_id(object_id) for object_id in order]
    object_in_tray = copy.deepcopy(self.objects_in_tray)
    res = []
    for item in order_ids:
      if object_in_tray.get(item, None):
        res.append(self.assembly_database.id_to_name(item))
    return res 

  def prep_taskboard_task(self):
    """
    Equip the set screw tool and M3 tool, and move to the position before task start.
    """

    self.a_bot.go_to_named_pose("home")

    self.equip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    self.b_bot.go_to_named_pose("home")
    
    self.equip_tool("b_bot", "set_screw_tool")
    self.b_bot.go_to_named_pose("horizontal_screw_ready")

    self.move_b_bot_to_setscrew_initial_pos()
  
  def prep_taskboard_task_simultaneous(self):
    def do_with_a():
      self.a_bot.go_to_named_pose("home")
      self.equip_tool("a_bot", "screw_tool_m3")
      self.a_bot.go_to_named_pose("feeder_pick_ready")
    def do_with_b():
      self.b_bot.go_to_named_pose("home")
      self.equip_tool("b_bot", "set_screw_tool")
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      self.move_b_bot_to_setscrew_initial_pos()
    return self.do_tasks_simultaneous(do_with_a, do_with_b, timeout=50.0)

  def move_b_bot_to_setscrew_initial_pos(self):
    screw_approach = copy.deepcopy(self.at_set_screw_hole)
    screw_approach.pose.position.x = -0.03
    self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.5)
    screw_approach.pose.position.x = -0.005
    self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.1)

  def do_screw_tasks_from_prep_position(self):
    ### - Set screw
    
    # Move into the screw hole with motor on
    self.vision.activate_camera("b_bot_inside_camera")
    self.do_task("M2 set screw")
    
    # TODO: check set screw success with a_bot, do spiral motion with b_bot otherwise
    
    ### SCREW M3 WITH A_BOT
    self.vision.activate_camera("a_bot_outside_camera")
    screw_picked = self.pick_screw_from_feeder("a_bot", screw_size = 3)
    self.a_bot.go_to_named_pose("feeder_pick_ready")

    # Move b_bot back, a_bot to screw
    self.unequip_tool("b_bot", "set_screw_tool")
    self.equip_tool("b_bot", "screw_tool_m4")
    self.b_bot.go_to_named_pose("feeder_pick_ready")

    if screw_picked:
      self.subtask_completed["M3 screw"] = self.do_task("M3 screw")
    self.unequip_tool("a_bot", "screw_tool_m3")
    self.a_bot.go_to_named_pose("home")
    
    #### SCREW M4 WITH B_BOT
    self.subtask_completed["M4 screw"] = self.do_task("M4 screw")

    self.b_bot.go_to_named_pose("home")
    self.unequip_tool("b_bot", "screw_tool_m4")

  def full_taskboard_task(self, do_screws=True, skip_tray_placing=True):
    """
    (THIS IS NOT THE MAIN FUNCTION ANYMORE. See full_taskboard_task_simultaneous instead.)
    Starts the taskboard task from the fully prepped position (set screw tool and M3 tool equipped).
    """
    #####
    self.subtask_completed = {
      "M2 set screw": True,
      "M3 screw": False,
      "M4 screw": False,
      "belt": False,
      "bearing": False,
      "screw_bearing": False,
      "motor pulley": False,
      "shaft": False,  
      "idler pulley": False,
    }
    if do_screws:
      # self.do_screw_tasks_from_prep_position()
      a_success, b_success = self.do_screw_tasks_simultaneous()
      self.subtask_completed["M3 screw"] = a_success
      self.subtask_completed["M4 screw"] = b_success


    if not skip_tray_placing:
      self.take_tray_from_agv()

    order = ["belt", "motor pulley", "shaft", "idler pulley", "bearing"]
    task_complete = False

    # Loop through the remaining items
    self.confirm_to_proceed("Continue into loop to retry parts?")
    unsuccessful_attempts = 0
    while not task_complete:
      for item in order:
        if not self.execute_step(item):
          unsuccessful_attempts += 1
      task_complete = all(self.subtask_completed.values())

    self.publish_status_text("FINISHED")

  def full_taskboard_task_simultaneous(self, do_screws=True, skip_tray_placing=True):
    """
    Start the taskboard task from the fully prepped position (set screw tool and M3 tool equipped).
    Includes simultaneous executions.
    """
    #####
    self.subtask_completed = {
      "M2 set screw": True,
      "M3 screw": False,
      "M4 screw": False,
      "belt": False,
      "bearing": False,
      "screw_bearing": False,
      "motor pulley": False,
      "shaft": False,  
      "idler pulley": False,
    }
    if do_screws:
      # self.do_screw_tasks_from_prep_position()
      success = self.do_screw_tasks_simultaneous()
      if success:
        self.subtask_completed["M3 screw"] = True
        self.subtask_completed["M4 screw"] = True
    else:
      self.subtask_completed["M3 screw"] = True
      self.subtask_completed["M4 screw"] = True

    self.unequip_tool("a_bot")
    self.unequip_tool("b_bot")

    if not skip_tray_placing:
      self.take_tray_from_agv()

    # Do belt and idler pulley first
    self.subtask_completed["belt"] = self.do_task("belt")
    self.subtask_completed["idler pulley"] = self.do_task("idler pulley", simultaneous=True)

    # Pick bearing, do motor_pulley
    self.publish_status_text("Target: pick bearing")
    def a_bot_task():
      self.pick_bearing("a_bot")
    def b_bot_task():
      self.b_bot.go_to_named_pose("home")
    self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180.0)
    self.a_success = False
    self.b_success = False
    def a_bot_task(): # orient/insert bearing
      self.a_success = self.orient_bearing("taskboard", robot_name="a_bot", part1=True, part2=False)
      self.a_bot.gripper.forget_attached_item()
    def b_bot_task(): # pick/orient/insert motor pulley
      self.subtask_completed["motor pulley"] = self.do_task("motor pulley")
      self.b_success = self.subtask_completed["motor pulley"]
      self.b_bot.go_to_named_pose("home")
    self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180.0)
    
    self.orient_bearing("taskboard", robot_name="a_bot", part1=False, part2=True)
    self.subtask_completed["bearing"] = self.insert_bearing("taskboard_bearing_target_link", robot_name="a_bot")
    
    print("task 1:", self.a_success, self.b_success)

    # Align bearing, pick screw with a_bot
    if self.a_success and self.b_success:
      rospy.loginfo(">>> Prepare for fastening bearing")
      def a_bot_task2(): # prepare a_bot with screw tool m4 / pick screw from feeder
        self.a_success = self.a_bot.go_to_named_pose("home")
        self.a_success = self.equip_tool("a_bot", 'screw_tool_m4')
        self.a_success &= self.pick_screw_from_feeder_python("a_bot", screw_size=4)
      def b_bot_task2(): # align bearing holes
        self.b_success = self.align_bearing_holes(task="taskboard")
        self.b_success &= self.b_bot.go_to_named_pose("home")
      self.do_tasks_simultaneous(a_bot_task2, b_bot_task2, timeout=120.0)
    print("task 2:", self.a_success, self.b_success)

    # Fasten bearing, insert shaft
    if self.a_success and self.b_success:
      print(">>>> fastening bearing")
      def a_bot_task3(): # fasten bearing
        self.subtask_completed["screw_bearing"] = self.fasten_bearing(task="taskboard", robot_name="a_bot", simultaneous=True)
      def b_bot_task3(): # pick/orient/insert motor pulley
        self.subtask_completed["shaft"] = self.do_task("shaft")
        self.vision.activate_camera("a_bot_outside_camera")
        self.b_bot.go_to_named_pose("home")
      self.do_tasks_simultaneous(a_bot_task3, b_bot_task3, timeout=300.0)
    print("task 3:", self.subtask_completed["screw_bearing"], self.subtask_completed["shaft"])

    self.despawn_object("bearing")
    self.ab_bot.go_to_named_pose("home")

    order = ["belt", "motor pulley", "shaft", "idler pulley", "bearing"]
    task_complete = False
    # Loop through the remaining items
    self.confirm_to_proceed("Continue into loop to retry parts?")
    unsuccessful_attempts = 0
    while not task_complete:
      for item in order:
        if not self.execute_step(item):
          unsuccessful_attempts += 1
      print(self.subtask_completed)
      task_complete = all(self.subtask_completed.values())

    self.publish_status_text("FINISHED")
    return True

  def do_screw_tasks_simultaneous(self):
    """
    Start from prep poses, finish before carrying tray.
    """
    ### - Set screw
    self.publish_status_text("M3 set screw")
    
    self.vision.activate_camera("b_bot_inside_camera")
    self.a_bot_success = False
    self.b_bot_success = False
    def do_with_a():
      self.a_bot_success = self.pick_screw_from_feeder("a_bot", screw_size=3)
    def do_with_b():
      self.b_bot_success = self.do_task("M2 set screw")
    self.do_tasks_simultaneous(do_with_a, do_with_b, timeout=120.0)
    # TODO: Consider failure cases
    
    if not self.b_bot_success:
      rospy.logerr("b_bot failed to do set screw, continue anyways")

    if not self.a_bot_success:
      if not self.pick_screw_from_feeder("a_bot", screw_size=3):
        rospy.logerr("Fail to pick screw with a_bot twice... abort")
        return False

    self.a_bot_success = False
    self.b_bot_success = False
    def prep_b_bot():
      self.b_bot_success = self.unequip_tool("b_bot", "set_screw_tool")
      self.b_bot_success &= self.equip_tool("b_bot", "screw_tool_m4")
      self.b_bot_success &= self.b_bot.go_to_named_pose("feeder_pick_ready")
      self.b_bot_success &= self.pick_screw_from_feeder("b_bot", screw_size=4)
      self.b_bot_success &= self.b_bot.go_to_named_pose("feeder_pick_ready")
    def a_bot_task():
      self.a_bot_success = self.do_task("M3 screw")

    self.do_tasks_simultaneous(a_bot_task, prep_b_bot, timeout=120.0)

    if not self.a_bot_success:
      rospy.logerr("a_bot fail but we will continue")

    if not self.b_bot_success:
      rospy.logerr("b_bot fail to do prep tools")

    if self.b_bot_success:
      self.b_bot_success = False
      def a_3():
        self.unequip_tool("a_bot", "screw_tool_m3")
        self.a_bot.go_to_named_pose("home")
      def b_3():
        self.b_bot_success = self.do_task("M4 screw")
        self.b_bot_success &= self.b_bot.go_to_named_pose("home")
      self.do_tasks_simultaneous(a_3, b_3, timeout=120.0)

    if not self.a_bot_success or not self.b_bot_success:
      self.unequip_tool("a_bot", "screw_tool_m3")
      self.tools.set_suction("screw_tool_m3", suction_on=False, eject=False, wait=False)
      self.tools.set_suction("screw_tool_m4", suction_on=False, eject=False, wait=False)
      self.tools.set_motor("screw_tool_m3", "tighten", duration = 1.0)
      self.tools.set_motor("screw_tool_m4", "tighten", duration = 1.0)
      self.unequip_tool("b_bot", "screw_tool_m4")
      self.ab_bot.go_to_named_pose("home")
      return False

    return self.a_bot_success, self.b_bot_success

  def execute_step(self, item):
      assert not rospy.is_shutdown(), "did ros master die?"
      self.unequip_tool("b_bot") # Just in case
      self.b_bot.gripper.open(wait=False) # Release any possible item that got stuck in the gripper

      if not self.subtask_completed[item]:
        self.confirm_to_proceed("Reattempt " + str(item) + "?")
        self.subtask_completed[item] = self.do_task(item)
        if item == "bearing" and self.subtask_completed[item]: 
          if not self.subtask_completed["screw_bearing"]:
            self.do_task("screw_bearing")
        
        rospy.loginfo("============================================")
        rospy.loginfo("==== Task: %s completed with status: %s ====" % (item, self.subtask_completed[item]))
        rospy.loginfo("============================================")      
      
      return self.subtask_completed[item]

  def do_task(self, task_name, fake_execution_for_calibration=False, simultaneous=False):
    self.publish_status_text("Target: " + task_name)

    if task_name == "belt":
      if not self.use_real_robot:
        rospy.logwarn("Cannot simulate task: belt")
        return True

      self.ab_bot.go_to_named_pose("home")
      
      self.a_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="a_bot_outside_camera_color_frame", speed=.8, move_lin=False)

      self.vision.activate_camera("a_bot_outside_camera")
      self.activate_led("a_bot")
      res = self.get_3d_poses_from_ssd()
      r2 = self.get_feasible_grasp_points("belt")
      if r2:
        pick_goal = r2[0]
        pick_goal.pose.position.z = 0.000
      else:
        rospy.logerr("Could not find belt grasp pose! Aborting.")
        return False
      
      self.confirm_to_proceed("Pick tool with b_bot?")
      rospy.sleep(1)
      self.a_bot.activate_ros_control_on_ur()
      self.b_bot.activate_ros_control_on_ur()
      # Equip the belt tool with b_bot
      if not self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_pick_hook.urp"):
        return False
      rospy.sleep(2)  # Wait for b_bot to have moved out of the way
      self.allow_collisions_with_robot_hand("tray", "a_bot")
      self.allow_collisions_with_robot_hand("tray_center", "a_bot")
      self.simple_pick("a_bot", pick_goal, gripper_force=100.0, grasp_width=.08, axis="z")
      self.a_bot.move_lin_rel(relative_translation=[0,0,.1])
      self.allow_collisions_with_robot_hand("tray", "a_bot", False)
      self.allow_collisions_with_robot_hand("tray_center", "a_bot", False)
      
      # Move to ready pose, check if belt was picked
      def a_bot_task():
        a_bot_wait_with_belt_pose = [0.646294116973877, -1.602117200891012, 2.0059760252581995, -1.3332312864116211, -0.8101084868060511, -2.4642069975482386]
        self.a_bot.move_joints(a_bot_wait_with_belt_pose)
      def b_bot_task():
        b_bot_look_at_belt = [1.95739448, -1.40047674, 1.92903739, -1.98750128, -2.1883457, 1.7778782]
        self.b_bot.move_joints(b_bot_look_at_belt, speed=0.3)
        self.vision.activate_camera("b_bot_outside_camera")
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180.0)
      
      
      if not self.vision.check_pick_success("belt"):
        rospy.logerr("Belt pick has failed. Return tool and abort.")
        self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
        rospy.sleep(2)
        pick_goal.pose.position.x = 0  # In tray_center
        pick_goal.pose.position.y = 0
        pick_goal.pose.position.z += 0.06
        self.a_bot.move_lin(pick_goal)
        self.a_bot.gripper.open(opening_width=0.07, wait=False)
        self.a_bot.go_to_named_pose("home")
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
        return False

      self.confirm_to_proceed("Load and execute the belt threading programs?")
      success_a = self.a_bot.load_program(program_name="wrs2020/taskboard_belt_v7.urp")
      success_b = self.b_bot.load_program(program_name="wrs2020/taskboard_belt_v7.urp")
      if success_a and success_b:
        print("Loaded belt program on a_bot.")
        rospy.sleep(1)
        success = self.a_bot.execute_loaded_program()
        success = self.b_bot.execute_loaded_program()
        if success:
          print("Starting belt threading execution.")
          rospy.sleep(2)
          self.a_bot.close_ur_popup()
          self.b_bot.close_ur_popup()
      else:
        print("Problem loading. Not executing belt procedure.")
        self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
        rospy.sleep(3)
        self.drop_in_tray("a_bot")
        self.a_bot.go_to_named_pose("home")
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
        return False
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))

      # b_bot is now above the tray, looking at the 
      # TODO(felixvd): Use vision to check belt threading success
      
      self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
      rospy.sleep(2)
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
      return True

    # ==========================================================

    if task_name == "M2 set screw":
      # Equip and move to the screw hole
      # self.equip_tool("b_bot", "set_screw_tool")
      # self.b_bot.go_to_named_pose("horizontal_screw_ready")
      rospy.loginfo("=== set screw: start ===")
      self.vision.activate_camera("b_bot_inside_camera")
      screw_approach = copy.deepcopy(self.at_set_screw_hole)
      screw_approach.pose.position.x = -0.005
      self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.b_bot.go_to_pose_goal(self.at_set_screw_hole, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True, speed=0.02)
      rospy.loginfo("=== set screw: at at_set_screw_hole ===")
      # This expects to be exactly above the set screw hole
      self.confirm_to_proceed("Turn on motor and move into screw hole?")
      self.tools.set_motor("set_screw_tool", "tighten", duration = 15.0, skip_final_loosen_and_retighten=True)
      dist = .002
      self.b_bot.move_lin_rel(relative_translation=[-dist, 0, 0], speed=0.03, wait=True)
      rospy.loginfo("=== set screw: move in ===")
      # Stop spiral motion if the tool action finished, regardless of success/failure
      self.b_bot.execute_spiral_trajectory("YZ", max_radius=0.001, radius_direction="+Y", steps=50,
                                          revolutions=2, target_force=0, check_displacement_time=10,
                                          termination_criteria=None, timeout=6, end_effector_link="b_bot_set_screw_tool_tip_link")
      # self.skill_server.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .002)
      if self.use_real_robot:
        rospy.sleep(2.0) # Wait for the screw to be screwed in a little bit
      d = .003
      rospy.loginfo("=== set screw: move in 2 by " + str(d) + " m. ===")
      self.b_bot.move_lin_rel(relative_translation=[-d, 0, 0], speed=0.002, wait=False)
      
      if self.use_real_robot:
        rospy.sleep(2.0)
      self.confirm_to_proceed("Go back?")
      if self.b_bot.is_protective_stopped():
        rospy.logwarn("Robot was protective stopped after set screw insertion!")
        self.b_bot.unlock_protective_stop()
        rospy.sleep(1)
        if self.b_bot.is_protective_stopped():
          return False

      self.b_bot.move_lin_rel(relative_translation=[0.05, 0, 0], speed=0.2)
      # Go back
      # self.b_bot.go_to_pose_goal(screw_approach, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)

      self.b_bot.go_to_named_pose("horizontal_screw_ready", speed=0.5, acceleration=0.5)
      # self.confirm_to_proceed("Unequip tool?")
      # self.b_bot.go_to_named_pose("home", speed=0.5, acceleration=0.5)
      # self.unequip_tool("b_bot", "set_screw_tool")
    
    # ==========================================================

    if task_name == "M3 screw":
      if not self.a_bot.robot_status.carrying_tool and not self.a_bot.robot_status.held_tool_id == "screw_tool_m3":
        self.equip_tool("a_bot", "screw_tool_m3")
      if not fake_execution_for_calibration and self.use_real_robot:
        if not self.pick_screw_from_feeder("a_bot", screw_size = 3):
          rospy.logerr("Fail pick screw from feeder m3")
          return False
      
      if not self.a_bot.go_to_named_pose("horizontal_screw_ready"):
        rospy.logerr("Fail to go to pose horizontal_screw_ready")
        return False

      self.allow_collisions_with_robot_hand("screw_tool_m4", "b_bot", False)
      approach_pose = geometry_msgs.msg.PoseStamped()
      approach_pose.header.frame_id = "taskboard_m3_screw_link"
      approach_pose.pose.position.x = -.04
      approach_pose.pose.position.y = -.002    # MAGIC NUMBER (y-axis of the frame points right)
      approach_pose.pose.position.z = -.006  # MAGIC NUMBER (z-axis of the frame points down)
      approach_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      if not self.a_bot.go_to_pose_goal(approach_pose, speed=0.5, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True):
        rospy.logerr("Fail to go to approach pose")
        return False

      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m3_screw_link"
      hole_pose.pose.position.y = -.002  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.006  # MAGIC NUMBER (z-axis of the frame points down)
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0))
      if not fake_execution_for_calibration:
        if not self.screw("a_bot", hole_pose, screw_size = 3, skip_final_loosen_and_retighten=False, spiral_radius=0.004, duration=40):
          rospy.logerr("Fail to screw m3")
          self.a_bot.move_lin_rel([0.05,0,0], speed=0.2)
          self.a_bot.go_to_named_pose("horizontal_screw_ready")
          self.unequip_tool("a_bot", "screw_tool_m3")
          return False
      else:
        hole_pose.pose.position.x -= 0.005
        if not self.a_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True):
          return False
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        if not self.a_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="a_bot_screw_tool_m3_tip_link", move_lin = True):
          return False
      if not self.a_bot.go_to_named_pose("horizontal_screw_ready"):
        rospy.logerr("Fail to go to horizontal_screw_ready")
        return False

    # ==========================================================

    if task_name == "M4 screw":
      if not self.b_bot.robot_status.carrying_tool and not self.b_bot.robot_status.held_tool_id == "screw_tool_m4":
        self.equip_tool("b_bot", "screw_tool_m4")
      self.vision.activate_camera("b_bot_outside_camera")
      if not fake_execution_for_calibration and self.use_real_robot:
        if not self.pick_screw_from_feeder("b_bot", screw_size = 4):
          rospy.logerr("Fail pick screw from feeder m4")
          return False
      if not self.b_bot.go_to_named_pose("horizontal_screw_ready"):
        rospy.logerr("Fail to go to horizontal_screw_ready")
        return False
      self.allow_collisions_with_robot_hand("screw_tool_m4", "a_bot", False)
      hole_pose = geometry_msgs.msg.PoseStamped()
      hole_pose.header.frame_id = "taskboard_m4_screw_link"
      hole_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/12, 0, 0))
      hole_pose.pose.position.y = -.001  # MAGIC NUMBER (y-axis of the frame points right)
      hole_pose.pose.position.z = -.001  # MAGIC NUMBER (z-axis of the frame points down)
      if not fake_execution_for_calibration:
        if not self.screw("b_bot", hole_pose, screw_size = 4, skip_final_loosen_and_retighten=False, spiral_radius=0.003, duration=40):
          rospy.logerr("Failed to screw m4")
          self.b_bot.move_lin_rel([0.05,0,0], speed=0.2)
          self.b_bot.go_to_named_pose("home")
          self.unequip_tool("4_bot", "screw_tool_m4")
          return False
      else:
        hole_pose.pose.position.x = -.01
        approach_pose = copy.deepcopy(hole_pose)
        approach_pose.pose.position.x -= .05
        if not self.b_bot.go_to_pose_goal(approach_pose, speed=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True):
          rospy.logerr("Fail to go to pose approach_pose")
          return False
        if not self.b_bot.go_to_pose_goal(hole_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True):
          rospy.logerr("Fail to go to pose hole_pose")
          return False
        self.confirm_to_proceed("Screw tool on hole. Press enter to move back.")
        if not self.b_bot.go_to_pose_goal(approach_pose, speed=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin = True):
          rospy.logerr("Fail to go to pose approach_pose")
          return False
        self.confirm_to_proceed("Did it go back?")
      if not self.b_bot.go_to_named_pose("horizontal_screw_ready"):
        rospy.logerr("Fail to go to horizontal_screw_ready")
        return False
      if not fake_execution_for_calibration:
        self.unequip_tool("b_bot", "screw_tool_m4")

    # ==========================================================

    if task_name == "motor pulley":
      success = self.pick_and_insert_motor_pulley(task = "taskboard")
      self.unlock_base_plate()  # To ensure that nothing moved due to a failed insertion
      self.lock_base_plate()
      self.b_bot.gripper.forget_attached_item()
      self.despawn_object("motor_pulley")
      return success

    # ==========================================================

    if task_name == "bearing":
      success = self.pick_up_and_insert_bearing(task="taskboard")
      self.a_bot.gripper.forget_attached_item()
      self.b_bot.gripper.forget_attached_item()
      if success:
        success = self.align_bearing_holes(task="taskboard")
      self.despawn_object("bearing")
      return success
      
    if task_name == "screw_bearing":
      self.equip_tool('b_bot', 'screw_tool_m4')
      success = self.fasten_bearing(task="taskboard")
      self.unequip_tool('b_bot', 'screw_tool_m4')
      return success
    
    # ==========================================================

    if task_name == "shaft":
      success = self.pick_and_insert_shaft("taskboard")
      self.b_bot.gripper.forget_attached_item()
      self.despawn_object("taskboard_idler_pulley_small")
      return success
    
    # ==========================================================
    
    if task_name == "idler pulley":
      success = self.pick_and_insert_idler_pulley("taskboard", simultaneous=simultaneous)
      self.despawn_object("taskboard_idler_pulley_small")
      self.a_bot.gripper.forget_attached_item()
      return success
