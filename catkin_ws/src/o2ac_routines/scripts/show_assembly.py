#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Team o2ac
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
#  * Neither the name of Team o2ac nor the names of its
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

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi, radians, sin, cos
tau = 2.0*pi  # Part of math from Python 3.6
import math
import time
import numpy as np
import random

from o2ac_msgs.srv import *
import moveit_msgs.msg
import visualization_msgs.msg

from ur_control import conversions, transformations

from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_assembly_database.assembly_reader import AssemblyReader
from o2ac_routines.assembly import O2ACAssembly
import o2ac_routines.helpers as helpers

import moveit_commander

import subprocess
import os
import sys, signal
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
  try:
    rospy.init_node('assembly_visualization', anonymous=False)
    assembly_database = AssemblyReader()
    assembly_marker_publisher = rospy.Publisher("o2ac_assembly_markers", visualization_msgs.msg.Marker, queue_size = 100)
    planning_scene_interface = moveit_commander.PlanningSceneInterface(synchronous=True)
    header_frame = ""

    def reset_scene():
      planning_scene_interface.remove_attached_object()  # Detach objects
      rospy.sleep(0.5) # Wait half a second for the detach to finish, just in case
      planning_scene_interface.remove_world_object()  # Clear all objects

    def set_assembly(assembly_name):
      assembly_database.change_assembly(assembly_name)
      pose = geometry_msgs.msg.PoseStamped()
      pose.header.frame_id = 'attached_base_origin_link'
      pose.pose.orientation.w = 1.0
      assembly_database.publish_assembly_frames(pose, prefix="assembled_")

    assembly_marker_id_counter = 0
    def publish_part_in_assembled_position(object_name, marker_only=False):
      if marker_only:
        marker = assembly_database.get_assembled_visualization_marker(object_name)
        assembly_marker_publisher.publish(marker)
        assembly_marker_id_counter += 1
        return True

      object_id = assembly_database.name_to_id(object_name)
      collision_object = assembly_database.get_collision_object(object_name, use_simplified_collision_shapes=False)
      collision_object.header.frame_id = "assembled_part_" + str(object_id).zfill(2)  # Fill with leading zeroes
      planning_scene_interface.apply_collision_object(collision_object)

    def spawn_panel(panel_name = "panel_bearing", offset_xyz=[0,0,0], offset_rpy=[0,0,0]):
      co = assembly_database.get_collision_object(panel_name)
      co.header.frame_id = "tray_center"
      co.pose = conversions.to_pose([offset_xyz[0], offset_xyz[1], 0.001+offset_xyz[2], 
                                     offset_rpy[0], offset_rpy[1], +offset_rpy[2]])
      planning_scene_interface.apply_collision_object(co)
      planning_scene_interface.allow_collisions(panel_name, "")

    set_assembly("wrs_assembly_2020")

    while True:
      rospy.loginfo("Enter load2020, load2021, load2021flipped, load2019surprise to change assembly.")
      rospy.loginfo("Enter 31, 32 to spawn panels.")
      rospy.loginfo("Enter 4, 4idler, 4bearing, 4motor to publish parts at target position.")
      rospy.loginfo("Enter 55, 551, 552 to spawn parts in example layout.")
      rospy.loginfo("Enter reset to reset the scene")
      rospy.loginfo("Enter 'robots' to load robot controllers")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      tic_start = time.time()
      if i == "load2020":
        set_assembly("wrs_assembly_2020")
      if i == "load2021":
        set_assembly("wrs_assembly_2021")
      if i == "load2021flipped":
        set_assembly("wrs_assembly_2021_flipped")
      if i == "load2019surprise":
        set_assembly("wrs_assembly_2019_surprise")
      # if i == "loadsurprise":
      #   set_assembly("wrs_assembly_2021_surprise")

      if i == "3":
        spawn_panel(offset_xyz=[-0.05 , -0.05, 0], offset_rpy=[tau/4, 0, 0])
      if i == "31":
        i = str(random.choice(range(311, 315)))
      if i == "311":
        spawn_panel("panel_bearing", offset_xyz=[-.02, 0.03, 0], offset_rpy=[tau/4, 0, 0])
      if i == "312":
        spawn_panel("panel_bearing", offset_xyz=[-.05, -0.02, 0], offset_rpy=[0, 0, 0])
      if i == "313":
        spawn_panel("panel_bearing", offset_xyz=[.04, -0.03, 0], offset_rpy=[0, 0, tau/2])
      if i == "314":
        spawn_panel("panel_bearing", offset_xyz=[-.01, 0.03, 0], offset_rpy=[0, 0, tau/4])
      if i == "32":
        i = str(random.choice(range(321, 325)))
      if i == "321":
        spawn_panel("panel_motor", offset_xyz=[-.02, 0.03, 0], offset_rpy=[tau/4, 0, 0])
      if i == "322":
        spawn_panel("panel_motor", offset_xyz=[-.05, -0.02, 0], offset_rpy=[0, 0, 0])
      if i == "323":
        spawn_panel("panel_motor", offset_xyz=[.04, -0.03, 0], offset_rpy=[0, 0, tau/2])
      if i == "324":
        spawn_panel("panel_motor", offset_xyz=[-.01, 0.03, 0], offset_rpy=[0, 0, tau/4])
      
      if i == "4":
        parts = ['base', 'panel_motor', 'panel_bearing', 'motor', 'motor_pulley', 'bearing',
          'shaft', 'end_cap', 'bearing_spacer', 'output_pulley', 'idler_spacer', 'idler_pulley', 'idler_pin']
        for part in parts:
          publish_part_in_assembled_position(part)
      elif i == "4base":
        parts = ['base', 'panel_motor', 'panel_bearing']
        for part in parts:
          publish_part_in_assembled_position(part)
      elif i == "4idler":
        parts = ['idler_spacer', 'idler_pulley', 'idler_pin']
        for part in parts:
          publish_part_in_assembled_position(part)
      elif i == "4bearing":
        parts = ['bearing', 'shaft', 'end_cap', 'bearing_spacer', 'output_pulley']
        for part in parts:
          publish_part_in_assembled_position(part)
      elif i == "4motor":
        parts = ['motor', 'motor_pulley']
        for part in parts:
          publish_part_in_assembled_position(part)
      elif i == "41":
        publish_part_in_assembled_position("base")
      elif i == "421":
        publish_part_in_assembled_position("panel_bearing")
      elif i == "422":
        publish_part_in_assembled_position("panel_motor")
      elif i == "43":
        publish_part_in_assembled_position("bearing")
      elif i == "44":
        publish_part_in_assembled_position("motor")
      elif i == "45":
        publish_part_in_assembled_position("shaft")
        publish_part_in_assembled_position("endcap")
      elif i == "46":
        publish_part_in_assembled_position("pulley")
      elif i == '55':
        spawn_objects_for_demo(base_plate_in_tray=True, layout_number=2)
      elif i == '551':
        spawn_objects_for_demo(layout_number=4)
      elif i == '552':
        spawn_objects_for_demo(base_plate_in_tray=True, layout_number=1)
      elif i == '553':
        spawn_objects_for_demo(base_plate_in_tray=True, layout_number=3)
      elif i == "reset":
        reset_scene()
      elif i == 'x':
        break
      elif i == "":
        continue
      
      ###### Planning section
      def plan(panel_name = "panel_bearing", mesh_filename = "03-PANEL2.stl", plan_type = "grasp_8"):
        demo_sh_command = "rosrun o2ac_pose_distribution_updater print_scene " + panel_name + " " + mesh_filename +  " " + plan_type + " > tmp0 && echo Planning starts && rosrun o2ac_pose_distribution_updater planner_test tmp0 > tmp1 && echo Visualization of the plan starts && rosrun o2ac_pose_distribution_updater simulation tmp1"
        os.system(demo_sh_command)
        print("================")
      if i == "plan11":
        plan("panel_bearing", "03-PANEL2.stl", "any")
      elif i == "plan12":
        plan("panel_bearing", "03-PANEL2.stl", "placed")
      elif i == "plan13":
        plan("panel_bearing", "03-PANEL2.stl", "grasp_8")
      
      if i == "plan21":
        plan("panel_bearing", "02-PANEL.stl", "any")
      elif i == "plan22":
        plan("panel_bearing", "02-PANEL.stl", "placed")
      elif i == "plan23":
        plan("panel_bearing", "02-PANEL.stl", "grasp_8")

      ###### Robot section 
      if i == "robots":
        c = O2ACAssembly()
        tray_views = c.define_local_tray_views(robot_name="a_bot")
      if i == "gopose":
        c.a_bot.go_to_pose_goal(tray_views[0], end_effector_link="", speed=.8, acceleration=.5)
      elif i == "pickplacebearingpanel":
        if not c.pick_panel_with_handover("panel_bearing", simultaneous=False):
          break
        c.center_panel("panel_bearing", store=True)
        c.place_panel("a_bot", "panel_bearing", pick_again=True, fake_position=True)
      elif i == "pickplacebearingpanelwithuncertainty" or i == "ppbu":
        pose_with_uncertainty=geometry_msgs.msg.PoseWithCovarianceStamped()
        if not c.pick_panel_with_handover("panel_bearing", simultaneous=False, pose_with_uncertainty=pose_with_uncertainty):
          break
        c.center_panel_with_uncertainty("panel_bearing", store=True, pose_with_uncertainty=pose_with_uncertainty)
        c.place_panel("a_bot", "panel_bearing", pick_again=True, fake_position=True, pose_with_uncertainty=pose_with_uncertainty)
      elif i == "pickmotorpanel":
        c.pick_panel_with_handover("panel_motor", simultaneous=False)
      elif i == "pickplacemotorpanel":
        if not c.pick_panel_with_handover("panel_motor", simultaneous=False):
          break
        c.center_panel("panel_motor", store=True)
        c.place_panel("a_bot", "panel_motor", pick_again=True, fake_position=True)
      print("This took: %.3f seconds" % (time.time() - tic_start))
  except rospy.ROSInterruptException:
    pass
