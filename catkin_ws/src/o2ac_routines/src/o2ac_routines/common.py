#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, OMRON SINIC X
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

from o2ac_routines.base import *
from math import radians, degrees, sin, cos, pi
from ur_control.constants import TERMINATION_CRITERIA
import numpy as np

class O2ACCommon(O2ACBase):
  """
  This class contains the higher-level routines 
  """
  def __init__(self):
    super(O2ACCommon, self).__init__()
    self.rospack = rospkg.RosPack()

    self.small_item_ids = [8,9,10,14]
    self.large_item_ids = [1,2,3,4,5,7,11,12,13]
    self.belt_id        = [6]
    
    self.define_tray_views()

  def define_tray_views(self):
    """
    Define the poses used to position the camera to look into the tray.

    Example usage: self.b_bot.go_to_pose_goal(self.tray_view_high, 
                                        end_effector_link="b_bot_outside_camera_color_frame", 
                                        speed=.1, acceleration=.04)
    """
    high_height = .37
    low_height = .22
    x_offset = .04  # At low_height
    y_offset = .07  # At low_height

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "tray_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    ps.pose.position.z = high_height

    # Centered views (high up and close)
    self.tray_view_high = copy.deepcopy(ps)
    ps.pose.position.z = low_height
    self.tray_view_low = copy.deepcopy(ps)

    # Close views in corners
    ps.pose.position.x = x_offset
    ps.pose.position.y = y_offset
    self.tray_view_close_front_b = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = y_offset
    self.tray_view_close_back_b = copy.deepcopy(ps)
    ps.pose.position.x = x_offset
    ps.pose.position.y = -y_offset
    self.tray_view_close_front_a = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = -y_offset
    self.tray_view_close_back_a = copy.deepcopy(ps)

    self.close_tray_views = [self.tray_view_low, self.tray_view_close_front_b, self.tray_view_close_back_b, self.tray_view_close_front_a, self.tray_view_close_back_a]
    self.close_tray_views_rot_left = [rotatePoseByRPY(radians(20),0,0, pose) for pose in self.close_tray_views]
    self.close_tray_views_rot_right = [rotatePoseByRPY(radians(-20),0,0, pose) for pose in self.close_tray_views]
    self.close_tray_views_rot_left_more = [rotatePoseByRPY(radians(50),0,0, pose) for pose in self.close_tray_views]
    self.close_tray_views_rot_left_90 = [rotatePoseByRPY(radians(90),0,0, pose) for pose in self.close_tray_views]

  def publish_part_in_assembled_position(self, object_name, test_header_frame=""):
    """ Move or publish a part as a collision object in its final assembled position.
        This is used to "finish" assembling a part.
    """
    # Remove from scene or detach from robot
    self.planning_scene_interface.remove_attached_object(object_name)
    object_id = self.assembly_database.name_to_id(object_name)
    collision_object = self.assembly_database.get_collision_object(object_name)
    if test_header_frame:
      collision_object.header.frame_id = test_header_frame
    else:
      collision_object.header.frame_id = "assembled_part_" + str(object_id).zfill(2)  # Fill with leading zeroes
    self.planning_scene_interface.apply_collision_object(collision_object)
    return
    

  ######## Higher-level routines used in both assembly and taskboard

  def load_MTC_solution(self, solution_file):
    """Load the result of a motion-plan from a file."""
    if not solution_file == '':
      # Load the solution
      file = self.rospack.get_path('o2ac_routines') + '/MP_solutions/' + solution_file
      with open(file, 'rb') as f:
        result = pickle.load(f)
    return result

  def execute_MTC_solution(self, solution, speed = 1.0):
    """
    Execute the result of a task plan.
    The type of the input 'solution' is mtc_msgs.msg.Solution
    """
    
    skip_stage_execution = False   # True while a subroutine like "equip/unequip" is active
    success = False
    screw_counter = 0
    if speed > 1.0:
      speed = 1.0
    start_state = self.robots.get_current_state()
    currently_attached_collision_objects = start_state.attached_collision_objects
    for sub_trajectory in solution.sub_trajectory:
      stage_name = sub_trajectory.info.creator_name
      if stage_name == 'Fasten screw (dummy)':
        rospy.sleep(2)

      # If a "start" block is encountered, skip the trajectories inside and execute the actual action they represent
      if stage_name == 'equip_tool_m4_start':
        skip_stage_execution = True
        self.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if stage_name == 'unequip_tool_m4_start':
        skip_stage_execution = True
        self.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if stage_name == 'pick_screw_m4_start':
        skip_stage_execution = True
        self.pick_screw_from_feeder('b_bot', screw_size=4)
      if stage_name == 'fasten_screw_m4_start':
        skip_stage_execution = True
        target_pose = geometry_msgs.msg.PoseStamped()
        if screw_counter == 0:
          target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_1'
          screw_counter += 1
        else:
          target_pose.header.frame_id = 'move_group/base/screw_hole_panel2_2'
        target_pose.pose.orientation.w = 1
        self.fasten_screw('b_bot', target_pose)
      
      # Resume executing the trajectories
      if (stage_name == 'equip_tool_m4_end' or stage_name == 'unequip_tool_m4_end' or stage_name == 'pick_screw_m4_end' or 
         stage_name == 'fasten_screw_m4_end'):
        skip_stage_execution = False
        currently_attached_collision_objects = self.robots.get_current_state().attached_collision_objects
        # The collision objects need to be updated because they might have changed during the trajectory
        continue
      
      # Check for any self-contained stages to be executed
      if stage_name == 'push plate with b_bot' and not skip_stage_execution:
        # Execute positioning UR program
        self.b_bot.load_and_execute_program(program_name="wrs2020_push_motor_plate.urp", wait=True)
        continue
      if stage_name == 'move a_bot right wrs_subtask_motor_plate':
        self.move_lin_rel("a_bot", relative_translation=[0, -0.02, 0], relative_to_robot_base=False, max_wait=5.0)
      if stage_name == 'move a_bot back wrs_subtask_motor_plate':
        self.move_lin_rel("a_bot", relative_translation=[0,  0.02, 0], relative_to_robot_base=False, max_wait=5.0)

      # Execute trajectories
      if sub_trajectory.scene_diff.robot_state.joint_state.name and not skip_stage_execution:  # If the robot state is changed (robot moved, object attached/detached)
        # Update attached collision objects
        if not currently_attached_collision_objects == sub_trajectory.scene_diff.robot_state.attached_collision_objects:
          coll_objs_to_detach = [collision_object for collision_object in currently_attached_collision_objects if collision_object not in sub_trajectory.scene_diff.robot_state.attached_collision_objects]
          coll_objs_to_attach = [collision_object for collision_object in sub_trajectory.scene_diff.robot_state.attached_collision_objects if collision_object not in currently_attached_collision_objects]
          for attached_object in coll_objs_to_detach:
            # print('Detaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            robot_name_ = attached_object.link_name[:5]
            self.active_robots[robot_name_].detach_object(attached_object_name)
          for attached_object in coll_objs_to_attach:
            # print('Attaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            robot_name_ = attached_object.link_name[:5]
            self.active_robots[robot_name_].robot_group.attach_object(attached_object_name, attached_object.link_name, touch_links=  # MODIFY attach_tool in base.py to attach_object ++ ROBOT NAME???
              [robot_name_ + "_gripper_tip_link", 
              robot_name_ + "_left_inner_finger_pad", 
              robot_name_ + "_left_inner_finger", 
              robot_name_ + "_left_inner_knuckle",
              robot_name_ + "_right_inner_finger_pad", 
              robot_name_ + "_right_inner_finger",
              robot_name_ + "_right_inner_knuckle"])
            currently_attached_collision_objects.append(attached_object)
          currently_attached_collision_objects = [attached_collision_object for attached_collision_object in currently_attached_collision_objects if attached_collision_object not in coll_objs_to_detach]

        # Skip stage if joint_names is empty, because the stage performs no motions
        if not sub_trajectory.trajectory.joint_trajectory.joint_names: 
          continue

        # Execute stage
        robot_name = sub_trajectory.trajectory.joint_trajectory.joint_names[0][:5]
        arm_group = self.active_robots[robot_name].robot_group
        if len(sub_trajectory.trajectory.joint_trajectory.joint_names) == 1:  # If only one joint is in the group, it is the gripper
          # Gripper motion
          hand_group = self.active_robots[robot_name].gripper_group
          hand_closed_joint_values = hand_group.get_named_target_values('close')
          hand_open_joint_values = hand_group.get_named_target_values('open')
          if stage_name == 'open hand':
            self.active_robots[robot_name].gripper.send_command('open')
          elif stage_name == 'close hand':
            self.active_robots[robot_name].gripper.send_command('close')
          elif 0.01 > abs(hand_open_joint_values[sub_trajectory.trajectory.joint_trajectory.joint_names[0]] - sub_trajectory.trajectory.joint_trajectory.points[-1].positions[0]):
            self.active_robots[robot_name].gripper.send_command('open')
          elif 0.01 < abs(hand_open_joint_values[sub_trajectory.trajectory.joint_trajectory.joint_names[0]] - sub_trajectory.trajectory.joint_trajectory.points[-1].positions[0]):
            self.active_robots[robot_name].gripper.send_command('close', True)
          
        else: # The robots move
          # First check that the trajectory is safe to execute (= robot is at start of trajectory)
          if not all_close(self.active_robots[robot_name].robot_group.get_current_joint_values(), 
                            sub_trajectory.trajectory.joint_trajectory.points[0].positions,
                            0.01):
            rospy.logerr("Robot " + robot_name + " is not at the start of the next trajectory! Aborting.")
            rospy.logerr("Stage name: " + stage_name)
            rospy.logwarn("self.active_robots[robot_name].robot_group.get_current_joint_values():")
            print(self.active_robots[robot_name].robot_group.get_current_joint_values())
            rospy.logwarn("sub_trajectory.trajectory.joint_trajectory.points[0].positions:")
            print(sub_trajectory.trajectory.joint_trajectory.points[0].positions)
            return False

          # Prepare robot motion
          self.active_robots[robot_name].activate_ros_control_on_ur()
          plan = arm_group.retime_trajectory(self.robots.get_current_state(), sub_trajectory.trajectory, 
                                             velocity_scaling_factor=speed, acceleration_scaling_factor=speed/2)
          arm_group.set_max_velocity_scaling_factor(speed)

          # Execute
          plan_success = arm_group.execute(plan, wait=True)
          success = success and plan_success
    return True

  def pick_and_move_object_with_robot(self, item_name, item_target_pose, robot_name, speed=0.1):
    """This function picks the item and move it to the target pose.
    It needs to be in the planning scene as a collision object."""
    # TODO: Implement this with MTC
    success = False
    return success
  

  ####### Vision

  def look_for_item_in_tray(self, item_name, robot_name="b_bot"):
    """
    This function will look for an item in the tray. After calling this function, the item
    is published to the planning scene.
    """

    # Look from top first
    self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.15)
    success = self.detect_object_in_camera_view(item_name)

    # # TODO: Also move robot if object requires a close-up view (shaft, pin...)
    # if not success:
    #   poses = [self.tray_view_close_front_b, self.tray_view_close_back_b, self.tray_view_close_front_a, self.tray_view_close_back_a]
    #   for pose in poses:
    #     self.active_robots[robot_name].go_to_pose_goal(pose, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    #     success = self.detect_object_in_camera_view(item_name)
    #     if success:
    #       break

    if not success:
      rospy.logdebug("Failed to find " + item_name)
    else:
      rospy.logdebug("Found " + item_name)
    return success

  def adjust_tip_position_visually(self, robot_name, use_inside_camera=False, use_outside_camera=False):
    """WIP: This function will adjust the position of the robot so that the visible tip of the object
    or tool is aligned with the target hole.
    Only one camera can be used.
    use_inside_camera should be True for grasped parts (and maybe the set screw tool).
    use_outside_camera should be True for the screw tools."""
    # TODO: Call a vision action
    success = False
    return success

  def touch_environment_with_grasped_object_to_reduce_uncertainty(self, robot_name, initial_uncertainty=[0,0,0]):
    """WIP: This touches the environment with a recently grasped object to ascertain its position."""
    # TODO: Use in-hand pose estimation particle filter
    success = False
    return success

  ########

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=0.3, speed_slow=0.02, gripper_command="close", 
          gripper_force=40.0, grasp_width=0.140,
          approach_height=0.05, item_id_to_attach = "", lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.1, 
          gripper_velocity = .1, axis="x", sign=+1):
    """
    This function (outdated) performs a grasp with the robot, but it is not updated in the planning scene.
    It does not use the object in simulation. It can be used for simple tests and prototyping, but should
    be replaced by the pick() function for the real competition.

    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    The attachment will be visible in the MoveIt planning scene. The object and its subframes can be used
    as an end effector.
    """
    
    robot = self.active_robots[robot_name]
    if gripper_command=="do_nothing":
      pass
    else: 
      robot.gripper.send_command(command=grasp_width, wait=False) # Open

    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    if axis =="x":
      object_pose.pose.position.x += approach_height * sign
    if axis =="z":
      object_pose.pose.position.z += approach_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= approach_height * sign
    if axis =="z":
      object_pose.pose.position.z -= approach_height * sign

    rospy.loginfo("Moving down to object")
    if axis =="x":
      object_pose.pose.position.x += grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z += grasp_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z -= grasp_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      robot.gripper.send_command(command="close", force = gripper_force, velocity = gripper_velocity)

    if item_id_to_attach:
      try:
        robot.robot_group.attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_gripper_tip_link", 
        robot_name + "_robotiq_85_left_finger_tip_link", 
        robot_name + "_robotiq_85_left_inner_knuckle_link", 
        robot_name + "_robotiq_85_right_finger_tip_link", 
        robot_name + "_robotiq_85_right_inner_knuckle_link"])
      except:
        rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

    if lift_up_after_pick:
      rospy.sleep(1.0)
      rospy.loginfo("Going back up")

      if axis =="x":
        object_pose.pose.position.x += approach_height * sign
      if axis =="z":
        object_pose.pose.position.z += approach_height * sign
      rospy.loginfo("Going to height " + str(object_pose.pose.position.z))
      self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
      if axis =="x":
        object_pose.pose.position.x -= approach_height * sign
      if axis =="z":
        object_pose.pose.position.z -= approach_height * sign
    return True

  def simple_place(self, robot_name, object_pose, place_height=0.05, speed_fast=0.1, speed_slow=0.02, 
      gripper_command="open", approach_height=0.05, item_id_to_detach = "", lift_up_after_place = True, acc_fast=1.0, acc_slow=.1):
    """
    A very simple place operation. item_id_to_detach is used to update the planning scene by
    removing an item that has been attached (=grasped) by the robot in the MoveIt planning scene.
    It is ignored if empty.

    This procedure works by changing the x axis of the target pose's frame. 
    It may produce dangerous motions in other configurations.
    """
    #self.skill_server.publish_marker(object_pose, "place_pose")
    self.log_to_debug_monitor("Place", "operation")
    rospy.loginfo("Going above place target")
    object_pose.pose.position.x -= approach_height
    self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=False)
   
    rospy.loginfo("Moving to place target")
    object_pose.pose.position.x += place_height
    self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=False)
    object_pose.pose.position.x -= place_height
    
    robot = self.active_robots[robot_name]

    #gripper open
    if gripper_command=="do_nothing":
      pass
    else: 
      robot.gripper.send_command(gripper=robot_name, command="open")

    if item_id_to_detach:
      robot.robot_group.detach_object(item_id_to_detach)

    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_fast, move_lin=False)  
    return True

  def simple_grasp_generation(self, object_pose, grasp_width=0.08, grasp_z_height=0.0):
    """
    Returns a list of one grasp for an object.
    Based only on border distance and distance to other objects.
    """
    # This grasp pose opens the gripper along the workspace_center's y-axis
    grasp_along_y = copy.deepcopy(object_pose)
    grasp_along_y.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    grasp_along_y.pose.position.z = 0.02

    # This one opens the gripper along the workspace_center's x-axis
    grasp_along_x = copy.deepcopy(grasp_along_y)
    grasp_along_x.pose = rotatePoseByRPY(pi/4, 0, 0, grasp_along_x.pose)
    dist_far = 0.05  # Along the gripper's opening direction
    dist_close = 0.015  # In the direction that the gripper does not open
    
    grasp = []
    
    (dx, dy) = self.distances_from_tray_border(grasp_along_y)
    if dy > dist_far and dx > dist_close:
      grasp_pose = grasp_along_y
      direction = "y"
    else:
      (dx, dy) = self.distances_from_tray_border(grasp_along_x)
      if dx > dist_far and dy > dist_close:
        grasp_pose = grasp_along_x
        direction = "x"
    
    if not grasp_pose:
      rospy.loginfo("too close to border. discarding. border distance was: " + str(d))
      return False
    
    for obj, pose in self.objects_in_tray.items():
      if obj == 6: # Skip the belt
        continue
      dx = abs(pose.pose.position.x - grasp_pose.pose.position.x)
      dy = abs(pose.pose.position.y - grasp_pose.pose.position.y)
      if dx < 1e-4 and dy < 1e-4:
        continue  # It's the item itself or a duplicate
      item_too_close = False
      if direction == "y":
        if dx < 0.015 and dy < 0.04:
          item_too_close = True
      elif direction == "x":
        if dy < 0.015 and dx < 0.04:
          item_too_close = True
      if item_too_close:
        rospy.loginfo("too close to another item. discarding. distance: " + str(pose_dist(pose.pose, grasp_pose.pose)) + ", id: " + str(obj))
        return False
    return [grasp_pose]
  
  def pick_from_two_poses_topdown(self, robot_name, object_name, object_pose):
    """ Plan a pick from above and execute it with MTC.
        This is meant for cylindrical objects like the bearing.
        Two grasp poses are evaluated. The gripper opens along either x and y and faces down.
        An object with the name `object_name` has to be placed in the scene
        
        object_pose is the PoseStamped of the object center. The orientation is ignored.
    """
    grasp_poses = []
    pose_in_tray = self.listener.transformPose("tray_center", object_pose)
    
    orientation_grasp_along_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    if robot_name == "b_bot":
      orientation_grasp_along_y = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, -tau/4))
    else:  # a_bot
      orientation_grasp_along_y = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, tau/4))
    
    grasp_along_x = copy.deepcopy(pose_in_tray)
    grasp_along_x.pose.orientation = orientation_grasp_along_x
    grasp_along_y = copy.deepcopy(pose_in_tray)
    grasp_along_y.pose.orientation = orientation_grasp_along_y

    grasp_poses = [grasp_along_x, grasp_along_y]
    return self.pick_MTC_helper(robot_name, object_name, grasp_poses)
  
  def pick(self, robot_name, object_name, grasp_name=""):
    """ Plan a pick operation and execute it with MTC.
        grasp_name is the name of a grasp on the parameter server.
    """
    if not grasp_name:
      grasp_name = "default_grasp"
    grasp_pose = self.assembly_database.get_grasp_pose(object_name, grasp_name)
    if not grasp_pose:
      rospy.logerr("Could not load grasp pose " + grasp_name + " for object " + object_name + ". Aborting pick.")
      return False
    return self.pick_MTC_helper(robot_name, object_name, [grasp_pose])
    
  def pick_MTC_helper(self, robot_name, object_name, grasp_poses):
    """ Plan a pick operation and execute it with MTC.
        grasp_poses is a vector of grasp poses.
    """
    res = self.plan_pick_place(robot_name, object_name, grasp_poses)
    success = True
    try:
      success = res.success
    except:
      success = False
    if not success:
      rospy.logerr("Could not plan pick for object " + object_name)
      return False

    return self.execute_MTC_solution(res.solution, speed=0.1)

  def simple_grasp_sanity_check(self, grasp_pose, grasp_width=0.08, border_dist=0.06):
    """
    Returns true if the grasp pose is further than 5 cm away from the tray border,
    and no other detected objects are closer than 5 cm.

    grasp_pose is a PoseStamped.
    """
    (dx, dy) = self.distance_from_tray_border(grasp_pose)
    if dx < border_dist and dy < border_dist:
      rospy.loginfo("too close to border. discarding. border distance was: " + str(d))
      return False
    for obj, pose in self.objects_in_tray.items():
      if obj == 6: # Hard-code skipping the belt
        rospy.logwarn("Skipping the belt during grasp check")
        continue
      if pose_dist(pose.pose, grasp_pose.pose) < 0.05:
        if pose_dist(pose.pose, grasp_pose.pose) < 1e-6:
          continue  # It's the item itself or a duplicate
        rospy.loginfo("too close to another item. discarding. distance: " + str(pose_dist(pose.pose, grasp_pose.pose)) + ", id: " + str(obj))
        return False
    return True
  
  def is_grasp_pose_feasible(self, grasp_pose, border_dist=0.08):
    # TODO: Consider the grasp width and actual collisions using the PlanningScene
    return self.simple_grasp_sanity_check(grasp_pose, border_dist)
    
  def get_feasible_grasp_points(self, object_in_scene):
    """
    Returns a list of PoseStamped grasp points for an object that is currently in the scene.
    object_in_scene can be the string or the id number of the object.
    """
    if isinstance(object_in_scene, str):
      object_id = self.assembly_database.name_to_id(object_in_scene)
    else:
      object_id = object_in_scene

    if object_id not in self.objects_in_tray:
      rospy.logerr("Grasp points requested for " + str(object_id) + " but it is not seen in tray.")
      return False
    
    if object_id in self.belt_id:
      res = self.get_3d_poses_from_ssd()
      grasp_poses = []
      for idx, pose in enumerate(res.poses):
        if res.class_ids[idx] == 6:
          if self.is_grasp_pose_feasible(pose, border_dist=0.05):
            grasp_poses.append(pose)
      return grasp_poses

    if object_id in self.small_item_ids:
      # For the shaft, use the orientation from the SSD
      if self.assembly_database.id_to_name(object_id) == "shaft":
        return [self.objects_in_tray[object_id]]
      # For other small items, use any pose from above that works
      return self.simple_grasp_generation(object_pose=self.objects_in_tray[object_id], grasp_z_height=0.0)
      # TODO: Consider the idler spacer, which can stand upright or lie on the side.
      
    if object_id in self.large_item_ids:
      # For large items, use any pose from above that works
      # TODO: Get grasp poses from database
      return self.simple_grasp_generation(object_pose=self.objects_in_tray[object_id], grasp_z_height=0.02)
    
    return False
  
  def look_and_get_grasp_point(self, object_id):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """
    if not self.use_real_robot: # For simulation
      rospy.logwarn("Returning position near center (simulation)")
      return conversions.to_pose_stamped("tray_center", [-0.03078, 0.06248, 0.02, 0.0,0.7071,0.0,0.7071])

    # Make sure object_id is the id number
    if isinstance(object_id, str):
      try:
        object_id_num = self.assembly_database.name_to_id(object_id)
      except:
        object_id_num = []
      if not object_id_num:
        rospy.logerr("Could not find object id " + object_id + " in database!")
        return False
      rospy.logwarn("look_and_get_grasp_point got " + object_id + " but will use id number " + str(object_id_num))
      return self.look_and_get_grasp_point(object_id_num)

    self.vision.activate_camera("b_bot_outside_camera")
    self.activate_led("b_bot")
    # TODO: Merge with detect_object_in_camera_view in base.py
    # self.b_bot.go_to_pose_goal(self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.1)
    # self.get_3d_poses_from_ssd()
    if object_id in self.objects_in_tray:
      del self.objects_in_tray[object_id]

    if not object_id in self.objects_in_tray:
      # for close_view_batch in [self.close_tray_views, self.close_tray_views_rot_left, self.close_tray_views_rot_right]:
      for view in [self.tray_view_high] + self.close_tray_views + self.close_tray_views_rot_left + self.close_tray_views_rot_right + self.close_tray_views_rot_left_more + self.close_tray_views_rot_left_90:
        if rospy.is_shutdown():
          break
        self.b_bot.go_to_pose_goal(view, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.3)
        rospy.sleep(0.3)
        self.get_3d_poses_from_ssd()
        if object_id in self.objects_in_tray:
          rospy.loginfo("Getting grasp points for object_id : " + str(object_id) + " at pose:")
          rospy.loginfo(self.objects_in_tray[object_id].pose.position)
          grasp_points = self.get_feasible_grasp_points(object_id)
          # Get another view from up close (if the view was already close, the object may have been on the edge of the image). 
          # TODO: Use another function to confirm item position and get best grasp point
          close_view = self.listener.transformPose("tray_center", self.objects_in_tray[object_id])
          close_view.pose.position.x += 0.0  # To avoid noise from direct reflections of the structured light
          close_view.pose.position.z = copy.deepcopy(self.close_tray_views[0].pose.position.z)
          close_view.pose.orientation = copy.deepcopy(view.pose.orientation)
          
          rospy.loginfo("Looking closer at object_id " + str(object_id) + " at pose: ")
          rospy.loginfo(self.objects_in_tray[object_id].pose.position)
          self.b_bot.go_to_pose_goal(close_view, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.3)
          rospy.sleep(0.5)
          self.get_3d_poses_from_ssd()
          grasp_points_close = self.get_feasible_grasp_points(object_id)
          if grasp_points_close and grasp_points:
            rospy.loginfo("Got a better grasp point by looking closer. Before: ")
            rospy.loginfo(grasp_points_close[0].pose.position)
            rospy.loginfo("After: ")
            rospy.loginfo(grasp_points[0].pose.position)
            grasp_point = grasp_points_close

          if grasp_points:
            return grasp_points[0]
          else:
            rospy.logerr("Could not find suitable grasp pose! Aborting.")
            return False
          break
    rospy.logerr("Could not find item id " + str(object_id) + " in tray!")
    return False

  def distances_from_tray_border(self, object_pose):
    """
    Returns the distance from the tray border as an (x, y) tuple.
    x, y are in the tray coordinate system.
    Distance is signed (negative is outside the tray).
    """
    # Inside tray width and length: 25.5 cm, 37.5 cm
    l_x_half = .255/2.0
    l_y_half = .375/2.0
    # print("object_pose = ")
    # print(object_pose)
    object_pose_in_world = self.listener.transformPose("tray_center", object_pose)
    xdist = l_x_half - abs(object_pose_in_world.pose.position.x)
    ydist = l_y_half - abs(object_pose_in_world.pose.position.y)
    return (xdist, ydist)
  
  def move_and_center_with_gripper(self, robot_name, object_pose, object_width):
    """
    Centers cylindrical object by moving the gripper, by moving the robot to the pose and closing/opening.
    Rotates once and closes/opens again. Does not move back afterwards.
    """
    robot = self.active_robots[robot_name]
    object_pose_in_world = self.listener.transformPose("world", object_pose)
    object_pose_in_world.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    robot.go_to_pose_goal(object_pose_in_world, move_lin=True)

    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.1)
    robot.gripper.send_command(command=object_width+0.02, force = 90.0, velocity = 0.001)
    
    object_pose_in_world_rotated = copy.deepcopy(object_pose_in_world)
    object_pose_in_world_rotated.pose = rotatePoseByRPY(0,0,tau/4, object_pose_in_world_rotated.pose)
    
    robot.go_to_pose_goal(object_pose_in_world, move_lin=True)
    
    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.1)
    robot.gripper.send_command(command=object_width+0.02, force = 90.0, velocity = 0.001)
    
    robot.go_to_pose_goal(object_pose_in_world, move_lin=True)
    return True
  
  def center_with_gripper(self, robot_name, grasp_width):
    """
    Centers cylindrical object at the current location, by closing/opening the gripper and rotating the robot's last joint.
    Moves back to the initial position.
    """
    robot = self.active_robots[robot_name]
    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.1)
    robot.gripper.send_command(command=grasp_width+0.03, force = 90.0, velocity = 0.001)

    # rotate gripper 90deg
    initial_pose_goal = robot.robot_group.get_current_joint_values()
    pose_goal = robot.robot_group.get_current_joint_values()
    pose_goal[-1] += tau/4.0 
    success = robot.move_joints(pose_goal, speed=0.5, wait=True)
    if not success:
      rospy.logerr("Fail to rotate 90deg %s" % success)
      return False
    
    # close-open
    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.003)
    robot.gripper.send_command(command=grasp_width+0.03, force = 90.0, velocity = 0.001)

    # rotate gripper -90deg
    return robot.move_joints(initial_pose_goal, speed=0.5, wait=True)

  def centering_pick(self, robot_name, object_pose, speed=0.2, acc=0.01, object_width=0.08, approach_height=0.1, 
          item_id_to_attach = "", lift_up_after_pick=False, gripper_force=40.0):
    """
    This function picks an object with the robot directly from above, but centers the object with the gripper first.
    Should be used only for cylindrical objects.
    
    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    """
    robot = self.active_robots[robot_name]

    pick_pose = copy.deepcopy(object_pose)
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))

    approach_pose = copy.deepcopy(pick_pose)
    approach_pose.pose.position.z += approach_height

    success = robot.go_to_pose_goal(approach_pose, speed=speed, acceleration=acc, move_lin=True)
    if not success:
      rospy.logerr("Fail to complete approach pose")
      return False
    
    rospy.loginfo("Moving down to object")
    robot.gripper.send_command(command=object_width+0.03)
    
    success = robot.go_to_pose_goal(pick_pose, speed=speed, acceleration=acc, move_lin=True)
    if not success:
      rospy.logerr("Fail to complete pick pose")
      return False

    # Center object
    self.center_with_gripper(robot_name, object_width)

    # Grasp object
    robot.gripper.send_command(command="close", force = gripper_force)

    if item_id_to_attach:
      try:
        robot.robot_group.attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_gripper_tip_link", 
        robot_name + "_robotiq_85_left_finger_tip_link", 
        robot_name + "_robotiq_85_left_inner_knuckle_link", 
        robot_name + "_robotiq_85_right_finger_tip_link", 
        robot_name + "_robotiq_85_right_inner_knuckle_link"])
      except:
        rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

    if lift_up_after_pick:
      rospy.sleep(0.5)
      rospy.loginfo("Going back up")

      rospy.loginfo("Going to height " + str(approach_pose.pose.position.z))
      robot.go_to_pose_goal(approach_pose, speed=speed, acceleration=acc, move_lin=True)
    return True

  def drop_shaft_in_v_groove(self):
    """
    Places the shaft in the v groove with b_bot.
    """
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "vgroove_aid_drop_point_link"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*(0,0,0,1))
    ps.pose.position = geometry_msgs.msg.Point(-0.05, 0, 0)
    self.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_gripper_tip_link", move_lin = False)
    ps.pose.position = geometry_msgs.msg.Point(0, 0, 0)
    self.b_bot.go_to_pose_goal(ps, end_effector_link="b_bot_gripper_tip_link")

  def check_if_shaft_in_v_groove(self):
    """
    Returns True if the shaft is in the v_groove
    """
    look_at_shaft_pose = [2.177835941, -1.700065275, 2.536958996, -2.40987076, -1.529889408, 0.59719228]
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    self.b_bot.move_joints(look_at_shaft_pose)

    res = self.vision.call_shaft_notch_detection()
    print("=== shaft notch detection returned:")
    print(res)

  def turn_shaft_until_groove_found(self):
    look_at_shaft_pose = [2.177835941, -1.700065275, 2.536958996, -2.40987076, -1.529889408, 0.59719228]
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    self.b_bot.move_joints(look_at_shaft_pose)

    times_turned = 0
    success = self.b_bot.load_program(program_name="wrs2020/shaft_turning.urp", recursion_depth=3)  
    if not success:
      return False
    while times_turned < 6:
      rospy.loginfo("Turn shaft once")
      self.b_bot.execute_loaded_program()
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(10))
      times_turned += 1
      res = self.vision.call_shaft_notch_detection()
      if res.shaft_notch_detected_at_top or res.shaft_notch_detected_at_bottom:
        return res
    return False

  def look_at_motor(self):
    # b_bot_joint_angles = [1.9093738794326782, -1.1168301564506073, 1.8244155089007776, -0.8763039273074646, -1.244535271321432, 0.048961393535137177]
    # b_bot_outside_camera_optical_frame in vgroove_aid_lin: xyz: -0.011832; 0.13308; 0.085104 q: 0.83999; 0.0043246; 0.0024908; 0.54257
    camera_look_pose = geometry_msgs.msg.PoseStamped()
    camera_look_pose.header.frame_id = "vgroove_aid_link"
    camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0.84, 0.0043246, 0.0024908, 0.54257))
    camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.0118, 0.133, 0.0851)
    camera_look_pose.pose.position.z += 0.2
    self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    camera_look_pose.pose.position.z -= 0.2
    self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    angle = self.get_motor_angle()

  ######## Bearing

  def align_bearing_holes(self, max_adjustments=10, task=""):
    """
    Align the bearing holes.
    """
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot", on=False)
    adjustment_motions = 0
    times_looked_without_action = 0
    times_perception_failed_in_a_row = 0
    times_we_added_random_action = 0
    times_it_looked_like_success = 0
    
    grasp_pose = geometry_msgs.msg.PoseStamped()
    if task == "taskboard":
      grasp_pose.header.frame_id = "taskboard_bearing_target_link"
    elif task == "assembly":
      grasp_pose.header.frame_id = "assembled_part_07_inserted"
    else:
      rospy.logerr("Incorrect task argument, frame could be determined! Breaking out of align_bearing_holes.")
      return False
    grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0, 0, 0, 1.0))

    camera_look_pose = copy.deepcopy(grasp_pose)
    camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0.5, 0.5, 0.5, 0.5))
    camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.155, -0.005, -0.0)

    def rotate_bearing_by_angle(angle):
      self.b_bot.gripper.open()
      start_pose = copy.deepcopy(grasp_pose)
      start_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(-angle/2.0, 0, 0))
      end_pose = copy.deepcopy(grasp_pose)
      end_pose.pose.position.z += 0.0005  # Avoid pulling the bearing out little by little
      end_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(angle/2.0, 0, 0))
      self.b_bot.go_to_pose_goal(start_pose, speed=.1, acceleration=.04, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.b_bot.gripper.close()
      self.b_bot.go_to_pose_goal(end_pose, speed=.1, acceleration=.04, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.b_bot.gripper.open()

    success = False
    while adjustment_motions < max_adjustments:
      # Look at tb bearing
      if self.b_bot.gripper.opening_width < 0.06:
        self.b_bot.gripper.open()
      
      self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.25, acceleration=.1)
      if np.random.uniform() > 0.4:  # = 60% chance
        self.activate_led("b_bot", on=False)
      else:
        self.activate_led("b_bot", on=True)
      rospy.sleep(1)  # If we don't wait, the camera image is blurry

      # Get angle and turn
      angle = self.get_bearing_angle()
      if angle:
        rospy.loginfo("Bearing detected angle: %3f, try to correct", degrees(angle))
        times_perception_failed_in_a_row = 0
        if abs(degrees(angle)) > 3.0:
          if task == "assembly" and abs(degrees(angle)) > 29:
            rospy.logwarn("Limiting maximum angle from " + str(degrees(angle)) + " because otherwise motion would fail!")
            angle = radians(29)
          rotate_bearing_by_angle(angle)
          adjustment_motions += 1
        else:
          rospy.loginfo("Bearing angle offset " + str(angle) + " is < 5 deg. Good.")
          times_it_looked_like_success += 1
      else:
        rospy.logwarn("Bearing angle not found in image.")
        times_perception_failed_in_a_row += 1
      if times_it_looked_like_success > 3:
        rospy.loginfo("Bearing angle looked correct " + str(times_it_looked_like_success) + " times. Judged successful.")
        return True
      if times_perception_failed_in_a_row > 10:
        # If our perception fails continuously, we try to get out of a local minimum/unlucky lighting situation
        # by rotating randomly in a direction
        times_perception_failed_in_a_row = 0
        times_we_added_random_action += 1
        if times_we_added_random_action > 4:
          rospy.logerr("Bearing perception failed too often. Breaking out")
          return False
        rotate_bearing_by_angle(radians(5))
    rospy.logerr("Did not manage to align the bearing holes.")
    return False

  def pick_up_and_insert_bearing(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False
    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")

    goal = self.look_and_get_grasp_point("bearing")
    if not goal:
      rospy.logerr("Could not find bearing in tray. Skipping procedure.")
      return False
    self.vision.activate_camera("b_bot_inside_camera")
    goal.pose.position.x -= 0.01 # MAGIC NUMBER
    goal.pose.position.z = 0.0115
    
    self.simple_pick("b_bot", goal, gripper_force=100.0, approach_height=0.05, axis="z")

    #TODO add bearing to scene
    # if not self.pick_from_two_poses_topdown("b_bot", "bearing", goal):
    #   rospy.logerr("Fail to pick bearing from tray")
    #   return False

    if self.b_bot.gripper.opening_width < 0.01:
      rospy.logerr("Fail to grasp bearing")
      return
    elif self.b_bot.gripper.opening_width < 0.045:
      rospy.loginfo("bearing found to be upwards")
      self.playback_sequence("bearing_orient")
      # success_b = self.b_bot.load_program(program_name="wrs2020/bearing_orient_totb.urp")
    else:
      rospy.loginfo("bearing found to be upside down")
      self.playback_sequence("bearing_orient_down")
      # success_b = self.b_bot.load_program(program_name="wrs2020/bearing_orient_down_totb.urp")
      #'down' means the small area contacts with tray.

    if self.b_bot.gripper.opening_width < 0.01:
      rospy.logerr("Bearing not found in gripper. Must have been lost. Aborting.")
      #TODO(felixvd): Look at the regrasping/aligning area next to the tray
      return False

    if task == "taskboard" or task == "assembly":
      self.playback_sequence("bearing_move_to_" + task)
    else:
      rospy.logerr("Task could not be read. Breaking out of pick_up_and_insert_bearing")
      return False

    # Insert bearing
    if not self.insert_bearing(task=task):
      rospy.logerr("insert_bearing returned False. Breaking out")
      return False

    self.bearing_holes_aligned = self.align_bearing_holes(task=task)
    return self.bearing_holes_aligned

  def insert_bearing(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      bearing_target_link = "taskboard_bearing_target_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      bearing_target_link = "assembled_part_07_inserted"

    duration = 30.0
    
    target_force = get_target_force('-X', 10.0)
    selection_matrix = [0., 0.5, 0.5, .8, .8, .8]

    target_pose = conversions.to_pose_stamped(bearing_target_link, [-0.0, 0, -.005, 0, 0, 0, 1.])
    target_in_robot_base = self.listener.transformPose("b_bot_base_link", target_pose)
    target_x = target_in_robot_base.pose.position.x
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.003) # relax constraint

    rospy.loginfo("** STARTING FORCE CONTROL **")
    result = self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.002, radius_direction="+Z", steps=100, revolutions=3, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(1.0), wiggle_revolutions=0.,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))

    if result != TERMINATION_CRITERIA:
      rospy.logerr("** Insertion Failed!! **")
      return

    self.b_bot.gripper.open(wait=True)

    self.b_bot.move_lin_rel(relative_translation = [0.016,0,0], acceleration = 0.015, speed=.03)

    pre_push_position = self.b_bot.force_controller.joint_angles()

    self.b_bot.gripper.close(velocity=0.01, wait=True)

    target_x += 0.001
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.006) # relax constraint

    rospy.loginfo("** STARTING FORCE CONTROL 2**")
    self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.0, radius_direction="+Z", steps=100, revolutions=3, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(1.0), wiggle_revolutions=2.,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE 2 with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))
    
    self.b_bot.gripper.open(wait=True)

    rospy.loginfo("** Second insertion done, moving back via MoveIt **")
    self.b_bot.move_lin_rel(relative_translation = [0.025,0,0], acceleration = 0.015, speed=.03)
    return True

  def fasten_bearing(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False
    self.a_bot.go_to_named_pose("home")
    self.equip_tool('b_bot', 'screw_tool_m4')
    self.vision.activate_camera("b_bot_outside_camera")
    intermediate_screw_bearing_pose = [31.0 /180.0*3.14, -137.0 /180.0*3.14, 121.0 /180.0*3.14, -114.0 /180.0*3.14, -45.0 /180.0*3.14, -222.0 /180.0*3.14]
    
    _task = task  # Needs to be defined here so the nested function can access it
    def pick_and_fasten_bearing_screw(screw_number, pick_screw=True):
      """Returns tuple (screw_success, break_out_of_loop)"""
      # Pick screw
      if pick_screw:
        self.b_bot.move_joints(intermediate_screw_bearing_pose)
        self.b_bot.go_to_named_pose("feeder_pick_ready")
        pick_success = self.pick_screw_from_feeder("b_bot", screw_size=4)
        if not pick_success:
          rospy.logerr("Could not pick screw. Why?? Breaking out.")
          self.unequip_tool('b_bot', 'screw_tool_m4')
          return (False, True)
      
      # Fasten screw
      self.b_bot.move_joints(intermediate_screw_bearing_pose)
      self.b_bot.go_to_named_pose("horizontal_screw_ready")
      screw_pose = geometry_msgs.msg.PoseStamped()
      if _task == "taskboard":
        screw_pose.header.frame_id = "/taskboard_bearing_target_screw_" + str(n) + "_link"
      elif _task == "assembly":
        screw_pose.header.frame_id = "/assembled_part_07_screw_hole_" + str(n)
      else:
        rospy.logerr("Incorrect task argument, frame could be determined! Breaking out.")
        return False
      
      screw_pose.pose.position.x = 0.00  ## Why does this seem to be loose at 0.0?
      screw_pose.pose.position.z = 0.00  ## MAGIC NUMBER
      screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/12, 0, 0) )
      if task == "assembly":
        # The target frame is oriented differently in taskboard and assembly.
        screw_pose.pose = rotatePoseByRPY(tau/4, 0, 0, screw_pose.pose)
      screw_pose_approach = copy.deepcopy(screw_pose)
      screw_pose_approach.pose.position.x -= 0.07
      
      self.b_bot.go_to_pose_goal(screw_pose_approach, end_effector_link = "b_bot_screw_tool_m4_tip_link", move_lin=False)
      screw_success = self.skill_server.do_screw_action("b_bot", screw_pose, screw_size=4)
      self.b_bot.go_to_pose_goal(screw_pose_approach, end_effector_link = "b_bot_screw_tool_m4_tip_link", move_lin=False)
      self.b_bot.move_joints(intermediate_screw_bearing_pose)
      return (screw_success, False)

    screw_status = dict()
    for n in [1,3,2,4]:  # Cross pattern
      screw_status[n] = "empty"
      if rospy.is_shutdown():
        break
      (screw_success, breakout) = pick_and_fasten_bearing_screw(n)
      if breakout:
        break
      if screw_success:
        screw_status[n] = "done"
      if not screw_success and self.tools.screw_is_suctioned["m4"]:
        screw_status[n] = "empty"
      if not screw_success and not self.tools.screw_is_suctioned["m4"]:
        screw_status[n] = "maybe_stuck_in_hole"
      rospy.loginfo("Screw " + str(n) + " detected as " + screw_status[n])
    
    # Reattempt 
    all_screws_done = all(value == "done" for value in screw_status.values())
    while not all_screws_done and not rospy.is_shutdown():
      for n in [1,2,3,4]:
        if rospy.is_shutdown():
          break
        if screw_status[n] == "empty":
          (screw_success, breakout) = pick_and_fasten_bearing_screw(n)
        elif screw_status[n] == "maybe_stuck_in_hole":
          (screw_success, breakout) = pick_and_fasten_bearing_screw(n, pick_screw=False)
        if screw_success:
          screw_status[n] = "done"
        if not screw_success and self.tools.screw_is_suctioned["m4"]:
          screw_status[n] = "empty"
        if not screw_success and not self.tools.screw_is_suctioned["m4"]:
          screw_status[n] = "maybe_stuck_in_hole"
        rospy.loginfo("Screw " + str(n) + " detected as " + screw_status[n])
      all_screws_done = all(value == "done" for value in screw_status.values())

    self.b_bot.move_joints(intermediate_screw_bearing_pose)
    self.b_bot.go_to_named_pose("tool_pick_ready")
    return all_screws_done

  ########  Motor pulley

  def insert_motor_pulley(self, task="", attempts=1):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      bearing_target_link = "taskboard_small_shaft"
    elif task == "assembly":
      rospy.logerr("look this up")
      bearing_target_link = "assembled_part_07_inserted"

    duration = 20.0
    
    target_force = get_target_force('-X', 8.0)
    selection_matrix = [0., 0.3, 0.3, 0.6, 0.6, 0.95]

    target_pose = conversions.to_pose_stamped(bearing_target_link, [0.009, -0.000, -0.009, 0,0,0])
    target_in_robot_base = self.listener.transformPose("b_bot_base_link", target_pose)
    target_x = target_in_robot_base.pose.position.x - 0.0045
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.001) # relax constraint

    rospy.loginfo("** STARTING FORCE CONTROL **")
    result = self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.002, radius_direction="+Z", steps=100, revolutions=5, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=2.,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))

    self.b_bot.gripper.open(wait=True, opening_width=0.07)

    if result != TERMINATION_CRITERIA:
      self.b_bot.gripper.close(wait=True)
      if self.b_bot.gripper.opening_width > 0.01 and attempts > 0: # try again the pulley is still there
        self.b_bot.move_lin_rel(relative_translation = [0.01,0,0], acceleration = 0.015, speed=.03)
        return self.insert_motor_pulley(task, attempts=attempts-1)
      else:
        self.b_bot.gripper.open(wait=True, opening_width=0.07)
        rospy.logerr("** Insertion Failed!! **")
        return

    self.b_bot.gripper.close(wait=True)
    target_force = get_target_force('-X', 10.0)

    target_x += 0.01
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.005) # relax constraint

    rospy.loginfo("** STARTING FORCE CONTROL **")
    result = self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.0, radius_direction="+Z", steps=100, revolutions=3, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(2.0), wiggle_revolutions=2.,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))

    if result != TERMINATION_CRITERIA:
      rospy.logerr("** Insertion Failed!! **")
      return

    self.b_bot.gripper.open(wait=True)

    self.b_bot.move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)

    return True

  ########  Idler pulley

  def pick_and_insert_idler_pulley(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      idler_puller_target_link = "taskboard_long_hole_top_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      idler_puller_target_link = "assembly_long_hole_middle_link"

    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")
    
    object_pose = self.look_and_get_grasp_point("taskboard_idler_pulley_small")
    
    if not object_pose:
      rospy.logerr("Could not find idler pulley in tray. Skipping procedure.")
      return False

    self.vision.activate_camera("b_bot_inside_camera")
    object_pose.pose.position.x -= 0.01 # MAGIC NUMBER
    object_pose.pose.position.z = 0.014

    rospy.loginfo("Picking idler pulley at: ")
    self.b_bot.go_to_named_pose("home")

    at_object_pose = copy.deepcopy(object_pose)
    at_object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
    approach_pose = copy.deepcopy(at_object_pose)
    approach_pose.pose.position.z += .1

    self.a_bot.gripper.open(wait=False, opening_width=0.07)

    if not self.a_bot.go_to_pose_goal(approach_pose):
      rospy.logerr("Fail to complete approach pose")
      return False
    
    rospy.loginfo("Moving down to object")
    self.a_bot.gripper.open(opening_width=0.08)
    if not self.a_bot.go_to_pose_goal(at_object_pose):
      rospy.logerr("Fail to complete pick pose")
      return False

    if not self.center_with_gripper("a_bot", grasp_width=.05):
      rospy.logerr("Fail to complete center_with_gripper")
      return False
    if not self.grasp_idler_pulley():
      rospy.logerr("Fail to complete grasp_idler_pulley")
      return False
    if not self.insert_idler_pulley(idler_puller_target_link):
      rospy.logerr("Fail to complete insert_idler_pulley")
      return False
    if not self.prepare_screw_tool_idler_pulley(idler_puller_target_link):
      rospy.logerr("Fail to complete prepare_screw_tool_idler_pulley")
      return False
    self.a_bot.gripper.open(opening_width=0.05)
    if not self.playback_sequence("idler_pulley_equip_nut_tool"):
      rospy.logerr("Fail to complete equip_nut_tool")
      return False

    success = self.fasten_idler_pulley_with_nut_tool(idler_puller_target_link)
    if not success:
      rospy.logerr("Fail to complete fasten_idler_pulley_with_nut_tool")

    if not self.playback_sequence("idler_pulley_unequip_nut_tool"):
      rospy.logerr("Fail to complete unequip_nut_tool")

    if not self.playback_sequence("idler_pulley_release_screw_tool"):
      rospy.logerr("Fail to complete idler_pulley_release_screw_tool")
      return False

    self.unequip_tool("b_bot", "padless_tool_m4")
    return success
    
  def center_idler_pulley(self):
    # Center first time
    self.a_bot.gripper.close(force=40.0, velocity=0.013)
    self.a_bot.gripper.open(velocity=0.013)

  def grasp_idler_pulley(self):
    # incline 45deg
    success = self.a_bot.move_lin_rel(relative_translation=[0, 0.01, 0.002], relative_rotation=[tau/8.0, 0, 0])
    if not success:
      rospy.logerr("Fail to incline a_bot 45 deg %s" % success)
      return False
    
    self.a_bot.gripper.close()

    if self.use_real_robot:
      if self.a_bot.gripper.opening_width < 0.01:
        rospy.logerr("Fail to grasp Idler Pulley")
        return False

    # Move up 15 cm
    return self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.15])

  def insert_idler_pulley(self, target_link):
    rospy.loginfo("Going to near tb (a_bot)")
    d = 0.07
    # MAGIC NUMBERS (offset from TCP to tip of idler pulley thread)
    # x = -0.003 was the original target
    approach_pose = conversions.to_pose_stamped(target_link, [(-0.07),  0.009, 0.0, tau/4.0, 0, tau/8.])
    near_tb_pose = conversions.to_pose_stamped(target_link,  [(-0.015), 0.009, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose = conversions.to_pose_stamped(target_link,    [(-0.006), 0.009, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose_world = self.listener.transformPose("world", in_tb_pose)
    success = self.a_bot.move_lin(approach_pose, speed=0.4)
    if not success:
      return False
    
    rospy.loginfo("Approach ridge (a_bot)")
    success = self.a_bot.move_lin(near_tb_pose, speed=0.4)
    if not success:
      return False
        
    rospy.loginfo("Moving into ridge (a_bot)")
    insertion_offsets = [0.0]
    d2 = 0.0007
    for i in range(6):
      insertion_offsets.append(d2*(i+1))
      insertion_offsets.append(-d2*(i+1))

    for offset in insertion_offsets:
      selection_matrix = [0.,1.,1.,1,1,1]
      success = self.a_bot.linear_push(5, "+X", max_translation=0.015, timeout=10.0, slow=True, selection_matrix=selection_matrix)

      if success and self.a_bot.robot_group.get_current_pose().pose.position.x <= in_tb_pose_world.pose.position.x:
        return True
      else:
        # Go back, try again
        self.a_bot.move_lin_rel(relative_translation=[0.01, 0, 0], speed=0.1, relative_to_robot_base=True)
        near_tb_pose_with_offset = copy.deepcopy(near_tb_pose)
        near_tb_pose_with_offset.pose.position.y += offset
        self.a_bot.move_lin(near_tb_pose_with_offset, speed=0.05)

    # No force control alternative
    # for i in range(3):
    #   if not self.a_bot.move_lin_rel(relative_translation=[-0.01, 0, 0], speed=0.05, relative_to_robot_base=True):
    #     if self.a_bot.is_protective_stopped():
    #       self.a_bot.unlock_protective_stop()
    #       rospy.loginfo("Fail insertion %s, moving 0.0015 to the right (-Y)" % str(i+1))
    #       self.a_bot.move_lin_rel(relative_translation=[0.01, -0.0015*(i+1), 0], speed=0.05)
    #   else:
    #     return True
    return False

  def prepare_screw_tool_idler_pulley(self, target_link):
    """ target_link is e.g. long_hole_top_link
    """
    self.equip_tool("b_bot", "padless_tool_m4")
    self.b_bot.go_to_named_pose("screw_ready")
    if not self.playback_sequence("idler_pulley_push_with_screw_tool"):
      return False

    rospy.loginfo("Going to near tb (b_bot)") # Push with tool
    target_rotation = np.deg2rad([30.0, 0.0, 0.0]).tolist()
    xyz_light_push = [-0.01, -0.001, 0.001]  # MAGIC NUMBERS
    near_tb_pose = conversions.to_pose_stamped(target_link, xyz_light_push + target_rotation)
    success = self.b_bot.move_lin(near_tb_pose, speed=0.05, acceleration=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link")
    if not success:
      return False

    self.tools.set_motor("padless_tool_m4", "tighten", duration=10.0)
    self.insert_screw_tool_tip_into_idler_pulley_head(target_link)
    
    # xyz_hard_push = [0.001, -0.001, 0.001]  # MAGIC NUMBERS
    # push_pose = conversions.to_pose_stamped(target_link, xyz_hard_push + target_rotation)
    # return self.b_bot.move_lin(push_pose, speed=0.05, acceleration=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link")

    ## Incline the tool slightly 
    self.planning_scene_interface.allow_collisions("padless_tool_m4", "taskboard_plate")
    xyz_hard_push = [0.001, -0.001, 0.001]  # MAGIC NUMBERS (target without inclination)
    inclination_angle_deg = 4.0
    inclined_orientation_hard_push = np.deg2rad([30.0, inclination_angle_deg, 0.0]).tolist()
    s = sin(np.deg2rad(inclination_angle_deg)) * 0.008  # 8 mm is roughly the distance from the taskboard surface to the 
                                                        # head of the screw, so adding this offset should result in a rotation
                                                        # around the screw head.
    xyz_hard_push[2] -= s
    push_pose = conversions.to_pose_stamped(target_link, xyz_hard_push + inclined_orientation_hard_push)
    return self.b_bot.move_lin(push_pose, speed=0.02, acceleration=0.02, end_effector_link="b_bot_screw_tool_m4_tip_link")

  def insert_screw_tool_tip_into_idler_pulley_head(self, target_link):

    target_force = get_target_force('-X', 0.0)
    selection_matrix = [0., 0.9, 0.9, 1, 1, 1]

    termination_criteria = lambda cpose, standby_time: False # disabled

    rospy.loginfo("** STARTING FORCE CONTROL **")
    self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.003, radius_direction="+Y", 
                                                  steps=50, revolutions=2, timeout=10.0,
                                                  target_force=target_force, selection_matrix=selection_matrix,
                                                  termination_criteria=termination_criteria,
                                                  displacement_epsilon=0.0015, check_displacement_time=3.0)
    rospy.loginfo("** FORCE CONTROL COMPLETE **")

    return True

  def fasten_idler_pulley_with_nut_tool(self, target_link):
    approach_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    self.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.4)

    success = False
    idler_pulley_screwing_succeeded = False
    offsets = [0.0, -0.003, -0.006, 0.009, 0.006, 0.003]
    for offset in offsets:
      if idler_pulley_screwing_succeeded:
        success = True
        break
      # Move nut tool forward so nut touches the screw
      d = offset  # 
      approach_pose = conversions.to_pose_stamped(target_link, [0.06, 0.0, d - 0.005, 0.0, 0.0, 0.0])
      self.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
      
      pushed_into_screw = conversions.to_pose_stamped(target_link, [0.011, 0.0, d - 0.005, 0.0, 0.0, 0.0])
      self.a_bot.move_lin(pushed_into_screw, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2)
      
      response = self.tools.set_motor("padless_tool_m4", "tighten", duration=3.0, wait=True, skip_final_loosen_and_retighten=True)
      idler_pulley_screwing_succeeded = response.motor_stalled

    retreat_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not self.a_bot.move_lin(retreat_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2):
      return False
    
    return success
  ######## Shaft

  def pick_and_insert_shaft(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      target_link = "taskboard_assy_part_07_inserted"
    elif task == "assembly":
      target_link = "assembly_assy_part_07_inserted"
    
    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")

    goal = self.look_and_get_grasp_point("shaft")
    if not goal:
      rospy.logerr("Could not find shaft in tray. Skipping procedure.")
      return False
    goal.pose.position.z = 0.001
    goal.pose.position.x -= 0.01
    print("shaft goal", goal)
    self.vision.activate_camera("b_bot_inside_camera")
    self.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.05, axis="z")

    rospy.loginfo("Going to approach pose (b_bot)")
    rotation = np.deg2rad([-22.5, -88.5, -157.5]).tolist()  # Arbitrary
    approach_pose = conversions.to_pose_stamped(target_link, [-0.25, 0.002, 0.00] + rotation)
    if not self.b_bot.move_lin(approach_pose, speed=0.4):
      return False
    
    rospy.loginfo("Going to hole (b_bot)")
    at_hole_pose = conversions.to_pose_stamped(target_link, [-0.09, 0.000, -0.002] + rotation)
    if not self.b_bot.move_lin(at_hole_pose, speed=0.2):
      return False

    self.insert_shaft(target_link)

  def insert_shaft(self, target_link):
    """
    Insert shaft with force control using b_bot. The shaft has to be in front of the hole already.
    """
    # FIXME: Needs tuning
    self.b_bot.linear_push(5, "-X", max_translation=0.06, timeout=20.0)
    after_push_x = self.b_bot.force_controller.end_effector()[0]
    print("Pose after linear push", after_push_x)

    # Parameters for insertion
    duration = 15.0
    target_force = get_target_force('-X', 10.0)
    selection_matrix = [0., 0.3, 0.3, 0.95, 0.95, 0.95]

    target_x = after_push_x + 0.01
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.003) # relax constraint

    rospy.loginfo("** STARTING FORCE CONTROL **")
    result = self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.0025, radius_direction="+Z", steps=100, revolutions=3, timeout=duration,
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(0.0), wiggle_revolutions=2.0,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))

    self.b_bot.gripper.open()

    if result != TERMINATION_CRITERIA:
      self.b_bot.gripper.close(wait=True)
      if self.b_bot.gripper.opening_width > 0.005: 
        rospy.loginfo("The shaft is in the hole, so let's keep trying to push from behind")
        self.b_bot.gripper.open(wait=True, opening_width=0.07)
      else:
        self.b_bot.gripper.open(wait=True, opening_width=0.07)
        rospy.logerr("** Insertion Failed!! **")
        return

    # Move back to push (without grasping)
    self.b_bot.move_lin_rel(relative_translation = [0.065,0,-0.02], acceleration = 0.015, speed=.03)
    self.b_bot.gripper.close()

    target_force = get_target_force('-X', 15.0)
    termination_criteria = lambda cpose, standby_time: cpose[0] >= target_x or \
                                                       (standby_time and cpose[0] >= target_x-0.01) # relax constraint
    radius = 0.0

    rospy.loginfo("** STARTING FORCE CONTROL 2**")
    result = self.b_bot.execute_spiral_trajectory(plane="YZ", max_radius=0.003, radius_direction="+Z", steps=100, revolutions=3, timeout=duration,
                                                        target_force=target_force, selection_matrix=selection_matrix,
                                                        termination_criteria=termination_criteria)
    rospy.loginfo("** FORCE CONTROL COMPLETE 2 with distance %s **" % round(target_x - self.b_bot.force_controller.end_effector()[0],5))
    rospy.logwarn(" position x: %s" % round(self.b_bot.force_controller.end_effector()[0],5))

    self.b_bot.gripper.open()

    rospy.loginfo("** Second insertion done, moving back via MoveIt **")
    self.b_bot.move_lin_rel(relative_translation = [0.025,0,0], acceleration = 0.015, speed=.03)
    
    if result != TERMINATION_CRITERIA:
      rospy.logerr("** Insertion Failed!! **")
      return False
    return True

  ########

  def fasten_screw_vertical(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):
    """
    This works for the two L-plates when they are facing forward.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
    
    res = self.skill_server.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
    return res  # Bool

  def fasten_screw_horizontal(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):
    """
    This should work for the motor and bearing.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
    
    success = self.skill_server.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
    return success

  def pick_nut(self, robot_name):
    """Pick the nut from the holder. The nut tool has to be equipped.
    Use this command to equip: do_change_tool_action(self, "a_bot", equip=True, screw_size = 66)"""
    rospy.logerr("Not implemented yet")
    return False
    
    self.a_bot.go_to_named_pose("home", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    self.a_bot.go_to_named_pose("nut_pick_ready", speed=1.0, acceleration=1.0, force_ur_script=self.use_real_robot)

    nut_pose = geometry_msgs.msg.PoseStamped()
    nut_pose.header.frame_id = "nut_holder_collar_link"
    nut_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    approach_pose = copy.deepcopy(nut_pose)
    approach_pose.pose.position.x -= .03
    self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=self.speed_fast, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    # spiral_axis = "Y"
    # push_direction = "Z+"
    # self.skill_server.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.tools.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.skill_server.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.active_robots[robot_name].go_to_pose_goal(nut_pose, speed=.005, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    rospy.sleep(3)
    # self.skill_server.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def move_camera_to_pose(self, pose_goal, robot_name="b_bot", camera_name="inside_camera"):
    return self.active_robots[robot_name].go_to_pose_goal(pose_goal, end_effector_link=robot_name+"_"+camera_name+"_color_optical_frame")

  def jigless_recenter(self, robot_carrying_the_item):
      pass

  def print_objects_in_tray(self):
    """ Print the position of all objects in the tray coordinate center to the command line.
        This can be used to spawn an example scene. """
    # TODO: Get all objects from planning_scene instead of hard-coding
    objects = ['panel_motor', 'panel_bearing', 'motor', 'motor_pulley', 'bearing',
      'shaft', 'end_cap', 'bearing_spacer', 'output_pulley', 'idler_spacer', 'idler_pulley', 'idler_pin', 'base']
    ps = geometry_msgs.msg.PoseStamped()
    ps.pose.orientation.w = 1.0
    for object_name in objects:
      ps.header.frame_id = "/move_group/" + object_name
      ps_in_tray = self.listener.transformPose("tray_center", ps)
      print(object_name + ":  (translation, rotation rpy)")
      xyz = [ps_in_tray.pose.position.x, ps_in_tray.pose.position.y, ps_in_tray.pose.position.z]
      rpy = tf_conversions.transformations.euler_from_quaternion([ps_in_tray.pose.orientation.x,
                                                                  ps_in_tray.pose.orientation.y,
                                                                  ps_in_tray.pose.orientation.z,
                                                                  ps_in_tray.pose.orientation.w])
      print("%0f, %0f, %0f, %0f, %0f, %0f" % (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
      