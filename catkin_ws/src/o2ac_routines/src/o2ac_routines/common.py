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

    Example usage: self.go_to_pose_goal("b_bot", self.tray_view_high, 
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
    
    skip_stage_execution = False   # True while a subroutine like "equip/unequip"
    success = False
    screw_counter = 0
    if speed > 1.0:
      speed = 1.0
    start_state = self.robots.get_current_state()
    currently_attached_collision_objects = start_state.attached_collision_objects
    for solution in solution.sub_trajectory:
      stage_name = solution.info.creator_name
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
        self.skill_server.pick_screw_from_feeder('b_bot', screw_size=4)
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
        self.load_and_execute_program(robot="b_bot", program_name="wrs2020_push_motor_plate.urp", wait=True)
        continue
      if stage_name == 'move a_bot right wrs_subtask_motor_plate':
        rospy.logwarn("===================== DEBUG1")
        self.skill_server.move_lin_rel("a_bot", relative_translation=[0, -0.02, 0], use_robot_base_csys=True, max_wait=5.0)
        rospy.logwarn("===================== DEBUG1b")
      if stage_name == 'move a_bot back wrs_subtask_motor_plate':
        rospy.logwarn("===================== DEBUG2")
        self.skill_server.move_lin_rel("a_bot", relative_translation=[0,  0.02, 0], use_robot_base_csys=True, max_wait=5.0)
        rospy.logwarn("===================== DEBUG2b")

      # Execute trajectories
      if solution.scene_diff.robot_state.joint_state.name and not skip_stage_execution:  # If the robot state is changed (robot moved, object attached/detached)
        # Update attached collision objects
        if not currently_attached_collision_objects == solution.scene_diff.robot_state.attached_collision_objects:
          # for co in currently_attached_collision_objects:
          #   print('IN MEMORY CO: ', co.object.id)
          # for co in solution.scene_diff.robot_state.attached_collision_objects:
          #   print('IN SOLUTION CO: ', co.object.id)
          coll_objs_to_detach = [collision_object for collision_object in currently_attached_collision_objects if collision_object not in solution.scene_diff.robot_state.attached_collision_objects]
          coll_objs_to_attach = [collision_object for collision_object in solution.scene_diff.robot_state.attached_collision_objects if collision_object not in currently_attached_collision_objects]
          for attached_object in coll_objs_to_detach:
            # print('Detaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].detach_object(attached_object_name)
          for attached_object in coll_objs_to_attach:
            # print('Attaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].attach_object(attached_object_name, attached_object.link_name, touch_links=  # MODIFY attach_tool in base.py to attach_object ++ ROBOT NAME???
              [attach_to + "_robotiq_85_tip_link", 
              attach_to + "_robotiq_85_left_finger_tip_link", 
              attach_to + "_robotiq_85_left_inner_knuckle_link", 
              attach_to + "_robotiq_85_right_finger_tip_link", 
              attach_to + "_robotiq_85_right_inner_knuckle_link"])
            currently_attached_collision_objects.append(attached_object)
          currently_attached_collision_objects = [attached_collision_object for attached_collision_object in currently_attached_collision_objects if attached_collision_object not in coll_objs_to_detach]

        # Skip stage if joint_names is empty, because the stage performs no motions
        if not solution.trajectory.joint_trajectory.joint_names: 
          continue

        # Execute stage
        robot_name = solution.trajectory.joint_trajectory.joint_names[0][:5]
        arm_group = self.groups[robot_name]
        if len(solution.trajectory.joint_trajectory.joint_names) == 1:  # If only one joint is in the group, it is the gripper
          # Gripper motion
          hand_group = self.groups[robot_name + '_robotiq_85']
          hand_closed_joint_values = hand_group.get_named_target_values('close')
          hand_open_joint_values = hand_group.get_named_target_values('open')
          if stage_name == 'open hand':
            self.send_gripper_command(robot_name, 'open')
          elif stage_name == 'close hand':
            self.send_gripper_command(robot_name, 'close')
          elif 0.01 > abs(hand_open_joint_values[solution.trajectory.joint_trajectory.joint_names[0]] - solution.trajectory.joint_trajectory.points[-1].positions[0]):
            rospy.logwarn("Actuating gripper (backup path)!")
            self.send_gripper_command(robot_name, 'open')
          elif 0.01 < abs(hand_open_joint_values[solution.trajectory.joint_trajectory.joint_names[0]] - solution.trajectory.joint_trajectory.points[-1].positions[0]):
            rospy.logwarn("Actuating gripper (backup path)!")
            self.send_gripper_command(robot_name, 'close', True)
          
        else: # The robots move
          rospy.loginfo("========")
          rospy.logwarn("self.groups[robot_name].get_current_joint_values():")
          print(self.groups[robot_name].get_current_joint_values())
          rospy.logwarn("solution.trajectory.joint_trajectory.points[0].positions:")
          print(solution.trajectory.joint_trajectory.points[0].positions)
          # First check that the trajectory is safe to execute (= robot is at start of trajectory)
          if not all_close(self.groups[robot_name].get_current_joint_values(), 
                            solution.trajectory.joint_trajectory.points[0].positions,
                            0.01):
            rospy.logerr("Robot " + robot_name + " is not at the start of the next trajectory! Aborting.")
            rospy.logerr("Stage name: " + stage_name)
            rospy.logwarn("self.groups[robot_name].get_current_joint_values():")
            print(self.groups[robot_name].get_current_joint_values())
            rospy.logwarn("solution.trajectory.joint_trajectory.points[0].positions:")
            print(solution.trajectory.joint_trajectory.points[0].positions)
            return False

          # Prepare robot motion
          self.activate_ros_control_on_ur(robot_name)
          plan = arm_group.retime_trajectory(self.robots.get_current_state(), solution.trajectory, speed)
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
    self.go_to_pose_goal(robot_name, self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
    success = self.detect_object_in_camera_view(item_name)

    # # TODO: Also move robot if object requires a close-up view (shaft, pin...)
    # if not success:
    #   poses = [self.tray_view_close_front_b, self.tray_view_close_back_b, self.tray_view_close_front_a, self.tray_view_close_back_a]
    #   for pose in poses:
    #     self.go_to_pose_goal(robot_name, pose, end_effector_link="b_bot_outside_camera_color_frame", speed=.1, acceleration=.04)
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

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=0.1, speed_slow=0.02, gripper_command="close", 
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
    
    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    if axis =="x":
      object_pose.pose.position.x += approach_height * sign
    if axis =="z":
      object_pose.pose.position.z += approach_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= approach_height * sign
    if axis =="z":
      object_pose.pose.position.z -= approach_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command=grasp_width) # Open

    rospy.loginfo("Moving down to object")
    if axis =="x":
      object_pose.pose.position.x += grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z += grasp_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z -= grasp_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command="close", force = gripper_force, velocity = gripper_velocity)

    if item_id_to_attach:
      try:
        self.groups[robot_name].attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_robotiq_85_tip_link", 
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
      self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True)
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
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, acceleration=acc_fast, move_lin=False)
   
    rospy.loginfo("Moving to place target")
    object_pose.pose.position.x += place_height
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acc_slow, move_lin=False)
    object_pose.pose.position.x -= place_height
    
    #gripper open
    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command="open")

    if item_id_to_detach:
      self.groups[robot_name].detach_object(item_id_to_detach)

    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      self.go_to_pose_goal(robot_name, object_pose, speed=speed_fast, move_lin=False)  
    return True

  def simple_grasp_sanity_check(self, grasp_pose, grasp_width=0.08, border_dist=0.06):
    """
    Returns true if the grasp pose is further than 5 cm away from the tray border,
    and no other detected objects are closer than 5 cm.

    grasp_pose is a PoseStamped.
    """
    d = self.distance_from_tray_border(grasp_pose)
    if d[0] < border_dist or d[1] < border_dist:
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
      # For small items, the object should be the only grasp pose.
      grasp_pose = self.objects_in_tray[object_id]
      return [grasp_pose]
      # TODO: Consider the idler spacer, which can stand upright or lie on the side.
      
    if object_id in self.large_item_ids:
      # TODO: Generate alternative grasp poses
      # TODO: Get grasp poses from database
      grasp_pose = self.objects_in_tray[object_id]
      grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
      grasp_pose.pose.position.z = 0.02
      if self.is_grasp_pose_feasible(grasp_pose, border_dist=0.05):
        return [grasp_pose]
    
    return False
  
  def look_and_get_grasp_point(self, object_id):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """
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

    self.activate_camera("b_bot_outside_camera")
    self.activate_led("b_bot")
    self.open_gripper("b_bot", wait=False)
    # TODO: Merge with detect_object_in_camera_view in base.py
    # self.go_to_pose_goal("b_bot", self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.1)
    # self.get_3d_poses_from_ssd()
    if object_id in self.objects_in_tray:
      del self.objects_in_tray[object_id]

    if not object_id in self.objects_in_tray:
      # for close_view_batch in [self.close_tray_views, self.close_tray_views_rot_left, self.close_tray_views_rot_right]:
      for view in [self.tray_view_high] + self.close_tray_views + self.close_tray_views_rot_left + self.close_tray_views_rot_right + self.close_tray_views_rot_left_more + self.close_tray_views_rot_left_90:
        self.go_to_pose_goal("b_bot", view, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.3)
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
          self.go_to_pose_goal("b_bot", close_view, end_effector_link="b_bot_outside_camera_color_frame", speed=.3, acceleration=.3)
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

  def distance_from_tray_border(self, object_pose):
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
  
  def center_with_gripper(self, robot_name, object_pose, object_width, use_ur_script=False):
    """
    Centers cylindrical object by moving the gripper, by moving the robot to the pose and closing/opening.
    Rotates once and closes/opens again. Does not move back afterwards.
    """
    if use_ur_script:
      success = self.load_program(robot=robot_name, program_name="wrs2020/center_object.urp", recursion_depth=3)
      if success:
        rospy.sleep(1)
        self.execute_loaded_program(robot=robot_name)
      else:
        rospy.logerr("Problem loading. Not executing center_with_gripper.")
        return False
      wait_for_UR_program("/"+robot_name, rospy.Duration.from_sec(20))
      if self.is_robot_protective_stopped(robot_name):
        self.unlock_protective_stop(robot_name)
        return False
    object_pose_in_world = self.listener.transformPose("world", object_pose)
    object_pose_in_world.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/2, 0))
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    self.send_gripper_command(gripper=robot_name, command="close", force = 1.0, velocity = 0.1)
    self.send_gripper_command(gripper=robot_name, command=object_width+0.02, force = 90.0, velocity = 0.001)
    object_pose_in_world_rotated = copy.deepcopy(object_pose_in_world)
    object_pose_in_world_rotated.pose = rotatePoseByRPY(0,0,tau/4, object_pose_in_world_rotated.pose)
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    self.send_gripper_command(gripper=robot_name, command="close", force = 1.0, velocity = 0.1)
    self.send_gripper_command(gripper=robot_name, command=object_width+0.02, force = 90.0, velocity = 0.001)
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_slow, acceleration=acc_slow, move_lin=True)
    return True

  def centering_pick(self, robot_name, pick_pose, speed_fast=0.1, speed_slow=0.02, object_width=0.08, approach_height=0.05, 
          item_id_to_attach = "", lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.1, gripper_force=40.0):
    """
    This function picks an object with the robot directly from above, but centers the object with the gripper first.
    Should be used only for cylindrical objects.
    
    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    """
    
    if speed_fast > 1.0:
      acceleration=speed_fast
    else:
      acceleration=1.0

    object_pose_in_world = self.listener.transformPose("world", pick_pose)
    object_pose_in_world.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    object_pose_in_world.pose.position.z += approach_height
    rospy.logdebug("Going to height " + str(object_pose_in_world.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    object_pose_in_world.pose.position.z -= approach_height

    self.send_gripper_command(gripper=robot_name, command=object_width+0.03)

    rospy.loginfo("Moving down to object")
    
    # Center object
    self.center_with_gripper(robot_name, object_pose_in_world, object_width)

    # Grasp object
    self.send_gripper_command(gripper=robot_name, command="close", force = gripper_force)

    if item_id_to_attach:
      try:
        self.groups[robot_name].attach_object(item_id_to_attach, robot_name + "_ee_link", touch_links= 
        [robot_name + "_robotiq_85_tip_link", 
        robot_name + "_robotiq_85_left_finger_tip_link", 
        robot_name + "_robotiq_85_left_inner_knuckle_link", 
        robot_name + "_robotiq_85_right_finger_tip_link", 
        robot_name + "_robotiq_85_right_inner_knuckle_link"])
      except:
        rospy.logerr(item_id_to_attach + " could not be attached! robot_name = " + robot_name)

    if lift_up_after_pick:
      rospy.sleep(0.5)
      rospy.loginfo("Going back up")

      object_pose_in_world.pose.position.z += approach_height
      rospy.loginfo("Going to height " + str(object_pose_in_world.pose.position.z))
      self.go_to_pose_goal(robot_name, object_pose_in_world, speed=speed_fast, acceleration=acc_fast, move_lin=True)
    return True

  def drop_shaft_in_v_groove(self):
    """
    Places the shaft in the v groove with b_bot.
    """
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "vgroove_aid_drop_point_link"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*(0,0,0,1))
    ps.pose.position = geometry_msgs.msg.Point(-0.05, 0, 0)
    self.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_robotiq_85_tip_link", move_lin = False)
    ps.pose.position = geometry_msgs.msg.Point(0, 0, 0)
    self.go_to_pose_goal("b_bot", ps, end_effector_link="b_bot_robotiq_85_tip_link")

  def check_if_shaft_in_v_groove(self):
    """
    Returns True if the shaft is in the v_groove
    """
    look_at_shaft_pose = [2.177835941, -1.700065275, 2.536958996, -2.40987076, -1.529889408, 0.59719228]
    self.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    self.move_joints("b_bot", look_at_shaft_pose)

    res = self.call_shaft_notch_detection()
    print("=== shaft notch detection returned:")
    print(res)

  def turn_shaft_until_groove_found(self):
    look_at_shaft_pose = [2.177835941, -1.700065275, 2.536958996, -2.40987076, -1.529889408, 0.59719228]
    self.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    self.move_joints("b_bot", look_at_shaft_pose)

    times_turned = 0
    success = self.load_program(robot="b_bot", program_name="wrs2020/shaft_turning.urp", recursion_depth=3)  
    if not success:
      return False
    while times_turned < 6:
      rospy.loginfo("Turn shaft once")
      self.execute_loaded_program(robot="b_bot")
      wait_for_UR_program("/b_bot", rospy.Duration.from_sec(10))
      times_turned += 1
      res = self.call_shaft_notch_detection()
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
    self.go_to_pose_goal("b_bot", camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    camera_look_pose.pose.position.z -= 0.2
    self.go_to_pose_goal("b_bot", camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    angle = self.get_motor_angle()

  def align_bearing_holes(self, max_adjustments=10, task="assembly"):
    """
    Align the bearing holes.
    """
    self.activate_camera("b_bot_inside_camera")
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
      grasp_pose.header.frame_id = "assembled_assy_part_07_inserted"
    else:
      rospy.logerr("Incorrect argument, frame could be determined! Breaking out.")
      return False
    grasp_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0, 0, 0, 1.0))

    camera_look_pose = copy.deepcopy(grasp_pose)
    camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0.5, 0.5, 0.5, 0.5))
    camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.155, -0.005, -0.0)

    def rotate_bearing_by_angle(angle):
      self.open_gripper("b_bot")
      start_pose = copy.deepcopy(grasp_pose)
      start_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(-angle/2.0, 0, 0))
      end_pose = copy.deepcopy(grasp_pose)
      end_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(angle/2.0, 0, 0))
      self.go_to_pose_goal("b_bot", start_pose, speed=.1, acceleration=.04, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.close_gripper("b_bot")
      self.go_to_pose_goal("b_bot", end_pose, speed=.1, acceleration=.04, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.open_gripper("b_bot")

    success = False
    while adjustment_motions < max_adjustments:
      # Look at tb bearing
      self.open_gripper("b_bot")
      
      self.go_to_pose_goal("b_bot", camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
      self.activate_led("b_bot", on=False)
      rospy.sleep(1)  # Without a wait, the camera image is blurry

      # Get angle and turn
      angle = self.get_bearing_angle()
      if angle:
        rospy.loginfo("Bearing detected angle: %3f, try to correct", degrees(angle))
        times_perception_failed_in_a_row = 0
        if abs(degrees(angle)) > 1.5:
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

  ########

  def fasten_screw_vertical(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):
    """
    This works for the two L-plates when they are facing forward.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    res = self.skill_server.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.go_to_named_pose("feeder_pick_ready", robot_name)
    return res  # Bool

  def fasten_screw_horizontal(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):
    """
    This should work for the motor and bearing.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.go_to_named_pose("horizontal_screw_ready", robot_name)
    
    success = self.skill_server.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.go_to_named_pose("horizontal_screw_ready", robot_name)
    return success

  def pick_nut(self, robot_name):
    """Pick the nut from the holder. The nut tool has to be equipped.
    Use this command to equip: do_change_tool_action(self, "a_bot", equip=True, screw_size = 66)"""
    rospy.logerr("Not implemented yet")
    return False
    
    self.go_to_named_pose("home", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("nut_pick_ready", "a_bot", speed=1.0, acceleration=1.0, force_ur_script=self.use_real_robot)

    nut_pose = geometry_msgs.msg.PoseStamped()
    nut_pose.header.frame_id = "nut_holder_collar_link"
    nut_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    approach_pose = copy.deepcopy(nut_pose)
    approach_pose.pose.position.x -= .03
    self.go_to_pose_goal(robot_name, approach_pose, speed=self.speed_fast, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    # spiral_axis = "Y"
    # push_direction = "Z+"
    # self.skill_server.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.skill_server.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.go_to_pose_goal(robot_name, nut_pose, speed=.005, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    rospy.sleep(3)
    # self.skill_server.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.go_to_pose_goal(robot_name, approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def move_camera_to_pose(self, pose_goal, robot_name="b_bot", camera_name="inside_camera"):
    return self.go_to_pose_goal(robot_name, pose_goal, end_effector_link=robot_name+"_"+camera_name+"_color_optical_frame")

  def jigless_recenter(self, robot_carrying_the_item):
      pass
