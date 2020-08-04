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

  ######## Higher-level routines used in both assembly and taskboard

  def pick(self, object_name, grasp_parameter_location = '', lift_direction_reference_frame = '', lift_direction = [], robot_name = '', save_solution_to_file=''):
    """This function creates a motion-plan for picking the item referred to by 'object_name' input.
    The item needs to be in the planning scene as a collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_pick_action(object_name, grasp_parameter_location, lift_direction_reference_frame, lift_direction, robot_name)
    for solution in result.solution.sub_trajectory:
      scene_diff = solution.scene_diff
      planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
      planning_scene_diff_req.scene = scene_diff
      # self.apply_planning_scene_diff.call(planning_scene_diff_req)   # DEBUG: Update the scene pretending the action has been completed

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting pick() after writing solution")
    # TODO: EXECUTION OF PICK PLAN?
    return result.success

  def place(self, object_name, object_target_pose, release_object_after_place = True, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion-plan for placing the item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_place_action(object_name, object_target_pose, release_object_after_place, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting place() after writing solution")
    # TODO: EXECUTION OF PLACE PLAN?
    return result.success

  def release(self, object_name, pose_to_retreat_to = '', save_solution_to_file=''):
    """This function creates a motion-plan for releasing the placed item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_release_action(object_name, pose_to_retreat_to)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting release() after writing solution")
    # TODO: EXECUTION OF RELEASE PLAN?
    return result.success

  def pick_place(self, object_name, object_target_pose, grasp_parameter_location = '', release_object_after_place = True, object_subframe_to_place = '',
    lift_direction_reference_frame = '', lift_direction = [], approach_place_direction_reference_frame = '', approach_place_direction = [], robot_names = '', force_robot_order = False, save_solution_to_file=''):
    """This function creates a motion-plan for picking and then placing item referred to by 'object_name' input.
    The item needs to be in the planning scene as a collision object.
    If the 'object_subframe_to_place' input provided, this subframe will be positioned instead of the object.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_pickplace_action(object_name, object_target_pose, grasp_parameter_location, release_object_after_place, object_subframe_to_place, lift_direction_reference_frame, 
      lift_direction, approach_place_direction_reference_frame, approach_place_direction, robot_names, force_robot_order)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting pickplace() after writing solution")
    # TODO: EXECUTION OF PICKPLACE PLAN?
    return result.success

  def fasten(self, object_name, object_target_pose, object_subframe_to_place = '', approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion-plan for moving an item referred to by 'object_name' input.
    The item needs to be in the planning scene as an attached collision object.
    The motion is: going to approach pose->approach->retreat.
    The resulted motion-plan is stored in a file, if the name for the file (save_solution_to_file) is provided."""
    result = self.do_plan_fastening_action(object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting fasten() after writing solution")
    # TODO: EXECUTION OF PLACE PLAN?
    return result.success

  def subassembly(self, object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame = '', approach_place_direction = [], save_solution_to_file=''):
    """This function creates a motion=plan for the subassembly task of attaching the 'object' on the base plate
    """
    result = self.do_plan_subassembly_action(object_name, object_target_pose, object_subframe_to_place, approach_place_direction_reference_frame, approach_place_direction)

    path = self.rospack.get_path('o2ac_routines')
    path += '/MP_solutions/'
    if result.success and not save_solution_to_file == '':
      file = path + save_solution_to_file
      with open(file,'wb') as f:
        pickle.dump(result, f)
      rospy.loginfo("Exiting subassembly() after writing solution")
    # TODO: EXECUTION OF PLACE PLAN?
    return result.success
  
  def load_MP_solution(self, solution_file):
    """Load the result of a motion-plan from a file."""
    if not solution_file == '':
      # Load the solution
      file = self.rospack.get_path('o2ac_routines') + '/MP_solutions/' + solution_file
      with open(file, 'rb') as f:
        result = pickle.load(f)
    return result

  def execute_MP_solution(self, solution, speed = 1.0):
    """Execute the result of a motion plan."""
    # Execute the solution
    executing_routine = False
    success = False
    if speed > 1.0:
      speed = 1.0
    start_state = self.robots.get_current_state()
    attached_collision_objects = start_state.attached_collision_objects
    for solution in solution.sub_trajectory:
      stage_name = solution.info.creator_name
      if stage_name == 'Fasten screw (dummy)':
        rospy.sleep(2)
      # print('STAGE NAME: ' + stage_name + '!!!!!!!!!!!!!!!!!!!!!!!!!!')
      # raw_input()
      if stage_name == 'pick_tool_start':
        executing_routine = True
        self.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if stage_name == 'place_tool_start':
        executing_routine = True
        self.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if stage_name == 'pick_screw_start':
        executing_routine = True
        self.pick_screw_from_feeder('b_bot', 4)
      if solution.scene_diff.robot_state.joint_state.name and not executing_routine:  # If the robot state is changed (robot moved, object attached/detached)
        # Update attached collision objects
        if not attached_collision_objects == solution.scene_diff.robot_state.attached_collision_objects:
          coll_objs_to_detach = [collision_object for collision_object in attached_collision_objects if collision_object not in solution.scene_diff.robot_state.attached_collision_objects]
          coll_objs_to_attach = [collision_object for collision_object in solution.scene_diff.robot_state.attached_collision_objects if collision_object not in attached_collision_objects]
          for attached_object in coll_objs_to_attach:
            print('Attaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].attach_object(attached_object_name, attached_object.link_name, touch_links=  # MODIFY attach_tool in base.py to attach_object ++ ROBOT NAME???
              [attach_to + "_robotiq_85_tip_link", 
              attach_to + "_robotiq_85_left_finger_tip_link", 
              attach_to + "_robotiq_85_left_inner_knuckle_link", 
              attach_to + "_robotiq_85_right_finger_tip_link", 
              attach_to + "_robotiq_85_right_inner_knuckle_link"])
            attached_collision_objects.append(attached_object)
          for attached_object in coll_objs_to_detach:
            print('Detaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            attach_to = attached_object.link_name[:5]
            self.groups[attach_to].detach_object(attached_object_name)
          attached_collision_objects = [attached_collision_object for attached_collision_object in attached_collision_objects if attached_collision_object not in coll_objs_to_detach]

        # If joint_names is empty, the stage performs no motions and can be skipped
        if solution.trajectory.joint_trajectory.joint_names: 
          robot_name = solution.trajectory.joint_trajectory.joint_names[0][:5]
          arm_group = self.groups[robot_name]
          if len(solution.trajectory.joint_trajectory.joint_names) == 1:  # If only one joint is in the group, it is the gripper
            # Gripper motion
            hand_group = self.groups[robot_name + '_robotiq_85']
            hand_closed_joint_values = hand_group.get_named_target_values('close')
            hand_open_joint_values = hand_group.get_named_target_values('open')
            if 0.01 > abs(hand_open_joint_values[solution.trajectory.joint_trajectory.joint_names[0]] - solution.trajectory.joint_trajectory.points[-1].positions[0]):
              self.send_gripper_command(robot_name, 'open')
            else:
              self.send_gripper_command(robot_name, 'close', True)
          else:
            # raw_input()
            # Robot motion
            self.activate_ros_control_on_ur(robot_name)
            plan = arm_group.retime_trajectory(self.robots.get_current_state(), solution.trajectory, speed)
            arm_group.set_max_velocity_scaling_factor(speed)
            plan_success = arm_group.execute(plan, wait=True)
            success = success and plan_success
      if stage_name == 'pick_tool_end':
        executing_routine = False
        attached_collision_objects = self.robots.get_current_state().attached_collision_objects
      if stage_name == 'place_tool_end':
        executing_routine = False
        attached_collision_objects = self.robots.get_current_state().attached_collision_objects
      if stage_name == 'pick_screw_end':
        executing_routine = False
        attached_collision_objects = self.robots.get_current_state().attached_collision_objects
    return True

  def pick_and_move_object_with_robot(self, item_name, item_target_pose, robot_name, speed=0.1):
    """This function picks the item and move it to the target pose.
    It needs to be in the planning scene as a collision object."""
    # TODO: Implement this with MTC
    success = False
    return success
  
  def look_for_item_in_tray(self, item_name, robot_name):
    """
    This function will look for an item in the tray. After calling this function, the item
    should be published to the planning scene.
    """
    # TODO: Call the vision action to implement this
    success = False
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


  def belt_circle_motion(self, speed = 0.02):
    """
    This function moves the belt tool around the pulley(s) to insert it into the groove.
    """
    rospy.logerr("Not implemented yet")
    return False
    self.toggle_collisions(collisions_on=False)
    if go_fast:
      self.send_gripper_command("b_bot", "close")
      speed_fast = 1.5
      speed_slow = .1
    else:
      self.send_gripper_command("b_bot", "close")
      speed_fast = .2
      speed_slow = .02
    
    turning_around_large_pulley = True
    if turning_around_large_pulley:
      r_pulley=0.036
    else:
      r_pulley=0.019

    theta_offset = 90  # To adjust the starting angle
    theta_belt= 0 + theta_offset
    theta_increase=40

    start_pose = geometry_msgs.msg.PoseStamped()
    start_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    start_pose.header.frame_id = "workspace_center"
    start_pose = self.listener.transformPose("taskboard_large_pulley", start_pose)    
    start_pose.pose.position.x = cos(radians(theta_belt))*r_pulley
    start_pose.pose.position.y = sin(radians(theta_belt))*r_pulley
    start_pose.pose.position.z = 0

    approach_pose = copy.deepcopy(start_pose)
    approach_pose.pose.position.z = 0.03
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)
    self.go_to_pose_goal(robot_name, start_pose, speed=speed_slow, move_lin=True)

    rotation_count = 0
    while rotation_count < rotations:
      rotation_count += 1
      theta_belt= 0 + theta_offset
      next_pose = geometry_msgs.msg.PoseStamped()
      next_pose.pose.orientation = start_pose.pose.orientation
      next_pose.header.frame_id = "taskboard_large_pulley"
      while theta_belt <= 340+theta_offset and not rospy.is_shutdown():
          #By default, the Spiral_Search function will maintain contact between both mating parts at all times
          theta_belt=theta_belt+theta_increase
          x=cos(radians(theta_belt))*r_pulley
          y=sin(radians(theta_belt))*r_pulley
          next_pose.pose.position.x = x
          next_pose.pose.position.y = y
          print(theta_belt)
          #  print(radians(theta_belt))
          print(cos(radians(theta_belt)))
          print(cos(radians(theta_belt))*r_pulley)
          print(next_pose.pose.position)
          self.go_to_pose_goal(robot_name, next_pose, move_lin=True)
      
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)
    
    self.toggle_collisions(collisions_on=True)
    # -------------
    return True

  ########

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=0.1, speed_slow=0.02, gripper_command="close", approach_height=0.05, 
          item_id_to_attach = "", lift_up_after_pick=True, force_ur_script=False, acc_fast=1.0, acc_slow=.1, gripper_force=40.0,
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
      self.send_gripper_command(gripper=robot_name, command="open")

    rospy.loginfo("Moving down to object")
    if axis =="x":
      object_pose.pose.position.x += grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z += grasp_height * sign
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acc_slow, high_precision=True, move_lin=True)
    if axis =="x":
      object_pose.pose.position.x -= grasp_height * sign
    if axis =="z":
      object_pose.pose.position.z -= grasp_height * sign

    if gripper_command=="do_nothing":
      pass
    else: 
      self.send_gripper_command(gripper=robot_name, command="close", force = gripper_force, velocity = gripper_velocity,)

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
    #self.publish_marker(object_pose, "place_pose")
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


  ########

  def pick_screw_from_feeder(self, robot_name, screw_size, attempts = 1):
    """
    Picks a screw from one of the feeders. The screw tool already has to be equipped!
    Use this command to equip the screw tool: do_change_tool_action(self, "b_bot", equip=True, screw_size = 4)
    """
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    # self.log_to_debug_monitor("Pick screw from feeder", "operation")

    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", robot_name, speed=0.2, acceleration=0.2, force_ur_script=False)

    pose_feeder = geometry_msgs.msg.PoseStamped()
    pose_feeder.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    if robot_name == "b_bot":
      pose_feeder.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(-60), 0, 0))
    elif robot_name == "a_bot":
      pose_feeder.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(60), 0, 0))
    else:
      rospy.logerr("robot_name is not a_bot or b_bot but instead: " + robot_name)
      return False
    pose_feeder.pose.position.x = 0.0

    attempt = 0
    screw_picked = False
    while attempt < attempts:
      self.do_pick_screw_action(robot_name, pose_feeder, screw_size = screw_size, use_complex_planning = True, tool_name = "screw_tool")
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if screw_picked:
        self.go_to_named_pose("feeder_pick_ready", robot_name, speed=0.2, acceleration=0.2, force_ur_script=False)
        rospy.loginfo("Successfully picked the screw")
        return True
      if not self.use_real_robot:
        self.go_to_named_pose("feeder_pick_ready", robot_name, speed=0.2, acceleration=0.2, force_ur_script=False)
        rospy.loginfo("Pretending the screw is picked, because this is simulation.")
        return True
      attempt += 1

    if not screw_picked:
      self.go_to_named_pose("feeder_pick_ready", robot_name, speed=0.2, acceleration=0.2, force_ur_script=False)
      rospy.loginfo("Failed to pick the screw")
    return False

  def fasten_screw(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):

    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.go_to_named_pose("feeder_pick_ready", robot_name)
    
    self.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    self.go_to_named_pose("feeder_pick_ready", robot_name)
    return True

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
    # self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.go_to_pose_goal(robot_name, nut_pose, speed=.005, move_lin = True, end_effector_link="a_bot_nut_tool_m6_link")
    rospy.sleep(3)
    # self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.go_to_pose_goal(robot_name, approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def move_camera_to_pose(self, pose_goal, robot_name="b_bot", camera_name="inside_camera"):
    return self.go_to_pose_goal(robot_name, pose_goal, end_effector_link=robot_name+"_"+camera_name+"_color_optical_frame")

  def jigless_recenter(self, robot_carrying_the_item):
      pass
