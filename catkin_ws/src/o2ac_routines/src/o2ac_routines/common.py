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

  ######## Higher-level routines used in both assembly and taskboard

  def pick(self, object_name, grasp_parameter_location = '', lift_direction_reference_frame = '', lift_direction = [], speed=0.1):
    """This function picks the item. It needs to be in the planning scene as a collision object."""
    result = self.do_pick_action(object_name, grasp_parameter_location, lift_direction_reference_frame, lift_direction)
    for solution in result.solution.sub_trajectory:
      scene_diff = solution.scene_diff
      print(scene_diff.robot_state.attached_collision_objects)
      planning_scene_diff_req = moveit_msgs.srv.ApplyPlanningSceneRequest()
      planning_scene_diff_req.scene = scene_diff
      self.apply_planning_scene_diff.call(planning_scene_diff_req)
    # TODO: EXECUTION OF PICK PLAN?
    return result.success

  def pick_and_move_object_with_robot(self, item_name, item_target_pose, robot_name, speed=0.1):
    """This function picks the item and move it to the target pose.
    It needs to be in the planning scene as a collision object."""
    # TODO: Implement this with MTC
    success = False
    return success

  def pick_place(self, robot_name, object_name, object_target_pose, object_subframe_to_place, speed = 1.0):
    """This function picks the object and places its subframe 'object_subframe_to_place' at 'object_target_pose'
    using the robot referred to by 'robot_name'
    """
    result = self.do_pickplace_action(robot_name, object_name, object_target_pose, object_subframe_to_place)
    success = False
    if speed > 1.0:
      speed = 1.0
    if result.success:
      # Execute pick-place task
      success = True
      i = 0
      for solution in result.solution.sub_trajectory:
        if solution.trajectory.joint_trajectory.joint_names: 
          #  Joint names is not empty, this is stage that performs motion
          if len(solution.trajectory.joint_trajectory.joint_names) == 1:
            # gripper motion
            if i == 1:
              self.send_gripper_command(robot_name, 'close', True)
            else:
              self.send_gripper_command(robot_name, 'open')
          else:
            #robot motion
            self.activate_ros_control_on_ur(robot_name)
            group = self.groups[robot_name]
            plan = group.retime_trajectory(self.robots.get_current_state(), solution.trajectory, speed)
            plan_success = group.execute(plan, wait=True)
            success = success and plan_success
            group.stop()
    else:
      rospy.logwarn("Planning pick-place task failed")
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

  def pick_screw_from_feeder(self, screw_size, attempts = 1):
    """
    Picks a screw from one of the feeders. The screw tool already has to be equipped!
    Use this command to equip the screw tool: do_change_tool_action(self, "b_bot", equip=True, screw_size = 4)
    """
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False

    self.log_to_debug_monitor("Pick screw from feeder", "operation")

    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)

    pose_feeder = geometry_msgs.msg.PoseStamped()
    pose_feeder.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    pose_feeder.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose_feeder.pose.position.x = 0.0

    attempt = 0
    screw_picked = False
    while attempt < attempts:
      self.set_feeder_power(False)
      self.do_pick_screw_action("b_bot", pose_feeder, screw_size = screw_size, use_complex_planning = True, tool_name = "screw_tool")
      self.set_feeder_power(True)
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if screw_picked:
        rospy.loginfo("Successfully picked the screw")
        return True
      if not self.use_real_robot:
        rospy.loginfo("Pretending the screw is picked, because this is simulation.")
        return True
      attempt += 1

    return False

  def pick_nut(self, robot_name):
    """Pick the nut from the holder. The nut tool has to be equipped.
    Use this command to equip: do_change_tool_action(self, "b_bot", equip=True, screw_size = 66)"""
    rospy.logerr("Not implemented yet")
    return False
    approach_pose = copy.deepcopy(nut_pose)
    approach_pose.pose.position.z += .02  # Assumes that z points upward
    self.go_to_pose_goal(robot_name, approach_pose, speed=self.speed_fast, move_lin = True, end_effector_link=end_effector_link)
    spiral_axis = "Y"
    push_direction = "Z+"
    self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.set_motor("nut_tool_m6", direction="loosen", duration=10)
    self.horizontal_spiral_motion(robot_name, max_radius = .006, radius_increment = .02, spiral_axis=spiral_axis)
    self.do_linear_push(robot_name, 10, direction=push_direction, wait = True)
    self.go_to_pose_goal(robot_name, approach_pose, speed=.03, move_lin = True, end_effector_link=end_effector_link)

  def jigless_recenter(self, robot_carrying_the_item):
      pass