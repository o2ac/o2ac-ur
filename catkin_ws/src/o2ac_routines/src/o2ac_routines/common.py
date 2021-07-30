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

from o2ac_routines import helpers
from o2ac_routines.base import *
from math import radians, degrees, sin, cos, pi
tau = 2*pi
from o2ac_routines.thread_with_trace import ThreadTrace
from ur_control.constants import DONE, TERMINATION_CRITERIA
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np

CORNER = "corner"
TOO_CLOSE_TO_OTHER_OBJECTS = "too_close_to_other_object"
TOO_CLOSE_TO_BORDER = "too_close_to_border"
Y_BORDER_SAFE = "y_border_safe"
X_BORDER_SAFE = "x_border_safe"

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
    self.assembly_marker_id_counter = 1  # For tracking visualization markers
    self.assembly_marker_publisher = rospy.Publisher("o2ac_assembly_markers", visualization_msgs.msg.Marker, queue_size = 100)

    self.define_tray_views()

  def define_tray_views(self):
    """
    Define the poses used to position the camera to look into the tray.

    Example usage: self.b_bot.go_to_pose_goal(self.tray_view_high, 
                                        end_effector_link="b_bot_outside_camera_color_frame", 
                                        speed=.1, acceleration=.04)
    """
    high_height = .38
    low_height = .24
    x_offset = .055  # At low_height
    y_offset = .095  # At low_height

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "tray_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    ps.pose.position.z = high_height

    # Centered views (high up and close)
    self.tray_view_high = copy.deepcopy(ps)
    self.tray_view_high.pose.position.x += 0.015  # Magic number (the camera frames used for planning are uncalibrated)
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
    self.planning_scene_interface.remove_attached_object(name=object_name)

    # # DEBUGGING: Remove object from the scene
    # if True:
    #   self.planning_scene_interface.remove_world_object(name=object_name)
      
    # marker = self.assembly_database.get_visualization_marker(object_name, self.assembly_marker_id_counter)
    # self.assembly_marker_id_counter += 1
    # self.assembly_marker_publisher.publish(marker)

    object_id = self.assembly_database.name_to_id(object_name)
    collision_object = self.assembly_database.get_collision_object(object_name)
    if test_header_frame:
      collision_object.header.frame_id = test_header_frame
    else:
      collision_object.header.frame_id = "assembled_part_" + str(object_id).zfill(2)  # Fill with leading zeroes
    self.planning_scene_interface.apply_collision_object(collision_object)

    # Remove collisions with scene to avoid unnecessary calculations
    self.planning_scene_interface.allow_collisions(object_name, "")
    # Do check collisions with moving parts (robot hands, tools) (cameras should be included, but ohwell)
    self.allow_collisions_with_robot_hand(object_name, "a_bot", allow=False)
    self.allow_collisions_with_robot_hand(object_name, "b_bot", allow=False)
    self.planning_scene_interface.disallow_collisions(object_name, "screw_tool_m3")
    self.planning_scene_interface.disallow_collisions(object_name, "screw_tool_m4")

    # Make sure the object is detached from all robots
    # for robot in self.active_robots.values():
    #   if object_name == robot.gripper.last_attached_object:
    #     robot.gripper.last_attached_object = None
    
    return
  
  def reset_assembly_visualization(self):
    """ Clears all visualization markers """
    m = visualization_msgs.msg.Marker()
    m.action = m.DELETEALL
    self.assembly_marker_publisher.publish(m)

  def set_assembly(self, assembly_name="wrs_assembly_2020"):
    self.assembly_database.change_assembly(assembly_name)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'attached_base_origin_link'
    pose.pose.orientation.w = 1.0
    self.assembly_database.publish_assembly_frames(pose, prefix="assembled_")
    return True

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
      # TODO(cambel): interesting, maybe insertion can go here?
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
            self.active_robots[robot_name_].gripper.detach_object(attached_object_name)
          for attached_object in coll_objs_to_attach:
            # print('Attaching object ' + attached_object.object.id + ' in stage ' + stage_name)
            attached_object_name = attached_object.object.id
            robot_name_ = attached_object.link_name[:5]
            self.active_robots[robot_name_].gripper.attach_object(attached_object_name, attached_object.link_name)
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

  def get_large_item_position_from_top(self, item_name, robot_name="b_bot", skip_moving=False):
    """
    This function look at the tray from the top only, and publishes the result to the planning scene.
    
    Returns False if object was not found, the pose if it was.
    """

    # Look from top first
    self.vision.activate_camera(robot_name+"_outside_camera")
    if not skip_moving:
      if not self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, end_effector_link=robot_name+"_outside_camera_color_frame"):
        rospy.logerr("Failed to move to camera view pose in get_large_item_position_from_top.")
        return False
    object_pose = self.detect_object_in_camera_view(item_name)

    if object_pose:
      rospy.logdebug("Found " + item_name + ". Publishing to planning scene.")
      # TODO: Apply Place action of pose estimation to constrain object to the tray_center plane
      # Workaround: Just set z to 0 
      if item_name == "base":
        object_pose.header.stamp = rospy.Time.now()
        self.listener.waitForTransform("tray_center", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(1))
        object_pose = self.listener.transformPose("tray_center", object_pose)
        #### fake the orientation of the plate (assumes that it is always flat on the tray) ###
        orientation_euler = list(transformations.euler_from_quaternion(conversions.from_pose_to_list(object_pose.pose)[3:]))
        orientation_euler[0] = np.sign(orientation_euler[0]) * tau/4
        orientation_euler[1] = 0
        object_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(*orientation_euler))
        object_pose.pose.position.z = 0.0

      obj = self.assembly_database.get_collision_object(item_name)
      obj.header.frame_id = object_pose.header.frame_id
      obj.pose = object_pose.pose
      self.planning_scene_interface.add_object(obj)

      rospy.sleep(1)
      ob = self.constrain_into_tray(item_name).pose
    else:
      rospy.logdebug("Failed to find " + item_name)
      return False
      
    return object_pose

  def look_and_get_object_pose(self, object_id, robot_name="b_bot"):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """
    if not self.use_real_robot: # For simulation
      rospy.logwarn("Returning position near center (simulation)")
      return conversions.to_pose_stamped("tray_center", [-0.01, 0.05, 0.02] + np.deg2rad([0,90.,0]).tolist())

    # Make sure object_id is the id number
    if isinstance(object_id, str):
      try:
        object_id_num = self.assembly_database.name_to_id(object_id)
      except:
        rospy.logerr("Could not find object id " + object_id + " in database!")
        return False
      rospy.logwarn("look_and_get_grasp_point got " + object_id + " but will use id number " + str(object_id_num))
      return self.look_and_get_object_pose(object_id_num, robot_name)

    self.vision.activate_camera(robot_name + "_outside_camera")
    self.activate_led(robot_name)

    if object_id in self.objects_in_tray:
      del self.objects_in_tray[object_id]

    if self.use_dummy_vision:
      self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True)
      rospy.logwarn("Using dummy vision! Setting object pose to tray center.")
      self.objects_in_tray[object_id] = conversions.to_pose_stamped("tray_center", [ 0,0,0, 0, 0, 0])
      return self.objects_in_tray[object_id]

    for view in [self.tray_view_high] + self.close_tray_views + self.close_tray_views_rot_left + self.close_tray_views_rot_right + self.close_tray_views_rot_left_more + self.close_tray_views_rot_left_90:
      assert not rospy.is_shutdown()
      self.active_robots[robot_name].go_to_pose_goal(view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True)
      rospy.sleep(0.5)
      self.get_3d_poses_from_ssd()

      object_pose = copy.deepcopy(self.objects_in_tray.get(object_id, None))
      if object_pose:
        rospy.loginfo("Getting grasp points for object_id : " + str(object_id) + " at pose:")
        rospy.loginfo(self.objects_in_tray[object_id].pose.position)

        # Get another view from up close (if the view was already close, the object may have been on the edge of the image). 
        close_view = self.listener.transformPose("tray_center", self.objects_in_tray[object_id])
        close_view.pose.position.x += 0.025  # Avoid LED glare
        close_view.pose.position.z = copy.deepcopy(self.close_tray_views[0].pose.position.z)
        close_view.pose.orientation = copy.deepcopy(view.pose.orientation)
        
        rospy.loginfo("Looking closer at object_id " + str(object_id))
        self.active_robots[robot_name].go_to_pose_goal(close_view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.3, acceleration=.3)
        rospy.sleep(0.5)
        self.get_3d_poses_from_ssd()
        
        close_object_pose = copy.deepcopy(self.objects_in_tray.get(object_id, None))

        if close_object_pose:
          return close_object_pose
        else:
          rospy.logwarn("(close view) Could not find item id " + str(object_id) + " in tray!")
          rospy.logwarn("returning previous best estimation")
          return object_pose
    rospy.logerr("Could not find item id " + str(object_id) + " in tray!")
    return False

  def look_and_get_grasp_point(self, object_id, robot_name="b_bot", options={}):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """
    center_on_corner       = options.get("center_on_corner", False)
    grasp_width            = options.get("grasp_width", 0.06)
    center_on_close_border = options.get("center_on_close_border", False)
    grab_and_drop          = options.get("grab_and_drop", False)
    declutter_with_tool    = options.get("declutter_with_tool", False)

    object_pose = self.look_and_get_object_pose(object_id, robot_name)
    options.update({"rotation_offset": -1 if robot_name == "b_bot" else 1})
    if object_pose:
      rospy.loginfo("Object found: checking for feasible grasps")
      grasps = self.get_feasible_grasp_points(object_id, object_pose=object_pose, options=options)
      print("grasps found?", grasps)
      if grasps:
        if grasps == CORNER and center_on_corner:
          if not self.move_towards_tray_center_from_corner(robot_name, object_pose, options):
            rospy.logerr("Fail to move_towards_tray_center_from_corner")
            return False
          return self.look_and_get_grasp_point(object_id, robot_name, {'grasp_width': grasp_width})
        elif grasps == TOO_CLOSE_TO_BORDER and center_on_close_border:
            rospy.loginfo("Moving away from border")
            if not self.move_towards_center_from_border(robot_name, object_pose, options):
              rospy.logerr("Fail to move_towards_center_from_border")
              return False
            return self.look_and_get_grasp_point(object_id, robot_name, {'grasp_width': grasp_width})
        elif grasps == TOO_CLOSE_TO_OTHER_OBJECTS:
          if grab_and_drop:
            if not self.grab_and_drop(robot_name, object_pose, {'grasp_width': grasp_width}):
              rospy.logerr("Fail to grab and drop")
              return False
          elif declutter_with_tool:
            if not self.declutter_with_tool(robot_name, object_pose):
              rospy.logerr("Fail to declutter_with_tool")
              return False
          else:
            return False
          return self.look_and_get_grasp_point(object_id, robot_name, {'grasp_width': grasp_width, 'grab_and_drop': True, 'center_on_corner': True}) # May end in infinite loop?
        else:
          return grasps[0]
      rospy.logerror("No feasible grasps! %s" % grasps)
    return False

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

  def constrain_into_tray(self, item_name=""):
    """ For an L-plate or base_plate, make sure the object does not exceed the tray border.
    """
    if item_name not in ["base", "panel_motor", "panel_bearing"]:
      rospy.logerr("Unknown item_name received in constrain_into_tray: " + item_name)
      return False
    if item_name == "base":
      dims = [0, 0.12, 0, 0.2]  # Object dimensions: [min_x, max_x, min_y, max_y]
    if item_name == "panel_motor":
      dims = [0, 0.06, 0, 0.06]
    if item_name == "panel_bearing":
      dims = [0, 0.116, 0, 0.09]

    # Check each corner point. If outside tray border, move the object.
    object_pose = conversions.to_pose_stamped("move_group/"+item_name, [0,0,0, 0,0,0])
    object_pose_in_tray = self.get_transformed_collision_object_pose(item_name, object_pose, "tray_center")
    
    if item_name == "base":
      axes = [0, 2]  # x, z
    if item_name == "panel_motor" or item_name == "panel_bearing":
      axes = [0, 1]  # x, y
    corner_points = [ [dims[0], dims[2]], [dims[0], dims[3]], [dims[1], dims[2]], [dims[1], dims[3]] ]
    for point in corner_points:
      object_corner_list = conversions.from_pose_to_list(object_pose.pose)
      object_corner_list[axes[0]] += point[0]
      object_corner_list[axes[1]] += point[1]
      object_corner_pose = conversions.to_pose_stamped(object_pose.header.frame_id, object_corner_list)

      dx, dy = self.distances_from_tray_border(object_corner_pose)
      if dx < 0 or dy < 0:
        if dx < 0:
          object_pose_in_tray.pose.position.x -= np.sign(object_pose_in_tray.pose.position.x) * abs(dx)
        if dy < 0:
          object_pose_in_tray.pose.position.y -= np.sign(object_pose_in_tray.pose.position.y) * abs(dy)
        try:
          self.listener.waitForTransform("move_group/"+item_name, object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(1))
          object_pose = self.listener.transformPose("move_group/"+item_name, object_pose_in_tray)
          rospy.loginfo("Moved ""dx:", dx, "dy:", dy)
        except Exception as e:
          print(e)
          pass
      
    # Update collision object position
    obj = self.assembly_database.get_collision_object(item_name)
    obj.header.frame_id = object_pose_in_tray.header.frame_id
    obj.pose = object_pose_in_tray.pose
    self.planning_scene_interface.add_object(obj)

    return object_pose_in_tray

  def get_transformed_grasp_pose(self, object_name, grasp_name, target_frame="tray_center"):
    """ Get an object's grasp pose in the target_frame"""
    grasp_pose = self.assembly_database.get_grasp_pose(object_name, grasp_name)
    return self.get_transformed_collision_object_pose(object_name, grasp_pose, target_frame)
  
  def get_transformed_collision_object_pose(self, object_name, object_pose, target_frame="tray_center"):
    """ Get the pose of a MoveIt CollisionObject in the target_frame"""
    try:
      object_pose.header.frame_id = "move_group/" + object_name
      self.listener.waitForTransform(target_frame, object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(1))
    except Exception as e:
      rospy.logwarn(e)
      return False
    return self.listener.transformPose(target_frame, object_pose)

  ########

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=0.5, speed_slow=0.3, gripper_command="close", 
          gripper_force=40.0, grasp_width=0.140, minimum_grasp_width=0.0,
          approach_height=0.05, item_id_to_attach = "", 
          lift_up_after_pick=True, acc_fast=1.0, acc_slow=.1, 
          gripper_velocity = .1, axis="x", sign=+1,
          retreat_height = None, approach_with_move_lin=True):
    """
    This function (outdated) performs a grasp with the robot, but it is not updated in the planning scene.
    It does not use the object in simulation. It can be used for simple tests and prototyping, but should
    be replaced by the pick() function for the real competition.

    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    The attachment will be visible in the MoveIt planning scene. The object and its subframes can be used
    as an end effector.
    """
    rospy.loginfo("Entered simple_pick")
    
    robot = self.active_robots[robot_name]
    if gripper_command=="do_nothing":
      pass
    else: 
      robot.gripper.send_command(command=grasp_width, wait=False) # Open

    approach_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += approach_height * sign
    approach_pose.pose.position = conversions.to_point(op)

    rospy.logdebug("Going to height " + str(op[get_direction_index(axis)]))
    if not robot.go_to_pose_goal(approach_pose, speed=speed_fast, acceleration=acc_fast, move_lin=approach_with_move_lin, wait=True):
      rospy.logerr("Fail to go to approach_pose")
      return False

    rospy.logdebug("Moving down to object")
    grasp_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += grasp_height * sign
    grasp_pose.pose.position = conversions.to_point(op)
    rospy.logdebug("Going to height " + str(op[get_direction_index(axis)]))

    if not robot.go_to_pose_goal(grasp_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True):
      rospy.logerr("Fail to go to grasp_pose")
      return False

    if gripper_command=="do_nothing":
      pass
    else: 
      robot.gripper.send_command(command="close", force = gripper_force, velocity = gripper_velocity)

    if item_id_to_attach:
      self.allow_collisions_with_robot_hand(item_id_to_attach, robot_name)
      robot.gripper.attach_object(object_to_attach=item_id_to_attach)

    if lift_up_after_pick:
      rospy.sleep(1.0)
      rospy.logdebug("Going back up")

      if retreat_height is None:
        retreat_height = approach_height
      retreat_pose = copy.deepcopy(object_pose)
      op = conversions.from_point(object_pose.pose.position)
      op[get_direction_index(axis)] += retreat_height * sign
      retreat_pose.pose.position = conversions.to_point(op)
      rospy.logdebug("Going to height " + str(retreat_pose.pose.position.z))
      if not robot.go_to_pose_goal(retreat_pose, speed=speed_fast, acceleration=acc_fast, move_lin=True):
        rospy.logerr("Fail to go to lift_up_pose. Opening.")
        robot.gripper.open(grasp_width)
        return False
      robot.gripper.close() # catch false grasps
    
    # if not self.simple_gripper_check(robot_name, minimum_grasp_width):
    if minimum_grasp_width > robot.gripper.opening_width and self.use_real_robot:
      rospy.logerr("Gripper opening width after pick less than minimum (" + str(minimum_grasp_width) + "): " + str(robot.gripper.opening_width) + ". Return False.")
      return
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
      robot.gripper.open()

    if item_id_to_detach:
      robot.robot_group.detach_object(item_id_to_detach)

    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      self.active_robots[robot_name].go_to_pose_goal(object_pose, speed=speed_fast, move_lin=False)  
    return True

  def simple_grasp_generation(self, object_pose, options):
    """
    Returns a list of one grasp for an object.
    Based only on border distance and distance to other objects.
    object_pose:
    options: dict, dictionary of options to apply for the simple grasp generation
             grasp_width=0.06, grasp_z_height=0.02, rotation_offset=1, check_for_close_items=True, check_too_close_to_border=False, min_dist_to_border=0.02, allow_pick_near_border=True
    check_for_close_items: bool, if True, check that there are no objects too close to the target object, otherwise just check distance to tray's borders
    check_too_close_to_border: bool, if True, check that the border is not only feasible by the `grasp_width` but also that it is not too close to the tray's border by a minimum distance
                               useful for very small items (shaft, end cap, ...) but unnecessary for big items
    """
    grasp_z_height            = options.get("grasp_z_height", 0.02)
    rotation_offset           = options.get("rotation_offset", 1)
    allow_pick_near_border    = options.get("allow_pick_near_border", True)

    # This grasp pose opens the gripper along the workspace_center's y-axis
    grasp_along_y = copy.deepcopy(object_pose)
    grasp_along_y.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    grasp_along_y.pose.position.z = grasp_z_height

    # This one opens the gripper along the workspace_center's x-axis
    grasp_along_x = copy.deepcopy(grasp_along_y)
    grasp_along_x.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, rotation_offset*tau/4))
    grasps_solutions = [] # valid grasps based on distance to tray's border and closer items
    
    safe_conditions = self.grasp_sanity_check(object_pose, options)

    if isinstance(safe_conditions, list):
      if TOO_CLOSE_TO_BORDER in safe_conditions and allow_pick_near_border:
        rospy.logwarn("Too close to a border, returning a safe grasp pose")
        grasp = grasp_along_y if safe_conditions[0] == Y_BORDER_SAFE else grasp_along_x
        print("grasp pose pre modification")
        print(grasp.pose)
        grasp.pose.position.x = grasp.pose.position.x if abs(grasp.pose.position.x) < 0.11 else np.sign(grasp.pose.position.x) * 0.11 
        grasp.pose.position.y = grasp.pose.position.y if abs(grasp.pose.position.y) < 0.17 else np.sign(grasp.pose.position.y) * 0.17
        print("grasp pose post modification")
        print(grasp.pose)
        
        grasps_solutions.append(grasp)
        return grasps_solutions
      elif TOO_CLOSE_TO_BORDER in safe_conditions and not allow_pick_near_border:
        return TOO_CLOSE_TO_BORDER
      for sc in safe_conditions:
        grasp = grasp_along_y if sc == Y_BORDER_SAFE else grasp_along_x
        grasps_solutions.append(grasp)
      return grasps_solutions
    return safe_conditions

  def grasp_sanity_check(self, object_pose, options):
    """
      dist_close: float, min distance allowed in the direction that the gripper does not open
    """  
    grasp_width               = options.get("grasp_width", 0.06)
    check_for_close_items     = options.get("check_for_close_items", True)
    check_too_close_to_border = options.get("check_too_close_to_border", False)
    min_dist_to_border        = options.get("min_dist_to_border", 0.02)

    dist_far = (grasp_width+0.02) / 2.0  # Along the gripper's opening direction
    
    safe_conditions = [] # relative to tray's border or other items proximity
    solutions = []

    (dx, dy) = self.distances_from_tray_border(object_pose)

    if dy > dist_far:
      safe_conditions.append(Y_BORDER_SAFE)
    else:
      rospy.loginfo("Too close to the Y border")
    
    if dx > dist_far:
      safe_conditions.append(X_BORDER_SAFE)
    else:
      rospy.loginfo("Too close to the X border")

    rospy.loginfo("Border distances were %0.3f, %0.3f, dist_far: %0.3f distclose: %0.3f" % (dx, dy, dist_far, min_dist_to_border))
    # First check that this is not a corner
    if not safe_conditions:
      rospy.logerr("Too close to borders. Discarding. border distances were %0.3f, %0.3f, min dist: %0.3f" % (dx, dy, dist_far))
      return CORNER

    # Then check that non of the borders are too close
    print("check border", check_too_close_to_border)
    if check_too_close_to_border and (dy < min_dist_to_border or dx < min_dist_to_border):
      rospy.logerr("Too close to a border")
      rospy.logerr("Border distances were %0.3f, %0.3f, min dist: %0.3f" % (dx, dy, min_dist_to_border))
      safe_conditions.append(TOO_CLOSE_TO_BORDER)
      return safe_conditions
      
    # Finally check that other objects are not too close
    print("check for close items", check_for_close_items)
    if check_for_close_items:
      for condition in safe_conditions:
        item_too_close = False
        for obj, pose in self.objects_in_tray.items():
          if obj == 6: # Skip the belt
            continue
          dx = abs(pose.pose.position.x - object_pose.pose.position.x)
          dy = abs(pose.pose.position.y - object_pose.pose.position.y)
          if dx < 1e-4 and dy < 1e-4:
            continue  # It's the item itself or a duplicate
          if condition == Y_BORDER_SAFE:
            if dx < min_dist_to_border and dy < dist_far:
              item_too_close = True
          elif condition == X_BORDER_SAFE:
            if dy < min_dist_to_border and dx < dist_far:
              item_too_close = True
        if item_too_close:
          rospy.loginfo("Too close to another item. Discarding. distance: " + str(pose_dist(pose.pose, object_pose.pose)) + ", id: " + str(obj))
        else:
          solutions.append(condition)
      
      if not solutions:
        rospy.logerr("Too close to another item. Not feasible grasp found!")
        return TOO_CLOSE_TO_OTHER_OBJECTS
      return solutions
    else:
      return safe_conditions
  
  def pick_from_two_poses_topdown(self, robot_name, object_name, object_pose, grasp_width=0.06):
    """ Plan a pick from above and execute it with MTC.
        This is meant for cylindrical objects like the bearing.
        Two grasp poses are evaluated. The gripper opens along either x and y and faces down.
        An object with the name `object_name` has to be placed in the scene
        
        object_pose is the PoseStamped of the object center. The orientation is ignored.
    """
    self.active_robots[robot_name].gripper.open(opening_width=grasp_width) # Open gripper to avoid silly failure. can we do better?
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
    self.active_robots[robot_name].gripper.open() # Open gripper to avoid silly failure. can we do better?
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
    self.disable_scene_object_collisions()
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

  def thin_out_grasp_poses(self, grasp_poses, radius=0.02):
    """ Takes a vector of grasp_poses and removes candidates that are too close to one another (distance < radius).
        Candidates with higher distance from the border are preferred.
        This is used for the belt grasp pose candidates.

        This procedure may discard more grasp poses than necessary.
    """
    new_grasp_poses = []
    for g1 in grasp_poses:
      keep_g1 = True
      for g2 in grasp_poses:
        if g1 == g2:
          continue
        if pose_dist(g1.pose, g2.pose) < radius:
          dx1, dy1 = self.distances_from_tray_border(g1)
          dx2, dy2 = self.distances_from_tray_border(g2)
          if min(dx1, dy1) < min(dx2, dy2):
            keep_g1 = False
            break
      if keep_g1:
        new_grasp_poses.append(g1)
    return new_grasp_poses

  def too_close_to_border(self, grasp_pose, border_dist=0.06):
    """ Returns true if the point is too close to the border of the tray (= too far away from the tray center).
    """
    (dx, dy) = self.distances_from_tray_border(grasp_pose)
    if dx < border_dist or dy < border_dist:
      return True
    return False

  def simple_grasp_sanity_check(self, grasp_pose, grasp_width=0.08, border_dist=0.06):
    """
    Returns true if the grasp pose is further than 5 cm away from the tray border,
    and no other detected objects are closer than 5 cm.

    grasp_pose is a PoseStamped.
    """
    (dx, dy) = self.distances_from_tray_border(grasp_pose)
    if dx < border_dist or dy < border_dist:
      rospy.loginfo("too close to border. discarding. border distances were %0.3f, %0.3f" % (dx, dy))
      return False
    for obj, pose in self.objects_in_tray.items():
      if obj == 6: # Hard-code skipping the belt
        # rospy.loginfo("Skipping the belt grasp points during grasp sanity check")
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
    
  def get_feasible_grasp_points(self, object_in_scene, object_pose=None, options={}):
    """
    Returns a list of PoseStamped grasp points for an object that is currently in the scene.
    object_in_scene can be the string or the id number of the object.
    """
    # , grasp_width=0.06, check_for_close_items=True, check_too_close_to_border=False, rotation_offset=1, min_dist_to_border=0.02, allow_pick_near_border=True
    if isinstance(object_in_scene, str):
      object_id = self.assembly_database.name_to_id(object_in_scene)
    else:
      object_id = object_in_scene

    if not object_pose and object_id not in self.objects_in_tray:
      rospy.logerr("Grasp points requested for " + str(object_id) + " but it is not seen in tray.")
      return False
    
    if object_id in self.belt_id:
      # We get the belt grasp candidates directly from the vision because they are not stored anywhere from a previous view
      res = self.get_3d_poses_from_ssd()
      grasp_poses = []
      for idx, pose in enumerate(res.poses):
        if res.class_ids[idx] == 6:
          if self.is_grasp_pose_feasible(pose, border_dist=0.05):
            grasp_poses.append(pose)
      grasp_poses = self.thin_out_grasp_poses(grasp_poses)
      return grasp_poses

    object_ps = object_pose if object_pose is not None else self.objects_in_tray[object_id]

    if object_id in self.small_item_ids:
      # For the shaft, use the orientation from the SSD
      if self.assembly_database.id_to_name(object_id) == "shaft":
        res = self.grasp_sanity_check(object_ps, options)
        print("shaft>>> sanity check", res)
        if isinstance(res, list):
          if TOO_CLOSE_TO_BORDER in res:
            return TOO_CLOSE_TO_BORDER
          if len(res) == 2:
            return [object_ps] # pick is possible along x and y so try as usual
          elif len(res) == 1 and res[0] in (X_BORDER_SAFE, Y_BORDER_SAFE): 
            # TODO (cambel): can we do better for the shaft? Use shaft orientation to see if the pick is possible
            # There are object too close to the shaft on one side
            return TOO_CLOSE_TO_OTHER_OBJECTS
        else:
          return res

      # For other small items, use any pose from above that works
      return self.simple_grasp_generation(object_pose=object_ps, options=options)
      # TODO: Consider the idler spacer, which can stand upright or lie on the side.
      
    if object_id in self.large_item_ids:
      # For large items, use any pose from above that works
      # TODO: Get grasp poses from database
      return self.simple_grasp_generation(object_pose=object_ps, options=options)
    
    return [None]

  def distances_from_tray_border(self, object_pose):
    """
    Returns the distance from the tray border as an (x, y) tuple.
    x, y are in the tray coordinate system.
    Distance is signed (negative is outside the tray).
    """
    # Inside tray width and length: 25.5 cm, 37.5 cm
    l_x_half = .255/2.0
    l_y_half = .375/2.0
    object_pose_in_tray = self.listener.transformPose("tray_center", object_pose)
    xdist = l_x_half - abs(object_pose_in_tray.pose.position.x)
    ydist = l_y_half - abs(object_pose_in_tray.pose.position.y)
    return (xdist, ydist)

  def declutter_with_tool(self, robot_name, starting_pose):
    if robot_name == "a_bot":
      self.a_bot.go_to_named_pose("home")
    
    robot_name == "b_bot"
    
    self.allow_collisions_with_robot_hand("plunger_tool_link", "b_bot")
    if not self.playback_sequence("plunger_tool_equip"):
      rospy.logerr("Fail to equip tool")
      return False

    # if we are too close to the border, move to a fixed distance from the borders
    dx, dy = self.distances_from_tray_border(starting_pose)
    at_start_pose = copy.deepcopy(starting_pose)
    at_start_pose.pose.position.x = at_start_pose.pose.position.x if dx > 0.035 else np.sign(at_start_pose.pose.position.x) * 0.085
    at_start_pose.pose.position.y = at_start_pose.pose.position.y if dy > 0.035 else np.sign(at_start_pose.pose.position.y) * 0.145

    seq = []

    safe_approach_pose = copy.deepcopy(at_start_pose)
    safe_approach_pose.pose.position.z = 0.20
    
    at_tray_border_pose = copy.deepcopy(safe_approach_pose)
    at_tray_border_pose.pose.position.z = 0.1

    # Move the pose inwards to avoid hitting the tray border
    if self.too_close_to_border(at_tray_border_pose, border_dist=0.04):
      at_tray_border_pose.pose.position.x -= 0.03 * np.sign(at_tray_border_pose.pose.position.x)
      at_tray_border_pose.pose.position.y -= 0.03 * np.sign(at_tray_border_pose.pose.position.y)
    
    spiral_trajectory = compute_trajectory(conversions.from_pose_to_list(at_tray_border_pose.pose), 
                                      "XY", 0.03, "+Y", steps=50, revolutions=3, from_center=True,  trajectory_type="spiral")
    spiral_trajectory = [conversions.to_pose_stamped(at_tray_border_pose.header.frame_id, t) for t in spiral_trajectory]

    seq.append(helpers.to_sequence_item(safe_approach_pose))
    seq.append(helpers.to_sequence_item(at_tray_border_pose))
    seq.append(helpers.to_sequence_trajectory(spiral_trajectory, 0.0, speed=0.2, default_frame=at_tray_border_pose.header.frame_id))
    seq.append(helpers.to_sequence_item_relative([0,0,0.08,0,0,0]))

    if not self.execute_sequence("b_bot", seq, 'declutter_with_tool'):
      rospy.logerr("Fail to declutter")
    
    if not self.playback_sequence("plunger_tool_unequip"):
      rospy.logerr("Fail to unequip tool")
      return False

    self.b_bot.go_to_named_pose("home")

    return True

  def move_towards_center_with_tool(self, robot_name, target_pose, distance=0.10, direction=None, start_with_spiral = False):
    if robot_name == "a_bot":
      self.a_bot.go_to_named_pose("home")
    
    # Put pose into tray_center and assign orientation
    self.listener.waitForTransform("tray_center", target_pose.header.frame_id, target_pose.header.stamp, rospy.Duration(1.0))
    target_pose_tray = self.listener.transformPose("tray_center", target_pose)
    target_pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    
    robot_name == "b_bot"
    robot = self.active_robots[robot_name]

    if not self.playback_sequence("plunger_tool_equip"):
      rospy.logerr("Fail to equip tool")
      return False

    safe_approach_pose = copy.deepcopy(target_pose_tray)
    if direction == 'x':
      safe_approach_pose.pose.position.x -= np.sign(safe_approach_pose.pose.position.x) * 0.015
    if direction == 'y':
      safe_approach_pose.pose.position.y -= np.sign(safe_approach_pose.pose.position.y) * 0.015
    safe_approach_pose.pose.position.z = 0.06
    if not robot.go_to_pose_goal(safe_approach_pose, end_effector_link="b_bot_plunger_tip_link"):
      rospy.logerr("Fail to approach 1")
      self.playback_sequence("plunger_tool_unequip")
      self.b_bot.go_to_named_pose("home")
      return False

    touch_pose = copy.deepcopy(target_pose_tray)
    touch_pose.pose.position.z = -0.002
    if not robot.go_to_pose_goal(touch_pose, speed=0.05, end_effector_link="b_bot_plunger_tip_link"):
      rospy.logerr("Fail to approach 2")
      self.playback_sequence("plunger_tool_unequip")
      self.b_bot.go_to_named_pose("home")
      return False
    
    if start_with_spiral and not self.too_close_to_border(touch_pose, border_dist=0.015):
      spiral_trajectory = compute_trajectory(conversions.from_pose_to_list(touch_pose.pose), 
                                      "XY", 0.01, "+Y", steps=50, revolutions=1, from_center=True,  trajectory_type="spiral")
      spiral_trajectory = [conversions.to_pose_stamped(touch_pose.header.frame_id, t) for t in spiral_trajectory]
      seq = [helpers.to_sequence_trajectory(spiral_trajectory, 0.0, speed=0.2, default_frame=touch_pose.header.frame_id)]
      if not self.execute_sequence("b_bot", seq, 'spiral_for_tool', end_effector_link="b_bot_plunger_tip_link"):
        rospy.logerr("Fail to do spiral")

    if not self.move_towards_tray_center(robot_name, distance=distance, go_back_halfway=False, one_direction=direction, speed=0.1, acc=0.1, end_effector_link="b_bot_plunger_tip_link"):
      rospy.logerr("Fail to move towards center")
      self.playback_sequence("plunger_tool_unequip")
      self.b_bot.go_to_named_pose("home")
      return False      

    if not self.active_robots[robot_name].move_lin_rel(relative_translation=[0, 0, 0.08]):
      rospy.logerr("Fail to move up")
      return False      

    if not self.playback_sequence("plunger_tool_unequip"):
      rospy.logerr("Fail to unequip tool")
      return False

    self.b_bot.go_to_named_pose("home")
    
    return True

  def move_towards_center_from_border_with_tool(self, robot_name, object_pose):
    border_pose = copy.deepcopy(object_pose)
    (dx, dy) = self.distances_from_tray_border(object_pose)
    direction = None
    if dx < dy: # Use the close border
      direction = 'x'
      border_pose.pose.position.x = 0.12 if np.sign(border_pose.pose.position.x) == 1 else -0.13 # non-symmetric tray_center
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, tau/4+np.sign(border_pose.pose.position.x)*radians(15), -tau/2))
    else:
      direction = 'y'
      border_pose.pose.position.y = 0.186 if np.sign(border_pose.pose.position.y) == 1 else -0.19 # non-symmetric tray_center
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, tau/4-np.sign(border_pose.pose.position.y)*radians(15), tau/4))

    return self.move_towards_center_with_tool(robot_name, border_pose, direction=direction)

  def move_towards_center_from_border(self, robot_name, object_pose, options):
    with_tool = options.get('with_tool', False)
    go_back_halfway = options.get('go_back_halfway', True)

    if with_tool:
      return self.move_towards_center_from_border_with_tool(robot_name, object_pose)

    robot = self.active_robots[robot_name]
    rotation_offset = -1 if robot_name == "b_bot" else 1
    
    robot.gripper.close()

    border_pose = copy.deepcopy(object_pose)
    (dx, dy) = self.distances_from_tray_border(object_pose)
    if dx < dy: # Use the close border
      direction = 'x'
      border_pose.pose.position.x = np.sign(border_pose.pose.position.x) * 0.11
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, rotation_offset*tau/4))
    else:
      direction = 'y'
      border_pose.pose.position.y = np.sign(border_pose.pose.position.y) * 0.17
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))

    return self.move_towards_tray_center_with_push(robot_name, border_pose, approach_height=0.02, direction=direction, go_back_halfway=go_back_halfway)

  def move_towards_tray_center_from_corner_with_tool(self, robot_name, object_pose):
    corner_pose = copy.deepcopy(object_pose)

    corner_pose.pose.position.x = np.sign(corner_pose.pose.position.x) * 0.121
    corner_pose.pose.position.y = 0.178 if np.sign(corner_pose.pose.position.y) == 1 else -0.184 

    return self.move_towards_center_with_tool(robot_name, corner_pose)

  def move_towards_tray_center_from_corner(self, robot_name, object_pose, options={}):
    approach_height = options.get('approach_height', 0.05)
    with_tool       = options.get('with_tool', False)
    go_back_halfway = options.get('go_back_halfway', False)

    if with_tool:
      return self.move_towards_tray_center_from_corner_with_tool(robot_name, object_pose)

    rospy.loginfo("Going to corner to push object with gripper!")
    robot = self.active_robots[robot_name]

    robot.gripper.close(wait=True)

    corner_pose = copy.deepcopy(object_pose)
    corner_pose.pose.position.x = np.sign(corner_pose.pose.position.x) * 0.105
    corner_pose.pose.position.y = np.sign(corner_pose.pose.position.y) * 0.165
    corner_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/6))

    return self.move_towards_tray_center_with_push(robot_name, corner_pose, approach_height, go_back_halfway=go_back_halfway)

  def move_towards_tray_center_with_push(self, robot_name, start_pose, approach_height, direction=None, go_back_halfway=True):
    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=True)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=True)

    robot = self.active_robots[robot_name]

    approach_pose = copy.deepcopy(start_pose)
    approach_pose.pose.position.z = 0.06

    if not robot.go_to_pose_goal(approach_pose):
      rospy.logerr("Fail to approach 1")
      return False

    approach_pose.pose.position.z = approach_height
    if not robot.go_to_pose_goal(approach_pose, speed=0.05):
      rospy.logerr("Fail to approach 2")
      return False

    robot.linear_push(force=8, direction="-Z", max_translation=0.06)

    if not self.move_towards_tray_center(robot_name, distance=0.1, one_direction=direction, go_back_halfway=go_back_halfway):
      rospy.logerr("Fail to move towards center")

    if not robot.move_lin_rel(relative_translation=[0, 0, approach_height]):
      rospy.logerr("Fail to move up")

    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=False)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=False)
    return True

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
  
  def center_with_gripper(self, robot_name, opening_width, gripper_force=40, required_width_when_closed=0.0, clockwise=False, move_back_to_initial_position=True):
    """
    Centers cylindrical object at the current location, by closing/opening the gripper and rotating the robot's last joint.

    If required_width_when_closed is set, the function returns False when the gripper closes and detects a smaller width. This
    assumes that the object is not 
    """
    robot = self.active_robots[robot_name]
    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.1)
    if required_width_when_closed:
      if not self.simple_gripper_check(robot_name, min_opening_width=required_width_when_closed):
        return False
    robot.gripper.send_command(command=opening_width, force=gripper_force, velocity = 0.001)

    # rotate gripper 90deg
    initial_pose = robot.get_current_pose_stamped()
    offset = -tau/4.0 if clockwise else tau/4.0
    success = robot.move_lin_rel(relative_rotation=[offset, 0, 0], speed=1.0, relative_to_tcp=True)
    if not success:
      rospy.logerr("Fail to rotate 90deg %s" % success)
      return False
    
    # close-open
    robot.gripper.send_command(command="close", force = 1.0, velocity = 0.003)
    if required_width_when_closed:
      if not self.simple_gripper_check(robot_name, min_opening_width=required_width_when_closed):
        robot.gripper.send_command(command=opening_width, force=gripper_force, velocity = 0.13)
        if move_back_to_initial_position:
          robot.go_to_pose_goal(initial_pose, speed=1.0, move_lin=True)
        return False
    robot.gripper.send_command(command=opening_width, force=gripper_force, velocity = 0.001)

    # rotate gripper -90deg
    if move_back_to_initial_position:
      success = robot.go_to_pose_goal(initial_pose, speed=1.0, move_lin=True)
    return success

  def centering_pick(self, robot_name, object_pose, speed_fast=0.5, speed_slow=0.2, object_width=0.08, approach_height=0.1, 
          item_id_to_attach = "", lift_up_after_pick=False, gripper_force=40.0, approach_move_lin=True):
    """
    This function picks an object with the robot directly from above, but centers the object with the gripper first.
    Should be used only for cylindrical objects.
    
    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    """
    robot = self.active_robots[robot_name]

    pick_pose = copy.deepcopy(object_pose)

    (dx, dy) = self.distances_from_tray_border(object_pose)
    rospy.loginfo("Border distances were %0.3f, %0.3f" % (dx, dy))

    border_dist = object_width
    if abs(dx) <= border_dist or abs(dy) <= border_dist: # if too close to a border, pull object towards middle  
      pick_pose = self.pull_object_towards_middle(robot_name, pick_pose, move_distance=0.05, grasp_width=object_width)
      pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
      robot.gripper.send_command(command=object_width+0.03)
      if not pick_pose:
        rospy.logerr("Fail to pull object towards middle")
        return False
    else: # otherwise just approach the object
      approach_pose = copy.deepcopy(pick_pose)
      approach_pose.pose.position.z += approach_height

      success = robot.go_to_pose_goal(approach_pose, speed=speed_fast, acceleration=speed_fast/2.0, move_lin=approach_move_lin)
      if not success:
        rospy.logerr("Fail to complete approach pose")
        return False
    
      rospy.loginfo("Moving down to object")
      robot.gripper.send_command(command=object_width+0.03)
    
      success = robot.go_to_pose_goal(pick_pose, speed=speed_slow, acceleration=speed_slow/2.0, move_lin=True)
      if not success:
        rospy.logerr("Fail to complete pick pose")
        return False

    # Center object
    self.center_with_gripper(robot_name, object_width+0.03)

    # Grasp object
    robot.gripper.send_command(command="close", force = gripper_force)

    if item_id_to_attach:
      robot.gripper.attach_object(object_to_attach=item_id_to_attach)

    if lift_up_after_pick:
      rospy.sleep(0.5)
      rospy.loginfo("Going back up")

      rospy.loginfo("Going to height " + str(approach_height))
      robot.move_lin_rel(relative_translation=[0, 0, approach_height], speed=speed_fast, acceleration=speed_fast/2.0)
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

    res = self.vision.call_shaft_hole_detection()
    print("=== shaft notch detection returned:")
    print(res)

  def check_screw_hole_visible_on_shaft_in_v_groove(self):
    """
    Looks at the end of the shaft and returns True if 
    """
    look_at_shaft_end_pose = conversions.to_pose_stamped("vgroove_aid_link", [ -0.013, 0.124, 0.130, radians(130.0), 0, 0])
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    
    self.b_bot.go_to_pose_goal(look_at_shaft_end_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=0.15)
    self.confirm_to_proceed("Looking at shaft tip")

    res = self.vision.call_shaft_hole_detection()
    print("=== shaft screw_hole detection returned:")
    print(res)
    return res

  def turn_shaft_until_groove_found(self):
    if not self.use_real_robot:
      res = o2ac_msgs.msg.shaftNotchDetectionActionResult()
      res.shaft_notch_detected_at_top = True
      return res

    # look_at_shaft_pose = [2.177835941, -1.700065275, 2.536958996, -2.40987076, -1.529889408, 0.59719228]
    look_at_shaft_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.008, 0, 0, tau/2., 0, 0])
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    self.b_bot.go_to_pose_goal(look_at_shaft_pose)

    times_turned = 0
    use_ros = True

    if use_ros:
      self.b_bot.go_to_pose_goal(look_at_shaft_pose, speed=0.1)

      for _ in range(6):
        res = self.vision.call_shaft_hole_detection()
        if not res:
          return False
        if res.shaft_notch_detected_at_top or res.shaft_notch_detected_at_bottom:
          return res
        rospy.loginfo("Turn shaft once")
        self.b_bot.move_lin_rel(relative_rotation=[0,0,-tau/12.], speed=0.1, relative_to_tcp=True)
        self.b_bot.gripper.close(velocity=0.03)
        self.b_bot.go_to_pose_goal(look_at_shaft_pose, speed=0.1)
        self.b_bot.gripper.open(opening_width=0.03, velocity=0.03)
    else:
      success = self.b_bot.load_program(program_name="wrs2020/shaft_turning.urp", recursion_depth=3)  
      if not success:
        return False
      while times_turned < 6:
        rospy.loginfo("Turn shaft once")
        self.b_bot.execute_loaded_program()
        wait_for_UR_program("/b_bot", rospy.Duration.from_sec(10))
        times_turned += 1
        res = self.vision.call_shaft_hole_detection()
        if res.shaft_notch_detected_at_top or res.shaft_notch_detected_at_bottom:
          return res
    return False

  def look_at_motor(self):
    b_bot_joint_angles = [1.56942403, -2.099094530, 1.399054352, -0.850922183, -1.570098225, 0.000927209854]
    
    # b_bot_outside_camera_optical_frame in vgroove_aid_link: xyz: -0.011832; 0.13308; 0.085104 q: 0.83999; 0.0043246; 0.0024908; 0.54257
    camera_look_pose = geometry_msgs.msg.PoseStamped()
    camera_look_pose.header.frame_id = "vgroove_aid_link"
    camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(0.84, 0.0043246, 0.0024908, 0.54257))
    camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.0118, 0.133, 0.0851)
    # camera_look_pose.pose.position.z += 0.2
    # self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    # camera_look_pose.pose.position.z -= 0.2
    self.b_bot.move_joints(b_bot_joint_angles)
    self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.1, acceleration=.04)
    return self.get_motor_angle()

  def simple_insertion_check(self, robot_name, opening_width, min_opening_width=0.01, velocity=0.03):
    """ Simple check: Open close the gripper slowly, check the gripper opening width
        end state: close!
    """
    self.active_robots[robot_name].gripper.open(wait=True, opening_width=opening_width, velocity=velocity)
    self.active_robots[robot_name].gripper.close(wait=True, velocity=velocity)
    return self.simple_gripper_check(robot_name, min_opening_width=min_opening_width)

  def grab_and_drop(self, robot_name, object_pose, options=dict()):
    grasp_width_ = options.get('grasp_width', 0.08)
    grasp_width = grasp_width_ + 0.02 # extra opening of gripper to try to catch multiple objects, not too much to avoid grasping too many?
    options_ = {'grasp_z_height': 0.002, 'grasp_width': grasp_width, 'check_for_close_items': False}
    if options.get('use_grasp_pose_directly_in_simple_pick', False):
      grasp_pose = object_pose
    else:
      grasp_pose = self.simple_grasp_generation(object_pose, options_)[0]
    
    robot = self.active_robots[robot_name]
    robot.gripper.open(opening_width=grasp_width)
    success = self.simple_pick(robot_name, grasp_pose, approach_height=0.05, lift_up_after_pick=True, axis="z", approach_with_move_lin=False)
    robot.gripper.open()
    if not success:      
      rospy.logerr("Fail to simple pick (grab_and_drop)")
    return success

  def pick_from_centering_area_and_drop_in_tray(self, robot_name):
    """
    Called if a rearranging procedure has failed. Picks up whatever is in the space and drops it in the tray.
    """
    robot = self.active_robots[robot_name]

    prefix = "right" if robot_name == "b_bot" else "left"
    approach_pose = conversions.to_pose_stamped(prefix + "_centering_link", [-0.10,0,0,0,0,0])
    at_object_pose = conversions.to_pose_stamped(prefix + "_centering_link", [-0.005,0,0,0,0,0])

    robot.gripper.open() 
    
    if not robot.go_to_pose_goal(approach_pose):
      rospy.logerr("fail to go to approach_pose (fallback_b_bot_outside_tray_centering)")
      return False
    if not robot.go_to_pose_goal(at_object_pose, speed=0.3):
      rospy.logerr("fail to go to at_object_pose (fallback_b_bot_outside_tray_centering)")
      return False

    robot.gripper.close()

    if not robot.go_to_pose_goal(approach_pose):
      rospy.logerr("fail to go to approach_pose (fallback_b_bot_outside_tray_centering)")
      return False

    return self.drop_in_tray(robot_name)
  
  def drop_in_tray(self, robot_name):
    robot = self.active_robots[robot_name]

    rotation_offset = -1 if robot_name == "b_bot" else 1
    above_tray       = conversions.to_pose_stamped("tray_center", [0, 0, 0.15, 0, tau/4, rotation_offset*tau/4])
    tray_center_pose = conversions.to_pose_stamped("tray_center", [0, 0, 0.06, 0, tau/4, rotation_offset*tau/4])
    
    if not robot.go_to_pose_goal(above_tray):
      rospy.logerr("fail to go to above_tray (drop_in_tray)")
      return False

    if not robot.go_to_pose_goal(tray_center_pose):
      rospy.logerr("fail to go to tray_center_pose (drop_in_tray)")
      return False

    robot.gripper.open() 
    return True

  @check_for_real_robot
  def simple_gripper_check(self, robot_name, min_opening_width=0.001):
    self.active_robots[robot_name].gripper.close() # confirm that there is something grasped
    if robot_name == "a_bot":
      min_opening_width += 0.005 # a_bot gripper is not very precise...
    success = self.active_robots[robot_name].gripper.opening_width > min_opening_width
    if not success:
      rospy.logerr("Fail to grasp. opening_width: %s" % self.active_robots[robot_name].gripper.opening_width)
    return success

  ######## Bearing

  def pick_and_fasten_screw(self, robot_name, screw_pose, screw_size, approach_distance=0.07, intermediate_pose=None, speed=0.7):
    """Returns bool, screw success"""
    # Pick screw
    self.active_robots[robot_name].go_to_named_pose("screw_ready", speed=speed)
    self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready", speed=speed)
    pick_success = self.pick_screw_from_feeder_python(robot_name, screw_size=screw_size)

    if not pick_success:
      rospy.logerr("Could not pick screw. Why?? Breaking out.")
      self.unequip_tool('b_bot', 'screw_tool_m4')
      return False
    
    if not self.active_robots[robot_name].go_to_named_pose("screw_ready", speed=speed):
      return False
    if not self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready", speed=speed):
      return False

    if intermediate_pose:
      if not self.active_robots[robot_name].go_to_pose_goal(intermediate_pose, 
                                                     end_effector_link = robot_name+"_screw_tool_m%s_tip_link"%screw_size,
                                                     speed=speed):
        rospy.logerr("Fail to go to intermediate pose")
        return False
    
    screw_pose_approach = copy.deepcopy(screw_pose)
    screw_pose_approach.pose.position.x -= approach_distance
    
    if not self.active_robots[robot_name].go_to_pose_goal(screw_pose_approach, 
                                                          end_effector_link = robot_name+"_screw_tool_m%s_tip_link"%screw_size,
                                                          move_lin=False):
      rospy.logerr("Fail to go to approach screw hole pose")
      return False
    
    return self.screw(robot_name, screw_pose, screw_size=screw_size, screw_height=0.02, skip_final_loosen_and_retighten=False, spiral_radius=0.003)

  @check_for_real_robot
  def align_bearing_holes(self, max_adjustments=10, task=""):
    """
    Align the bearing holes.
    """
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot", on=False)
    adjustment_motions = 0
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
    camera_look_pose.pose.orientation = geometry_msgs.msg.Quaternion(*(-0.5, 0.5, -0.5, 0.5))
    camera_look_pose.pose.position = geometry_msgs.msg.Point(-0.155, 0.005, 0.0)

    def rotate_bearing_by_angle(angle):
      self.b_bot.gripper.open()
      start_pose = copy.deepcopy(grasp_pose)
      start_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(-angle/2.0, 0, 0))
      end_pose = copy.deepcopy(grasp_pose)
      end_pose.pose.position.z += 0.0005  # Avoid pulling the bearing out little by little
      end_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(angle/2.0, 0, 0))
      self.b_bot.go_to_pose_goal(start_pose, speed=.2, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.b_bot.gripper.close()
      self.b_bot.go_to_pose_goal(end_pose, speed=.2, end_effector_link = "b_bot_bearing_rotate_helper_link")
      self.b_bot.gripper.open()

    while adjustment_motions < max_adjustments:
      # Look at tb bearing
      if self.b_bot.gripper.opening_width < 0.06:
        self.b_bot.gripper.open()
      
      self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.3, acceleration=.15)
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
    if task == "taskboard":
      bearing_target_link = "taskboard_bearing_target_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      bearing_target_link = "assembled_part_07_inserted"

    self.ab_bot.go_to_named_pose("home")

    if not self.pick_bearing():
      return False

    if not self.orient_bearing(task):
      return False

    # Insert bearing
    if not self.insert_bearing(bearing_target_link):
      rospy.logerr("insert_bearing returned False. Breaking out")
      return False

    return True

  def pick_bearing(self, robot_name="b_bot"):
    options = {'center_on_corner': True, 'check_for_close_items': False, 'grasp_width': 0.08, 'go_back_halfway': False}
    goal = self.look_and_get_grasp_point("bearing", robot_name, options=options)
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find bearing in tray. Skipping procedure.")
      return False
    self.vision.activate_camera(robot_name + "_inside_camera")
    goal.pose.position.z = 0.0
    # self.spawn_object("bearing", goal, goal.header.frame_id)
    goal.pose.position.x -= 0.01 # MAGIC NUMBER
    goal.pose.position.z = 0.0115

    # if not self.pick_from_two_poses_topdown(robot_name, "bearing", goal, grasp_width=0.07):
    if not self.simple_pick(robot_name, goal, gripper_force=50.0, grasp_width=.07, axis="z", grasp_height=0.002):
      rospy.logerr("Fail to pick bearing from tray")
      return False

    self.active_robots[robot_name].gripper.detach_object("bearing")
    # self.despawn_object("bearing")
    self.active_robots[robot_name].gripper.last_attached_object = None # Forget about this object
    return True

  def orient_bearing(self, task, robot_name="b_bot"):
    if task == "taskboard":
      bearing_target_link = "taskboard_bearing_target_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      bearing_target_link = "assembled_part_07_inserted"

    self.active_robots[robot_name].gripper.close() # catch false grasps
    if not self.simple_gripper_check(robot_name, min_opening_width=0.01):
      rospy.logerr("Fail to grasp bearing")
      return False
    elif self.active_robots[robot_name].gripper.opening_width < 0.045:
      rospy.loginfo("bearing found to be upwards")
      if not self.playback_sequence("bearing_orient_" + robot_name):
        rospy.logerr("Could not complete orient sequence")
        self.pick_from_centering_area_and_drop_in_tray(robot_name)
        return False
    else:
      rospy.loginfo("bearing found to be upside down")
      if not self.playback_sequence("bearing_orient_down_" + robot_name):
        rospy.logerr("Could not complete orient down sequence")
        self.pick_from_centering_area_and_drop_in_tray(robot_name)
        return False
      #'down' means the small area contacts with tray.
    if self.active_robots[robot_name].gripper.opening_width < 0.01 and self.use_real_robot:
      rospy.logerr("Bearing not found in gripper. Must have been lost. Aborting.")
      #TODO(felixvd): Look at the regrasping/aligning area next to the tray
      return False

    prefix = "right" if robot_name == "b_bot" else "left"
    at_tray_border_pose = conversions.to_pose_stamped(prefix + "_centering_link", [-0.15, 0, 0.10, radians(-100), 0, 0])

    rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/4, 0, radians(35.0)]
    approach_pose       = conversions.to_pose_stamped(bearing_target_link, [-0.050, -0.001, 0.005] + rotation)
    if task == "taskboard":
      preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.017,  0.000, 0.002 ]+ rotation)
    elif task == "assembly":
      preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.017, -0.000, 0.006] + rotation)

    trajectory = [(at_tray_border_pose, 0.01, 0.4), (approach_pose, 0.02, 0.4), (preinsertion_pose, 0, 0.2)]
    if not self.active_robots[robot_name].move_lin_trajectory(trajectory=trajectory, timeout=15):
      rospy.logerr("Could not go to preinsertion")
      self.pick_from_centering_area_and_drop_in_tray(robot_name)
      return False

    return True

  def insert_bearing(self, target_link, try_recenter=True, try_reinsertion=True, robot_name="b_bot"):
    """ Only inserts the bearing, does not align the holes.
    """
    robot = self.active_robots[robot_name]
    insertion_direction = "-X" if robot_name == "b_bot" else "+X"
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.004, 0.000, 0.002, 0, 0, 0, 1.])
    selection_matrix = [0., 0.3, 0.3, .8, .8, .8]
    result = robot.do_insertion(target_pose_target_frame, insertion_direction=insertion_direction, force=10.0, timeout=20.0, 
                                radius=0.002, relaxed_target_by=0.003, selection_matrix=selection_matrix)
    
    if result != TERMINATION_CRITERIA:
      # move back
      robot.move_lin_rel(relative_translation = [0.005,0,0], acceleration = 0.015, speed=.03)

      # TODO(cambel): check that the bearing is not slightly inserted
      #               otherwise we may trigger a safety lock
      if try_reinsertion:
        # Try to insert again
        rospy.logwarn("** Insertion Incomplete, trying again **")
        self.insert_bearing(target_link, try_recenter=False, try_reinsertion=False, robot_name=robot_name)
      elif try_recenter:
        # Try to recenter the bearing
        rospy.logwarn("** Insertion Incomplete, trying from centering again **")
        self.playback_sequence("bearing_orient_down_" + robot_name)
        self.insert_bearing(target_link, try_recenter=False, try_reinsertion=True, robot_name=robot_name)
      else:
        robot.move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
        self.drop_in_tray(robot_name)

    robot.gripper.open(wait=True)
    robot.move_lin_rel(relative_translation = [0.01,0,0], acceleration = 0.015, speed=.03)
    robot.gripper.close(velocity=0.05, wait=True)

    result = robot.do_insertion(target_pose_target_frame, insertion_direction=insertion_direction, force=10.0, timeout=30.0, 
                                radius=0.0, wiggle_direction="X", wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=1.0,
                                relaxed_target_by=0.003, selection_matrix=selection_matrix)
    
    success = result in (TERMINATION_CRITERIA, DONE)

    # Go back regardless of success
    # TODO(cambel): implement fallback in case of error
    robot.gripper.open(wait=True)
    success &= robot.move_lin_rel(relative_translation = [0.025,0,0], acceleration = 0.015, speed=.03)
    return success

  def fasten_bearing(self, task="", only_retighten=False, robot_name="b_bot"):
    if not task in ["taskboard", "assembly"]:
      rospy.logerr("Invalid task specification: " + task)
      return False
    
    if not self.equip_tool(robot_name, 'screw_tool_m4'):
      rospy.logerr("Fail to equip tool abort!")
      return False

    speed = 0.7
    self.vision.activate_camera(robot_name + "_outside_camera")
    
    screw_poses = []
    for i in [1,2,3,4]:
      screw_pose = geometry_msgs.msg.PoseStamped()
      if task == "taskboard":
        screw_pose.header.frame_id = "taskboard_bearing_target_screw_" + str(i) + "_link"
      elif task == "assembly":
        screw_pose.header.frame_id = "assembled_part_07_screw_hole_" + str(i)
      else:
        rospy.logerr("Invalid task specification: " + task)
        return False
      if task == "taskboard":
        screw_pose.pose.position.z += -.001  # MAGIC NUMBER
      elif task == "assembly":
        screw_pose.pose.position.z += .0025  # MAGIC NUMBER
      screw_pose.pose.position.x += .006  # This needs to be quite far forward, because the thread is at the plate level (behind the frame)
      offset = -1 if robot_name == "a_bot" else 1
      screw_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(offset*tau/12, 0, 0) )
      screw_poses.append(screw_pose)

    # Initialize screw status
    screw_status = dict()
    for n in [1,2,3,4]:
      if only_retighten:
        screw_status[n] = "maybe_stuck_in_hole"
      else:
        screw_status[n] = "empty"
    
    self.confirm_to_proceed("intermediate pose")
    robot = self.active_robots[robot_name]
    robot.go_to_named_pose("screw_ready", speed=speed)
    
    # Go to bearing and fasten all the screws
    all_screws_done = False
    tries = 0
    first_screw = True
    while not all_screws_done and tries < 8:
      for n in [1,3,2,4]:  # Cross pattern
        assert not rospy.is_shutdown(), "lost connection to ros?"
        if screw_status[n] == "done":
          continue

        if not first_screw and screw_status[n] == "empty": # go back
          robot.go_to_named_pose("horizontal_screw_ready", speed=speed)
          robot.go_to_named_pose("screw_ready", speed=speed)
        
        if first_screw and screw_status[n] == "maybe_stuck_in_hole": # get ready for screwing
          robot.go_to_named_pose("screw_ready", speed=speed)
          robot.go_to_named_pose("horizontal_screw_ready", speed=speed)

        if first_screw:
          first_screw = False

        if screw_status[n] == "empty":
          success = self.pick_and_fasten_screw(robot_name, screw_poses[n-1], screw_size=4)
          screw_status[n] = "done" if success else "maybe_stuck_in_hole"
        elif screw_status[n] == "maybe_stuck_in_hole":
          screw_pose_approach = copy.deepcopy(screw_poses[n-1])
          screw_pose_approach.pose.position.x -= 0.07
          robot.go_to_pose_goal(screw_pose_approach, end_effector_link = robot_name + "_screw_tool_m4_tip_link", move_lin=False)
          success = self.screw(robot_name, screw_poses[n-1], screw_size=4, screw_height=0.02, skip_final_loosen_and_retighten=False, spiral_radius=0.003)
          screw_status[n] = "done" if success else "maybe_stuck_in_hole"
        
        if robot.is_protective_stopped():
          rospy.logerr("Critical error: %s  protective stopped. Something is wrong. Wait to unlock, move back, abort." % robot_name)
          robot.unlock_protective_stop()
          robot.move_lin_rel(relative_translation=[-0.05, 0, 0], relative_to_tcp=True, end_effector_link = robot_name + "_screw_tool_m4_tip_link")
          break
        
        rospy.loginfo("Screw " + str(n) + " detected as " + screw_status[n])

      all_screws_done = all(value == "done" for value in screw_status.values())
      tries += 1

    robot.go_to_named_pose("horizontal_screw_ready", speed=speed)
    robot.go_to_named_pose("screw_ready", speed=speed)
    
    if not self.unequip_tool(robot_name, 'screw_tool_m4'):
      rospy.logerr("Fail to unequip tool abort!")

    return all_screws_done

  ########  Motor pulley

  def pick_and_insert_motor_pulley(self, task):
    if task == "taskboard":
      target_link = "taskboard_small_shaft"
    elif task == "assembly":
      rospy.logerr("look this up")
      target_link = "assembled_part_07_inserted"
    
    if not self.pick_motor_pulley():
      return False

    if not self.playback_sequence(routine_filename="motor_pulley_orient"):
      rospy.logerr("Fail to complete the playback sequence motor pulley orient")
      self.pick_from_centering_area_and_drop_in_tray("b_bot")
      return False

    return self.insert_motor_pulley(target_link)

  def pick_motor_pulley(self, attempt=2):
    options = {'grasp_width': 0.06, 'center_on_corner': True, 'approach_height': 0.02, 'grab_and_drop': True}
    goal = self.look_and_get_grasp_point("motor_pulley", options=options)
    
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find motor_pulley in tray. Skipping procedure.")
      return False
    goal.pose.position.x -= 0.01 # MAGIC NUMBER
    goal.pose.position.z = 0.0
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot", False)
    
    if not self.simple_pick("b_bot", goal, gripper_force=50.0, grasp_width=.06, axis="z", grasp_height=0.002):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check("b_bot"):
      rospy.logerr("Gripper did not grasp the pulley --> Stop")
      return self.pick_motor_pulley(attempt=attempt-1)

    return True

  def insert_motor_pulley(self, target_link, attempts=1):
    pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.006, -0.000, -0.003] + np.deg2rad([180, 35, 0]).tolist()) # Manually defined target pose in object frame
    self.b_bot.go_to_pose_goal(pre_insertion_pose, speed=0.1)
    
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [0.015, -0.000, -0.009, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame

    selection_matrix = [0., 0.3, 0.3, 0.95, 1.0, 1.0]

    result = self.b_bot.do_insertion(target_pose_target_frame, radius=0.0005, 
                                                      insertion_direction="-X", force=6.0, timeout=15.0, 
                                                      wiggle_direction="X", wiggle_angle=np.deg2rad(5.0), wiggle_revolutions=1.,
                                                      relaxed_target_by=0.005, selection_matrix=selection_matrix)
    success = (result == TERMINATION_CRITERIA)

    if not success:
      if attempts > 0: # try again the pulley is still there   
        rospy.logwarn("** Insertion Incomplete, trying again **")
        return self.insert_motor_pulley(target_link, attempts=attempts-1)
      else:
        # TODO(cambel): return to tray to drop pulley
        self.b_bot.move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
        self.drop_in_tray("b_bot")
        rospy.logerr("** Insertion Failed!! **")
        return False

    self.b_bot.gripper.open(opening_width=0.04, wait=True)
    success &= self.b_bot.move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
    return success

  ########  Idler pulley
    
  def pull_object_towards_middle(self, robot_name, object_pose, move_distance=0.04, grasp_width=0.08):
    """ Moves to and returns the new pose of the object.
    """
    rotation_offset = -1 if robot_name == "b_bot" else 1
    grasp_poses = self.simple_grasp_generation(object_pose, grasp_width=grasp_width, 
                                grasp_z_height=object_pose.pose.position.z, rotation_offset=rotation_offset, check_for_close_items=False)
    self.simple_pick(robot_name, grasp_poses[0], lift_up_after_pick=False, axis="z")
    new_pose = self.move_towards_tray_center(robot_name, move_distance)
    return new_pose

  def pick_and_insert_idler_pulley(self, task="", simultaneous=False):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      idler_puller_target_link = "taskboard_long_hole_top_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      idler_puller_target_link = "assembly_long_hole_middle_link"

    self.ab_bot.go_to_named_pose("home")
    
    options = {'check_for_close_items': False, 'center_on_corner': True, 'center_on_close_border': True, 'min_dist_to_border': 0.04, 'allow_pick_near_border': False}
    object_pose = self.look_and_get_grasp_point("taskboard_idler_pulley_small", robot_name="a_bot", options=options)

    if not isinstance(object_pose, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find idler pulley in tray. Skipping procedure.")
      return False
    
    self.vision.activate_camera("a_bot_inside_camera")
    object_pose.pose.position.x -= 0.01 # MAGIC NUMBER
    object_pose.pose.position.z = 0.018

    rospy.loginfo("Picking idler pulley at: ")
    self.b_bot.go_to_named_pose("home")

    at_object_pose = copy.deepcopy(object_pose)

    self.a_bot.gripper.open(wait=False, opening_width=0.07)
    at_object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))

    approach_pose = copy.deepcopy(at_object_pose)
    approach_pose.pose.position.z += .03

    if not self.a_bot.go_to_pose_goal(approach_pose, speed=1.0):
      rospy.logerr("Fail to complete approach pose")
      return False  

    rospy.loginfo("Moving down to object")
    

    self.a_success = False
    self.b_success = False
    def a_bot_task():
      self.a_bot.gripper.open(opening_width=0.08)
      if not self.a_bot.go_to_pose_goal(at_object_pose):
        rospy.logerr("Fail to complete pick pose")
        return False

      self.a_bot.gripper.open(wait=False, opening_width=0.07)
      if not self.center_with_gripper("a_bot", opening_width=.08):
        rospy.logerr("Fail to complete center_with_gripper")
        return False
      if not self.grasp_idler_pulley():
        rospy.logerr("Fail to complete grasp_idler_pulley")
        return False
      if not self.insert_idler_pulley(idler_puller_target_link):
        rospy.logerr("Fail to complete insert_idler_pulley")
        self.drop_in_tray("a_bot")
        return False
      self.a_success = True
      return True
    def b_bot_task():
      self.vision.activate_camera("b_bot_outside_camera")
      if not self.equip_tool("b_bot", "padless_tool_m4"):
        rospy.logerr("Fail to equip padless_tool_m4")
        return False
      if not self.b_bot.go_to_named_pose("screw_ready"):
        return False
      self.b_success = True
      return True

    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
      if not (self.a_success and self.b_success):
        rospy.logerr("Fail to do simultaneous tasks")
        return False
    else: # sequential 
      if not a_bot_task():
        return False
      if not b_bot_task():
        return False

    if not self.prepare_screw_tool_idler_pulley(idler_puller_target_link):
      rospy.logerr("Fail to do prepare_screw_tool_idler_pulley")
      return False
    
    self.a_bot.gripper.open(opening_width=0.05)
    self.a_bot.move_lin_rel(relative_translation=[-0.15, 0, 0.0], relative_to_tcp=True, speed=1.0)
    if not self.playback_sequence("idler_pulley_equip_nut_tool"):
      rospy.logerr("Fail to complete equip_nut_tool")
      return False

    self.vision.activate_camera("a_bot_inside_camera")
    success = self.fasten_idler_pulley_with_nut_tool(idler_puller_target_link)
    if not success:
      rospy.logerr("Fail to complete fasten_idler_pulley_with_nut_tool")

    def a_bot_task2():
      return self.playback_sequence("idler_pulley_unequip_nut_tool")

    def b_bot_task2():
      if not self.playback_sequence("idler_pulley_return_screw_tool"):
        rospy.logerr("Fail to complete idler_pulley_return_screw_tool")
        return False
      return self.unequip_tool("b_bot", "padless_tool_m4")

    if simultaneous:
      if not self.do_tasks_simultaneous(a_bot_task2, b_bot_task2, timeout=60):
        return False
    else:
      if not a_bot_task2():
        return False
      if not b_bot_task2():
        return False

    return True
    
  def center_idler_pulley(self):
    # Center first time
    self.a_bot.gripper.close(force=40.0, velocity=0.013)
    self.a_bot.gripper.open(velocity=0.013)

  def grasp_idler_pulley(self, attempt=1):
    # Incline 45 deg
    success = self.a_bot.move_lin_rel(relative_translation=[0, 0.01, 0.002], relative_rotation=[tau/8.0, 0, 0])
    if not success:
      rospy.logerr("Fail to incline a_bot 45 deg %s" % success)
      self.a_bot.gripper.close()
      if not self.move_towards_tray_center("a_bot", 0.05, go_back_halfway=False):
        return False
      self.a_bot.gripper.open()
      return self.grasp_idler_pulley(attempt=attempt-1)
    
    self.a_bot.gripper.close()

    if self.a_bot.gripper.opening_width < 0.01 and self.use_real_robot:
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
    near_tb_pose = conversions.to_pose_stamped(target_link,  [(-0.016), 0.009, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose = conversions.to_pose_stamped(target_link,    [(-0.008), 0.009, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose_world = self.listener.transformPose("world", in_tb_pose)
    success = self.a_bot.move_lin(approach_pose, speed=0.7)
    if not success:
      return False
    
    rospy.loginfo("Approach ridge (a_bot)")
    success = self.a_bot.move_lin(near_tb_pose, speed=0.7)
    if not success:
      return False
        
    rospy.loginfo("Moving into ridge (a_bot)")
    insertion_offsets = [0.0]
    d2 = 0.0005
    for i in range(6):
      insertion_offsets.append(d2*(i+1))
      insertion_offsets.append(-d2*(i+1))

    for offset in insertion_offsets:
      selection_matrix = [0.,1.,1.,1,1,1]
      success = self.a_bot.linear_push(5, "+X", max_translation=0.015, timeout=10.0, slow=True, selection_matrix=selection_matrix)

      if not self.use_real_robot:
        return True

      if success and self.a_bot.robot_group.get_current_pose().pose.position.x <= in_tb_pose_world.pose.position.x:
        return True
      else:
        # Go back, try again
        self.a_bot.move_lin_rel(relative_translation=[0.01, 0, 0], speed=0.1, relative_to_robot_base=True)
        near_tb_pose_with_offset = copy.deepcopy(near_tb_pose)
        near_tb_pose_with_offset.pose.position.y += offset
        self.a_bot.move_lin(near_tb_pose_with_offset, speed=0.05)
    return False

  def prepare_screw_tool_idler_pulley(self, target_link):
    """ target_link is e.g. long_hole_top_link
    """
    self.equip_tool("b_bot", "padless_tool_m4")
    if not self.use_real_robot:
      self.allow_collisions_with_robot_hand("padless_tool_m4", "a_bot")
    self.b_bot.go_to_named_pose("screw_ready")
    if not self.playback_sequence("idler_pulley_ready_screw_tool"):
      rospy.logerr("Fail to complete idler_pulley_ready_screw_tool")
      return False

    rospy.loginfo("Going to near tb (b_bot)") # Push with tool
    target_rotation = np.deg2rad([30.0, 0.0, 0.0]).tolist()
    xyz_light_push = [-0.01, -0.001, 0.001]  # MAGIC NUMBERS
    near_tb_pose = conversions.to_pose_stamped(target_link, xyz_light_push + target_rotation)
    success = self.b_bot.move_lin(near_tb_pose, speed=0.05, acceleration=0.05, end_effector_link="b_bot_screw_tool_m4_tip_link")
    if not success:
      return False

    self.vision.activate_camera("a_bot_inside_camera")
    self.tools.set_motor("padless_tool_m4", "tighten", duration=12.0)
    self.insert_screw_tool_tip_into_idler_pulley_head()
    
    ## Incline the tool slightly 
    self.planning_scene_interface.allow_collisions("padless_tool_m4", "taskboard_plate")
    xyz_hard_push = [0.001, -0.001, 0.001]  # MAGIC NUMBERS (target without inclination)
    inclination_angle_deg = 3.0
    inclined_orientation_hard_push = np.deg2rad([30.0, inclination_angle_deg, 0.0]).tolist()
    s = sin(np.deg2rad(inclination_angle_deg)) * 0.008  # 8 mm is roughly the distance from the taskboard surface to the 
                                                        # head of the screw, so adding this offset should result in a rotation
                                                        # around the screw head.
    xyz_hard_push[2] -= s
    push_pose = conversions.to_pose_stamped(target_link, xyz_hard_push + inclined_orientation_hard_push)
    return self.b_bot.move_lin(push_pose, speed=0.02, acceleration=0.02, end_effector_link="b_bot_screw_tool_m4_tip_link")

  def insert_screw_tool_tip_into_idler_pulley_head(self):

    target_force = get_target_force('-X', 0.0)
    selection_matrix = [0., 0.9, 0.9, 1, 1, 1]

    self.b_bot.execute_spiral_trajectory("YZ", max_radius=0.003, radius_direction="+Y", steps=50,
                                         revolutions=2, target_force=0, selection_matrix=selection_matrix, check_displacement_time=10,
                                         termination_criteria=None, timeout=6.0, end_effector_link="b_bot_screw_tool_m4_tip_link")

    return True

  def fasten_idler_pulley_with_nut_tool(self, target_link):
    approach_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not self.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=1.0):
      return False

    success = False
    idler_pulley_screwing_succeeded = False
    offsets = [0.0, -0.003, -0.006, 0.009, 0.006, 0.003]
    for offset in offsets:
      if idler_pulley_screwing_succeeded:
        success = True
        break
      # Move nut tool forward so nut touches the screw
      d = offset  # 
      approach_pose = conversions.to_pose_stamped(target_link, [0.06, 0.0, d + 0.004, 0.0, 0.0, 0.0])
      if not self.a_bot.move_lin(approach_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2):
        return False
      
      pushed_into_screw = conversions.to_pose_stamped(target_link, [0.011, 0.0, d + 0.004, 0.0, 0.0, 0.0])
      if not self.a_bot.move_lin(pushed_into_screw, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2):
        return False
      
      response = self.tools.set_motor("padless_tool_m4", "tighten", duration=3.0, wait=True, skip_final_loosen_and_retighten=True)
      if not self.use_real_robot:
        idler_pulley_screwing_succeeded = True
      else:
        idler_pulley_screwing_succeeded = response.motor_stalled

    retreat_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not self.a_bot.move_lin(retreat_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=0.2):
      return False
    
    return success
  
  ######## Shaft

  def pick_and_center_shaft(self):
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 3:
      picked = self.pick_shaft(attempt_nr=attempt_nr)
      if not picked:
        attempt_nr += 1
        continue

      picked = self.centering_shaft()  # This also checks for success via grasp_width
      if not picked:
        attempt_nr += 1
        continue
    return picked

  def pick_and_insert_shaft(self, task=""):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      target_link = "taskboard_assy_part_07_inserted"
    elif task == "assembly":
      target_link = "assembly_assy_part_07_inserted"
    
    self.allow_collisions_with_robot_hand("shaft", "b_bot", True)
    
    if not self.pick_and_center_shaft():
      return False

    self.b_bot.move_lin_rel(relative_translation=[0, 0.01, 0.1], speed=.5)

    if not self.align_shaft(target_link, pre_insert_offset=0.065):
      return False

    success = self.insert_shaft(target_link)
    
    self.b_bot.gripper.open(wait=False)
    self.b_bot.gripper.forget_attached_item()
    self.despawn_object("shaft")
    self.b_bot.go_to_named_pose("home")
    return success
  
  def align_shaft(self, target_link, pre_insert_offset=0.09):
    rospy.loginfo("Going to approach pose (b_bot)")
    rotation = np.deg2rad([-22.5, -88.5, -157.5]).tolist()  # Arbitrary

    post_pick_pose = conversions.to_pose_stamped(target_link, [-0.15, 0.0, -0.10] + rotation)
    above_pose     = conversions.to_pose_stamped(target_link, [0.0, 0.002, -0.10] + rotation)
    behind_pose    = conversions.to_pose_stamped(target_link, [0.09, 0.002, -0.05] + rotation)
    pre_insertion_pose = conversions.to_pose_stamped(target_link, [pre_insert_offset, 0.001, 0.001] + rotation)

    trajectory = [[post_pick_pose, 0.05, 0.8], [above_pose, 0.05, 0.5], [behind_pose, 0.01, 0.5], [pre_insertion_pose, 0.0, 0.2]]
    rospy.loginfo("Going to position shaft to pre-insertion (b_bot)")
    if not self.b_bot.move_lin_trajectory(trajectory, speed=0.5, acceleration=0.25):
      rospy.logerr("Fail to position shaft to pre-insertion")
      return False
    return True

  def pick_shaft(self, attempt_nr=0, called_recursively=False):
    options = {'center_on_corner': True, 'approach_height': 0.02, 'grab_and_drop': True, 'center_on_close_border': True, 'with_tool': True}
    goal = self.look_and_get_grasp_point("shaft", options=options)
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      print("goal", type(goal), goal)
      rospy.logerr("Could not find shaft in tray. Skipping procedure.")
      return False
    
    # # Spawn object FIXME(cambel)
    gp = conversions.from_pose_to_list(goal.pose)
    gp[:2] += [0.075/2.0, 0.0] # Magic Numbers for visuals 
    gp[2] = 0.005
    euler_gp = tf_conversions.transformations.euler_from_quaternion(gp[3:])
    shaft_pose = conversions.to_pose_stamped("tray_center", gp[:3].tolist() + [0, 0, -tau/2-euler_gp[0]])
    self.spawn_object("shaft", shaft_pose, shaft_pose.header.frame_id)
    self.planning_scene_interface.allow_collisions("shaft", "")
    
    goal.pose.position.z = 0.001 # Magic Numbers for grasping
    goal.pose.position.x -= 0.01

    self.vision.activate_camera("b_bot_inside_camera")

    if self.too_close_to_border(goal, border_dist=0.02):
      if not called_recursively:
        self.move_towards_center_from_border_with_tool(robot_name="b_bot", object_pose=goal)
        self.pick_shaft(self, attempt_nr=0, called_recursively=True)
      else:
        rospy.logerr("Shaft too close to border. Trying to pick even though correction seems to have failed.")

    picked_ok = self.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.03, approach_height=0.1, 
                              item_id_to_attach="shaft", minimum_grasp_width=0.004,  axis="z", lift_up_after_pick=True,
                              speed_slow=0.1)
    
    
    if not picked_ok:
    # if not self.pick("b_bot", object_name="shaft", grasp_pose=goal):
      rospy.logerr("Failed to pick shaft")
      if attempt_nr < 3:
        rospy.loginfo("Try again")
        print("goal.pose.orientation", goal.pose.orientation)
        goal_rotated = helpers.rotatePoseByRPY(tau/4, 0, 0, goal)
        print("goal_rotated.pose.orientation", goal_rotated.pose.orientation)
        options = {"use_grasp_pose_directly_in_simple_pick":True}
        if attempt_nr == 0:
          self.grab_and_drop("b_bot", goal_rotated, options=options)
        if attempt_nr > 0:
          # TODO: Don't do this randomly. Try each side of the shaft.
          goal.pose.position.x += np.random.uniform() * 0.006 - .003
          goal.pose.position.y += np.random.uniform() * 0.006 - .003
          self.declutter_with_tool("b_bot", goal)
        return self.pick_shaft(attempt_nr=attempt_nr+1)
      return False
    return True

  def insert_shaft(self, target_link, attempts=1, target=0.06):
    """
    Insert shaft with force control using b_bot. The shaft has to be in front of the hole already.
    """

    self.b_bot.linear_push(3, "+X", max_translation=0.08, timeout=30.0)
    after_push_pose = self.b_bot.get_current_pose_stamped()

    current_pose = self.b_bot.robot_group.get_current_pose()
    target_pose_target_frame = self.listener.transformPose(target_link, current_pose)
    target_pose_target_frame.pose.position.x = 0.04 # Magic number

    selection_matrix = [0., 0.2, 0.2, 0.95, 1, 1]
    self.b_bot.move_lin_rel(relative_translation = [-0.003,0,0], acceleration = 0.015, speed=.03) # Release shaft for next push
    result = self.b_bot.do_insertion(target_pose_target_frame, insertion_direction="+X", force=5.0, timeout=15.0, 
                                                        wiggle_direction="X", wiggle_angle=np.deg2rad(10.0), wiggle_revolutions=1.0,
                                                        radius=0.002, relaxed_target_by=0.0, selection_matrix=selection_matrix)
    success = result in (TERMINATION_CRITERIA, DONE)

    current_pose = self.b_bot.robot_group.get_current_pose()
    target_pose_target_frame = self.listener.transformPose(target_link, current_pose)
     
    rotation = np.deg2rad([-22.5, -88.5, -157.5]).tolist()  # Arbitrary
    pre_insertion_pose = conversions.to_pose_stamped(target_link, [0.12, 0.001, 0.02] + rotation)

    if not success or not self.simple_insertion_check("b_bot", 0.02, min_opening_width=0.001):
      # TODO(cambel): implement a fallback
      rospy.logerr("** Insertion Failed!! **")

      cp = self.b_bot.get_current_pose_stamped()
      if cp.pose.position.x > after_push_pose.pose.position.x + 0.005:
        # IF there was a bit of an insertion, drop the shaft and move away
        # TODO(cambel): fall back for the shaft stuck in the bearing
        self.b_bot.gripper.open(wait=True)
        self.b_bot.move_lin(pre_insertion_pose, speed=0.05)
      elif attempts > 1:
        self.b_bot.move_lin_rel([-0.01,0,0])
        self.insert_shaft(target_link, attempts=attempts-1, target=target)
      else:
        self.b_bot.move_lin_rel([-0.05,0,0])
        self.b_bot.move_lin_rel([0,0,0.1])
        self.drop_in_tray("b_bot")
      return False

    # Move back to push (without grasping)
    self.b_bot.gripper.open(wait=True)

    if not self.b_bot.move_lin(pre_insertion_pose, speed=0.3):
      rospy.logerr("** Fail to return to pre insertion pose **")
      return False
    self.b_bot.gripper.close(wait=True)

    target_pose_target_frame.pose.position.x = target # Magic number

    for _ in range(6):
      result = self.b_bot.do_insertion(target_pose_target_frame, insertion_direction="+X", force=15.0, timeout=10.0, 
                                                    radius=0.002, relaxed_target_by=0.005, selection_matrix=selection_matrix, 
                                                    check_displacement_time=3)
      success = result == TERMINATION_CRITERIA
      success &= self.b_bot.move_lin_rel(relative_translation = [-0.003,0,0], acceleration = 0.015, speed=.03) # Release shaft for next push
      if success:
        break

    if not self.b_bot.move_lin(pre_insertion_pose, speed=0.05):
      rospy.logerr("** Fail to return to pre insertion pose **")
      return False

    if not success:
      rospy.logerr("** Insertion Failed!! **")
      if attempts > 0:
        # Fall back if the shaft is stuck
        self.b_bot.gripper.open(wait=True)

        rotation = np.deg2rad([-22.5, -88.5, -157.5]).tolist()  # Arbitrary
        re_pick_pose = conversions.to_pose_stamped(target_link, [0.06, 0.000, -0.002] + rotation)
        if not self.b_bot.move_lin(re_pick_pose, speed=0.05):
          rospy.logerr("** Fail to return to pre insertion pose **")
          return False
        self.b_bot.gripper.close(wait=True)

        if self.b_bot.gripper.opening_width < 0.004 and self.use_real_robot:
          rospy.logerr("Fail to grasp Shaft")
          self.b_bot.gripper.open(wait=True)
          return False

        return self.insert_shaft(target_link, attempts-1)
      return False
    return True

#### shaft orientation
  def orient_shaft(self):    
    if not self.centering_shaft():
      return False

    approach_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.100, 0, 0, tau/2., 0, 0])
    on_vgroove =       conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.000, 0, 0, tau/2., 0, 0])
    inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.007, 0, 0, tau/2., 0, 0])

    if not self.b_bot.go_to_pose_goal(approach_vgroove, move_lin=False):
      rospy.logerr("Fail to go to approach_vgroove")
      return False

    if not self.b_bot.go_to_pose_goal(on_vgroove, speed=0.1):
      rospy.logerr("Fail to go to on_vgroove")
      return False
    if not self.b_bot.go_to_pose_goal(inside_vgroove, speed=0.1):
      rospy.logerr("Fail to go to inside_vgroove")
      return False

    self.b_bot.gripper.open(opening_width=0.06)

    shaft_has_hole_at_top = self.check_screw_hole_visible_on_shaft_in_v_groove()
    if shaft_has_hole_at_top:
      self.b_bot.go_to_pose_goal(inside_vgroove, speed=0.1)
      self.b_bot.gripper.close()
      self.b_bot.go_to_pose_goal(approach_vgroove, speed=0.1)
    else:  # Turn gripper around
      approach_turned = copy.deepcopy(approach_vgroove)
      approach_turned.pose = helpers.rotatePoseByRPY(tau/2, 0, 0, approach_turned.pose)
      in_groove_turned = copy.deepcopy(approach_turned)
      in_groove_turned.pose.position.x = inside_vgroove.pose.position.x

      self.b_bot.go_to_pose_goal(approach_vgroove, speed=0.1)
      self.b_bot.go_to_pose_goal(approach_turned, speed=0.1)
      self.b_bot.go_to_pose_goal(in_groove_turned, speed=0.1)
      self.b_bot.gripper.close()
      self.b_bot.go_to_pose_goal(approach_turned, speed=0.1)
    
    self.b_bot.go_to_named_pose("home")
    self.b_bot.gripper.last_attached_object = None # clean attach/detach memory
    return True

  def centering_shaft(self):
    """ Push grasped shaft into the tray holder, and regrasp it centered.
    """
    shaft_length = 0.075
    approach_centering = conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length,     0.150, tau/2., tau/4., tau/4.])
    on_centering =       conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length,    -0.004, tau/2., tau/4., tau/4.])
    shaft_center =       conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length/2., -0.008, tau/2., tau/4., tau/4.])

    # if not self.b_bot.move_lin_trajectory([(approach_centering, 0.005, 0.6), (on_centering, 0.01, 0.4)]):
    #   return False

    # TODO(cambel): make this a trajectory
    if not self.b_bot.go_to_pose_goal(approach_centering):
      rospy.logerr("Fail to go to approaching_centering")
      self.b_bot.go_to_named_pose("home")  # Fallback because sometimes the planning fails for no reason??
      if not self.b_bot.go_to_pose_goal(approach_centering):
        rospy.logerr("Fail to go to approaching_centering AGAIN")
        return False
    if not self.b_bot.go_to_pose_goal(on_centering):
      rospy.logerr("Fail to go to on_centering")
      return False

    self.b_bot.gripper.open(opening_width=0.03)
    self.b_bot.gripper.close(velocity=0.013, force=40)  # Minimum force and speed
    # self.b_bot.linear_push(3, "-Y", max_translation=0.05, timeout=15.0)
    self.b_bot.move_lin_rel(relative_translation=[0, -.055, 0], speed=.05)
    self.b_bot.gripper.open(opening_width=0.03)
    if not self.b_bot.go_to_pose_goal(shaft_center, speed=0.1):
      rospy.logerr("Fail to go to relative shaft center")
      return False

    self.b_bot.gripper.close()
    if not self.simple_gripper_check("b_bot", 0.004):
      rospy.logerr("No shaft detected in gripper. Going back up and aborting.")
      self.b_bot.go_to_pose_goal(approach_centering)
      return False
  
    return True

  def orient_shaft_end_cap(self):

    if not self.playback_sequence("end_cap_orient"):
      rospy.logerr("Fail to end_cap_orient")
      return False

    # Look at the end cap to determine the orientation
    ready_to_put_on_shaft = self.is_the_placed_end_cap_upside_down(dy=0.04, dz=-0.02)

    # Pick it up again
    at_object_pose = conversions.to_pose_stamped("left_centering_link", [-0.005, 0, 0.0, -tau/2, 0, 0] )
    self.a_bot.go_to_pose_goal(at_object_pose, speed=0.5)

    self.center_with_gripper("a_bot", opening_width=0.05)

    self.a_bot.gripper.close()
    above_pose = conversions.to_pose_stamped("left_centering_link", [-0.1, 0, 0.0, -tau/2, 0, 0] )
    self.a_bot.go_to_pose_goal(above_pose, speed=0.5)

    if not ready_to_put_on_shaft:  # Do reorientation procedure
      approach_centering = conversions.to_pose_stamped("simple_holder_tip_link", [0.0, 0, 0.1,        0, tau/4., tau/4.])
      close_to_tip       = conversions.to_pose_stamped("simple_holder_tip_link", [0.0, -0.008,  0.01, 0, tau/4., tau/4.])
      push_down          = conversions.to_pose_stamped("simple_holder_tip_link", [0.0, -0.008, -0.02, 0, tau/4., tau/4.])
      prepare_second_push= conversions.to_pose_stamped("simple_holder_tip_link", [0.0, -0.03,   0.05, 0, tau/4., tau/4.])
      close_to_edge      = conversions.to_pose_stamped("simple_holder",          [0.08, -0.10, 0.001, tau/4., tau/4., tau/4.])
      push_edge          = conversions.to_pose_stamped("simple_holder",          [0.02, -0.10, 0.001, tau/4., tau/4., tau/4.])
      
      if not self.a_bot.go_to_pose_goal(approach_centering, move_lin=False):
        rospy.logerr("Fail to go to approach_centering")
        return False
      if not self.a_bot.go_to_pose_goal(close_to_tip, speed=0.5):
        rospy.logerr("Fail to go to close_to_tip")
        return False
      if not self.a_bot.go_to_pose_goal(push_down, speed=0.01):
        rospy.logerr("Fail to go to push_down")
        return False
      if not self.a_bot.go_to_pose_goal(prepare_second_push, speed=0.5):
        rospy.logerr("Fail to go to prepare_second_push")
        return False
      if not self.a_bot.go_to_pose_goal(close_to_edge, speed=0.5):
        rospy.logerr("Fail to go to close_to_edge")
        return False
      if not self.a_bot.go_to_pose_goal(push_edge, speed=0.01):
        rospy.logerr("Fail to go to push_edge")
        return False

      self.a_bot.gripper.open(velocity=0.03, opening_width=0.03)
      if not self.center_with_gripper("a_bot", opening_width=0.05, clockwise=True):
        rospy.logerr("Fail to go to center with gripper")
        return False

      self.a_bot.gripper.close()
      
      if not self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.15]):
        rospy.logerr("Fail to go to centering_pose")
        return False
    else:
      self.a_bot.go_to_named_pose("home")
    return True

  @check_for_real_robot
  def is_the_placed_end_cap_upside_down(self, dy=0.0, dz=0.0, led_on=False):
    """ Look at the end cap placed next a few times, return.

        dy, dz add an offset to the camera position.
    """
    self.vision.activate_camera("a_bot_outside_camera")
    self.activate_led("a_bot", on=led_on)

    cam_height = -self.tray_view_low.pose.position.z -.005

    def go_and_record(view_pose, results):
      self.a_bot.go_to_pose_goal(view_pose, speed=0.5, end_effector_link="a_bot_outside_camera_color_frame")
      self.get_3d_poses_from_ssd()
      obj_id = self.assembly_database.name_to_id("end_cap")
      res = copy.copy(self.object_in_tray_is_upside_down.get(obj_id, None))
      if res is not None:
        results.append(res)

    results = []
    
    go_and_record(conversions.to_pose_stamped("left_centering_link", [cam_height, dy, dz, radians(-90), 0, 0] ), results)
    go_and_record(conversions.to_pose_stamped("left_centering_link", [cam_height, dy, dz, radians(-110), 0, 0] ), results)
    go_and_record(conversions.to_pose_stamped("left_centering_link", [cam_height, dy, dz, radians(-135), 0, 0] ), results)
    go_and_record(conversions.to_pose_stamped("left_centering_link", [cam_height, dy, dz, radians(-160), 0, 0] ), results)
    go_and_record(conversions.to_pose_stamped("left_centering_link", [cam_height, dy, dz, radians(-180), 0, 0] ), results)
    
    if len(results) == 0:
      rospy.logerr("Failed to see the end cap!")
      if not led_on:
        rospy.loginfo("Retry with LED on")
        return self.is_the_placed_end_cap_upside_down(dy, dz, led_on=True)
      return False

    print(">>> end_cap views result:", np.array(results))
    is_upside_down = np.mean(np.array(results) * 1) >= 0.5
    print(">>> is upsidedown?:", is_upside_down)

    return is_upside_down

  def pick_end_cap(self):
    """ Returns "Success" and "endcap_is_upside_down" (upside down = ready to be placed on shaft)
    """
    options = {'check_for_close_items': True, 'declutter_with_tool': True, 'allow_pick_near_border': True}
    goal = self.look_and_get_grasp_point("end_cap", robot_name="a_bot", options=options)
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find shaft in tray. Skipping procedure.")
      return False

    goal.pose.position.z = -0.001 # Magic Numbers for grasping
    goal.pose.position.x -= 0.01
    goal = self.listener.transformPose("world", goal)  # This is not necessary

    self.vision.activate_camera("a_bot_inside_camera")  # Just for visualization
    if not self.simple_pick("a_bot", goal, axis="z", speed_fast=0.5, gripper_force=100.0, grasp_width=.04, 
                               approach_height=0.1, item_id_to_attach="", lift_up_after_pick=True, approach_with_move_lin=False):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check("a_bot", min_opening_width=0.001):
      return False

    return True

  def insert_end_cap(self, attempts=1):
    self.a_bot.linear_push(force=2.5, direction="-Z", max_translation=0.05, timeout=10.0)
    target_pose = self.a_bot.get_current_pose_stamped()
    target_pose.pose.position.z -= 0.002
    self.a_bot.move_lin_rel(relative_translation=[0,0,0.001]) # release pressure before insertion

    selection_matrix = [0.3, 0.3, 0., 0.95, 1, 1]

    # target_pose = conversions.to_pose_stamped("tray_center", [-0.002, -0.001, 0.237, 0, 0, 0])
    result = self.a_bot.do_insertion(target_pose, insertion_direction="-Z", force=2, timeout=15.0, 
                                  radius=0.003, revolutions=3, relaxed_target_by=0.0005, selection_matrix=selection_matrix,
                                  check_displacement_time=3., displacement_epsilon=0.0005)
    success = result in (TERMINATION_CRITERIA)

    if result == DONE and attempts > 0:
      return self.insert_end_cap(attempts=attempts-1)

    return success

  def fasten_end_cap(self):
    self.vision.activate_camera("a_bot_outside_camera")
    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    if not self.do_change_tool_action("a_bot", equip=True, screw_size = 4):
      rospy.logerr("Failed equip m4")
      return False

    self.confirm_to_proceed("pick screw")
    if not self.pick_screw_from_feeder("a_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      self.do_change_tool_action("a_bot", equip=False, screw_size = 4)
      self.b_bot.gripper.open()
      self.ab_bot.go_to_named_pose("home")
      return False
    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    
    self.confirm_to_proceed("go to above_hole_screw_pose")
    above_hole_screw_pose = conversions.to_pose_stamped("tray_center", [-0.001, 0.019, 0.388]+np.deg2rad([180, 30, 90]).tolist())
    if not self.a_bot.go_to_pose_goal(above_hole_screw_pose, speed=0.2, move_lin=False):
      rospy.logerr("Fail to go to above_hole_screw_pose")
      return False

    obj = self.assembly_database.get_collision_object("shaft")
    obj.header.frame_id = "b_bot_gripper_tip_link"
    obj.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, -tau/4, -tau/4))
    obj.pose = helpers.rotatePoseByRPY(0, 0, tau/2, obj.pose)
    obj.pose.position = conversions.to_point([-.006, 0, .0375])
    self.planning_scene_interface.add_object(obj)
    self.b_bot.gripper.attach_object(obj.id)

    self.confirm_to_proceed("try to screw")
    self.vision.activate_camera("b_bot_inside_camera")
    hole_screw_pose = conversions.to_pose_stamped("move_group/shaft/screw_hole", [0.0, 0.002, -0.010, 0, 0, 0])
    self.skill_server.do_screw_action("a_bot", hole_screw_pose, screw_height=0.02, screw_size=4, loosen_and_retighten_when_done=True)
    
    self.confirm_to_proceed("unequip tool")
    self.tools.set_suction("screw_tool_m4", suction_on=False, wait=False)

    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    
    if not self.do_change_tool_action("a_bot", equip=False, screw_size = 4):
      rospy.logerr("Failed unequip m4")
      return False
    
    return True

  ######## Motor

  def pick_and_center_motor(self):
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 10:
      picked = self.attempt_motor_tray_pick()
      if not picked:
        attempt_nr += 1
        continue
      
      # Place motor in centering area
      p_through = conversions.to_pose_stamped("right_centering_link", [-0.15, 0, 0.10, -90, 0, 0])
      p_drop    = conversions.to_pose_stamped("right_centering_link", [-0.03, 0, 0.0, -135, 0, 0])
      self.b_bot.go_to_pose_goal(p_through, speed=.5, wait=True)
      self.b_bot.go_to_pose_goal(p_drop, speed=.5, wait=True)
      self.b_bot.gripper.open()

      picked = self.confirm_motor_and_place_in_aid()
      if not picked:
        attempt_nr += 1
        continue
    return picked

  def attempt_motor_tray_pick(self, robot_name="b_bot"):
    """ Do one pick attempt, ignoring the motor's orientation.
    """
    self.vision.activate_camera("b_bot_outside_camera")
    self.active_robots[robot_name].go_to_pose_goal(self.tray_view_high, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, acceleration=.2)
    res = self.get_3d_poses_from_ssd()
    obj_id = self.assembly_database.name_to_id("motor")
    try:
      r2 = self.get_feasible_grasp_points(obj_id)
      p = r2[0]
      if len(r2) > 1 and np.random.uniform() > 0.5:  # Randomly choose between vertical and horizontal orientation
        pr = r2[1]
        # p = helpers.rotatePoseByRPY(tau/4, 0, 0, p)
      p.pose.position.z = 0.015
      self.simple_pick(robot_name, p, gripper_force=100.0, grasp_width=.085, axis="z")
      if self.active_robots[robot_name].gripper.opening_width < 0.031:
        rospy.logerr("Motor not successfully grasped")
        return False
    except:
      rospy.logerr("Did not see the motor in the tray")
      return False
    return True

  def confirm_motor_and_place_in_aid(self):
    """ Assumes that the motor is grasped in a random orientation. 
        Places it in the separate area, determines the orientation, centers it, and places it in the vgroove.
    """
    p_view = conversions.to_pose_stamped("right_centering_link", [-self.tray_view_high.pose.position.z, 0, 0, 0, 0, 0])
    p_view = self.listener.transformPose("tray_center", p_view)
    p_view.pose.orientation = self.tray_view_high.pose.orientation
    self.vision.activate_camera("b_bot_outside_camera")
    self.b_bot.go_to_pose_goal(p_view, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, wait=True)

    # Look at the motor from above, confirm that SSD sees it
    rospy.sleep(1.0)
    res = self.get_3d_poses_from_ssd()
    motor_placed = True
    try:
      motor_id = self.assembly_database.name_to_id("motor")
      if not motor_id in res.class_ids:
        motor_placed = False
    except:
      motor_placed = False
    if not motor_placed:
      rospy.logerr("Motor not detected by SSD! Return item and abort")
      p_pick = conversions.to_pose_stamped("right_centering_link", [-.002, 0, 0, 0, 0, 0])
      p_pick = self.listener.transformPose("tray_center", p_pick)
      p_pick.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      self.simple_pick("b_bot", object_pose=p_pick, gripper_force=100, axis="z", retreat_height=.1)
      self.drop_in_tray("b_bot")
      return False
    
    # Use CAD matching to determine orientation
    pose = self.get_large_item_position_from_top("motor", skip_moving=True)
    if not pose:
      rospy.logerr("Could not find motor via CAD matching!")
      return False
    
    self.planning_scene_interface.allow_collisions("motor")

    # TODO: Call Place action on CAD result to constrain motor to surface
    # Get motor grasp pose from CAD result
    rospy.sleep(0.5)  # To let the scene update with the motor
    p_motor = conversions.to_pose_stamped("move_group/motor/center", [0, 0, 0, 0, 0, 0])  # x-axis points along axis towards the front (shaft)
    p_motor_in_tray_center = self.listener.transformPose("tray_center", p_motor)
    p_motor_in_tray_center.pose = helpers.getOrientedFlatGraspPoseFromXAxis(p_motor_in_tray_center.pose)
    p_motor_in_centering_link = self.listener.transformPose("right_centering_link", p_motor_in_tray_center)
    p_motor_in_centering_link.pose.position.x = -0.006  # Grasp height

    # Check that motor direction matches the cables seen in RGB image
    # flag values are 0:right, 1:left, 2:top, 3:bottom  (documented in pose_estimation_func)
    ## theta = 0  --> top     --> flag = 2
    ## theta = 90 --> right   --> flag = 0
    ## theta = 180 --> bottom --> flag = 3
    ## theta = 270 --> left   --> flag = 1
    motor_rotation_flag = self.vision.get_motor_angle_from_top_view(camera="b_bot_outside_camera")
    q = p_motor_in_tray_center.pose.orientation
    rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    print("rpy ", rpy)
    print("motor_rotation_flag ", motor_rotation_flag)
    theta = rpy[0]
    print("degrees(theta) ", degrees(theta))
    if theta < 0:
      theta += tau

    if theta >= 0.0 and theta < tau/4:
      if motor_rotation_flag is not 2 and motor_rotation_flag is not 0:
        rospy.loginfo("")
        p_motor_in_centering_link = helpers.rotatePoseByRPY(-tau/2, 0, 0, p_motor_in_centering_link)
    elif theta >= tau/4 and theta < tau/2:
      if motor_rotation_flag is not 0 and motor_rotation_flag is not 3:
        p_motor_in_centering_link = helpers.rotatePoseByRPY(-tau/2, 0, 0, p_motor_in_centering_link)
    elif theta >= tau/2 and theta < tau*3/4:
      if motor_rotation_flag is not 3 and motor_rotation_flag is not 1:
        p_motor_in_centering_link = helpers.rotatePoseByRPY(-tau/2, 0, 0, p_motor_in_centering_link)
    elif theta >= tau*3/4 and theta < tau:
      if motor_rotation_flag is not 1 and motor_rotation_flag is not 2:
        p_motor_in_centering_link = helpers.rotatePoseByRPY(-tau/2, 0, 0, p_motor_in_centering_link)

    # Pick motor in defined orientation and move up
    self.planning_scene_interface.remove_world_object("motor")
    self.simple_pick("b_bot", object_pose=p_motor_in_centering_link, grasp_width=.05, axis="x", sign=-1, approach_height=0.07)
    
    self.confirm_to_proceed("motor picked?")
    # self.reset_scene_and_robots()
    # self.planning_scene_interface.remove_attached_object("motor")
    # rospy.sleep(0.5)

    if not self.b_bot.load_and_execute_program(program_name="wrs2020/motor_orient.urp"):
        return False
    # self.center_motor()
    # self.orient_motor_in_aid_edge()
    pass

  def look_and_align_motor_in_vgroove(self, upside_down=False):
    """ Assumes that the motor is in the vgroove_aid_motor_link. Rearranges the motor so that the shaft is at the top or bottom.
    """
    attempts = 0
    times_seen_at_correct_angle = 0
    if not upside_down:
      target_angle = 0
    else:
      target_angle = tau/2
    
    while not rospy.is_shutdown():
      self.b_bot.gripper.open()
      angle = self.look_at_motor()
      if not angle:
        continue
      self.rotate_motor_in_aid(angle-target_angle)
      
  def rotate_motor_in_aid(self, angle):
    pass
  
  def flip_motor_in_aid(self, angle):
    """ Turn motor in aid by 180 degrees.W
    """
    pass

  def insert_motor(self, target_link, attempts=1):
    target_pose = conversions.to_pose_stamped(target_link, [-0.019, -0.006, -0.015, -tau/4, radians(60), -tau/4])

    selection_matrix = [0., 0.2, 0.2, 0.95, 1, 1]
    
    self.b_bot.linear_push(2, "+X", max_translation=0.03, timeout=10.0)
    result = self.b_bot.do_insertion(target_pose, insertion_direction="+X", force=5.0, timeout=20.0, 
                                      wiggle_direction="X", wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=1.0,
                                      radius=0.003, relaxed_target_by=0.002, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA

    if not success:
      # TODO(cambel): implement a fallback
      rospy.logerr("** Insertion Failed!! **")
      return False

    self.b_bot.linear_push(4, "+Z", max_translation=0.02, timeout=10.0)

    return True

  def center_motor(self):
    """ Push grasped shaft into the tray holder, and regrasp it centered.
    """
    motor_length = 0.035
    approach_centering = conversions.to_pose_stamped("tray_left_stopper", [0.0, motor_length,     0.05, tau/2., tau/4., tau/4.])
    on_centering =       conversions.to_pose_stamped("tray_left_stopper", [0.0, motor_length,    -0.004, tau/2., tau/4., tau/4.])
    p_pick                = conversions.to_pose_stamped("right_centering_link", [-0.006, -0.081, 0.043, 0, 0, 0])
    
    if not self.b_bot.go_to_pose_goal(approach_centering):
      rospy.logerr("Fail to go to approaching_centering")
      self.b_bot.go_to_named_pose("home")  # Fallback because sometimes the planning fails for no reason??
      if not self.b_bot.go_to_pose_goal(approach_centering):
        rospy.logerr("Fail to go to approaching_centering AGAIN")
        return False
    if not self.b_bot.go_to_pose_goal(on_centering):
      rospy.logerr("Fail to go to on_centering")
      return False

    self.b_bot.linear_push(3, "-Y", max_translation=0.05, timeout=15.0)
    self.b_bot.gripper.open(opening_width=0.03)
    if not self.b_bot.go_to_pose_goal(p_pick, speed=0.1):
      rospy.logerr("Fail to go to motor pick position")
      return False

    self.b_bot.gripper.close()
    if not self.b_bot.gripper.opening_width > 0.025:
      rospy.logerr("No motor detected in gripper. Going back up and aborting.")
      self.b_bot.go_to_pose_goal(approach_centering)
      return False
  
    return True

  def orient_motor_in_aid_edge(self):
    """ Starting with the motor picked (after center_motor), drop motor in vgroove aid's edge a number of times,
        to align the shaft near the top.
    """

    above_first_drop = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.042, 0, 0.041, tau/2, 0, 0])
    at_first_drop    = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.018,  0, 0.045, tau/2, 0, 0])
    repick           = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.03,   0, 0.041, tau/2, 0, 0])
    redrop           = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.018,  0, 0.041, tau/2, 0, 0])
    repick_low       = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.023,  0, 0.041, tau/2, 0, 0])
    
    # Drop first time
    self.b_bot.go_to_pose_goal(above_first_drop)
    self.b_bot.go_to_pose_goal(at_first_drop)
    self.b_bot.gripper.open(opening_width=.06)
    
    # Repick and drop X times
    num_tries = 10
    for _ in range(num_tries):
      if num_tries < 6:
        self.b_bot.go_to_pose_goal(repick, move_lin=False)
      else:
        self.b_bot.go_to_pose_goal(repick_low, move_lin=False)
      self.b_bot.gripper.close()
      self.b_bot.go_to_pose_goal(redrop, move_lin=False)
      self.b_bot.gripper.open(opening_width=.06)
    
    # Place in vgroove aid
    self.b_bot.go_to_pose_goal(repick_low, speed=0.2)
    self.b_bot.gripper.close()
    self.b_bot.move_lin_rel(relative_translation=[0, 0, 0.08], speed=0.2)
    
    # TODO: Touch shaft tip to ensure motor position, then use relative motions
    # pre_touch =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.023, 0.0, 0.041, tau/2, radians(-4), 0])
    drop =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.009, 0, -0.016, tau/2., radians(-4), 0])
    
    self.b_bot.go_to_pose_goal(drop, speed=0.2)
    self.b_bot.gripper.open(opening_width=.06)

  def align_motor_pre_insertion(self):
    inclination = radians(15)
    above_vgroove  =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.2, 0, 0, tau/2., 0, 0])
    # inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.0, 0, -0.01, tau/2., radians(3.0), 0])
    inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.0, 0, -0.005, tau/2., radians(3.0), inclination])
    if not self.b_bot.go_to_pose_goal(inside_vgroove, speed=0.2):
      return False
    self.confirm_to_proceed("finetune sav")
    self.b_bot.gripper.close(force=70, velocity=0.01)
    if not self.b_bot.go_to_pose_goal(above_vgroove, speed=0.4, move_lin=False):
      rospy.logerr("fail to go above vgroove")
      return False

    midway        =   conversions.to_pose_stamped("assembled_part_02_back_hole", [ -0.100, 0, 0.1, -tau/4, tau/4-inclination, -tau/4])
    pre_insertion =   conversions.to_pose_stamped("assembled_part_02_back_hole", [ -0.045, -0.004, -0.019, -tau/4, tau/4-inclination, -tau/4])
    if not self.b_bot.go_to_pose_goal(midway, speed=0.4):
      rospy.logerr("fail to go midway")
      return False
    if not self.b_bot.go_to_pose_goal(pre_insertion, speed=0.4):
      rospy.logerr("fail to go pre_insertion")
      return False
    return True
  
  def fasten_motor(self, robot_name="a_bot", support_robot="b_bot"):
    """ Assumes that the tool is already equipped """
    assert robot_name != support_robot, "Same robot cannot fill both roles"
    robot = self.active_robots[robot_name]
    offset = -1 if robot_name == "a_bot" else 1
    speed = 0.2
    screw_pose = conversions.to_pose_stamped("assembled_part_02_motor_screw_hole_1", [0.004, 0, -0.001, offset*tau/12, 0, 0])
    intermediate_pose = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.05, -0.113, 0.006, radians(150), 0, tau/2])
    def a_bot_task():
      # pass
      self.pick_and_fasten_screw(robot_name, screw_pose, screw_size=3, approach_distance=0.07, intermediate_pose=intermediate_pose, speed=speed)
    def b_bot_task():
      pass
      tc = lambda a, b: self.tools.fastening_tool_client.get_state() != GoalStatus.ACTIVE
      selection_matrix = [1., 1., 1., 0.8, 1., 1.]
      self.active_robots[support_robot].execute_spiral_trajectory("XY", max_radius=0, radius_direction="+Y", steps=50,
                                                          revolutions=1, target_force=0, check_displacement_time=10,
                                                          wiggle_direction="Z", wiggle_angle=radians(10.0), wiggle_revolutions=1.0,
                                                          termination_criteria=tc, timeout=20, selection_matrix=selection_matrix)
    self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60.0)
    return True
  ########

  def move_towards_tray_center(self, robot_name, distance, speed=0.05, acc=0.025, go_back_halfway=True, one_direction=None, end_effector_link="", go_back_ratio=0.5):
    """ Moves from the current position of the robot towards the tray center at constant height.
        one_direction: 'x' or 'y'
    """
    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=True)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=True)

    p_start = self.active_robots[robot_name].get_current_pose_stamped()
    p_start = self.listener.transformPose("tray_center", p_start)
    p_center = conversions.to_pose_stamped("tray_center", [0,0,0] + conversions.from_quaternion(p_start.pose.orientation).tolist())

    d = helpers.pose_dist(p_start.pose, p_center.pose)
    if distance > d:
      ratio = distance/d
    else:
      ratio = distance/d
    p = helpers.interpolate_between_poses(p_start.pose, p_center.pose, ratio)
    p_new = conversions.to_pose_stamped("tray_center", conversions.from_pose_to_list(p))
    p_new.pose.position.z = p_start.pose.position.z # do not go down
    if one_direction == 'x':
      p_new.pose.position.y = p_start.pose.position.y
    if one_direction == 'y':
      p_new.pose.position.x = p_start.pose.position.x
    if not self.active_robots[robot_name].move_lin(p_new, speed=speed, acceleration=acc, end_effector_link=end_effector_link):
      return False
    
    if go_back_halfway:
      p = helpers.interpolate_between_poses(p_new.pose, p_start.pose, go_back_ratio)
      p_new = conversions.to_pose_stamped("tray_center", conversions.from_pose_to_list(p))
      p_new.pose.position.z = p_start.pose.position.z # do not go down
      if one_direction == 'x':
        p_new.pose.position.y = p_start.pose.position.y
      if one_direction == 'y':
        p_new.pose.position.x = p_start.pose.position.x
      if not self.active_robots[robot_name].move_lin(p_new, speed=speed, acceleration=acc, end_effector_link=end_effector_link):
        return False

    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=False)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=False)
    return p_new

  def fasten_screw_vertical(self, robot_name, screw_hole_pose, allow_collision_with_object="", screw_height = .02, screw_size = 4):
    """
    This works for the two L-plates when they are facing forward.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
    
    if allow_collision_with_object:
      self.planning_scene_interface.allow_collisions("screw_tool_m4", allow_collision_with_object)  # Has to be allowed after each screw attempt
    res = self.screw(robot_name, screw_hole_pose, screw_size, screw_height, stay_put_after_screwing=False, skip_final_loosen_and_retighten=False)
    if allow_collision_with_object:
      self.planning_scene_interface.allow_collisions("screw_tool_m4", allow_collision_with_object)  # Has to be allowed after each screw attempt
    self.active_robots[robot_name].go_to_named_pose("feeder_pick_ready")
    if allow_collision_with_object:
      self.planning_scene_interface.allow_collisions("screw_tool_m4", allow_collision_with_object) 
    return res  # Bool

  def fasten_screw_horizontal(self, robot_name, screw_hole_pose, screw_height = .02, screw_size = 4):
    """
    This should work for the motor and bearing.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    self.active_robots[robot_name].go_to_named_pose("horizontal_screw_ready")
    
    # success = self.skill_server.do_screw_action(robot_name, screw_hole_pose, screw_height, screw_size)
    success = self.screw(robot_name, screw_hole_pose, screw_size, screw_height, stay_put_after_screwing=False, skip_final_loosen_and_retighten=False)
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

#### subtasks assembly 

  def pick_bearing_spacer(self):
    bearing_spacer_pose = self.look_and_get_grasp_point("bearing_spacer")
    bearing_spacer_pose.pose.position.x -= 0.005 # Magic numbers

    self.vision.activate_camera("b_bot_inside_camera")
    if not self.simple_pick("b_bot", bearing_spacer_pose, grasp_height=0.001, gripper_force=50.0, grasp_width=.04, axis="z", approach_height=0.07, gripper_command=0.03):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check("b_bot", min_opening_width=0.002):
      rospy.logerr("Gripper did not grasp the bearing_spacer --> Stop")
      return False
    return True

  def pick_output_pulley(self):
    options = {'grasp_width': 0.06, 'check_for_close_items': False}
    output_pulley_pose = self.look_and_get_grasp_point("output_pulley", options=options)
    output_pulley_pose.pose.position.x -= 0.005 # Magic numbers
    output_pulley_pose.pose.position.z = 0.0 # Magic numbers

    self.vision.activate_camera("b_bot_inside_camera")
    if not self.simple_pick("b_bot", output_pulley_pose, grasp_height=0.011, gripper_force=50.0, grasp_width=.05, axis="z", approach_height=0.07, gripper_command=0.03):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check("b_bot"):
      rospy.logerr("Gripper did not grasp the output_pulley --> Stop")
      return False
    return True

  def panel_subtask2(self):
    self.a_bot.go_to_named_pose("home")
    self.b_bot.go_to_named_pose("home")

    # goal = self.look_and_get_grasp_point("panel_bearing")
    
    goal = conversions.to_pose_stamped("tray_center", [0.02, -0.06, 0.001, 0.0, 0.0, -tau/4])
    self.spawn_object("panel_bearing", goal, goal.header.frame_id)
    goal = conversions.to_pose_stamped("tray_center", [0.02, -0.1, 0.02, 0.0, tau/4, -tau/4])
    
    handover_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.15, 0.0, tau/4, 0.0])
    self.simple_hand_over("b_bot", "a_bot", handover_pose, "panel_bearing", "default_grasp", "grasp_7")

  def simple_hand_over(self, from_robot_name, to_robot_name, handover_pose, object_name, grasp_pose1, grasp_pose2):
    self.pick(from_robot_name, "panel_bearing", grasp_name=grasp_pose1)
    self.active_robots[from_robot_name].move_lin(handover_pose, speed=0.5)
    self.active_robots[from_robot_name].detach_object(object_name)

    grasp2 = self.assembly_database.get_grasp_pose(object_name, grasp_pose2)
    grasp2.header.frame_id = "move_group/" + grasp2.header.frame_id
    grasp2.header.stamp = rospy.Time.now() - rospy.Time(0.5)

    grasp2 = self.listener.transformPose("world", grasp2)
    self.simple_pick(to_robot_name, object_pose=grasp2, grasp_width=0.06, approach_height=-0.05, grasp_height=0.0, 
                     axis="y", item_id_to_attach=object_name, lift_up_after_pick=False, approach_with_move_lin=False)
    
    self.active_robots[from_robot_name].gripper.open()
    self.active_robots[from_robot_name].go_to_named_pose("home")

  def check_output_pulley_angle(self):
    # Look at pulley with b_bot
    approach_centering = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.1, 0, -0.15,  np.deg2rad(180), np.deg2rad(-60), 0])
    self.b_bot.go_to_pose_goal(approach_centering, speed=0.1, end_effector_link="b_bot_outside_camera_link", move_lin=False)
    self.vision.activate_camera("b_bot_outside_camera")

    self.vision.activate_pulley_screw_detection()

    while not rospy.is_shutdown():
      print("screws visible:", self.vision.check_if_pulley_screws_visible())
      rospy.sleep(0.1)


    # TODO: Turn pulley with a_bot, stop when screws visible

  def check_motor_pulley_angle(self):
    # Check bearing orientation
    approach_centering = conversions.to_pose_stamped("assembled_part_04_tip", [0.0, 0, -0.15,  0, -tau/4., 0])
    self.b_bot.go_to_pose_goal(approach_centering, speed=0.1, end_effector_link="b_bot_outside_camera_link",move_lin=False)
    self.vision.activate_camera("b_bot_outside_camera")

    # TODO(cambel): compute angle

  def insert_bearing_spacer(self, target_link, attempts=1):
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.04, 0.0, 0.0, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame

    selection_matrix = [0.0, 0.3, 0.3, 0.95, 1.0, 1.0]
    result = self.b_bot.do_insertion(target_pose_target_frame, radius=0.0005, 
                                                      insertion_direction="-X", force=5.0, timeout=15.0, 
                                                      relaxed_target_by=0.005, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA

    if not success:
      grasp_check = self.simple_insertion_check("b_bot", 0.06, min_opening_width=0.02)
      if grasp_check and attempts > 0: # try again the spacer is still there   
        return self.insert_bearing_spacer(target_link, attempts=attempts-1)
      elif not grasp_check or not attempts > 0:
        self.b_bot.gripper.open(wait=True, opening_width=0.08)
        self.b_bot.move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
        rospy.logerr("** Insertion Failed!! **")
        return False

    self.b_bot.gripper.open(wait=True, opening_width=0.08)
    self.b_bot.move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
    return success

  def insert_output_pulley(self, target_link, attempts=1):
    rospy.loginfo("Starting insertion of output pulley")
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.043, 0.0, 0.0, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame

    selection_matrix = [0.0, 0.3, 0.3, 0.95, 1.0, 1.0]
    result = self.b_bot.do_insertion(target_pose_target_frame, radius=0.003, 
                                                      insertion_direction="-X", force=8.0, timeout=15.0, 
                                                      relaxed_target_by=0.005, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA
    rospy.loginfo("insertion finished with status: %s" % result)

    if not success:
      grasp_check = self.simple_insertion_check("b_bot", 0.09, min_opening_width=0.02)
      if grasp_check and attempts > 0: # try again the pulley is still there   
        return self.insert_output_pulley(target_link, attempts=attempts-1)
      elif not grasp_check or not attempts > 0:
        self.b_bot.gripper.open(wait=True, opening_width=0.08)
        self.b_bot.move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
        rospy.logerr("** Insertion Failed!! **")
        return False

    rospy.loginfo("Preparing push")
    self.b_bot.gripper.open(wait=True, opening_width=0.09)
    self.b_bot.move_lin_rel(relative_translation=[0.02,0,0])
    self.b_bot.gripper.send_command(0.04, wait=False)
    rospy.loginfo("Starting push")
    success = self.b_bot.force_controller.linear_push(force=8, direction="-X", max_translation=0.1, timeout=20.)

    self.b_bot.gripper.open(wait=True, opening_width=0.09)
    self.b_bot.move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
    return success

  def take_tray_from_agv_preplanned(self, save_on_success=True, use_saved_plans=True):
    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)

    # Push the tray from the side
    self.a_bot.gripper.open(wait=False)
    self.b_bot.gripper.open(wait=False)

    if not self.playback_sequence("tray_orient", plan_while_moving=True, save_on_success=save_on_success, use_saved_plans=use_saved_plans):
      return False

    if not self.playback_sequence("tray_take_from_agv", plan_while_moving=True, save_on_success=save_on_success, use_saved_plans=use_saved_plans):
      return False

    self.ab_bot.go_to_named_pose("home")

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)

    return True

  def take_tray_from_agv(self, reverse=False, reverse_movement_for_calibration=False, preplanned=True):
    """
    Take the tray from the AGV and place it in the robot workspace.
    """
    if preplanned:
      return self.take_tray_from_agv_preplanned()

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)

    self.publish_status_text("Target: Tray")

    a_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center",  [0.04, -0.192, 0.121, 0, tau/4, 0])
    b_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [0.04, 0.190, 0.121, 0, tau/4, 0])
    b_bot_at_tray_agv = conversions.to_pose_stamped("agv_tray_center", [0.04, 0.190, 0.021, 0, tau/4, 0])
    a_bot_above_tray_table = conversions.to_pose_stamped("tray_center", [0.04, 0.190, 0.121, 0, tau/4, 0])
    b_bot_above_tray_table = conversions.to_pose_stamped("tray_center", [0, 0.190, 0.121, 0, tau/4, 0])
    b_bot_at_tray_table = conversions.to_pose_stamped("tray_center", [0, 0.190, 0.022, 0, tau/4, 0])
    
    self.a_bot.gripper.open(opening_width=0.08, wait=False)
    self.b_bot.gripper.open(opening_width=0.08, wait=False)

    # Gripper needs to be open for these poses
    a_bot_push_tray_side_start_high = conversions.to_pose_stamped("agv_tray_center", [-0.01, -0.29, 0.09, tau/4, tau/4, 0])
    b_bot_push_tray_side_start_high = conversions.to_pose_stamped("agv_tray_center", [-0.004, 0.28, 0.10, -.5, .5, .5, .5])
    a_bot_push_tray_side_start = conversions.to_pose_stamped("agv_tray_center", [-0.01, -0.29, -0.019, tau/4, tau/4, 0])
    b_bot_push_tray_side_start = conversions.to_pose_stamped("agv_tray_center", [-0.004, 0.28, -0.004, -.5, .5, .5, .5])
    a_bot_push_tray_side_goal = conversions.to_pose_stamped("agv_tray_center", [-0.01, -0.218, -0.019, tau/4, tau/4, 0])
    b_bot_push_tray_side_goal = conversions.to_pose_stamped("agv_tray_center", [-0.004, 0.209, -0.004, -.5, .5, .5, .5])
    a_bot_push_tray_side_retreat = conversions.to_pose_stamped("agv_tray_center", [-0.01, -0.25, 0.09, tau/4, tau/4, 0])
    b_bot_push_tray_side_retreat = conversions.to_pose_stamped("agv_tray_center", [-0.004, 0.24, 0.105, -.5, .5, .5, .5])

    a_bot_push_tray_front_start_high = conversions.to_pose_stamped("agv_tray_center", [-0.25, -0.063, .09, 0, tau/4, 0])
    a_bot_push_tray_front_start = conversions.to_pose_stamped("agv_tray_center", [-0.25, -0.063, -0.01, 0, tau/4, 0])
    a_bot_push_tray_front_goal = conversions.to_pose_stamped("agv_tray_center", [-0.11, -0.063, -0.01, 0, tau/4, 0])
    a_bot_push_tray_front_retreat = conversions.to_pose_stamped("agv_tray_center", [-0.17, -0.18, .09, 0, tau/4, 0])

    if not reverse_movement_for_calibration and not reverse:
      # Push the tray from the side
      self.a_bot.gripper.open(wait=False)
      self.b_bot.gripper.open(wait=False)
      self.ab_bot.go_to_goal_poses(a_bot_push_tray_side_start_high, b_bot_push_tray_side_start_high, planner="OMPL", speed=0.3)
      self.ab_bot.go_to_goal_poses(a_bot_push_tray_side_start, b_bot_push_tray_side_start, planner="OMPL", speed=0.5)
      self.ab_bot.go_to_goal_poses(a_bot_push_tray_side_goal, b_bot_push_tray_side_goal, planner="OMPL", speed=0.1)
      self.ab_bot.go_to_goal_poses(a_bot_push_tray_side_retreat, b_bot_push_tray_side_retreat, planner="OMPL", speed=0.5)

      # Push from the front
      self.a_bot.go_to_pose_goal(a_bot_push_tray_front_start_high, speed=0.8)
      self.a_bot.go_to_pose_goal(a_bot_push_tray_front_start, speed=0.6)
      self.a_bot.go_to_pose_goal(a_bot_push_tray_front_goal, speed=0.08)
      self.a_bot.go_to_pose_goal(a_bot_push_tray_front_retreat, speed=0.6)
      
      # Grasp and place the tray
      self.ab_bot.go_to_goal_poses(a_bot_above_tray_agv, b_bot_above_tray_agv, planner="OMPL")
      self.confirm_to_proceed("At above_tray_agv. Move to next?")

      slave_relation = self.ab_bot.get_relative_pose_of_slave("b_bot", "a_bot")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_agv, slave_relation)
      self.confirm_to_proceed("At at_tray_agv. Move to next?")

      self.a_bot.gripper.close(force=80)
      self.b_bot.gripper.close(force=80)

      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_agv, slave_relation, speed=0.05)
      self.confirm_to_proceed("At above_tray_agv. Move to next?")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_table, slave_relation, speed=0.1)
      self.confirm_to_proceed("At above_tray_table. Move to next?")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_table, slave_relation, speed=0.05)
      self.confirm_to_proceed("At at_tray_table. Move to next?")

      if self.a_bot.is_protective_stopped() or self.b_bot.is_protective_stopped():
        rospy.logerr("Protective stop!!")
        
      self.a_bot.gripper.open(opening_width=0.05, wait=False)
      self.b_bot.gripper.open(opening_width=0.05)

      # TODO(felixvd): Release potential protective stop, double check that the tray is in the right position in that case

      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_table, slave_relation, speed=0.5)

      self.ab_bot.go_to_named_pose("home")

      self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)
    elif reverse:
      # Grasp and place the tray
      self.ab_bot.go_to_goal_poses(a_bot_above_tray_table, b_bot_above_tray_table, planner="OMPL")
      self.confirm_to_proceed("At above_tray_table. Move to next?")

      slave_relation = self.ab_bot.get_relative_pose_of_slave("b_bot", "a_bot")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_table, slave_relation)
      self.confirm_to_proceed("At at_tray_table. Move to next?")

      self.a_bot.gripper.close(force=80)
      self.b_bot.gripper.close(force=80)

      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_table, slave_relation, speed=0.1)
      self.confirm_to_proceed("At above_tray_table. Move to next?")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_agv, slave_relation, speed=0.05)
      self.confirm_to_proceed("At above_tray_agv. Move to next?")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_agv, slave_relation, speed=0.05)
      self.confirm_to_proceed("At above_tray_agv. Move to next?")

      if self.a_bot.is_protective_stopped() or self.b_bot.is_protective_stopped():
        rospy.logerr("Protective stop!!")
        
      self.a_bot.gripper.open(wait=False)
      self.b_bot.gripper.open()

      # TODO(felixvd): Release potential protective stop, double check that the tray is in the right position in that case

      self.ab_bot.go_to_named_pose("home")

      self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)
    elif reverse_movement_for_calibration:
      self.ab_bot.go_to_goal_poses(a_bot_above_tray_agv, b_bot_above_tray_agv, planner="OMPL")
      slave_relation = self.ab_bot.get_relative_pose_of_slave("b_bot", "a_bot")
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_table, slave_relation)
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_table, slave_relation, speed=0.05)

      self.a_bot.gripper.close(force=80)
      self.b_bot.gripper.close(force=80)

      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_table, slave_relation, speed=0.05)
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_agv, slave_relation, speed=0.05)
      
      self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_agv, slave_relation, speed=0.05)
    
    self.publish_status_text("SUCCESS: Tray")
  
  def unload_drive_unit(self):
    a_bot_above_drive_unit = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.0025, -0.076, 0.060, 0, 0.891, tau/4])
    b_bot_above_drive_unit = conversions.to_pose_stamped("assembled_part_03_front_hole", [0.0025, -0.067, 0.078, 0, 0.883, tau/4])
    a_bot_at_drive_unit = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.0025, -0.026, 0.010, 0, 0.891, tau/4])
    b_bot_at_drive_unit = conversions.to_pose_stamped("assembled_part_03_front_hole", [0.0025, -0.017, 0.028, 0, 0.883, tau/4])
    
    b_bot_drive_unit_loosened = self.listener.transformPose("tray_center", b_bot_at_drive_unit)
    b_bot_drive_unit_loosened.pose.position.x -= 0.008
    b_bot_drive_unit_loosened.pose.position.y -= 0.012
    b_bot_drive_unit_up = copy.deepcopy(b_bot_drive_unit_loosened)
    b_bot_drive_unit_up.pose.position.z += 0.1

    b_bot_above_tray_target = conversions.to_pose_stamped("tray_center", [0.06, 0.1, 0.2, 0, 0.883, -tau/4])
    b_bot_at_tray_target = conversions.to_pose_stamped("tray_center", [0.06, 0.1, 0.078, 0, 0.883, -tau/4])

    self.publish_status_text("Target: Unload product")
    self.unlock_base_plate()

    # Grasp the drive unit
    self.a_bot.gripper.open(opening_width=0.05, wait=False)
    self.b_bot.gripper.open(opening_width=0.05, wait=False)

    self.ab_bot.go_to_goal_poses(a_bot_above_drive_unit, b_bot_above_drive_unit, planner="OMPL")
    self.ab_bot.go_to_goal_poses(a_bot_at_drive_unit, b_bot_at_drive_unit, planner="OMPL")

    self.a_bot.gripper.close(force=100)
    self.b_bot.gripper.close(force=100)

    slave_relation = self.ab_bot.get_relative_pose_of_slave(master_name="b_bot", slave_name="a_bot")
    print("slave_relation", slave_relation)

    # Move product to the tray

    # TODO(felixvd): Adjust placement height if unit incomplete. Avoid protective stops.
    # if self.drive_unit_completed:
    #   b_bot_at_tray_target

    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_drive_unit_loosened, slave_relation, speed=0.02)
    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_drive_unit_up, slave_relation, speed=0.05)
    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_target, slave_relation, speed=0.2)
    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_target, slave_relation, speed=0.05)

    self.a_bot.gripper.open(opening_width=0.07, wait=False)
    self.b_bot.gripper.open(opening_width=0.07)

    self.ab_bot.go_to_named_pose("home")
    self.publish_status_text("SUCCESS: Unload product")

  def do_tasks_simultaneous(self, function_a_bot, function_b_bot, timeout=30.0):
    a_thread = ThreadTrace(target=function_a_bot)
    a_thread.daemon = True
    b_thread = ThreadTrace(target=function_b_bot)
    b_thread.daemon = True
    a_thread.start()
    b_thread.start()

    start_time = rospy.Time.now()
    a_thread.join(timeout)
    if a_thread.is_alive():
      rospy.logerr("a_bot not done yet, breaking out")
      a_thread.kill()
      b_thread.kill()
      return False
    rospy.loginfo("a_bot DONE")
    
    time_passed = (rospy.Time.now() - start_time).secs
    time_left_until_timeout = timeout - time_passed
    if time_left_until_timeout < 0.0:
      rospy.logerr("simultaneous task timed out!")
      b_thread.kill()
      return False

    b_thread.join(time_left_until_timeout)
    if b_thread.is_alive():
      rospy.logerr("b_bot not done yet, abort")
      rospy.logerr("Started at " + str(start_time) + " and timeout was " + str(timeout))
      b_thread.kill()
      return False
    rospy.loginfo("b_bot DONE")
    return True
