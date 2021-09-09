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

import bz2
from o2ac_assembly_database.parts_reader import PartsReader
from o2ac_routines import helpers, markers_scene
from o2ac_routines.base import *
from math import radians, degrees, sin, cos, pi
tau = 2*pi
from o2ac_routines.thread_with_trace import ThreadTrace
from ur_control.constants import DONE, TERMINATION_CRITERIA
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_gazebo.gazebo_spawner import GazeboModels
from ur_gazebo.model import Model
import gazebo_msgs

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

    self.nut_tool_used = False
    self.end_cap_is_upside_down = False

    self.update_distribution_client = actionlib.SimpleActionClient("update_distribution", o2ac_msgs.msg.updateDistributionAction)

    self.use_storage_on_failure = False 
    self.is_bearing_in_storage = False 
    self.bearing_store_pose = conversions.to_pose_stamped("left_centering_link", [-0.020, 0.007, 0.0]+np.deg2rad([-35.179, 29.784, -19.294]).tolist())
    self.is_motor_pulley_in_storage = False 
    self.motor_pulley_store_pose = conversions.to_pose_stamped("right_centering_link", [-0.005, 0, 0.0] +np.deg2rad([-135, 0, 0]).tolist())

    # self.gazebo_scene = GazeboModels('o2ac_gazebo')
    
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

  def define_local_tray_views(self, high_height=.385, low_height=.24, robot_name="b_bot", include_rotated_views=False):
    """
    Define the poses used to position the camera to look into the tray.

    Example usage: self.b_bot.go_to_pose_goal(self.tray_view_high, 
                                        end_effector_link="b_bot_outside_camera_color_frame", 
                                        speed=.1, acceleration=.04)
    """
    if robot_name == "b_bot":
      x_offset = .055  # At low_height
      y_offset = .095  # At low_height
    if robot_name == "a_bot":
      x_offset = .055  # At low_height
      y_offset = .095  # At low_height

    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "tray_center"
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    ps.pose.position.y = -0.01 if robot_name == "b_bot" else 0
    ps.pose.position.x = 0 if robot_name == "b_bot" else 0.02
    ps.pose.position.z = high_height

    # Centered views (high up and close)
    tray_view_high = copy.deepcopy(ps)
    ps.pose.position.z = low_height
    tray_view_low = copy.deepcopy(ps)

    # Close views in corners
    ps.pose.position.x = x_offset
    ps.pose.position.y = y_offset
    tray_view_close_front_b = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = y_offset
    tray_view_close_back_b = copy.deepcopy(ps)
    ps.pose.position.x = x_offset
    ps.pose.position.y = -y_offset
    tray_view_close_front_a = copy.deepcopy(ps)
    ps.pose.position.x = -x_offset
    ps.pose.position.y = -y_offset
    tray_view_close_back_a = copy.deepcopy(ps)

    close_tray_views = [tray_view_low, tray_view_close_front_b, tray_view_close_back_b, tray_view_close_front_a, tray_view_close_back_a]
    if include_rotated_views:
      rot_20  = [rotatePoseByRPY(radians(20),0,0, pose) for pose in close_tray_views]
      rot_n20 = [rotatePoseByRPY(radians(-20),0,0, pose) for pose in close_tray_views]
      rot_50  = [rotatePoseByRPY(radians(50),0,0, pose) for pose in close_tray_views]
      rot_90  = [rotatePoseByRPY(radians(90),0,0, pose) for pose in close_tray_views]
      close_tray_views += rot_20
      close_tray_views += rot_n20
      close_tray_views += rot_50
      close_tray_views += rot_90
    return tray_view_high, close_tray_views

  def publish_part_in_assembled_position(self, object_name, test_header_frame="", disable_collisions=False, marker_only=False):
    """ Move or publish a part as a collision object in its final assembled position.
        This is used to "finish" assembling a part.
    """
    if marker_only:
      marker = self.assembly_database.get_assembled_visualization_marker(object_name)
      self.assembly_marker_publisher.publish(marker)
      return True

    # Remove from scene or detach from robot
    self.despawn_object(object_name)
    # self.planning_scene_interface.remove_attached_object(name=object_name)

    # # DEBUGGING: Remove object from the scene
    # if True:
    #   self.planning_scene_interface.remove_world_object(name=object_name)
      
    # marker = self.assembly_database.get_assembled_visualization_marker(object_name, self.assembly_marker_id_counter)
    # self.assembly_marker_id_counter += 1
    # self.assembly_marker_publisher.publish(marker)

    object_id = self.assembly_database.name_to_id(object_name)
    collision_object = self.assembly_database.get_collision_object(object_name, use_simplified_collision_shapes=True)
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

    if disable_collisions:
      self.planning_scene_interface.allow_collisions(object_name)

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
    self.database_name = assembly_name
    self.assembly_database.change_assembly(assembly_name)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'attached_base_origin_link'
    pose.pose.orientation.w = 1.0
    self.assembly_database.publish_assembly_frames(pose, prefix="assembled_")
    self.markers_scene.parts_database = PartsReader(assembly_name, load_meshes=False, verbose=False)
    # TODO(cambel): load the objects dimensions from somewhere
    self.dimensions_dataset = {}
    if self.assembly_database.db_name in ["wrs_assembly_2021", "wrs_assembly_2021_flipped"]:
      self.dimensions_dataset.update({"panel_bearing": [0.09, 0.116, 0.012]})
      self.dimensions_dataset.update({"panel_motor"  : [0.06, 0.06, 0.012]})
    elif self.assembly_database.db_name in ["wrs_assembly_2020", "wrs_assembly_2019_surprise"] :
      self.dimensions_dataset.update({"panel_bearing": [0.09, 0.116, 0.012]})
      self.dimensions_dataset.update({"panel_motor"  : [0.06, 0.07, 0.012]})
    else:
      pass
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
  @lock_vision
  def get_large_item_position_from_top(self, item_name, robot_name="b_bot", skip_moving=False):
    """
    This function look at the tray from the top only, and publishes the result to the planning scene.
    
    Returns False if object was not found, the pose if it was.
    """
    tray_view_high, close_tray_views = self.define_local_tray_views(low_height=.3, robot_name=robot_name)
    # Look from top first
    self.vision.activate_camera(robot_name+"_outside_camera")
    if not skip_moving:
      self.active_robots[robot_name].go_to_named_pose("above_tray", speed=1.0)
      view_poses = [tray_view_high] + close_tray_views
      for pose in view_poses:
        if not self.active_robots[robot_name].go_to_pose_goal(pose, end_effector_link=robot_name+"_outside_camera_color_frame", move_lin=True, retry_non_linear=True):
          rospy.logerr("Failed to move to camera view pose in get_large_item_position_from_top.")
          return False
        object_pose = self.detect_object_in_camera_view(item_name)
        if object_pose:
          break
    else:
      object_pose = self.detect_object_in_camera_view(item_name)

    if object_pose:
      rospy.loginfo("Found " + item_name + ". Publishing to planning scene.")
      # TODO: Apply Place action of pose estimation to constrain object to the tray_center plane
      # Workaround: Just set z to 0 
      object_pose.header.stamp = rospy.Time.now()
      try:
        self.listener.waitForTransform("tray_center", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(1))
      except:
        pass
      object_pose = self.listener.transformPose("tray_center", object_pose)
      if item_name in ("panel_motor", "panel_bearing", "base"):
        #### fake the orientation of the plate (assumes that it is always flat on the tray) ###
        orientation_euler = list(transformations.euler_from_quaternion(conversions.from_pose_to_list(object_pose.pose)[3:]))
        if item_name == "base":
          orientation_euler[0] = np.sign(orientation_euler[0]) * tau/4
        else:
          orientation_euler[0] = 0
        orientation_euler[1] = 0
        object_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(*orientation_euler))
        object_pose.pose.position.z = 0.0

        obj = self.assembly_database.get_collision_object(item_name)
        obj.header.frame_id = object_pose.header.frame_id
        obj.pose = object_pose.pose
        self.planning_scene_interface.add_object(obj)
        self.planning_scene_interface.allow_collisions(item_name)
        if self.use_gazebo_sim:
          # Spawn the part in gazebo
          print("spawn to gazebo")
          name = "panel_bearing"
          gazebo_pose = self.listener.transformPose("world", object_pose)
          models = [Model(name, pose=gazebo_pose.pose, reference_frame=gazebo_pose.header.frame_id, file_type="sdf")]
          print(models[0].pose)
          self.gazebo_scene.load_models(models)
        rospy.sleep(0.5)
        self.constrain_into_tray(item_name)
      return object_pose
    
    return False

  def update_collision_item_pose(self, item_name, pose_stamped):
      obj = self.assembly_database.get_collision_object(item_name)
      obj.header.frame_id = pose_stamped.header.frame_id
      obj.pose = pose_stamped.pose
      self.planning_scene_interface.add_object(obj)

  @lock_vision
  def look_and_get_object_pose(self, object_id, robot_name="b_bot"):
    """
    Looks at the tray from above and gets grasp points of items.
    Does very light feasibility check before returning.
    """

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
    self.active_robots[robot_name].go_to_named_pose("above_tray", speed=1.0)

    if object_id in self.objects_in_tray:
      del self.objects_in_tray[object_id]

    if self.use_dummy_vision or not self.use_real_robot:
      rospy.logwarn("Using dummy vision! Setting object pose to tray center.")
      self.objects_in_tray[object_id] = conversions.to_pose_stamped("tray_center", [0.01, 0.01, 0.02] + np.deg2rad([0,90.,0]).tolist())
      return self.objects_in_tray[object_id]

    tray_view_high, close_tray_views = self.define_local_tray_views(robot_name=robot_name, include_rotated_views=True)
    tray_views = [tray_view_high] + close_tray_views

    for view in tray_views:
      assert not rospy.is_shutdown()
      self.vision.activate_camera(robot_name + "_outside_camera")
      self.active_robots[robot_name].go_to_pose_goal(view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.5, acceleration=.3, wait=True, move_lin=True)
      rospy.sleep(0.5)
      
      tries = 5
      while tries > 0:
        if self.get_3d_poses_from_ssd():
          break
        rospy.sleep(0.3)
        tries -= 1

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
        self.vision.activate_camera(robot_name + "_outside_camera")
        self.active_robots[robot_name].go_to_pose_goal(close_view, end_effector_link=robot_name + "_outside_camera_color_frame", speed=.3, acceleration=.3)
        rospy.sleep(0.5)
        tries = 5
        while tries > 0:
          if self.get_3d_poses_from_ssd():
            break
          rospy.sleep(0.3)
          tries -= 1
        
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
    center_on_close_border = options.get("center_on_close_border", False)
    grab_and_drop          = options.get("grab_and_drop", False)
    declutter_with_tool    = options.get("declutter_with_tool", False)
    options.update({"robot_name": robot_name})

    object_pose = self.look_and_get_object_pose(object_id, robot_name)

    options.update({"rotation_offset": -1 if robot_name == "b_bot" else 1})
    if object_pose:
      rospy.loginfo("Object found: checking for feasible grasps")
      grasps = self.get_feasible_grasp_points(object_id, object_pose=object_pose, options=options)
      rospy.loginfo("grasps found? %s" % bool(grasps))
      if grasps:
        if grasps == CORNER and center_on_corner:
          rospy.loginfo("=== Pick fallback: Corner ===")
          if not self.move_towards_tray_center_from_corner(robot_name, object_pose, options):
            rospy.logerr("Fail to move_towards_tray_center_from_corner")
            return False
          options.update({"center_on_corner": False})
          return self.look_and_get_grasp_point(object_id, robot_name, options)
        elif grasps == TOO_CLOSE_TO_BORDER and center_on_close_border:
          rospy.loginfo("=== Pick fallback: Moving away from border===")
          if not self.move_towards_center_from_border(robot_name, object_pose, options):
            rospy.logerr("Fail to move_towards_center_from_border")
            return False
          options.update({"center_on_close_border": False})
          return self.look_and_get_grasp_point(object_id, robot_name, options)
        elif grasps == TOO_CLOSE_TO_OTHER_OBJECTS:
          rospy.loginfo("=== Pick fallback: Too close to other object===")
          fallback_methods = []
          if grab_and_drop:
            fallback_methods.append("grab_and_drop")
          if declutter_with_tool:
            fallback_methods.append("declutter_with_tool")

          fallback = random.choice(fallback_methods)
          if fallback == "grab_and_drop":
            rospy.loginfo("=== Pick fallback: grab and drop===")
            if not self.grab_and_drop(robot_name, object_pose, options):
              rospy.logerr("Fail to grab and drop")
              return False
          elif fallback == "declutter_with_tool":
            rospy.loginfo("=== Pick fallback: declutter with tool===")
            if not self.declutter_with_tool(robot_name, object_pose):
              rospy.logerr("Fail to declutter_with_tool")
              return False
          else:
            return False
          options.update({"grab_and_drop": True})
          options.update({"declutter_with_tool": False})
          options.update({'center_on_corner': True})
          return self.look_and_get_grasp_point(object_id, robot_name, options) # May end in infinite loop?
        else:
          return random.choice(grasps)
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
      axes = [2, 0]  # x, z
    if item_name == "panel_motor" or item_name == "panel_bearing":
      axes = [0, 1]  # x, y
    corner_points = [ [dims[0], dims[2]], [dims[0], dims[3]], [dims[1], dims[2]], [dims[1], dims[3]] ]
    for i, point in enumerate(corner_points):
      object_corner_list = conversions.from_pose_to_list(object_pose.pose)
      object_corner_list[axes[0]] += point[0]
      object_corner_list[axes[1]] += point[1]
      object_corner_pose = conversions.to_pose_stamped(object_pose.header.frame_id, object_corner_list)

      dx, dy = self.distances_from_tray_border(object_corner_pose, tray_length=0.365, tray_width=0.245) # workaround for slight different with real world tray
      # print("corner:",i, "dx, dy", dx, dy)
      if dx < 0 or dy < 0:
        if dx < 0:
          object_pose_in_tray.pose.position.x -= np.sign(object_pose_in_tray.pose.position.x) * abs(dx)
          rospy.loginfo("Moved dx: %s" % dx)
        if dy < 0:
          rospy.loginfo("Moved dy: %s" % dy)
          object_pose_in_tray.pose.position.y -= np.sign(object_pose_in_tray.pose.position.y) * abs(dy)
        try:
          self.listener.waitForTransform("move_group/"+item_name, object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(1))
          object_pose = self.listener.transformPose("move_group/"+item_name, object_pose_in_tray)
        except Exception as e:
          print(e)
          pass

    # Update collision object position
    obj = self.assembly_database.get_collision_object(item_name)
    obj.header.frame_id = object_pose_in_tray.header.frame_id
    obj.pose = object_pose_in_tray.pose
    self.planning_scene_interface.add_object(obj)
    rospy.sleep(0.1)

  def constrain_grasp_into_tray(self, robot_name, grasp_pose, grasp_width=0.06, object_width=0.0):
    """
      Check that the grasp pose + the gripper opening + the gripper finger width is within the tray.
      If the distance between the original grasp pose and the tray is less than the gripper finger with, return original pose, success=False
      Otherwise return the grasp pose updated where the gripper finger is barely inside the tray.
    """
    gripper_finger_width = 0.02 if robot_name == "b_bot" else 0.015
    dx, dy = self.distances_from_tray_border(grasp_pose)
    # The gripper's finger and half the object width must be available to make the grasp possible (assuming the grasp point is in the middle of the object)
    if dx < gripper_finger_width+object_width/2 or dy < gripper_finger_width+object_width/2:
      rospy.logwarn("Grasp pose too close to border, cannot correct.")
      return grasp_pose, False

    # print("grasp pose \n", grasp_pose.pose.position)
    grasp_pose_with_gripper_offset = self.listener.transformPose("tray_center", grasp_pose)
    corrected_grasp_pose = self.listener.transformPose("tray_center", grasp_pose)
    grasp_pose_with_gripper_offset.pose.position.x += np.sign(grasp_pose.pose.position.x) * (grasp_width/2 + gripper_finger_width)
    grasp_pose_with_gripper_offset.pose.position.y += np.sign(grasp_pose.pose.position.y) * (grasp_width/2 + gripper_finger_width)
    # print("grasp pose with gripper opening \n", grasp_pose_with_gripper_offset.pose.position)
    dx, dy = self.distances_from_tray_border(grasp_pose_with_gripper_offset)
    
    # print("distance with gripper correction \n", dx, dy)
    if dx < 0:
      corrected_grasp_pose.pose.position.x += np.sign(grasp_pose.pose.position.x) * dx
    if dy < 0:
      corrected_grasp_pose.pose.position.y += np.sign(grasp_pose.pose.position.y) * dy
    # print("grasp pose corrected in tray_center \n", corrected_grasp_pose.pose.position)
    result_pose = self.listener.transformPose(grasp_pose.header.frame_id, corrected_grasp_pose)
    # print("grasp pose corrected in original frame \n", result_pose.pose.position)
    return result_pose, True

  def get_transformed_grasp_pose(self, object_name, grasp_name, target_frame="tray_center"):
    """ Get an object's grasp pose in the target_frame"""
    grasp_pose = self.assembly_database.get_grasp_pose(object_name, grasp_name)
    return self.get_transformed_collision_object_pose(object_name, grasp_pose, target_frame)
  
  def get_transformed_collision_object_pose(self, object_name, object_pose=None, target_frame="tray_center"):
    """ Get the pose of a MoveIt CollisionObject in the target_frame"""
    obj_pose = object_pose if object_pose else conversions.to_pose_stamped("move_group/" + object_name, [0,0,0,0,0,0])
    tries = 0
    while tries < 10:
      try:
        obj_pose.header.frame_id = "move_group/" + object_name
        obj_pose.header.stamp = rospy.Time.now()
        self.listener.waitForTransform(target_frame, obj_pose.header.frame_id, obj_pose.header.stamp, rospy.Duration(1))
        return self.listener.transformPose(target_frame, obj_pose)
      except Exception as e:
        rospy.logwarn(e)
        tries =+ 1
    return False

  ########

  def simple_pick(self, robot_name, object_pose, grasp_height=0.0, speed_fast=1.0, speed_slow=0.4, gripper_command="close", 
          gripper_force=40.0, grasp_width=0.140, minimum_grasp_width=0.0,
          approach_height=0.05, item_id_to_attach = "", 
          lift_up_after_pick=True, acc_fast=1.0, acc_slow=.1, 
          gripper_velocity = .1, axis="x", sign=+1,
          retreat_height = None, approach_with_move_lin=True, 
          attach_with_collisions=False, pose_with_uncertainty=None, object_name="", pick_from_ground=True,
          allow_collision_with_tray=False):
    """
    This function (outdated) performs a grasp with the robot, but it is not updated in the planning scene.
    It does not use the object in simulation. It can be used for simple tests and prototyping, but should
    be replaced by the pick() function for the real competition.

    item_id_to_attach is used to attach the item to the robot at the target pick pose. It is ignored if empty.
    The attachment will be visible in the MoveIt planning scene. The object and its subframes can be used
    as an end effector.

    attach_with_collisions use the CollisionObject otherwise try to attach the visualization Marker
    """
    if allow_collision_with_tray:
      self.allow_collisions_with_robot_hand("tray", robot_name)
    rospy.loginfo("Entered simple_pick")
    if item_id_to_attach:
      self.allow_collisions_with_robot_hand(item_id_to_attach, robot_name)
    seq = []

    robot = self.active_robots[robot_name]
    if gripper_command=="do_nothing":
      pass
    else: 
      seq.append(helpers.to_sequence_gripper("open", gripper_opening_width=grasp_width, gripper_velocity=1.0, wait=False))

    approach_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += approach_height * sign
    approach_pose.pose.position = conversions.to_point(op)

    seq.append(helpers.to_sequence_item(approach_pose, speed=speed_fast, acc=0.7, linear=approach_with_move_lin))

    rospy.logdebug("Going to height " + str(op[get_direction_index(axis)]))

    rospy.logdebug("Moving down to object")
    grasp_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += grasp_height * sign
    grasp_pose.pose.position = conversions.to_point(op)
    rospy.logdebug("Going to height " + str(op[get_direction_index(axis)]))

    seq.append(helpers.to_sequence_item(grasp_pose, speed=speed_fast))

    if gripper_command=="do_nothing":
      pass
    else: 
      def post_cb():
        if item_id_to_attach:
          robot.gripper.attach_object(object_to_attach=item_id_to_attach, with_collisions=attach_with_collisions)
      seq.append(helpers.to_sequence_gripper("close", gripper_velocity=gripper_velocity, gripper_force=gripper_force, post_callback=post_cb))

    # # break seq here
    if not self.execute_sequence(robot_name, seq, "simple_pick"):
      rospy.logerr("Fail to simple pick with sequence")
      if allow_collision_with_tray:
        self.allow_collisions_with_robot_hand("tray", robot_name, False)
      return False

    if pose_with_uncertainty != None:
      tip_link = robot_name + "_gripper_tip_link"
      collision_object=self.assembly_database.get_collision_object(object_name)
      if pick_from_ground:
        now=rospy.Time.now()
        update_goal=o2ac_msgs.msg.updateDistributionGoal()
        update_goal.observation_type=update_goal.GRASP_OBSERVATION
        update_goal.gripped_object=collision_object
        self.listener.waitForTransform("world", tip_link, now, rospy.Duration(1.0))
        update_goal.gripper_pose.pose= tf_conversions.posemath.toMsg(tf_conversions.posemath.fromTf(self.listener.lookupTransform("world", tip_link, now)))
        update_goal.gripper_pose.header.frame_id="world"
        update_goal.gripper_pose.header.stamp= now
        update_goal.distribution_type=1
        update_goal.distribution=self.transform_pose_with_uncertainty(tip_link, pose_with_uncertainty, now=now)
        self.update_distribution_client.send_goal(update_goal)
        self.update_distribution_client.wait_for_result()
        update_result = self.update_distribution_client.get_result()
        pose_with_uncertainty.header=update_result.distribution.header
        pose_with_uncertainty.pose=update_result.distribution.pose
      else:
        transformed_pose=self.transform_pose_with_uncertainty(tip_link, pose_with_uncertainty)
        pose_with_uncertainty.header=transformed_pose.header
        pose_with_uncertainty.pose=transformed_pose.pose

      self.visualize_object_with_distribution(collision_object, pose_with_uncertainty, frame_locked=True)

    success = True
    if minimum_grasp_width > robot.gripper.opening_width and self.use_real_robot:
      rospy.logerr("Gripper opening width after pick less than minimum (" + str(minimum_grasp_width) + "): " + str(robot.gripper.opening_width) + ". Return False.")
      robot.gripper.open(opening_width=grasp_width)
      robot.gripper.forget_attached_item()
      if allow_collision_with_tray:
        self.allow_collisions_with_robot_hand("tray", robot_name, False)
      success = False

    if lift_up_after_pick:
      rospy.logdebug("Going back up")
      if retreat_height is None:
        retreat_height = approach_height
      retreat_pose = copy.deepcopy(object_pose)
      op = conversions.from_point(object_pose.pose.position)
      op[get_direction_index(axis)] += retreat_height * sign
      retreat_pose.pose.position = conversions.to_point(op)
      rospy.logdebug("Going to height " + str(retreat_pose.pose.position.z))
      if not robot.go_to_pose_goal(retreat_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True):
        rospy.logerr("Fail to go to lift_up_pose. Opening.")
        robot.gripper.open(grasp_width)
        if allow_collision_with_tray:
          self.allow_collisions_with_robot_hand("tray", robot_name, False)
        return False

    if allow_collision_with_tray:
        self.allow_collisions_with_robot_hand("tray", robot_name, False)
    return success

  def simple_place(self, robot_name, object_pose, place_height=0.05, speed_fast=1.0, speed_slow=0.3, 
                         gripper_command="open", gripper_opening_width=0.14, approach_height=0.05, axis="x", sign=+1,
                         item_id_to_detach = "", lift_up_after_place = True, acc_fast=0.6, acc_slow=0.15):
    """
    A very simple place operation. item_id_to_detach is used to update the planning scene by
    removing an item that has been attached (=grasped) by the robot in the MoveIt planning scene.
    It is ignored if empty.
    """

    seq = []

    rospy.loginfo("Going above place target")
    approach_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += approach_height * sign
    approach_pose.pose.position = conversions.to_point(op)
    seq.append(helpers.to_sequence_item(approach_pose, speed=speed_fast, acc=acc_fast, linear=False))
    # if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=speed_fast, acceleration=acc_fast, move_lin=False):
    #   rospy.logerr("fail to go to approach pose")
    #   return False
   
    rospy.loginfo("Moving to place target")
    place_pose = copy.deepcopy(object_pose)
    op = conversions.from_point(object_pose.pose.position)
    op[get_direction_index(axis)] += place_height * sign
    place_pose.pose.position = conversions.to_point(op)
    seq.append(helpers.to_sequence_item(place_pose, speed=speed_slow, acc=acc_slow, linear=False))
    # if not self.active_robots[robot_name].go_to_pose_goal(place_pose, speed=speed_slow, acceleration=acc_slow, move_lin=True):
    #   rospy.logerr("fail to go to place pose")
    #   return False
    
    robot = self.active_robots[robot_name]

    #gripper open
    if gripper_command=="do_nothing":
      pass
    else:
      seq.append(helpers.to_sequence_gripper('open', gripper_velocity=1.0, gripper_opening_width=gripper_opening_width))
      # robot.gripper.open()

    if item_id_to_detach:
      robot.robot_group.detach_object(item_id_to_detach)

    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      seq.append(helpers.to_sequence_item(approach_pose, speed=speed_fast, acc=acc_fast, linear=False))
      # if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=speed_fast, move_lin=False):
      #   rospy.logerr("fail to go to retrieve pose")
      #   return False
    
    if not self.execute_sequence(robot_name, seq, "simple place"):
      rospy.logerr("fail to simple place")
      return False

    return True

  def simple_grasp_generation(self, object_pose, options, object_id=-1):
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
    grasp_z_height  = options.get("grasp_z_height", 0.02)
    rotation_offset = options.get("rotation_offset", 1)
    object_width    = options.get("object_width", 0.01)
    grasp_width     = options.get("grasp_width",  0.06)
    robot_name      = options.get("robot_name",  "b_bot")

    # This grasp pose opens the gripper along the workspace_center's y-axis
    grasp_along_y = copy.deepcopy(object_pose)
    grasp_along_y.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, 0))
    grasp_along_y.pose.position.z = grasp_z_height

    # This one opens the gripper along the workspace_center's x-axis
    grasp_along_x = copy.deepcopy(grasp_along_y)
    grasp_along_x.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, rotation_offset*tau/4))
    grasps_solutions = [] # valid grasps based on distance to tray's border and closer items
    
    safe_conditions = self.grasp_sanity_check(object_pose, options, object_id=object_id)

    if isinstance(safe_conditions, list):
      if TOO_CLOSE_TO_BORDER in safe_conditions:
        rospy.logwarn("Too close to a border, returning a safe grasp pose")
        grasp = grasp_along_y if safe_conditions[0] == Y_BORDER_SAFE else grasp_along_x
        grasp, is_corrected = self.constrain_grasp_into_tray(robot_name, grasp, grasp_width=grasp_width, object_width=object_width)
        if not is_corrected:
          return TOO_CLOSE_TO_BORDER
        # print("grasp pose pre modification")
        # print(grasp.pose)
        # grasp.pose.position.x = grasp.pose.position.x if abs(grasp.pose.position.x) < 0.11 else np.sign(grasp.pose.position.x) * 0.11 
        # grasp.pose.position.y = grasp.pose.position.y if abs(grasp.pose.position.y) < 0.17 else np.sign(grasp.pose.position.y) * 0.17
        # print("grasp pose post modification")
        # print(grasp.pose)
        
        grasps_solutions.append(grasp)
        return grasps_solutions
        
      for sc in safe_conditions:
        grasp = grasp_along_y if sc == Y_BORDER_SAFE else grasp_along_x
        grasps_solutions.append(grasp)
      return grasps_solutions
    return safe_conditions

  def grasp_sanity_check(self, object_pose, options, object_id=-1):
    """
      dist_close: float, min distance allowed in the direction that the gripper does not open
    """  
    check_for_close_items     = options.get("check_for_close_items", True)
    check_too_close_to_border = options.get("check_too_close_to_border", False)
    min_dist_to_border        = options.get("min_dist_to_border", 0.02)
    grasp_width               = options.get("grasp_width", 0.06)
    object_width              = options.get("object_width", 0.01)
    robot_name                = options.get("robot_name",  "b_bot")

    object_pose_corrected, is_corrected = self.constrain_grasp_into_tray(robot_name, object_pose, grasp_width=grasp_width, object_width=object_width)
    safe_conditions = [] # relative to tray's border or other items proximity
    solutions = []

    dist_far = (grasp_width+0.02) / 2.0  # Along the gripper's opening direction
    if is_corrected:
      object_pose = object_pose_corrected
      safe_conditions.append(Y_BORDER_SAFE)
      safe_conditions.append(X_BORDER_SAFE)
    else:
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
    dx, dy = self.distances_from_tray_border(object_pose)
    if check_for_close_items:
      for condition in safe_conditions:
        item_too_close = False
        for obj, pose in self.objects_in_tray.items():
          if obj == object_id: # skip same object id 
            continue
          if obj == 6: # Skip the belt
            continue
          if obj == -1: # For undefined object id
            dx = abs(pose.pose.position.x - object_pose.pose.position.x)
            dy = abs(pose.pose.position.y - object_pose.pose.position.y)
            if dx < 1e-3 and dy < 1e-3:
              continue  # It's the item itself or a duplicate
          elif condition == Y_BORDER_SAFE:
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
  
  def pick(self, robot_name, object_name, grasp_name="", speed=0.1):
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
    
  def pick_MTC_helper(self, robot_name, object_name, grasp_poses, speed=0.1):
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

  def too_close_to_border(self, grasp_pose, border_dist=0.06, verbose=False):
    """ Returns true if the point is too close to the border of the tray (= too far away from the tray center).
    """
    (dx, dy) = self.distances_from_tray_border(grasp_pose)
    if verbose:
      rospy.loginfo("distance to tray dx: %s, dy: %s" % (dx, dy))
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
      tries = 5
      while tries > 0:
        res = self.get_3d_poses_from_ssd()
        if res:
          break
        rospy.sleep(1)
        tries -= 1
      if not res:
        return False
      grasp_poses = []
      for idx, pose in enumerate(res.poses):
        if res.class_ids[idx] == 6:
          if self.is_grasp_pose_feasible(pose, border_dist=0.05):
            grasp_poses.append(pose)
      grasp_poses = self.thin_out_grasp_poses(grasp_poses)
      return grasp_poses

    object_ps = object_pose if object_pose is not None else self.objects_in_tray[object_id]
    grasp_width = options.get("grasp_width", 0.06)
    object_width = options.get("object_width", 0.0)
    object_ps, _ = self.constrain_grasp_into_tray("b_bot", object_ps, grasp_width=grasp_width, object_width=object_width)


    if object_id in self.small_item_ids:
      # For the shaft, use the orientation from the SSD
      if self.assembly_database.id_to_name(object_id) == "shaft":
        object_ps, _ = self.constrain_grasp_into_tray("b_bot", object_ps, grasp_width=grasp_width, object_width=0.01)
        res = self.grasp_sanity_check(object_ps, options, object_id=object_id)
        print("shaft >>> sanity check", res)
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
      return self.simple_grasp_generation(object_pose=object_ps, options=options, object_id=object_id)
      # TODO: Consider the idler spacer, which can stand upright or lie on the side.
      
    if object_id in self.large_item_ids:
      # For large items, use any pose from above that works
      # TODO: Get grasp poses from database
      return self.simple_grasp_generation(object_pose=object_ps, options=options, object_id=object_id)
    
    return [None]

  def distances_from_tray_border(self, object_pose, tray_width=0.255, tray_length=0.375):
    """
    Returns the distance from the tray border as an (x, y) tuple.
    x, y are in the tray coordinate system.
    Distance is signed (negative is outside the tray).
    """
    # Inside tray width and length: 25.5 cm, 37.5 cm
    l_x_half = tray_width/2.0
    l_y_half = tray_length/2.0
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

  def move_towards_center_with_tool(self, robot_name, target_pose, distance=0.10, direction=None, start_with_spiral=False):
    if robot_name == "a_bot":
      self.a_bot.go_to_named_pose("home")
    
    self.allow_collisions_with_robot_hand("plunger_tool_link", "b_bot")

    robot_name == "b_bot"
    robot = self.active_robots[robot_name]

    if not self.playback_sequence("plunger_tool_equip"):
      rospy.logerr("Fail to equip tool")
      return False

    safe_approach_pose = copy.deepcopy(target_pose)
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

    touch_pose = copy.deepcopy(target_pose)
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

    if not self.move_towards_tray_center(robot_name, distance=distance, go_back_halfway=False, one_direction=direction, speed=0.05, acc=0.05, end_effector_link="b_bot_plunger_tip_link"):
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
    self.allow_collisions_with_robot_hand("plunger_tool_link", "b_bot", False)

    return True

  def move_towards_center_from_border_with_tool(self, robot_name, object_pose, start_with_spiral=False, distance=0.1):
    border_pose = copy.deepcopy(object_pose)
    (dx, dy) = self.distances_from_tray_border(object_pose)
    direction = None
    if dx < dy: # Use the close border
      direction = 'x'
      border_pose.pose.position.x = 0.12 if np.sign(border_pose.pose.position.x) == 1 else -0.133 # non-symmetric tray_center
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-tau/4, tau/4+np.sign(border_pose.pose.position.x)*radians(15), -tau/2))
    else:
      direction = 'y'
      border_pose.pose.position.y = 0.186 if np.sign(border_pose.pose.position.y) == 1 else -0.193 # non-symmetric tray_center
      border_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, tau/4-np.sign(border_pose.pose.position.y)*radians(15), tau/4))

    return self.move_towards_center_with_tool(robot_name, border_pose, direction=direction, start_with_spiral=start_with_spiral, distance=distance)

  def move_towards_center_from_border(self, robot_name, object_pose, options):
    with_tool = options.get('with_tool', False)
    go_back_halfway = options.get('go_back_halfway', True)

    if with_tool:
      rospy.loginfo("=== Pick fallback: Moving away from border with tool===")
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
    if robot_name == "b_bot":
      corner_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/6))
    else:
      corner_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4,  tau/6))

    return self.move_towards_tray_center_with_push(robot_name, corner_pose, approach_height, go_back_halfway=go_back_halfway)

  def move_towards_tray_center_with_push(self, robot_name, start_pose, approach_height, direction=None, go_back_halfway=True, distance=0.1):
    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=True)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=True)

    robot = self.active_robots[robot_name]

    approach_pose = copy.deepcopy(start_pose)
    approach_pose.pose.position.z = 0.06
    robot.gripper.close()

    if not robot.go_to_pose_goal(approach_pose):
      rospy.logerr("Fail to approach 1")
      return False

    approach_pose.pose.position.z = approach_height
    if not robot.go_to_pose_goal(approach_pose, speed=0.05):
      rospy.logerr("Fail to approach 2")
      return False

    robot.linear_push(force=10, direction="-Z", max_translation=0.06)

    if not self.move_towards_tray_center(robot_name, distance=distance, one_direction=direction, go_back_halfway=go_back_halfway):
      rospy.logerr("Fail to move towards center")

    self.allow_collisions_with_robot_hand("tray_center", robot_name, allow=True)
    self.allow_collisions_with_robot_hand("tray", robot_name, allow=True)

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
  
  def center_with_gripper(self, robot_name, opening_width, gripper_force=40, gripper_velocity=0.01, 
                                required_width_when_closed=0.0, clockwise=False, 
                                move_back_to_initial_position=True):
    """
    Centers cylindrical object at the current location, by closing/opening the gripper and rotating the robot's last joint.

    If required_width_when_closed is set, the function returns False when the gripper closes and detects a smaller width.
    """
    robot = self.active_robots[robot_name]
    robot.gripper.close(force=gripper_force, velocity=gripper_velocity)
    if required_width_when_closed:
      if not self.simple_gripper_check(robot_name, min_opening_width=required_width_when_closed):
        robot.gripper.send_command(command=opening_width, force=gripper_force, velocity=gripper_velocity)
        return False
    robot.gripper.send_command(command=opening_width, force=gripper_force, velocity=gripper_velocity)

    # rotate gripper 90deg
    initial_pose = robot.get_current_pose_stamped()
    offset = -tau/4.0 if clockwise else tau/4.0
    success = robot.move_lin_rel(relative_rotation=[offset, 0, 0], speed=1.0, relative_to_tcp=True, plan_only=True)
    if not success:
      rospy.logerr("Fail to plan - rotate 90deg %s. Retry with -90deg" % robot_name)
      if not robot.move_lin_rel(relative_rotation=[-offset, 0, 0], speed=1.0, relative_to_tcp=True):
        return False
    plan, _ = success
    plan = robot.robot_group.retime_trajectory(robot.robot_group.get_current_state(), plan, algorithm="time_optimal_trajectory_generation",
                                                  velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0)
    if not robot.execute_plan(plan):
      rospy.logerr("Fail to execute centering")
      return False

    # close-open
    robot.gripper.close(force=gripper_force, velocity=gripper_velocity)
    if required_width_when_closed:
      if not self.simple_gripper_check(robot_name, min_opening_width=required_width_when_closed):
        robot.gripper.send_command(command=opening_width, force=gripper_force, velocity=gripper_velocity)
        if move_back_to_initial_position:
          robot.go_to_pose_goal(initial_pose, speed=1.0)
        return False
    robot.gripper.send_command(command=opening_width, force=gripper_force, velocity=gripper_velocity)

    # rotate gripper -90deg
    if move_back_to_initial_position:
      plan, _ = robot.go_to_pose_goal(initial_pose, speed=1.0, plan_only=True)
      plan = robot.robot_group.retime_trajectory(robot.robot_group.get_current_state(), plan, algorithm="time_optimal_trajectory_generation",
                                                    velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0)
      if not robot.execute_plan(plan):
        rospy.logerr("Fail to go back after centering")
        return False
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

  @lock_vision
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

  @lock_vision
  def check_screw_hole_visible_on_shaft_in_v_groove(self):
    """
    Looks at the end of the shaft and returns True if 
    """

    look_at_shaft_end_pose = conversions.to_pose_stamped("vgroove_aid_link", [-0.012, 0.144, 0.066]+np.deg2rad([110.550, 0.030, -0.129]).tolist())
    self.vision.activate_camera("b_bot_inside_camera")
    self.activate_led("b_bot")
    
    self.b_bot.go_to_pose_goal(look_at_shaft_end_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=0.15, move_lin=True)

    if not self.use_real_robot:
      return True

    res = self.vision.call_shaft_hole_detection()
    if res:
      print("=== shaft screw_hole detection returned:")
      print(res)
      return res.has_hole
    raise ValueError("Fail to call shaft hole detection")

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

  @lock_vision
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
    rospy.loginfo("==== Fallback grab and drop (%s) ====" % robot_name)
    grasp_width_ = options.get('grasp_width', 0.08)
    grasp_width = grasp_width_ + 0.02 # extra opening of gripper to try to catch multiple objects, not too much to avoid grasping too many?
    options_ = {'grasp_z_height': 0.002, 'grasp_width': grasp_width, 'check_for_close_items': False}
    if options.get('use_grasp_pose_directly_in_simple_pick', False):
      grasp_pose = object_pose
    else:
      grasp_pose = random.choice(self.simple_grasp_generation(object_pose, options_))
    
    grasp_pose.pose.position.z = 0.002
    robot = self.active_robots[robot_name]
    robot.gripper.open(opening_width=grasp_width)
    grasp_pose, _  = self.constrain_grasp_into_tray(robot_name, grasp_pose, grasp_width=0.08)
    success = self.simple_pick(robot_name, grasp_pose, approach_height=0.05, lift_up_after_pick=True, axis="z", 
                               approach_with_move_lin=False, allow_collision_with_tray=True)
    drop_pose = self.listener.transformPose("tray_center", grasp_pose)
    drop_pose.pose.position.x = np.random.uniform(low=-0.05, high=0.05)
    drop_pose.pose.position.y = np.random.uniform(low=-0.05, high=0.05)
    robot.go_to_pose_goal(drop_pose)
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
    drop_pose = self.listener.transformPose("tray_center", tray_center_pose)
    drop_pose.pose.position.x = np.random.uniform(low=-0.08, high=0.08)
    drop_pose.pose.position.y = np.random.uniform(low=-0.08, high=0.08)
    
    if not robot.go_to_pose_goal(above_tray):
      rospy.logerr("fail to go to above_tray (drop_in_tray)")
      return False

    if not robot.go_to_pose_goal(drop_pose):
      rospy.logerr("fail to go to drop_pose (drop_in_tray)")
      return False

    robot.gripper.open() 
    return True

  @check_for_real_robot
  def simple_gripper_check(self, robot_name, min_opening_width=0.001, velocity=1.0):
    self.active_robots[robot_name].gripper.close(velocity=velocity) # confirm that there is something grasped
    if robot_name == "a_bot":
      min_opening_width += 0.005 # a_bot gripper is not very precise...
    success = self.active_robots[robot_name].gripper.opening_width > min_opening_width
    if not success:
      rospy.logerr("Fail to grasp. opening_width: %s" % self.active_robots[robot_name].gripper.opening_width)
      rospy.logerr("min_opening_width: %s" % min_opening_width)
    return success

  ######## Bearing

  def rotate_cylinder_by_angle(self, angle, robot_name, grasp_pose, 
                               grasp_width=0.085, ignore_collisions_with=""):
    """ From a given grasp pose, rotates relative to world frame x axis """
    if ignore_collisions_with:
      self.allow_collisions_with_robot_hand(ignore_collisions_with, robot_name, True)
    seq = []
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=grasp_width, gripper_velocity=1.0, wait=False))
    seq.append(helpers.to_sequence_item(grasp_pose, speed=0.2))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0, -angle/2.0, 0, 0], relative_to_tcp=False, speed=0.2))
    seq.append(helpers.to_sequence_gripper('close', gripper_force=40, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0, angle, 0, 0], relative_to_tcp=False, speed=0.2))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=grasp_width, gripper_velocity=1.0))
    self.execute_sequence(robot_name, seq, "rotate cylinder by angle")
    if ignore_collisions_with:
      self.allow_collisions_with_robot_hand(ignore_collisions_with, robot_name, False)

  def pick_and_fasten_screw(self, robot_name, screw_pose, screw_size, 
                             approach_distance=0.07, intermediate_pose=None, speed=0.7, 
                             duration=20.0, attempts=1, spiral_radius=0.003, 
                             save_plan_on_success=True, screw_set_center_pose=None, 
                             saved_plan=None, skip_initial_motion=False):
    """Returns bool, screw success"""
    fastening_tool_name = "screw_tool_m" + str(screw_size)
    if not self.active_robots[robot_name].robot_status.held_tool_id == fastening_tool_name:
      rospy.loginfo("Tool already in the robot skip motion")
      skip_initial_motion = True

    if not skip_initial_motion:
      waypoints = []
      # waypoints.append(("screw_ready",      0, 1.0))
      waypoints.append(("feeder_pick_ready", 0, 1.0))
      if not self.active_robots[robot_name].move_joints_trajectory(waypoints):
        rospy.logerr("Fail to go to screw_ready>feeder_pick_ready")
        return False
    
    # Pick screw
    pick_success = self.pick_screw_from_feeder_python(robot_name, screw_size=screw_size, skip_retreat=save_plan_on_success)

    if not pick_success:
      rospy.logerr("Could not pick screw. Why?? Breaking out.")
      self.unequip_tool('b_bot', 'screw_tool_m4')
      return False

    if not screw_set_center_pose:
      screw_set_center_pose = copy.deepcopy(screw_pose)
      screw_set_center_pose.pose.position.x -= 0.04
    feeder_to_hole_plan = None
    eef = robot_name+"_screw_tool_m%s_tip_link"%screw_size
    if save_plan_on_success:
      success = False
      if saved_plan:
        success = self.active_robots[robot_name].execute_plan(saved_plan)
        if not success:
          rospy.logerr("Failed to execute saved_plan. Try to compute plan again. are we in the same initial pose?")
        feeder_to_hole_plan = saved_plan
      if not success:
        saved_plan = None
        waypoints = []
        waypoints.append(("feeder_pick_ready",      0, 1.0))
        waypoints.append(("horizontal_screw_ready", 0, 1.0))
        if intermediate_pose:
          waypoints.append((self.active_robots[robot_name].compute_ik(intermediate_pose, timeout=0.02, retry=True, end_effector_link = eef), 0, 1.0))
        waypoints.append((self.active_robots[robot_name].compute_ik(screw_set_center_pose, timeout=0.02, retry=True, end_effector_link = eef), 0, 0.3))
        res = self.active_robots[robot_name].move_joints_trajectory(waypoints, plan_only=True)
        if not res:
          rospy.logerr("Fail to plan feeder_to_hole_plan")
          return False
        else:
          feeder_to_hole_plan, _ = res
          if not self.active_robots[robot_name].execute_plan(feeder_to_hole_plan):
            rospy.logerr("Failed to execute feeder_to_hole_plan")
            return False
    else:
      waypoints = []
      waypoints.append(("screw_ready",      0, 1.0))
      waypoints.append(("horizontal_screw_ready", 0, 1.0))
      if intermediate_pose:
        waypoints.append((self.active_robots[robot_name].compute_ik(intermediate_pose, timeout=0.02, retry=True, end_effector_link = eef), 0, 1.0))
      screw_pose_approach = copy.deepcopy(screw_pose)
      screw_pose_approach.pose.position.x -= approach_distance
      waypoints.append((self.active_robots[robot_name].compute_ik(screw_pose_approach, timeout=0.02, retry=True, end_effector_link = eef), 0, 0.3))
      if not self.active_robots[robot_name].move_joints_trajectory(waypoints):
        rospy.logerr("Fail to go to screw_pose_approach")
        return False
    
    success = self.screw(robot_name, screw_pose, screw_size=screw_size, screw_height=0.02, duration=duration, 
                         skip_final_loosen_and_retighten=False, spiral_radius=spiral_radius, attempts=attempts, 
                         retry_on_failure=True, stay_put_after_screwing=save_plan_on_success)
    if success:
      if save_plan_on_success:
        return feeder_to_hole_plan
      else:
        return True
    return False

  @check_for_real_robot
  @lock_vision
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
      self.allow_collisions_with_robot_hand("panel_bearing", "b_bot", True)
      self.b_bot.gripper.open()
      start_pose = copy.deepcopy(grasp_pose)
      start_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(-angle/2.0, 0, 0))
      end_pose = copy.deepcopy(grasp_pose)
      end_pose.pose.position.z += 0.0005  # Avoid pulling the bearing out little by little
      end_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                        *tf_conversions.transformations.quaternion_from_euler(angle/2.0, 0, 0))
      self.b_bot.go_to_pose_goal(start_pose, speed=.2, end_effector_link = "b_bot_bearing_rotate_helper_link", move_lin=True)
      self.b_bot.gripper.close(velocity=0.1, wait=True)
      self.b_bot.go_to_pose_goal(end_pose, speed=.2, end_effector_link = "b_bot_bearing_rotate_helper_link", move_lin=True)
      self.b_bot.gripper.open()
      self.allow_collisions_with_robot_hand("panel_bearing", "b_bot", False)

    while adjustment_motions < max_adjustments:
      # Look at tb bearing
      if self.b_bot.gripper.opening_width < 0.06:
        self.b_bot.gripper.open()
      
      self.b_bot.go_to_pose_goal(camera_look_pose, end_effector_link="b_bot_inside_camera_color_optical_frame", speed=.3, acceleration=.15, move_lin=True)
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
      if times_it_looked_like_success > 2:
        rospy.loginfo("Bearing angle looked correct " + str(times_it_looked_like_success) + " times. Judged successful.")
        return True
      if times_perception_failed_in_a_row > 5:
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

  def pick_up_and_insert_bearing(self, task="", robot_name="b_bot"):
    """ If bearing_is_in_storage, the bearing is picked from storage and not the tray.
    """
    if not task:
      rospy.logerr("Specify the task!")
      return False
    if task == "taskboard":
      bearing_target_link = "taskboard_bearing_target_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      bearing_target_link = "assembled_part_07_inserted"

    insert_has_failed_before = False
    if self.is_bearing_in_storage:
      insert_has_failed_before = True

    if not self.pick_bearing(robot_name=robot_name):
      return False

    if not self.orient_bearing(task, robot_name):
      return False

    if not insert_has_failed_before:
      # Insert bearing
      if not self.insert_bearing(bearing_target_link, task=task, robot_name=robot_name):
        rospy.logerr("insert_bearing returned False. Breaking out")
        return False
    else:
      return self.insert_bearing_fallback(task, bearing_target_link, "a_bot")


    return True

  def pick_bearing(self, robot_name="b_bot", attempts=5):
    if robot_name == "b_bot":
      options = {'center_on_corner': True, 'check_for_close_items': False, 'grasp_width': 0.08, 'go_back_halfway': False, 'object_width': 0.06}
    else:
      options = {'center_on_corner': False, 'check_for_close_items': False, 'grasp_width': 0.06, 'go_back_halfway': False, 'object_width': 0.0}
      
    if self.is_bearing_in_storage: # FIXME only works for Taskboard with a_bot
      self.a_bot.go_to_named_pose("centering_area")
      goal = copy.deepcopy(self.bearing_store_pose)
      goal = self.listener.transformPose("tray_center", goal)
      goal.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(*np.deg2rad([35,90.,0]).tolist()))
      goal.pose.position.z = -0.015
      self.is_bearing_in_storage = False
    else:
      goal = self.look_and_get_grasp_point("bearing", robot_name, options=options)
      if not isinstance(goal, geometry_msgs.msg.PoseStamped):
        rospy.logerr("Could not see bearing in tray. Skipping procedure.")
        return False
      self.vision.activate_camera(robot_name + "_inside_camera")
      goal.pose.position.z = 0.0
      goal.pose.position.x -= 0.01 # MAGIC NUMBER
      goal.pose.position.z = 0.0115

    bearing_pose = copy.deepcopy(goal)
    bearing_pose.pose.position.z -= 0.01
    self.markers_scene.spawn_item("bearing", bearing_pose)
    # self.spawn_object("bearing", bearing_pose)
    self.planning_scene_interface.allow_collisions("bearing", "")

    # if not self.pick_from_two_poses_topdown(robot_name, "bearing", goal, grasp_width=0.07):
    if not self.simple_pick(robot_name, goal, gripper_force=50.0, grasp_width=.07, 
                            axis="z", grasp_height=0.002, item_id_to_attach="bearing", 
                            minimum_grasp_width=0.01, allow_collision_with_tray=True):
      if attempts > 0:
        rospy.logwarn("Fail to pick bearing from tray. Try again remaining attempts:%s" % attempts)
        return self.pick_bearing(robot_name, attempts-1)
      rospy.logerr("Fail to pick bearing from tray")
      return False
    return True

  def orient_bearing(self, task, robot_name="b_bot", part1=True, part2=True):
    """ Reorients the bearing in the area next to the tray so that the lip can be grasped and
        the bearing inserted. Two possible end states, according to input parameters.
        
        part1 ends after reorienting and regrasping the bearing.
        part2 ends with the bearing in front of the hole, ready to be inserted with insert_bearing.
    """
    if task == "taskboard":
      bearing_target_link = "taskboard_bearing_target_link"
    elif task == "assembly":
      bearing_target_link = "assembled_part_07_inserted"
    else:
      rospy.logerr("No task specified! Assuming assembly")
      bearing_target_link = "assembled_part_07_inserted"

    if part1:  # Reorient and grasp the bearing
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
        #'down' means that the small area contacts with tray.
      if self.active_robots[robot_name].gripper.opening_width < 0.01 and self.use_real_robot:
        rospy.logerr("Bearing not found in gripper. Must have been lost. Aborting.")
        #TODO(felixvd): Look at the regrasping/aligning area next to the tray
        return False

    if part2:  # Move in front of insertion target
      prefix = "right" if robot_name == "b_bot" else "left"
      at_tray_border_pose = conversions.to_pose_stamped(prefix + "_centering_link", [-0.2, 0, 0, -tau/4, 0, 0])

      if task=="taskboard":
        rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/4, 0, radians(35.0)]
      else:
        rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/2, radians(-35.0), 0]

      approach_pose = conversions.to_pose_stamped(bearing_target_link, [-0.10, -0.001, 0.005] + rotation)
      if task == "taskboard":
        if robot_name == "b_bot":
          preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.017,  0.000, 0.002 ]+ rotation)
        elif robot_name == "a_bot":
          preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.016, 0.015, -0.003 ]+ rotation)
        else:
          raise ValueError("Unknown robot")
      elif task == "assembly":
        preinsertion_pose = conversions.to_pose_stamped(bearing_target_link, [-0.014, 0.001, 0.011] + rotation)

      trajectory = [(self.active_robots[robot_name].compute_ik(at_tray_border_pose, timeout=0.02, retry=True), 0.01, 1.0), 
                    (self.active_robots[robot_name].compute_ik(approach_pose, timeout=0.02, retry=True), 0.02, 1.0),
                    (self.active_robots[robot_name].compute_ik(preinsertion_pose, timeout=0.02, retry=True), 0, 0.2)]
      if not self.active_robots[robot_name].move_joints_trajectory(trajectory=trajectory, speed=1.0):
        rospy.logerr("Could not go to preinsertion")
        self.pick_from_centering_area_and_drop_in_tray(robot_name)
        return False
      self.confirm_to_proceed("finetune")
      self.despawn_object("bearing")
    return True

  def fallback_recenter_bearing(self, task="", robot_name="b_bot"): 
    """ Return the inclined bearing to the centering area, recenter, and orient """
    self.active_robots[robot_name].move_lin_rel(relative_translation=[-0.02, 0, 0], relative_to_tcp=True, speed=0.1) # Slowly go back to reduce risk of emergency stop
    
    if not self.playback_sequence("bearing_fallback_recenter_" + robot_name):
      rospy.logerr("Could not complete bearing fallback recenter sequence")
      self.pick_from_centering_area_and_drop_in_tray(robot_name)
      return False

    return self.orient_bearing(task, robot_name, part1=False, part2=True)

  def insert_bearing_fallback(self, task, target_link, robot_name="a_bot"):
    """
        If first fallback don't work, try 4 additional initial poses close to the expected pose
    """
    rospy.logerr("** Insertion still Failed. Insertion with extra search area **")
    # Try to 
    if task=="taskboard":
      rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/4, 0, radians(35.0)]
    else:
      rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/2, radians(-35.0), 0]

    if robot_name == "b_bot":
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.017,  0.000, 0.002 ]+ rotation)
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.004, 0.000, 0.002, 0, 0, 0, 1.])
    else:
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.016, 0.015, -0.003 ]+ rotation)
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.003, 0.014, -0.003, 0, 0, 0, 1.])
    selection_matrix = [0., 0.2, 0.2, 1.0, 1.0, 1.0]
    
    offsets = [[0.0015, 0.0015],[-0.0015, 0.0015],[0.0015, -0.0015],[-0.0015, -0.0015]]
    robot = self.active_robots[robot_name]

    for i in range(4):
      start_pose = copy.deepcopy(pre_insertion_pose)
      start_pose.pose.position.y += offsets[i][0]
      start_pose.pose.position.z += offsets[i][1]
      
      # move back and go to new initial pose
      robot.move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
      robot.go_to_pose_goal(start_pose, speed=0.1, move_lin=True)
      # Attempt insertion at new pose
      result = robot.do_insertion(target_pose_target_frame, radius=0.005, 
                                                        insertion_direction="-X", force=8.0, timeout=20.0, 
                                                        wiggle_direction=None, wiggle_angle=np.deg2rad(0), wiggle_revolutions=1.,
                                                        relaxed_target_by=0.003, selection_matrix=selection_matrix)
      success = (result == TERMINATION_CRITERIA)

      if not success:
        # One small extra check and push if we are a bit inside the small shaft
        current_pose = self.listener.transformPose(target_link, robot.get_current_pose_stamped())
        print("current pose bearing", current_pose.pose.position.x)
        if current_pose.pose.position.x > -0.014: # approx. -0.014
          robot.gripper.open(opening_width=0.04)
          robot.gripper.close()
          robot.linear_push(force=10, direction="-X", max_translation=0.01)
          success = True # assume partial success, will try second part insertion
          break
      else:
        break

    if success:
      robot.gripper.open(opening_width=0.08, wait=True)
      robot.move_lin_rel(relative_translation = [0.012,0,0], acceleration = 0.015, speed=.03)
      robot.gripper.close(force=30, velocity=0.01, wait=True)

      result = robot.do_insertion(target_pose_target_frame, insertion_direction="-X", force=10.0, timeout=30.0, 
                                  radius=0.0, wiggle_direction="X", wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=1.0,
                                  relaxed_target_by=0.003, selection_matrix=selection_matrix)
      
      success = result in (TERMINATION_CRITERIA, DONE)

      # Go back regardless of success
      robot.gripper.open(wait=True)
      robot.move_lin_rel(relative_translation = [0.15,0,0.05], speed=.3)
      return success

    rospy.logerr("** Insertion Incomplete, dropping bearing into tray **")
    if self.use_storage_on_failure:
      robot.move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.015, speed=.03)
      self.simple_place(robot_name, self.bearing_store_pose, place_height=0.0, gripper_opening_width=0.09, axis="x", sign=-1)
      robot.gripper.forget_attached_item()
      self.is_bearing_in_storage = True
    else:
      robot.move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.015, speed=.03)
      self.drop_in_tray(robot_name)
    return False

  @lock_impedance
  def insert_bearing(self, target_link, try_recenter=True, try_reinsertion=True, 
                      robot_name="b_bot", task="assembly", attempts=2):
    """ Only inserts the bearing, does not align the holes.
    """
    robot = self.active_robots[robot_name]
    insertion_direction = "-X"
    if robot_name == "b_bot":
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.004, 0.000, 0.002, 0, 0, 0, 1.])
    else:
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.003, 0.014, -0.003, 0, 0, 0, 1.])
    selection_matrix = [0., 0.3, 0.3, .8, .8, .8]
    result = robot.do_insertion(target_pose_target_frame, insertion_direction=insertion_direction, force=10.0, timeout=20.0, 
                                radius=0.0035, relaxed_target_by=0.003, selection_matrix=selection_matrix)
    
    if result != TERMINATION_CRITERIA:
      current_pose = self.listener.transformPose(target_link, self.active_robots[robot_name].get_current_pose_stamped())
      print("current pose bearing ", current_pose.pose.position.x)
      print("target pose bearing ", target_pose_target_frame.pose.position.x-0.008)
      self.confirm_to_proceed("finetune")
      if current_pose.pose.position.x > -0.014: # approx. -0.014
        self.active_robots[robot_name].gripper.open(opening_width=0.1)
        self.active_robots[robot_name].gripper.close()
        self.active_robots[robot_name].linear_push(force=10, direction="-X", max_translation=0.015, timeout=5)
        return self.insert_bearing(target_link, attempts=attempts-1, robot_name=robot_name, try_recenter=try_recenter, try_reinsertion=try_reinsertion, task=task)

      # TODO(cambel): check that the bearing is not slightly inserted
      #               otherwise we may trigger a safety lock
      if attempts > 0:
        if try_reinsertion:
          # Try to insert again
          # move back
          rospy.logwarn("** Insertion Incomplete, trying again **")
          robot.move_lin_rel(relative_translation = [0.005,0,0], acceleration = 0.015, speed=.03)
          return self.insert_bearing(target_link, try_recenter=True, try_reinsertion=False, robot_name=robot_name, task=task, attempts=attempts-1)
        else:
          # Try to recenter the bearing
          rospy.logwarn("** Insertion Incomplete, trying from centering again **")
          robot.move_lin_rel(relative_translation = [0.1,0,0], acceleration = 0.1, speed=.2)
          self.fallback_recenter_bearing(task=task, robot_name=robot_name)
          return self.insert_bearing(target_link, try_recenter=False, try_reinsertion=False, robot_name=robot_name, task=task, attempts=attempts-1)
      else:
        rospy.logerr("** Insertion Incomplete, dropping bearing into tray **")
        robot.move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.015, speed=.03)
        if self.use_storage_on_failure:
          self.simple_place(robot_name, self.bearing_store_pose, place_height=0.0, gripper_opening_width=0.09, axis="x", sign=-1)
          self.a_bot.gripper.forget_attached_item()
          self.is_bearing_in_storage = True
        else:
          self.drop_in_tray(robot_name)
        return False

    robot.gripper.open(opening_width=0.08, wait=True)
    robot.move_lin_rel(relative_translation = [0.012,0,0], acceleration = 0.015, speed=.03)
    robot.gripper.close(force=30, velocity=0.01, wait=True)

    result = robot.do_insertion(target_pose_target_frame, insertion_direction=insertion_direction, force=10.0, timeout=30.0, 
                                radius=0.0, wiggle_direction="X", wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=1.0,
                                relaxed_target_by=0.003, selection_matrix=selection_matrix)
    
    success = result in (TERMINATION_CRITERIA, DONE)

    # Go back regardless of success
    robot.gripper.open(wait=True)
    success &= robot.move_lin_rel(relative_translation = [0.15,0,0.05], speed=.3)
    return success

  def fasten_bearing(self, task="", only_retighten=False, robot_name="b_bot", 
                           simultaneous=False, with_extra_retighten=False,
                           skip_intermediate_pose=False):
    if not task in ["taskboard", "assembly"]:
      rospy.logerr("Invalid task specification: " + task)
      return False
    offset = -1 if robot_name == "a_bot" else 1
    screw_set_center_pose = None
    if task == "taskboard":
      screw_set_center_pose = conversions.to_pose_stamped("taskboard_bearing_target_link", [-0.03, 0, 0, offset*tau/12, 0, 0])
    elif task == "assembly":
      screw_set_center_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.03, 0, 0, offset*tau/12, 0, 0])
    else:
      rospy.logerr("Invalid task specification: " + task)
      return False
    
    screw_poses = []
    for i in [1,3,2,4]: # Cross pattern
      if task == "taskboard":
        screw_pose = conversions.to_pose_stamped("taskboard_bearing_target_screw_" + str(i) + "_link", [0, 0, 0, offset*tau/12, 0, 0])
        screw_pose.pose.position.z += -.001  # MAGIC NUMBER
      elif task == "assembly":
        screw_pose = conversions.to_pose_stamped("assembled_part_07_screw_hole_" + str(i), [0, 0, 0, offset*tau/12, 0, 0])
        screw_pose.pose.position.z += .0025  # MAGIC NUMBER
        screw_pose.pose.position.y -= .0022  # MAGIC NUMBER
      screw_pose.pose.position.x += .006  # This needs to be quite far forward, because the thread is at the plate level (behind the frame)
      screw_poses.append(screw_pose)

    return self.fasten_set_of_screws(screw_poses, screw_size=4, robot_name=robot_name, only_retighten=only_retighten,
                              skip_intermediate_pose=skip_intermediate_pose,
                              simultaneous=simultaneous, with_extra_retighten=with_extra_retighten, tries=15,
                              screw_set_center_pose=screw_set_center_pose)

  def fasten_set_of_screws(self, screw_poses, screw_size, robot_name, only_retighten=False, skip_intermediate_pose=False, 
                           simultaneous=False, with_extra_retighten=False, intermediate_pose=None, unequip_when_done=True,
                           skip_return=False, attempts=1, tries=8, screw_set_center_pose=None):
    if not self.equip_tool(robot_name, 'screw_tool_m%s' % screw_size):
      rospy.logerr("Fail to equip tool abort!")
      return False

    speed = 0.7
    screw_tool_link = robot_name + "_screw_tool_m%s_tip_link" % screw_size
    self.vision.activate_camera(robot_name + "_outside_camera")
    # Initialize screw status
    screw_status = dict()
    for n in range(len(screw_poses)):
      if only_retighten:
        screw_status[n] = "maybe_stuck_in_hole"
      else:
        screw_status[n] = "empty"
    
    if only_retighten and self.tools.screw_is_suctioned["m"+str(screw_size)]:
      rospy.logwarn("Screw already in tool, but we want to retighten! Breaking out without retightening.")
      return True

    robot = self.active_robots[robot_name]
    if not skip_intermediate_pose:
      self.confirm_to_proceed("intermediate pose")
      robot.go_to_named_pose("screw_ready", speed=speed)
    
    # Go to bearing and fasten all the screws
    all_screws_done = False
    first_screw = True
    feeder_to_hole_plan = None
    hole_to_feeder_plan = None
    if robot_name == "a_bot":
      rotation = [radians(80), 0, 0]
    elif robot_name == "b_bot":
      rotation = [-tau/6, 0, 0]
    pose_feeder = conversions.to_pose_stamped("m" + str(screw_size) + "_feeder_outlet_link", [0, 0, 0]+rotation)
    pose_feeder.pose.position.x -= 0.04
    if screw_set_center_pose:
      screw_set_center_pose = screw_set_center_pose
    else:
      screw_set_center_pose = copy.deepcopy(screw_poses[0])
      screw_set_center_pose.pose.position.x -= 0.05
    while not all_screws_done and tries > 0:
      for n in range(len(screw_poses)):
        assert not rospy.is_shutdown(), "lost connection to ros?"
        if screw_status[n] == "done":
          continue

        screw_pose_approach = copy.deepcopy(screw_poses[n])
        screw_pose_approach.pose.position.x -= 0.03
        
        seq = []
        if not first_screw and screw_status[n] == "empty": # go back
          if not hole_to_feeder_plan:
            waypoints = []
            if intermediate_pose:
              waypoints.append((self.active_robots[robot_name].compute_ik(intermediate_pose, timeout=0.02, retry=True, end_effector_link=screw_tool_link), 0, 1.0))
            waypoints.append(("horizontal_screw_ready", 0, 1.0))
            # waypoints.append(("screw_ready",            0, 1.0))
            waypoints.append(("feeder_pick_ready",      0, 1.0))
            waypoints.append((self.active_robots[robot_name].compute_ik(pose_feeder, timeout=0.02, retry=True, end_effector_link=screw_tool_link), 0, 0.3))
      
            res = self.active_robots[robot_name].move_joints_trajectory(waypoints, plan_only=True)
            if not res:
              rospy.logerr("Fail to plan hole_to_feeder_plan")
              return False
            else:
              hole_to_feeder_plan, _ = res
          if not self.active_robots[robot_name].execute_plan(hole_to_feeder_plan):
            rospy.logerr("Failed to execute hole_to_feeder_plan")
            return False
        
        if not skip_intermediate_pose and first_screw and screw_status[n] == "maybe_stuck_in_hole": # get ready for screwing
          seq.append(helpers.to_sequence_item("horizontal_screw_ready", speed=speed, linear=True))
          seq.append(helpers.to_sequence_item("screw_ready", speed=speed, linear=True))

        if seq:
          self.execute_sequence(robot_name, seq, "fasten_screws_return_seq")

        if screw_status[n] == "empty":
          feeder_to_hole_plan = self.pick_and_fasten_screw(robot_name, screw_poses[n], screw_size=screw_size, 
                                                           intermediate_pose=intermediate_pose, attempts=attempts,
                                                           save_plan_on_success=True, saved_plan=feeder_to_hole_plan,
                                                           screw_set_center_pose=screw_set_center_pose,
                                                           skip_initial_motion=(not first_screw))
          if self.active_robots[robot_name].is_protective_stopped():
            rospy.logerr("Screw was already there, robot collided! Releasing protective stop and marking screw as done.")
            while self.active_robots[robot_name].is_protective_stopped():
              self.active_robots[robot_name].unlock_protective_stop()
              rospy.sleep(1)
            self.active_robots[robot_name].go_to_pose_goal(screw_pose_approach, speed=0.2)
            screw_status[n] = "done"
            continue
          
          trajectory = [[screw_pose_approach, 0.0, 0.02], [screw_set_center_pose, 0.0, 0.3]]
          if not self.active_robots[robot_name].move_lin_trajectory(trajectory, end_effector_link=screw_tool_link):
            rospy.logerr("Failed to go to screw_set_center_pose")
            return False
          if feeder_to_hole_plan:
            screw_status[n] = "done" 
          else:
            rospy.sleep(0.2)
            if self.tools.screw_is_suctioned["m%s"%screw_size]:
              rospy.logwarn("Screw detected in tool after failing to screw!")
              screw_status[n] == "empty"
            else:
              rospy.logwarn("Screw failed, skiping")
              screw_status[n] == "maybe_stuck_in_hole"
            
        elif screw_status[n] == "maybe_stuck_in_hole":  # Retighten
          success = self.screw(robot_name, screw_poses[n], screw_size=screw_size, screw_height=0.02, 
                               skip_final_loosen_and_retighten=False, spiral_radius=0.003, retry_on_failure=True,
                               stay_put_after_screwing=True)
          robot.go_to_pose_goal(screw_pose_approach, end_effector_link=screw_tool_link, speed=0.1, move_lin=True)
          screw_status[n] = "done" if success else "maybe_stuck_in_hole"
        
        if first_screw:
          first_screw = False
        
        if robot.is_protective_stopped():
          rospy.logerr("Critical error: %s  protective stopped. Something is wrong. Wait to unlock, move back, abort." % robot_name)
          robot.unlock_protective_stop()
          robot.move_lin_rel(relative_translation=[-0.05, 0, 0], relative_to_tcp=True, end_effector_link=screw_tool_link)
          break
        
        rospy.loginfo("=== Screw " + str(n) + " detected as " + screw_status[n] + " ===")

      all_screws_done = all(value == "done" for value in screw_status.values())
      if not all_screws_done and simultaneous:
        rospy.logwarn("Fail to do all screws in simultaneous, waiting for other robot for 5s before remaining attempts: %s" % tries)
        rospy.sleep(5) # extra time for b_bot to get out of the way
      tries -= 1

    if not all_screws_done and simultaneous:
        rospy.logwarn("NOT done, but aborting")

    if with_extra_retighten:
      return self.fasten_set_of_screws(screw_poses[::-1], screw_size=screw_size, robot_name=robot_name, only_retighten=True,
                                       simultaneous=simultaneous, with_extra_retighten=False, 
                                       skip_intermediate_pose=True, intermediate_pose=intermediate_pose, 
                                       unequip_when_done=unequip_when_done)
    elif not skip_return:
      seq = []
      if intermediate_pose:
        seq.append(helpers.to_sequence_item(intermediate_pose, speed=speed, end_effector_link=screw_tool_link, linear=True))
      seq.append(helpers.to_sequence_item("horizontal_screw_ready", speed=speed, linear=True))
      seq.append(helpers.to_sequence_item("screw_ready", speed=speed, linear=True))
      self.execute_sequence(robot_name, seq, "fasten_screws_return_seq")
    
    if unequip_when_done:
      if not self.unequip_tool(robot_name, 'screw_tool_m%s' % screw_size):
        rospy.logerr("Fail to unequip tool abort!")

    return all_screws_done

  ########  Motor pulley

  def pick_and_insert_motor_pulley(self, task, robot_name="b_bot"):
    if task == "taskboard":
      target_link = "taskboard_small_shaft"
    elif task == "assembly":
      rospy.logerr("look this up")
      target_link = "assembled_part_05_center"

    insert_has_failed_before = False
    if self.is_motor_pulley_in_storage:
      insert_has_failed_before = True

    if not self.pick_motor_pulley(robot_name=robot_name):
      return False

    if not self.orient_motor_pulley(target_link, robot_name=robot_name):
      return False

    if not insert_has_failed_before:
      return self.insert_motor_pulley(target_link, robot_name=robot_name)
    else:
      return self.insert_motor_pulley_fallback(target_link, robot_name="b_bot")

  def orient_motor_pulley(self, target_link, robot_name="b_bot"):
    if not self.playback_sequence(routine_filename="motor_pulley_orient_"+robot_name):
      rospy.logerr("Fail to complete the playback sequence motor pulley orient")
      self.pick_from_centering_area_and_drop_in_tray(robot_name)
      return False

    if robot_name == "b_bot":
      approach_pose      = conversions.to_pose_stamped(target_link, [-0.05, 0.0, 0.0] + np.deg2rad([180, 35, 0]).tolist()) 
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.005, 0.001, -0.005] + np.deg2rad([180, 35, 0]).tolist()) # Manually defined target pose in object frame
    else:
      approach_pose      = conversions.to_pose_stamped(target_link, [-0.050, -0.000, 0.009] + np.deg2rad([180, -35, 0]).tolist()) 
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.007, 0.001, 0.008] + np.deg2rad([180, -35, 0]).tolist()) # Manually defined target pose in object frame
    
    if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=0.5):
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(pre_insertion_pose, speed=0.1):
      return False
    self.confirm_to_proceed("finetune")
    return True

  def pick_motor_pulley(self, attempt=5, robot_name="b_bot"):
    self.despawn_object("motor_pulley")
    options = {'grasp_width': 0.05, 'center_on_corner': True, 'approach_height': 0.02, 'grab_and_drop': True, 'object_width': 0.03}

    if self.is_motor_pulley_in_storage:
      goal = copy.deepcopy(self.motor_pulley_store_pose)
      goal = self.listener.transformPose("tray_center", goal)
      self.b_bot.go_to_named_pose("centering_area")
      self.is_motor_pulley_in_storage = False
    else:
      goal = self.look_and_get_grasp_point("motor_pulley", robot_name=robot_name, options=options)
      if not isinstance(goal, geometry_msgs.msg.PoseStamped):
        rospy.logerr("Could not find motor_pulley in tray. Skipping procedure.")
        return False
      # goal.pose.position.x -= 0.01 # MAGIC NUMBER
      goal.pose.position.z = 0.0

    motor_pulley_pose = copy.deepcopy(goal)
    motor_pulley_pose.pose.position.z = 0.005
    self.markers_scene.spawn_item("motor_pulley", motor_pulley_pose)
    # self.spawn_object("motor_pulley", motor_pulley_pose)
    self.planning_scene_interface.allow_collisions("motor_pulley", "")
    self.vision.activate_camera(robot_name + "_inside_camera")
    self.activate_led(robot_name, False)
    
    if not self.simple_pick(robot_name, goal, gripper_force=30.0, grasp_width=.05, 
                            axis="z", grasp_height=0.002, item_id_to_attach="motor_pulley",
                            allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick")
      if attempt > 0:
        rospy.logwarn("Could not pick motor_pulley in tray. Try again remaining attempts:%s" % attempt)
        return self.pick_motor_pulley(robot_name=robot_name, attempt=attempt-1)
      rospy.logerr("Could not pick motor_pulley in tray. Skipping procedure.")
      return False

    if not self.simple_gripper_check(robot_name, min_opening_width=0.02):
      rospy.logerr("Gripper did not grasp the pulley --> Stop")
      if attempt > 0:
        rospy.logwarn("Could not pick motor_pulley in tray. Try again remaining attempts:%s" % attempt)
        return self.pick_motor_pulley(robot_name=robot_name, attempt=attempt-1)
      rospy.logerr("Could not pick motor_pulley in tray. Skipping procedure.")
      return False

    return True

  def insert_motor_pulley_fallback(self, target_link, robot_name="b_bot"):
    """
        If first fallback don't work, try 4 additional initial poses close to the expected pose
    """
    rospy.logerr("** Insertion still Failed. Insertion with extra search area **")
    # Try to 
    if robot_name == "b_bot":
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [0.013, 0.001, -0.005, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame
      wiggle_direction="X"
      relaxed_by = 0.005
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.005, 0.001, -0.005] + np.deg2rad([180, 35, 0]).tolist()) # Manually defined target pose in object frame
    else:
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.007, 0.001, 0.008] + np.deg2rad([180, -35, 0]).tolist()) # Manually defined target pose in object frame
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [0.003, -0.002, 0.009, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame
      relaxed_by = 0.001
      wiggle_direction=None
    selection_matrix = [0., 0.2, 0.2, 1.0, 1.0, 1.0]
    
    offsets = [[0.0015, 0.0015],[-0.0015, 0.0015],[0.0015, -0.0015],[-0.0015, -0.0015]]
    for i in range(4):
      start_pose = copy.deepcopy(pre_insertion_pose)
      start_pose.pose.position.y += offsets[i][0]
      start_pose.pose.position.z += offsets[i][1]
      
      # move back and go to new initial pose
      self.active_robots[robot_name].move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
      self.active_robots[robot_name].go_to_pose_goal(start_pose, speed=0.1, move_lin=True)
      # Attempt insertion at new pose
      result = self.active_robots[robot_name].do_insertion(target_pose_target_frame, radius=0.005, 
                                                        insertion_direction="-X", force=8.0, timeout=15.0, 
                                                        wiggle_direction=wiggle_direction, wiggle_angle=np.deg2rad(5.0), wiggle_revolutions=1.,
                                                        relaxed_target_by=relaxed_by, selection_matrix=selection_matrix)
      success = (result == TERMINATION_CRITERIA)

      if not success:
        # One small extra check and push if we are a bit inside the small shaft
        current_pose = self.listener.transformPose(target_link, self.active_robots[robot_name].get_current_pose_stamped())
        print("current pose motor pulley ", current_pose.pose.position.x)
        if current_pose.pose.position.x > -0.003:
          self.active_robots[robot_name].gripper.open(opening_width=0.04)
          self.active_robots[robot_name].gripper.close()
          self.active_robots[robot_name].linear_push(force=10, direction="-X", max_translation=0.01)

          self.active_robots[robot_name].gripper.open(opening_width=0.04, wait=True)
          self.active_robots[robot_name].move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.3, speed=.03)
          return True # If we are this close, assume success
      else:
        self.active_robots[robot_name].gripper.open(opening_width=0.04, wait=True)
        self.active_robots[robot_name].move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.3, speed=.03)
        return True
  
    rospy.logerr("** Insertion Failed!! **")
    self.active_robots[robot_name].move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
    if self.use_storage_on_failure:
      self.active_robots[robot_name].move_lin_rel(relative_translation = [0.05,0,0], acceleration = 0.015, speed=.03)
      self.simple_place(robot_name, self.motor_pulley_store_pose, place_height=0.0, gripper_opening_width=0.09, axis="x", sign=-1, approach_height=0.1)
      self.active_robots[robot_name].gripper.forget_attached_item()
      self.is_motor_pulley_in_storage = True
    else:
      # return to tray to drop pulley
      self.drop_in_tray(robot_name)
    return False

  def insert_motor_pulley(self, target_link, attempts=2, robot_name="b_bot", retry_insertion=True):
    if robot_name == "b_bot":
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [0.013, 0.001, -0.005, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame
      wiggle_direction="X"
      relaxed_by = 0.005
    else:
      target_pose_target_frame = conversions.to_pose_stamped(target_link, [0.003, -0.002, 0.009, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame
      relaxed_by = 0.001
      wiggle_direction=None

    selection_matrix = [0., 0.2, 0.2, 1.0, 1.0, 1.0]
    result = self.active_robots[robot_name].do_insertion(target_pose_target_frame, radius=0.005, 
                                                      insertion_direction="-X", force=8.0, timeout=15.0, 
                                                      wiggle_direction=wiggle_direction, wiggle_angle=np.deg2rad(5.0), wiggle_revolutions=1.,
                                                      relaxed_target_by=relaxed_by, selection_matrix=selection_matrix)
    success = (result == TERMINATION_CRITERIA)

    if not success:
      self.confirm_to_proceed("finetune")
      self.active_robots[robot_name].linear_push(force=8, direction="-X", max_translation=0.01, timeout=5)
      current_pose = self.listener.transformPose(target_link, self.active_robots[robot_name].get_current_pose_stamped())
      print("current pose motor pulley ", current_pose.pose.position.x)
      if self.assembly_database.db_name == "taskboard":
        if current_pose.pose.position.x > -0.0034:
          self.active_robots[robot_name].gripper.open(opening_width=0.04)
          self.active_robots[robot_name].gripper.close()
          self.active_robots[robot_name].linear_push(force=10, direction="-X", max_translation=0.01)
          return self.insert_motor_pulley(target_link, attempts=attempts-1, robot_name=robot_name, retry_insertion=retry_insertion)

      if attempts > 0: # try again the pulley is still there
        if retry_insertion:  # = Retry directly after end of last insertion, wherever it stopped
          self.active_robots[robot_name].move_lin_rel(relative_translation = [0.005,0,0], acceleration = 0.015, speed=.03)
          return self.insert_motor_pulley(target_link, attempts=attempts-1, robot_name=robot_name, retry_insertion=False)
        else: # try reorient first
          rospy.logwarn("** Insertion Incomplete, trying again **")
          self.active_robots[robot_name].move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
          self.orient_motor_pulley(target_link, robot_name)
          return self.insert_motor_pulley(target_link, attempts=attempts-1, robot_name=robot_name, retry_insertion=True)
      else:
        rospy.logerr("** Insertion Failed!! **")
        self.active_robots[robot_name].move_lin_rel(relative_translation = [0.03,0,0], acceleration = 0.015, speed=.03)
        if self.use_storage_on_failure:
          self.simple_place(robot_name, self.motor_pulley_store_pose, place_height=0.0, gripper_opening_width=0.09, axis="x", sign=-1, approach_height=0.1)
          self.b_bot.gripper.forget_attached_item()
          self.is_motor_pulley_in_storage = True
        else:
          # return to tray to drop pulley
          self.drop_in_tray(robot_name)
        return False

    self.active_robots[robot_name].gripper.open(opening_width=0.04, wait=True)
    success &= self.active_robots[robot_name].move_lin_rel(relative_translation = [0.03,0,0], speed=.3)
    return success

  def rotate_motor_pulley(self, target_link, rotations=6, offset_from_center=-0.058, x_offset=0.033, skip_approach=False):
    """"
      Use the gripper finger's pad to rotate the motor pulley by going up and down
    """
    self.allow_collisions_with_robot_hand("panel_motor", "a_bot")
    self.allow_collisions_with_robot_hand("panel_bearing", "a_bot")
    approach_pose   = conversions.to_pose_stamped(target_link, [-0.05, -0.045,-0.004, tau/2, 0, 0])
    up_pose         = conversions.to_pose_stamped(target_link, [x_offset, offset_from_center,-0.018, tau/2, 0, 0])
    down_pose       = conversions.to_pose_stamped(target_link, [x_offset, offset_from_center, 0.012, tau/2, 0, 0])
    offset = 0.01 if offset_from_center < 0 else -0.01 # direction to get out of the way of the pulley
    center_out_pose = conversions.to_pose_stamped(target_link, [x_offset, offset_from_center+offset,-0.0035, tau/2, 0, 0])

    self.a_bot.gripper.open(wait=False)

    seq = []
    if not skip_approach:
      seq.append(helpers.to_sequence_item(approach_pose, speed=1.0))
    trajectory = [[center_out_pose, 0.0, 1.0], [up_pose, 0.0, 0.5], [down_pose, 0.0, 0.2]]
    trajectory *= rotations
    seq.append(["trajectory", trajectory])
    seq.append(helpers.to_sequence_item(center_out_pose, speed=1.0))
    if not skip_approach:
      seq.append(helpers.to_sequence_item(approach_pose, speed=1.0))
    
    self.execute_sequence("a_bot", seq, "rotate motor pulley")
    self.allow_collisions_with_robot_hand("panel_motor", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("panel_bearing", "a_bot", allow=False)
    return True

  def fasten_motor_pulley(self, target_link, simultaneous=False):
    self.equip_tool("b_bot", "set_screw_tool")

    b_bot_approach_pose    = conversions.to_pose_stamped(target_link, [0.001, -0.002, -0.072]+ np.deg2rad([174.3, -87.6, -135.8]).tolist())
    b_bot_above_hole_pose  = conversions.to_pose_stamped(target_link, [-0.004, -0.003,-0.007]+ np.deg2rad([174.3, -87.6, -135.8]).tolist())
    b_bot_centering_pose   = conversions.to_pose_stamped(target_link, [-0.005, -0.003,-0.007]+ np.deg2rad([-170.5, -88, -151]).tolist())
    b_bot_at_hole_pose     = conversions.to_pose_stamped(target_link, [-0.004, -0.002, 0.005]+ np.deg2rad([-170.5, -88, -151]).tolist())

    a_bot_approach_push_pose = conversions.to_pose_stamped(target_link, [-0.04, 0, 0, tau/2, 0, 0])
    a_bot_push_pose = conversions.to_pose_stamped(target_link, [-0.0025, 0, 0, tau/2, 0, 0])
    def a_task_push_pulley():
      self.a_bot.go_to_pose_goal(a_bot_approach_push_pose, speed=0.8, move_lin=True)
      self.a_bot.gripper.close(wait=False)
      self.a_bot.go_to_pose_goal(a_bot_push_pose, speed=0.1, move_lin=True)
    
    offset_from_center = -0.058 if self.assembly_database.db_name == "wrs_assembly_2020" else -0.056
    # Find both holes
    for i in range(2):
      if i > 0:
        # Rotate once to get the current hole out of the way
        self.confirm_to_proceed("rotate?")
        self.rotate_motor_pulley(target_link, rotations=1, offset_from_center=offset_from_center, x_offset=0.033)

      self.b_bot.go_to_pose_goal(b_bot_approach_pose, speed=1.0, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link")
      self.confirm_to_proceed("down1?")
      self.b_bot.go_to_pose_goal(b_bot_above_hole_pose, speed=0.05, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link")
      
      self.confirm_to_proceed("rotate?")
      self.rotate_motor_pulley(target_link, rotations=6, offset_from_center=offset_from_center, x_offset=0.033)
      
      self.confirm_to_proceed("down2?")
      def b_task_fastening_pose():
        self.b_bot.go_to_pose_goal(b_bot_centering_pose, speed=0.03, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link")
        self.b_bot.go_to_pose_goal(b_bot_at_hole_pose, speed=0.015, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link")
      if simultaneous:
        self.do_tasks_simultaneous(a_task_push_pulley, b_task_fastening_pose, timeout=60)
      else:
        b_task_fastening_pose()
        a_task_push_pulley()

      self.tools.set_motor("set_screw_tool", "tighten", duration=4.0, skip_final_loosen_and_retighten=True, wait=True)

      result = self.tools.fastening_tool_client.get_result()
      rospy.loginfo("Set screw motor result: %s" % result)
      motor_stalled = False
      if result is not None:
        motor_stalled = result.motor_stalled
      self.b_bot.go_to_pose_goal(b_bot_approach_pose, speed=0.2, move_lin=True, end_effector_link="b_bot_set_screw_tool_tip_link")
      if motor_stalled or not self.use_real_robot:
        break
      else:
        print("not stalled")
    
    def a_bot_task():
      self.a_bot.move_lin_rel([0.05,0,0], speed=0.5)
      self.a_bot.go_to_named_pose("above_tray")
    def b_bot_task():
      self.unequip_tool("b_bot", "set_screw_tool")
    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=60)
    else:
      b_bot_task()
      a_bot_task()
    return True

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

  def pick_idler_pulley(self, previous_object_pose=None, attempts=5):
    self.despawn_object("taskboard_idler_pulley_small")
    if not previous_object_pose: # Find the idler pulley pose when not given
      rospy.loginfo("Look for the idler pulley")
      self.vision.activate_camera("a_bot_outside_camera")
      options = {'check_for_close_items': False, 'center_on_corner': True, 'center_on_close_border': True, 
                'min_dist_to_border': 0.0, 'allow_pick_near_border': True, 'object_width': 0.0}
      object_pose = self.look_and_get_grasp_point("taskboard_idler_pulley_small", robot_name="a_bot", options=options)

      if not isinstance(object_pose, geometry_msgs.msg.PoseStamped):
        rospy.logerr("Could not find idler pulley in tray. Skipping procedure.")
        if attempts > 0:
          return self.pick_idler_pulley(previous_object_pose=None, attempts=attempts-1)
        return False
      
      object_pose, is_safe_grasp  = self.constrain_grasp_into_tray("a_bot", object_pose, grasp_width=0.06, object_width=0.04)

      dx, dy = self.distances_from_tray_border(object_pose)
      if not is_safe_grasp: # Too close to one of the borders but not a corner
        if dx > dy:
          object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
        else:
          object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, 0))
      else:
        object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
    else:
      is_safe_grasp = True
      object_pose = copy.deepcopy(previous_object_pose)

    object_pose.pose.position.x -= 0.01 # MAGIC NUMBER
    object_pose.pose.position.z = 0.018
  
    idler_pulley_pose = copy.deepcopy(object_pose)
    idler_pulley_pose.pose.position.z = 0.023
    self.markers_scene.spawn_item("taskboard_idler_pulley_small", idler_pulley_pose)
    # self.spawn_object("taskboard_idler_pulley_small", idler_pulley_pose)
    self.planning_scene_interface.allow_collisions("taskboard_idler_pulley_small", "")
    
    rospy.loginfo("Picking idler pulley at: ")
    self.b_bot.go_to_named_pose("home")

    at_object_pose = copy.deepcopy(object_pose)

    self.a_bot.gripper.open(wait=False, opening_width=0.06)

    approach_pose = copy.deepcopy(at_object_pose)
    approach_pose.pose.position.z += .03

    if not self.a_bot.go_to_pose_goal(approach_pose, speed=1.0):
      rospy.logerr("Fail to complete approach pose")
      return False  

    rospy.loginfo("Moving down to object")
    
    self.a_bot.gripper.open(opening_width=0.06)
    if not self.a_bot.go_to_pose_goal(at_object_pose):
      rospy.logwarn("Failed to pick. Abort")
      return False

    if not is_safe_grasp:
      if attempts > 0:
        rospy.logwarn("Too close to border, centering")
        self.a_bot.gripper.close()
        self.a_bot.gripper.attach_object("taskboard_idler_pulley_small")
        direction = "y" if dx > dy else "x"
        if not self.move_towards_tray_center("a_bot", distance=0.1, go_back_halfway=False, one_direction=direction):
          rospy.logerr("Fail to move to the center")
          return False
        self.a_bot.gripper.open(opening_width=0.06)
        if not previous_object_pose: # if we tried with vision before, try once with the new current pose
          current_pose = self.listener.transformPose("tray_center", self.a_bot.get_current_pose_stamped())
          current_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
          return self.pick_idler_pulley(previous_object_pose=current_pose, attempts=attempts-1)
        else: # we try the current pose and we are back here, try vision again
          return self.pick_idler_pulley(attempts=attempts-1)
      rospy.logerr("Fail to complete grasp_idler_pulley, ABORT")
      return False
      

    self.a_bot.gripper.open(wait=False, opening_width=0.06)
    self.allow_collisions_with_robot_hand("tray", "a_bot")
    if not self.center_with_gripper("a_bot", opening_width=.06):
      rospy.logerr("Fail to complete center_with_gripper")
      self.allow_collisions_with_robot_hand("tray", "a_bot", False)
      return False
    self.allow_collisions_with_robot_hand("tray", "a_bot", False)
    if not self.grasp_idler_pulley():
      if attempts > 0:
        rospy.logerr("Fail to complete grasp_idler_pulley, retry")
        self.a_bot.gripper.close()
        if not self.move_towards_tray_center("a_bot", 0.1, go_back_halfway=False):
          return False
        self.a_bot.gripper.open(opening_width=.06)
        if not previous_object_pose: # if we tried with vision before, try once with the new current pose
          current_pose = self.listener.transformPose("tray_center", self.a_bot.get_current_pose_stamped())
          current_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, tau/4, -tau/4))
          return self.pick_idler_pulley(previous_object_pose=current_pose, attempts=attempts-1)
        else: # we try the current pose and we are back here, try vision again
          return self.pick_idler_pulley(attempts=attempts-1)
      rospy.logerr("Fail to complete grasp_idler_pulley, ABORT")
      return False
    return True

  def pick_and_insert_idler_pulley(self, task="", simultaneous=False):
    if not task:
      rospy.logerr("Specify the task!")
      return False

    if task == "taskboard":
      idler_puller_target_link = "taskboard_long_hole_top_link"
    elif task == "assembly":
      rospy.logerr("look this up")
      idler_puller_target_link = "assembly_long_hole_top_link"

    self.ab_bot.go_to_named_pose("home")
    
    if not self.pick_idler_pulley():
      rospy.logerr('Fail to pick idler pulley')
      return False

    self.allow_collisions_with_robot_hand("taskboard_plate", "a_bot")
    self.a_success = False
    self.b_success = False
    def a_bot_task():
      if not self.insert_idler_pulley(idler_puller_target_link):
        rospy.logerr("Fail to complete insert_idler_pulley")
        self.a_bot.move_lin_rel(relative_translation=[0.1,0,0])
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
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=180)
      if not self.a_success or not self.b_success:
        rospy.logerr("Fail to do simultaneous tasks")
        return False
    else: # sequential 
      if not a_bot_task():
        return False
      if not b_bot_task():
        return False

    if not self.prepare_screw_tool_idler_pulley(idler_puller_target_link):
      rospy.logerr("Fail to do prepare_screw_tool_idler_pulley")
      self.unequip_tool("b_bot", "padless_tool_m4")
      self.a_bot.gripper.open(opening_width=0.05)
      self.a_bot.gripper.forget_attached_item()    
      self.a_bot.move_lin_rel(relative_translation=[-0.15, 0, 0.0], relative_to_tcp=True, speed=1.0)
      return False
    self.despawn_object("taskboard_idler_pulley_small")
    
    if not self.equip_nut_tool():
      rospy.logerr("Fail to complete equip_nut_tool")
      self.playback_sequence("idler_pulley_return_screw_tool")
      self.unequip_tool("b_bot", "padless_tool_m4")
      return False

    self.vision.activate_camera("a_bot_inside_camera")
    success = self.fasten_idler_pulley_with_nut_tool(idler_puller_target_link)
    if not success:
      rospy.logerr("Fail to complete fasten_idler_pulley_with_nut_tool")

    self.a_success = False
    self.b_success = False
    def a_bot_task2():
      self.a_success = self.unequip_nut_tool()
      return self.a_success
    def b_bot_task2():
      # rospy.sleep(10)
      self.b_success = self.return_padless_tool_idler_pulley()
      return self.b_success

    if simultaneous:
      if not self.do_tasks_simultaneous(a_bot_task2, b_bot_task2, timeout=60):
        return False
    else:
      if not a_bot_task2():
        return False
      if not b_bot_task2():
        return False

    return True

  def return_padless_tool_idler_pulley(self):
    waypoints = []
    waypoints.append((self.b_bot.move_lin_rel(relative_translation=[0.1, 0, 0.0], pose_only=True), 0, 0.1))
    # intermediate_pose = conversions.to_pose_stamped("workspace_center", [0.15, 0.0, 0.30, 50, 50, -140])
    # waypoints.append((intermediate_pose, 0, 1.0))
    waypoints.append(("horizontal_screw_ready", 0, 1.0))
    waypoints.append(("tool_pick_ready",        0, 1.0))
    if not self.b_bot.move_joints_trajectory(waypoints, speed=0.8):
      rospy.logerr("Fail to complete idler_pulley_return_screw_tool")
    return self.unequip_tool("b_bot", "padless_tool_m4")

  def equip_nut_tool(self):
    self.allow_collisions_with_robot_hand("padless_tool_m4", "a_bot", True)

    if not self.nut_tool_used:
      above_nut_aid = [1.51833, -1.9979, 1.83185, -1.41133, -1.57929, 3.09534]
      nut_holder_name = "nut_holder1"
    else:
      above_nut_aid = [1.91526, -1.81915, 1.72051, -1.47392, -1.58145, 3.49249]
      nut_holder_name = "nut_holder2"

    self.spawn_tool(nut_holder_name)
    self.planning_scene_interface.allow_collisions(nut_holder_name)
    def post_cb():
      self.a_bot.gripper.attach_object(nut_holder_name, with_collisions=True)
    
    self.a_bot.gripper.forget_attached_item()    
    seq = []
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.05, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_item_relative([-0.15, 0, 0, 0, 0, 0], relative_to_tcp=True, speed=1.0))
    seq.append(helpers.to_sequence_item(above_nut_aid, speed=1.0, linear=False))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.1, gripper_velocity=1.0, wait=False))
    seq.append(helpers.to_sequence_item_relative([0, 0, -0.25, 0, 0, 0], speed=0.6))
    seq.append(helpers.to_sequence_gripper('close', gripper_force=60, gripper_velocity=0.1, post_callback=post_cb))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0.25, 0, 0, 0], speed=0.4))

    success = self.execute_sequence("a_bot", seq, "equip nut tool")

    self.allow_collisions_with_robot_hand("padless_tool_m4", "a_bot", False)

    return success

  def unequip_nut_tool(self):
    if not self.nut_tool_used:
      above_nut_aid = [1.51833, -1.9979, 1.83185, -1.41133, -1.57929, 3.09534]
      nut_holder_name = "nut_holder1"
    else:
      above_nut_aid = [1.91526, -1.81915, 1.72051, -1.47392, -1.58145, 3.49249]
      nut_holder_name = "nut_holder2"

    self.planning_scene_interface.allow_collisions(nut_holder_name)
    def post_cb():
      self.a_bot.gripper.forget_attached_item()
      self.despawn_tool(nut_holder_name)
    
    seq = []
    # seq.append(helpers.to_sequence_joint_trajectory([above_nut_aid, self.a_bot.move_lin_rel([0, -0.01, -0.25], pose_only=True)], speed=[1.0,1.0]))
    seq.append(helpers.to_sequence_item(above_nut_aid, speed=1.0, linear=False, retime=True))
    seq.append(helpers.to_sequence_item_relative([0, -0.01, -0.25, 0, 0, 0], speed=1.0, retime=True))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.1, gripper_velocity=1.0, post_callback=post_cb))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0.25, 0, 0, 0], speed=1.0, retime=True))

    success = self.execute_sequence("a_bot", seq, "unequip nut tool")
    self.nut_tool_used = True

    return success

  def center_idler_pulley(self):
    # Center first time
    self.a_bot.gripper.close(force=40.0, velocity=0.013)
    self.a_bot.gripper.open(velocity=0.013)

  def grasp_idler_pulley(self):
    # Incline 45 deg
    if not self.a_bot.move_lin_rel(relative_translation=[0, 0.01, 0.0], relative_rotation=[tau/8.0, 0, 0], speed=1.0, timeout=3):
      return False
    
    self.a_bot.gripper.attach_object("taskboard_idler_pulley_small")
    self.a_bot.gripper.close(velocity=0.1)

    if self.a_bot.gripper.opening_width < 0.01 and self.use_real_robot:
        rospy.logerr("Fail to grasp Idler Pulley")
        return False

    # Move up 15 cm
    return self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.15], speed=0.5)

  def insert_idler_pulley(self, target_link):
    rospy.loginfo("Going to near tb (a_bot)")
    # MAGIC NUMBERS (offset from TCP to tip of idler pulley thread)
    approach_pose = conversions.to_pose_stamped(target_link, [(-0.15),   0.011,-0.05, tau/4.0, 0, tau/8.])
    near_tb_pose = conversions.to_pose_stamped(target_link,  [(-0.016), 0.011, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose = conversions.to_pose_stamped(target_link,    [(-0.011), 0.011, 0.0, tau/4.0, 0, tau/8.])
    in_tb_pose_world = self.listener.transformPose("world", in_tb_pose)

    waypoints = []
    waypoints.append((approach_pose, 0, 1.0))
    waypoints.append((near_tb_pose, 0, 1.0))
    if not self.a_bot.move_joints_trajectory(waypoints):
      rospy.logerr("Fail to go to TB pose")
      return False
    
    self.confirm_to_proceed("finetune")

    rospy.loginfo("Moving into ridge (a_bot)")
    insertion_offsets = [0.0]
    d2 = 0.0005
    for i in range(6):
      insertion_offsets.append(d2*(i+1))
      insertion_offsets.append(-d2*(i+1))

    for offset in insertion_offsets:
      selection_matrix = [0.,1.,1.,1,1,1]
      success = self.a_bot.linear_push(10, "-X", max_translation=0.01, timeout=10.0, slow=False, selection_matrix=selection_matrix)

      if not self.use_real_robot:
        return True

      print("target range", in_tb_pose_world.pose.position.x)
      print("current pose", self.a_bot.robot_group.get_current_pose().pose.position.x)
      self.confirm_to_proceed("ok?")
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
    self.allow_collisions_with_robot_hand("padless_tool_m4", "a_bot")

    rospy.loginfo("Going to near tb (b_bot)") # Push with tool
    target_rotation = np.deg2rad([30.0, 0.0, 0.0]).tolist()
    approach_pose = conversions.to_pose_stamped(target_link, [-0.05,0,0] + target_rotation)
    xyz_light_push = [-0.005, -0.002, 0.002]  # MAGIC NUMBERS
    near_tb_pose = conversions.to_pose_stamped(target_link, xyz_light_push + target_rotation)
    
    waypoints = []
    waypoints.append(("screw_ready", 0, 1.0))
    waypoints.append(("horizontal_screw_ready", 0, 1.0))
    waypoints.append((approach_pose, 0, 1.0))
    waypoints.append((near_tb_pose, 0, 1.0))
    if not self.b_bot.move_joints_trajectory(waypoints, speed=0.8, end_effector_link="b_bot_screw_tool_m4_tip_link"):
      rospy.logerr("Fail to prepare tool of for idler pulley")
      return False

    self.vision.activate_camera("a_bot_inside_camera")
    self.tools.set_motor("padless_tool_m4", "tighten", duration=8.0)
    self.insert_screw_tool_tip_into_idler_pulley_head()
    
    ## Incline the tool slightly 
    self.planning_scene_interface.allow_collisions("padless_tool_m4", "taskboard_plate")
    xyz_hard_push = [0.006, -0.001, 0.001]  # MAGIC NUMBERS (target without inclination)
    inclination_angle_deg = 1.0
    inclined_orientation_hard_push = np.deg2rad([30.0, inclination_angle_deg, 0.0]).tolist()
    s = sin(np.deg2rad(inclination_angle_deg)) * 0.008  # 8 mm is roughly the distance from the taskboard surface to the 
                                                        # head of the screw, so adding this offset should result in a rotation
                                                        # around the screw head.
    xyz_hard_push[2] -= s
    push_pose = conversions.to_pose_stamped(target_link, xyz_hard_push + inclined_orientation_hard_push)
    success = self.b_bot.move_lin(push_pose, speed=0.02, acceleration=0.02, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.confirm_to_proceed("finetune")
    return success

  def insert_screw_tool_tip_into_idler_pulley_head(self):
    selection_matrix = [0., 0.9, 0.9, 1, 1, 1]
    self.b_bot.execute_spiral_trajectory("YZ", max_radius=0.003, radius_direction="+Y", steps=30,
                                         revolutions=2, target_force=0, selection_matrix=selection_matrix, check_displacement_time=10,
                                         termination_criteria=None, timeout=4.0, end_effector_link="b_bot_screw_tool_m4_tip_link")
    return True

  def fasten_idler_pulley_with_nut_tool(self, target_link):
    """ Assumes that the nut tool is equipped and the screw tool is in the idler pulley.
        Holds the nut in different places and turns on the motor to fasten the pulley.

        target_link should be "taskboard_long_hole_top_link" or "assembled_part_03_pulley_ridge_top".
    """
    approach_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    waypoints = []
    waypoints.append((approach_pose, 0, 1.0))
    
    x_offset = -0.01  # The Taskboard pulley is shorter than the assembly one
    if target_link == "assembled_part_03_pulley_ridge_top":
      x_offset = 0.004

    success = False
    idler_pulley_screwing_succeeded = False
    # if careful_mode:
    offsets = [0.0, -0.001, -0.002, -0.003, -0.004, -0.005, -0.006, -0.007, -0.008, 
                 0.008, 0.007, 0.006, 0.005, 0.004, 0.003, 0.002, 0.001, 0.000]
    # else:
    #   offsets = [0.0, -0.002, -0.004, -0.006, -0.008, 0.008, 0.006, 0.004, 0.002]
    first_approach = True
    for offset in offsets:
      if idler_pulley_screwing_succeeded:
        success = True
        break
      # Move nut tool forward so nut touches the screw 
      d = offset  # 
      approach_pose     = conversions.to_pose_stamped(target_link, [0.060 + x_offset, 0.001, d + 0.004, 0.0, 0.0, 0.0])
      pushed_into_screw = conversions.to_pose_stamped(target_link, [0.009 + x_offset, 0.001, d + 0.004, 0.0, 0.0, 0.0])
      if not first_approach: 
        waypoints = []
      waypoints.append((approach_pose, 0, 1.0))
      waypoints.append((pushed_into_screw, 0, 0.2))
      if not self.a_bot.move_joints_trajectory(waypoints, speed=1.0, end_effector_link="a_bot_nut_tool_m4_hole_link"):
        rospy.logerr("Fail to prepare nut of for idler pulley")
        return False
      self.confirm_to_proceed("finetune")
      
      response = self.tools.set_motor("padless_tool_m4", "tighten", duration=3.0, wait=True, skip_final_loosen_and_retighten=True)
      if not self.use_real_robot:
        idler_pulley_screwing_succeeded = True
      else:
        idler_pulley_screwing_succeeded = response.motor_stalled
      
      if idler_pulley_screwing_succeeded: # tighten the nut a bit more...
        self.tools.set_motor("padless_tool_m4", "tighten", duration=3.0, wait=True, skip_final_loosen_and_retighten=True)

      if first_approach:
        first_approach = False

    retreat_pose = conversions.to_pose_stamped(target_link, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not self.a_bot.go_to_pose_goal(retreat_pose, end_effector_link="a_bot_nut_tool_m4_hole_link", speed=1.0):
      return False
    
    if success:
      rospy.loginfo("fasten_idler_pulley_with_nut_tool succeeded!")
    else:
      rospy.loginfo("fasten_idler_pulley_with_nut_tool failed!")
    return success
  
  ######## Shaft

  def pick_and_center_shaft(self):
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 2:
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
      target_link = "assembled_part_07_inserted"
    
    self.allow_collisions_with_robot_hand("shaft", "b_bot", True)
    
    if not self.pick_and_center_shaft():
      return False

    if not self.align_shaft(target_link, pre_insert_offset=0.065):
      self.drop_in_tray("b_bot")
      return False

    success = self.insert_shaft(target_link)
    
    self.b_bot.gripper.open(wait=False)
    self.b_bot.gripper.forget_attached_item()
    self.despawn_object("shaft")
    self.b_bot.go_to_named_pose("home")
    return success
  
  def align_shaft(self, target_link, pre_insert_offset=0.09, from_behind=True):
    rospy.loginfo("Going to approach pose (b_bot)")
    if from_behind:
      post_pick_pose = conversions.to_pose_stamped(target_link, [-0.15, 0.0, -0.10, -tau/4, -radians(50), -tau/4])
      success = self.b_bot.go_to_pose_goal(post_pick_pose)
      above_pose     = conversions.to_pose_stamped(target_link, [0.0, 0.002, -0.10, -tau/4, -radians(50), -tau/4])
      behind_pose    = conversions.to_pose_stamped(target_link, [0.09, 0.002, -0.05, -tau/4, -radians(50), -tau/4])
      if target_link == "assembled_part_07_inserted":
        pre_insertion_pose = conversions.to_pose_stamped(target_link, [pre_insert_offset, -0.002, 0.009, -tau/4, -radians(50), -tau/4])
      else:
        pre_insertion_pose = conversions.to_pose_stamped(target_link, [pre_insert_offset, -0.003, 0.002, -tau/4, -radians(50), -tau/4])
      trajectory = [[post_pick_pose, 0.0, 0.8], [above_pose, 0.03, 0.5], [behind_pose, 0.01, 0.5], [pre_insertion_pose, 0.0, 0.2]]
    else:
      rotation = [-tau/4, -radians(50), -tau/4]
      in_front_pose    = conversions.to_pose_stamped(target_link, [-0.09, 0.002, 0.0] + rotation)
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [pre_insert_offset, -0.001, 0.001] + rotation)
      trajectory = [[in_front_pose, 0.0, 0.5], [pre_insertion_pose, 0.0, 0.2]]
    rospy.loginfo("Going to position shaft to pre-insertion (b_bot)")
    success = False
    for i in range(4):
      success = self.b_bot.move_lin_trajectory(trajectory, speed=0.5, acceleration=0.25)
      if success: 
        break
      else:
        rospy.logerr("Fail to position shaft to pre-insertion: attempt #%s" % (i+1))
        rospy.sleep(2)
    if not success: # try without linear trajectory
      if from_behind:
        success = self.b_bot.go_to_pose_goal(post_pick_pose, move_lin=True, retry_non_linear=True)
        success &= self.b_bot.go_to_pose_goal(above_pose, move_lin=True, retry_non_linear=True)
        success &= self.b_bot.go_to_pose_goal(behind_pose, move_lin=True, retry_non_linear=True)
        success &= self.b_bot.go_to_pose_goal(pre_insertion_pose, move_lin=True, retry_non_linear=True)
      else:
        success = self.b_bot.go_to_pose_goal(in_front_pose, move_lin=True, retry_non_linear=True)
        success &= self.b_bot.go_to_pose_goal(pre_insertion_pose, move_lin=True, retry_non_linear=True)
    self.confirm_to_proceed("finetune")
    return success

  def pick_shaft(self, attempt_nr=0):
    self.despawn_object("shaft")
    options = {'center_on_corner': True, 'approach_height': 0.02, 
               'grab_and_drop': True, 'center_on_close_border': True,
               'with_tool': True, 'check_too_close_to_border': True, 'robot_name': "b_bot",
               "use_grasp_pose_directly_in_simple_pick":True}
    self.vision.activate_camera("b_bot_outside_camera")
    goal = self.look_and_get_grasp_point("shaft", options=options)
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find shaft in tray. Skipping procedure.")
      return False

    gp = conversions.from_pose_to_list(goal.pose)
    gp[2] = 0.005
    rotation = tf_conversions.transformations.euler_from_quaternion(gp[3:])[0]%(tau/2)
    print("gripper orientation from SSD", degrees(rotation))
    translation_correction = 0.075/2.0
    gp[:2] += [translation_correction*cos(rotation)-0.01, -translation_correction*sin(rotation)] # Magic Numbers for visuals 
    shaft_pose = conversions.to_pose_stamped("tray_center", gp[:3].tolist() + [0, 0, tau/2 - rotation ])
    self.markers_scene.spawn_item("shaft", shaft_pose)
    # self.spawn_object("shaft", shaft_pose)
    # self.planning_scene_interface.allow_collisions("shaft", "")
    
    goal.pose.position.z = 0.001 # Magic Numbers for grasping
    goal.pose.position.x -= 0.01

    self.vision.activate_camera("b_bot_inside_camera")

    picked_ok = self.simple_pick("b_bot", goal, gripper_force=100.0, grasp_width=.03, approach_height=0.1, 
                              item_id_to_attach="shaft", minimum_grasp_width=0.004,  axis="z", lift_up_after_pick=True,
                              speed_slow=0.5, allow_collision_with_tray=True)
    
    if not picked_ok:
      rospy.logerr("Failed to pick shaft")
      if attempt_nr < 6:
        return self.pick_shaft(attempt_nr=attempt_nr+1)
      return False
    return True

  def insert_shaft(self, target_link, attempts=1, target=0.065, from_behind=True):
    """
    Insert shaft with force control using b_bot. The shaft has to be in front of the hole already.
    """
    direction = "+X" if from_behind else "-X"
    self.b_bot.linear_push(3, direction, max_translation=0.01, timeout=30.0)
    after_push_pose = self.b_bot.get_current_pose_stamped()

    current_pose = self.b_bot.robot_group.get_current_pose()
    target_pose_target_frame = self.listener.transformPose(target_link, current_pose)
    target_pose_target_frame.pose.position.x = 0.006 if from_behind else -0.01 # Magic number

    selection_matrix = [0., 0.2, 0.2, 1, 1, 0.8]
    offset = -0.003 if from_behind else 0.003
    self.b_bot.move_lin_rel(relative_translation = [offset,0,0], acceleration = 0.1, speed=.03) # Release shaft for next push
    result = self.b_bot.do_insertion(target_pose_target_frame, insertion_direction=direction, force=5.0, timeout=15.0, 
                                    wiggle_direction="Z", wiggle_angle=np.deg2rad(10.0), wiggle_revolutions=1.0,
                                    radius=0.003, relaxed_target_by=0.0, selection_matrix=selection_matrix)
    success = result in (TERMINATION_CRITERIA, DONE)

    current_pose = self.b_bot.robot_group.get_current_pose()
    target_pose_target_frame = self.listener.transformPose(target_link, current_pose)
     
    if from_behind:
      rotation = [-tau/4, -radians(50), -tau/4]  # Arbitrary
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [0.12, -0.011, 0.010] + rotation)
    else:
      rotation = [-tau/4, -radians(50), -tau/4]  # Arbitrary
      pre_insertion_pose = conversions.to_pose_stamped(target_link, [-0.08, -0.011, 0.010] + rotation)

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
        offset = -0.01 if from_behind else 0.01
        self.b_bot.move_lin_rel([offset,0,0])
        self.insert_shaft(target_link, attempts=attempts-1, target=target, from_behind=from_behind)
      else:
        offset = -0.05 if from_behind else 0.05
        self.b_bot.move_lin_rel([offset,0,0])
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

    for _ in range(10):
      result = self.b_bot.do_insertion(target_pose_target_frame, insertion_direction=direction, force=15.0, timeout=10.0, 
                                                    radius=0.005, relaxed_target_by=0.005, selection_matrix=selection_matrix, 
                                                    check_displacement_time=3)
      success = result == TERMINATION_CRITERIA
      offset = -0.003 if from_behind else 0.003
      success &= self.b_bot.move_lin_rel(relative_translation = [offset,0,0], acceleration = 0.1, speed=.03) # Release shaft for next push
      if success:
        break

    if not self.b_bot.move_lin(pre_insertion_pose, speed=0.2):
      rospy.logerr("** Fail to return to pre insertion pose **")
      return False

    if not success:
      rospy.logerr("** Insertion Failed!! **")
      if attempts > 0:
        # Fall back if the shaft is stuck
        self.b_bot.gripper.open(wait=True)

        if from_behind:
          if target_link == "assembled_part_07_inserted":
            re_pick_pose = conversions.to_pose_stamped(target_link, [0.07, -0.002, 0.009] + rotation)
          else:
            re_pick_pose = conversions.to_pose_stamped(target_link, [0.07, 0.000, -0.002] + rotation)
        else:
          re_pick_pose = conversions.to_pose_stamped(target_link, [-0.07, 0.000, -0.002] + rotation)
        if not self.b_bot.move_lin(re_pick_pose, speed=0.05):
          rospy.logerr("** Fail to return to pre insertion pose **")
          return False
        self.b_bot.gripper.close(wait=True)

        if self.b_bot.gripper.opening_width < 0.004 and self.use_real_robot:
          rospy.logerr("Fail to grasp Shaft")
          self.b_bot.gripper.open(wait=True)
          return False

        return self.insert_shaft(target_link, attempts-1, target=target, from_behind=from_behind)
      return False
    return True

#### shaft orientation
  def orient_shaft(self):    
    if not self.centering_shaft():
      return False

    approach_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.100, 0, 0, tau/2., 0, 0])
    on_vgroove =       conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.000, 0, 0, tau/2., 0, 0])
    inside_vgroove =   conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.011, 0, 0, tau/2., 0, 0])

    if not self.b_bot.go_to_pose_goal(approach_vgroove, speed=1.0, move_lin=False):
      rospy.logerr("Fail to go to approach_vgroove")
      return False

    if not self.b_bot.go_to_pose_goal(on_vgroove, speed=0.5):
      rospy.logerr("Fail to go to on_vgroove")
      return False
    if not self.b_bot.go_to_pose_goal(inside_vgroove, speed=0.1):
      rospy.logerr("Fail to go to inside_vgroove")
      return False

    self.b_bot.gripper.open(opening_width=0.06)

    rospy.loginfo("Checking shaft hole")
    try: # Give up if the hole detection fails TODO: implement fallback
      shaft_has_hole_at_top = self.check_screw_hole_visible_on_shaft_in_v_groove()
    except Exception as e:
      print("fail to detect shaft hole", e)
      return False

    seq = []
    if shaft_has_hole_at_top:
      seq.append(helpers.to_sequence_item(inside_vgroove, speed=0.1))
      seq.append(helpers.to_sequence_gripper('close', gripper_force=40, gripper_velocity=1.0))
      seq.append(helpers.to_sequence_item(approach_vgroove, speed=0.3))
    else:  # Turn gripper around
      above_vgroove  = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.010, 0, 0, 0, 0, 0])
      inside_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.011, 0, 0, 0, 0, 0])
      seq.append(helpers.to_sequence_item_relative([0,0,0.03,0,0,0], speed=0.2))
      seq.append(helpers.to_sequence_item(above_vgroove, speed=0.3))
      seq.append(helpers.to_sequence_item(inside_vgroove, speed=0.2))
      seq.append(helpers.to_sequence_gripper('close', gripper_force=40, gripper_velocity=1.0))
      seq.append(helpers.to_sequence_item_relative([0,0,0.01,0,0,0], speed=0.2))
    
    self.execute_sequence("b_bot", seq, "orient shaft")
    self.b_bot.go_to_named_pose("home")
    self.b_bot.gripper.forget_attached_item() # clean attach/detach memory
    return True

  def centering_shaft(self):
    """ Push grasped shaft into the tray holder, and regrasp it centered.
    """
    shaft_length = 0.075
    approach_centering = conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length,     0.150, tau/2., tau/4., tau/4.])
    on_centering =       conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length,    -0.004, tau/2., tau/4., tau/4.])
    shaft_center =       conversions.to_pose_stamped("tray_left_stopper", [0.0, shaft_length/2., -0.008, tau/2., tau/4., tau/4.])

    seq = []
    seq.append(helpers.to_sequence_item(approach_centering, speed=1.0, linear=False))
    seq.append(helpers.to_sequence_item(on_centering, speed=0.5))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.03, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.0105, gripper_velocity=0.03))
    # seq.append(helpers.to_sequence_gripper('close', gripper_force=0, gripper_velocity=0.02))
    seq.append(helpers.to_sequence_item_relative([0, -0.055, 0, 0, 0, 0], speed=0.1))
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.03, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_item(shaft_center, speed=0.4, linear=True))
    def post_cb():
      shaft_pose = self.listener.transformPose("tray_center", self.b_bot.get_current_pose_stamped())
      shaft_pose.pose.position.y -= 0.035
      shaft_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, tau/4))
      self.markers_scene.spawn_item("shaft", shaft_pose)
      self.b_bot.gripper.attach_object("shaft")
    seq.append(helpers.to_sequence_gripper('close', gripper_force=40, gripper_velocity=1.0, post_callback=post_cb))

    if not self.execute_sequence("b_bot", seq, "centering_shaft"):
      rospy.logerr("Fail to center shaft or lost somewhere. Going back up and aborting.")
      self.b_bot.go_to_pose_goal(approach_centering)
      return False

    if not self.simple_gripper_check("b_bot", 0.004):
      rospy.logerr("No shaft detected in gripper. Going back up and aborting.")
      self.b_bot.go_to_pose_goal(approach_centering)
      return False

    self.b_bot.move_lin_rel(relative_translation=[0, 0.01, 0.1], speed=.3)
    return True

  def orient_shaft_end_cap(self, robot_name="a_bot", ignore_orientation=False):
    # Note only works for 'a_bot'
    # TODO: Make it work for both robots (not a priority)
    centering_frame = "left_centering_link" if robot_name == "a_bot" else "right_centering_link"
    place_pose = conversions.to_pose_stamped(centering_frame, [-0.005, 0, 0, -tau/4, 0, 0])
    
    if not self.simple_place(robot_name, place_pose, place_height=0.0, gripper_opening_width=0.03, 
                             lift_up_after_place=False, approach_height=0.15,
                             axis="x", sign=-1):
      return False
    
    # Look at the end cap to determine the orientation
    # ready_to_put_on_shaft = self.is_the_placed_end_cap_upside_down(dy=0.04, dz=-0.02)
    ready_to_put_on_shaft = False

    above_pose = conversions.to_pose_stamped("left_centering_link", [-0.1, 0, 0.0, -tau/4, 0, 0] )
    
    # self.a_bot.go_to_pose_goal(above_pose, speed=0.5)
    # self.a_bot.go_to_pose_goal(place_pose, speed=0.3, move_lin=True)
    self.center_with_gripper("a_bot", opening_width=0.05, gripper_force=0, gripper_velocity=0.01)
    self.a_bot.gripper.close(velocity=0.03, force=60)
    self.a_bot.go_to_pose_goal(above_pose, speed=0.5)

    if ignore_orientation:
      return True

    if not ready_to_put_on_shaft:  # Do reorientation procedure
      approach_centering = conversions.to_pose_stamped("simple_holder_tip_link", [0.0, 0, 0.1,      tau/4., tau/4., tau/4.])
      close_to_tip       = conversions.to_pose_stamped("simple_holder_tip_link", [-0.0075, 0,  0.01, tau/4., tau/4., tau/4.])
      push_down          = conversions.to_pose_stamped("simple_holder_tip_link", [-0.0075, 0, -0.02, tau/4., tau/4., tau/4.])
      prepare_second_push= conversions.to_pose_stamped("simple_holder_tip_link", [-0.030,  0,  0.05, tau/4., tau/4., tau/4.])
      close_to_edge      = conversions.to_pose_stamped("simple_holder",          [0.08, -0.10, 0.001, tau/4., tau/4., tau/4.])
      push_edge          = conversions.to_pose_stamped("simple_holder",          [0.02, -0.10, 0.001, tau/4., tau/4., tau/4.])
      
      seq = []
      seq.append(helpers.to_sequence_item(approach_centering, linear=False))
      seq.append(helpers.to_sequence_item(close_to_tip, speed=0.5))
      seq.append(helpers.to_sequence_item(push_down, speed=0.03))
      seq.append(helpers.to_sequence_item(prepare_second_push, speed=0.5))
      seq.append(helpers.to_sequence_item(close_to_edge, speed=0.5))
      seq.append(helpers.to_sequence_item(push_edge, speed=0.05))

      if not self.execute_sequence(robot_name, seq, "orient end cap"):
        rospy.logerr("Fail to go to orient end cap")
        return False

      self.a_bot.gripper.open(velocity=0.03, opening_width=0.03)
      if not self.center_with_gripper("a_bot", opening_width=0.05, clockwise=True, gripper_force=0, gripper_velocity=0.01):
        rospy.logerr("Fail to go to center with gripper")
        return False

      self.a_bot.gripper.close(velocity=0.03, force=60)
      
      if not self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.15]):
        rospy.logerr("Fail to go to centering_pose")
        return False
    else:
      self.a_bot.go_to_named_pose("centering_area")
    return True

  @lock_vision
  def is_the_placed_end_cap_upside_down(self, dy=0.0, dz=0.0, led_on=True):
    """ Look at the end cap placed next a few times, return.

        dy, dz add an offset to the camera position.
    """
    self.vision.activate_camera("a_bot_outside_camera")
    self.activate_led("a_bot", on=led_on)

    cam_height = -self.tray_view_low.pose.position.z + 0.05

    def go_and_record(view_pose, results):
      self.vision.activate_camera("a_bot_outside_camera")
      self.a_bot.go_to_pose_goal(view_pose, speed=0.5, end_effector_link="a_bot_outside_camera_color_frame")
      tries = 10
      while tries > 0:
        if self.get_3d_poses_from_ssd():
          break
        tries -= 1
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
    
    if not self.use_real_robot:
      return True

    if len(results) == 0:
      rospy.logerr("Failed to see the end cap!")
      if led_on:
        rospy.loginfo("Retry with LED off")
        return self.is_the_placed_end_cap_upside_down(dy, dz, led_on=False)
      return self.end_cap_is_upside_down # return value found at picking time

    print(">>> end_cap views result:", np.array(results))
    is_upside_down = np.mean(np.array(results) * 1) >= 0.5
    print(">>> is upsidedown?:", is_upside_down)

    return is_upside_down

  def pick_end_cap(self, attempts=5):
    """ Returns "Success" and "endcap_is_upside_down" (upside down = ready to be placed on shaft)
    """
    self.despawn_object("end_cap")
    options = {'check_for_close_items': True, 'declutter_with_tool': True, 'object_width': 0.005, 'grasp_width': 0.04}
    goal = self.look_and_get_grasp_point("end_cap", robot_name="a_bot", options=options)
    if not isinstance(goal, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Could not find shaft in tray. Skipping procedure.")
      return False
    self.end_cap_is_upside_down = copy.copy(self.object_in_tray_is_upside_down.get(self.assembly_database.name_to_id("end_cap"), False))

    goal.pose.position.z = -0.001 # Magic Numbers for grasping
    # goal.pose.position.x -= 0.01
    goal = self.listener.transformPose("tray_center", goal)  # This is not necessary
    marker_pose = copy.deepcopy(goal)
    marker_pose.pose.position.z = 0.007
    self.markers_scene.spawn_item("end_cap", marker_pose)

    self.vision.activate_camera("a_bot_inside_camera")  # Just for visualization
    self.allow_collisions_with_robot_hand("tray", "a_bot")
    if not self.simple_pick("a_bot", goal, axis="z", speed_fast=0.5, gripper_force=100.0, grasp_width=.04, 
                               approach_height=0.1, item_id_to_attach="end_cap", lift_up_after_pick=True, 
                               approach_with_move_lin=False, allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick")
      self.allow_collisions_with_robot_hand("tray", "a_bot", False)
      if attempts > 0:
        return self.pick_end_cap(attempts-1)
      return False
    self.allow_collisions_with_robot_hand("tray", "a_bot", False)

    if not self.simple_gripper_check("a_bot", min_opening_width=0.001):
      if attempts > 0:
        return self.pick_end_cap(attempts-1)
      return False

    return True

  def insert_end_cap(self, attempts=1):
    # self.a_bot.linear_push(force=2.5, direction="-Z", max_translation=0.02, timeout=10.0)
    # target_pose = self.a_bot.get_current_pose_stamped()
    # target_pose.pose.position.z -= 0.002
    target_pose = conversions.to_pose_stamped("tray_center", [-0.002, 0.002, 0.238]+np.deg2rad([-180, 90, -90]).tolist())
    self.a_bot.move_lin_rel(relative_translation=[0,0,0.001]) # release pressure before insertion

    selection_matrix = [0.15, 0.15, 0., 1.0, 1, 1]

    result = self.a_bot.do_insertion(target_pose, insertion_direction="-Z", force=4, timeout=15.0, 
                                  radius=0.001, revolutions=4, relaxed_target_by=0.001, selection_matrix=selection_matrix,
                                  check_displacement_time=3., displacement_epsilon=0.0005)
    success = result in (TERMINATION_CRITERIA)

    if result == DONE and attempts > 0:
      return self.insert_end_cap(attempts=attempts-1)

    return success

  def fasten_end_cap(self, skip_unequip_tool=False):
    self.vision.activate_camera("a_bot_outside_camera")

    if not self.do_change_tool_action("a_bot", equip=True, screw_size = 4):
      rospy.logerr("Failed equip m4")
      return False

    # We use the collision object to guide the screwing 
    self.despawn_object("shaft")
    obj = self.assembly_database.get_collision_object("shaft")
    obj.header.frame_id = "b_bot_gripper_tip_link"
    obj.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(tau/4, -tau/4, -tau/4))
    obj.pose = helpers.rotatePoseByRPY(0, 0, tau/2, obj.pose)
    obj.pose.position = conversions.to_point([-.006, 0, .038])
    self.planning_scene_interface.add_object(obj)
    self.planning_scene_interface.allow_collisions("shaft")
    self.b_bot.gripper.attach_object(obj.id, with_collisions=True)
    
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
    above_hole_screw_pose = conversions.to_pose_stamped("tray_center", [-0.003, 0.036, 0.4]+np.deg2rad([180, 30, 90]).tolist())
    if not self.a_bot.go_to_pose_goal(above_hole_screw_pose, speed=0.2, move_lin=True):
      rospy.logerr("Fail to go to above_hole_screw_pose")
      return False

    self.confirm_to_proceed("try to screw")
    self.vision.activate_camera("b_bot_inside_camera")
    hole_screw_pose = conversions.to_pose_stamped("move_group/shaft/screw_hole", [0.0, 0.002, -0.002, 0, 0, 0])
    hole_screw_pose_transformed = self.get_transformed_collision_object_pose("shaft/screw_hole", hole_screw_pose, "right_centering_link")
    self.screw("a_bot", hole_screw_pose_transformed, screw_height=0.02, screw_size=4, skip_final_loosen_and_retighten=False, attempts=0, spiral_radius=0.0025)
    
    self.confirm_to_proceed("unequip tool")
    self.tools.set_suction("screw_tool_m4", suction_on=False, wait=False)

    if not self.a_bot.go_to_named_pose("screw_ready"):
      return False
    
    if not skip_unequip_tool:
      if not self.do_change_tool_action("a_bot", equip=False, screw_size = 4):
        rospy.logerr("Failed unequip m4")
        return False
    
    # Go back to a simple marker
    self.despawn_object("shaft")
    shaft_pose = conversions.to_pose_stamped("b_bot_gripper_tip_link", conversions.from_pose_to_list(obj.pose))
    self.markers_scene.spawn_item("shaft", shaft_pose)
    self.b_bot.gripper.attach_object("shaft")

    return True

  ######## Motor

  def pick_and_store_motor(self):
    """ Half-blindly picks the motor and places it away from the tray.
    """
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 10:
      picked = self.attempt_motor_tray_pick()
      if not picked:
        attempt_nr += 1
        continue
      
      # Place motor in storage area
      p_through = conversions.to_pose_stamped("right_centering_link", [-0.15, -0.05, -0.05, -tau/4, 0, 0])
      p_drop    = conversions.to_pose_stamped("right_centering_link", [-0.03, -0.05, -0.05, -tau/4, 0, 0])
      self.b_bot.go_to_pose_goal(p_through, speed=1.0, wait=True)
      self.b_bot.go_to_pose_goal(p_drop, speed=0.5, wait=True)
      self.b_bot.gripper.open()
      self.despawn_object("motor")
      self.b_bot.go_to_pose_goal(p_through, speed=1.0, wait=True)

      if not picked:
        attempt_nr += 1
        continue
    return picked

  def pick_motor(self):
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 10:
      picked = self.attempt_motor_tray_pick()
      if not picked:
        attempt_nr += 1
        continue
    return picked

  def orient_motor(self):
    p_drop    = conversions.to_pose_stamped("right_centering_link", [-0.03, -0.05, -0.05, -tau/4, 0, 0])
    seq = []
    seq.append(helpers.to_sequence_item("centering_area", speed=0.8))
    seq.append(helpers.to_sequence_item(p_drop, speed=0.8))
    seq.append(helpers.to_sequence_gripper('open', gripper_velocity=1.0))
    if not self.execute_sequence("b_bot", seq, "orient_motor"):
      rospy.logerr("Fail to orient motor")
      self.b_bot.gripper.open()
      return False
    return self.confirm_motor_and_place_in_aid()

  def pick_and_orient_motor(self):
    picked = False
    attempt_nr = 0
    while not picked and attempt_nr < 10:
      picked = self.pick_motor()
      # Place motor in centering area
      picked = self.orient_motor()
      if not picked:
        attempt_nr += 1
        continue
    return picked

  def attempt_motor_tray_pick(self, robot_name="b_bot"):
    """ Do one pick attempt, ignoring the motor's orientation.
    """
    self.despawn_object("motor")
    options = {'grasp_width': 0.085, 'object_width': 0.04, 'check_for_close_items': False, 'check_too_close_to_border': False, 'center_on_corner': True}
    object_pose = self.look_and_get_grasp_point("motor", robot_name, options)
    if not isinstance(object_pose, geometry_msgs.msg.PoseStamped):
      rospy.logerr("Motor not found or not graspable")
      return False
    dx, dy = self.distances_from_tray_border(object_pose)
    print("Motor distance from tray's borders", dx, dy)
    if dx > 0.08 and dy > 0.08:
      # If not close to any tray border, attempt a grasp considering the cables orientation
      rgb_theta = None
      tries = 10
      while not rgb_theta and tries > 0:
        rgb_theta = self.vision.get_motor_angle_from_top_view(camera="b_bot_outside_camera")
        tries -= 1
      if rgb_theta is None:
        rospy.logerr("Motor not detected by `get motor angle`! Attempting blind grasp")
      else:
        rgb_theta = rgb_theta - tau/2 # rotate 180, align camera to motor tip
        rgb_theta = (rgb_theta) % tau # wrap angle to range [0, TAU]
        object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, tau/4, rgb_theta))
    motor_pose = copy.deepcopy(object_pose)
    motor_pose.pose.position.z = 0.035
    self.markers_scene.spawn_item("motor", motor_pose)
    self.planning_scene_interface.allow_collisions("motor")
    object_pose.pose.position.z = 0.014
    self.simple_pick(robot_name, object_pose, gripper_force=100.0, grasp_width=.085, axis="z", 
                     item_id_to_attach="motor", attach_with_collisions=False, allow_collision_with_tray=True)
    if not self.simple_gripper_check(robot_name, min_opening_width=0.031):
      rospy.logerr("Motor not successfully grasped")
      return False
    return True

  def motor_centering_fallback(self, remaining_attempts=3):
    self.despawn_object("motor")
    motor_placed, motor_pose = self.find_motor_centering_area()
    if not motor_placed:
      rospy.logerr("Motor not found during fallback. Abort.")
      self.b_bot.gripper.open(wait=False)
      return False
    motor_pose = self.listener.transformPose("right_centering_link", motor_pose)
    motor_pose.pose.position.x = -0.01  # Grasp height
    motor_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(-tau/4, 0, 0))
    if not self.simple_pick("b_bot", object_pose=motor_pose, grasp_width=.085, 
                            axis="x", sign=-1, approach_height=0.07, minimum_grasp_width=0.02,):
      rospy.logerr("Fail picking during fallback. Abort.")
      self.b_bot.gripper.open(wait=False)
      self.b_bot.go_to_named_pose("home")
      return False
    if not self.simple_gripper_check(robot_name="b_bot", min_opening_width=0.02) and remaining_attempts > 0:
      self.b_bot.gripper.open(wait=False)
      self.motor_centering_fallback(remaining_attempts=remaining_attempts-1)
    self.drop_in_tray("b_bot")
    return False

  @lock_vision
  def find_motor_centering_area(self):
    """ Return pose of motor if seen in centering area """
    motor_placed = False
    remaining_tries = 10
    while not motor_placed and remaining_tries > 0:
      p_view = conversions.to_pose_stamped("right_centering_link", [-self.tray_view_high.pose.position.z, -0.05, -0.05, 0, 0, 0])
      p_view = self.listener.transformPose("tray_center", p_view)
      p_view.pose.orientation = self.tray_view_high.pose.orientation
      self.vision.activate_camera("b_bot_outside_camera")
      self.b_bot.go_to_pose_goal(p_view, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, wait=True, move_lin=True)

      if not self.use_real_robot:
        return True, conversions.to_pose_stamped("right_centering_link", [-0.035,  -0.05, -0.05, 0, 0, 0])

      # Look at the motor from above, confirm that SSD sees it
      rospy.sleep(1.0)
      tries = 10
      res = None
      while tries > 0:
        res = self.get_3d_poses_from_ssd()
        if res:
          break
        tries -= 1
      try:
        motor_id = self.assembly_database.name_to_id("motor")
        motor_placed = (motor_id in res.class_ids)
      except:
        motor_placed = False
      remaining_tries -= 1
      if motor_placed:
        break
    return motor_placed, self.objects_in_tray.get(motor_id, None)

  @lock_vision
  def confirm_motor_pose(self, calibration=False, use_cad=False):
    """ 
      Confirm the position of the motor in the centering area
      calibration ON: Skip any fallbacks
      use_cad ON: Use cad to predict orientation of the motor (Optionally use the RGB matching to confirm)
      use_cad OFF: Use the RGB matching to confirm the orientation of the motor's cables
    """
    motor_placed, motor_pose = self.find_motor_centering_area()
    
    if not use_cad and motor_pose:
      visual_motor_pose = copy.deepcopy(motor_pose)
      visual_motor_pose.pose.position.z = -0.035
      self.markers_scene.spawn_item("motor", visual_motor_pose)

    if not motor_placed and not calibration:
      rospy.logerr("Motor not detected by SSD! Return item and abort")
      self.motor_centering_fallback()
      return False

    # Get a closer look for the detection of the motor's orientation
    p_view = conversions.to_pose_stamped("right_centering_link", [-0.30, -0.05, -0.05, 0, 0, 0])
    p_view = self.listener.transformPose("tray_center", p_view)
    p_view.pose.orientation = self.tray_view_high.pose.orientation
    self.b_bot.go_to_pose_goal(p_view, end_effector_link="b_bot_outside_camera_color_frame", speed=.5, wait=True)
    
    # Use CAD matching to determine orientation
    if use_cad:
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
      motor_pose = self.listener.transformPose("right_centering_link", p_motor_in_tray_center)
      motor_pose.pose.position.x = -0.006  # Grasp height
      
      # Check that motor direction matches the cables seen in RGB image
      rgb_theta = self.vision.get_motor_angle_from_top_view(camera="b_bot_outside_camera")
      q = motor_pose.pose.orientation
      rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
      rgb_theta = rgb_theta - tau/2 # rotate 180, align camera to motor tip
      rgb_theta = rgb_theta % tau # wrap angle to range [0, TAU]
      cad_theta = rpy[0] % tau # wrap angle to range [0, TAU]
      print("rgb_theta ", rgb_theta, "cad_theta", cad_theta)
      # If arrow and motor do not point in the same direction
      if rgb_theta and abs(rgb_theta - cad_theta) > tau/4: # If there is a big difference, discard CAD and rely on rgb estimation
        motor_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(rgb_theta, 0, 0))
      self.planning_scene_interface.remove_world_object("motor")
    else:
      # Use RGB only
      rgb_theta = None
      tries = 10
      while not rgb_theta and tries > 0:
        rgb_theta = self.vision.get_motor_angle_from_top_view(camera="b_bot_outside_camera")
        tries -= 1
      if rgb_theta is None:
        rospy.logerr("Motor not detected by `get motor angle`! Return item and abort")
        self.motor_centering_fallback()
        return False
      rgb_theta = rgb_theta - tau/2 # rotate 180, align camera to motor tip
      rgb_theta = (rgb_theta) % tau # wrap angle to range [0, TAU]
      motor_pose = self.listener.transformPose("right_centering_link", motor_pose)
      motor_pose.pose.position.x = -0.007  # Grasp height
      motor_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(rgb_theta, 0, 0))
    
    if motor_pose:
      visual_motor_pose = copy.deepcopy(motor_pose)
      visual_motor_pose.pose.position.x = -0.035
      self.markers_scene.spawn_item("motor", visual_motor_pose)

    return motor_pose

  def confirm_motor_and_place_in_aid(self, calibration=False):
    """ Assumes that the motor is grasped in a random orientation. 
        Places it in the separate area, determines the orientation, centers it, and places it in the vgroove.
    """
    p_motor_in_centering_link = self.confirm_motor_pose(calibration=calibration)
    if not p_motor_in_centering_link:
      # No need for more fallbacks
      return False

    # Pick motor in defined orientation and move up
    if not self.simple_pick("b_bot", object_pose=p_motor_in_centering_link, grasp_width=.08, 
                            axis="x", sign=-1, approach_height=0.07, minimum_grasp_width=0.02,
                            item_id_to_attach="motor"):
      rospy.logerr("Fail to pick motor")
      self.motor_centering_fallback()
      return False
    
    # For safety of the cameras, urscript is blind of them
    # pre_orient_pose = conversions.to_pose_stamped("right_centering_link", [-0.05, 0.0, 0.0, radians(-90), 0, 0])
    pre_orient_pose = [1.84255, -1.90216, 2.56565, -2.23376, -1.56659, -1.29859] # safer with joint target
    if not self.b_bot.move_joints(pre_orient_pose, speed=1.0):
      rospy.logerr("Fail to go to urscript start pose")
      self.motor_centering_fallback()
      return False

    self.confirm_to_proceed("motor picked?")

    if not calibration:
      if not self.orient_motor_in_aid_edge():
        self.b_bot.close_ur_popup()
        self.motor_centering_fallback()
        return False
    
    self.b_bot.gripper.forget_attached_item()
    motor_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.01, -0.02, -0.005, tau/2, 0, tau/4])
    self.markers_scene.spawn_item("motor", motor_pose)
    return True

  def orient_motor_in_aid_edge_urscript(self):
    if not self.use_real_robot:
      rospy.sleep(15.0) # assume time needed for centering with urscript
      return True
    if not self.b_bot.load_and_execute_program(program_name="wrs2020/motor_orient_2.urp"):
      rospy.logerr("Fail to orient with URScript")
      return False
    return helpers.wait_for_UR_program("/b_bot", rospy.Duration.from_sec(60))

  def flip_motor_in_aid(self, recursion=False):
    """ 
    Turn motor in aid by 180 degrees. 
    """
    approach_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ -0.03, 0.005, 0,  tau/2., 0, 0])
    grasp_pose    = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.004, 0.008, -0.000] + np.deg2rad([180, 3.002, 60.047]).tolist())
    drop_pose     = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.0035, -0.0042, -0.000] + np.deg2rad([-179.988, 3.051, -29.952]).tolist())
    self.b_bot.go_to_pose_goal(approach_pose)
    self.b_bot.go_to_pose_goal(grasp_pose, move_lin=True, speed=0.4, retime=True)
    self.b_bot.gripper.close(velocity=0.01)
    self.b_bot.move_lin_rel(relative_translation=[0,0,0.05], speed=0.4, retime=True)
    self.b_bot.move_lin_rel(relative_rotation=[tau/4,0,0], speed=1.0, retime=True)
    self.b_bot.go_to_pose_goal(drop_pose, speed=0.02)
    self.b_bot.gripper.open(velocity=0.01)
    self.b_bot.go_to_pose_goal(approach_pose, move_lin=True, speed=0.4, retime=True)
    if not recursion:
      return self.flip_motor_in_aid(recursion=True)
    align_pose = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [0.0, 0.0, -0.0528,  tau/2., 0, 0])
    self.b_bot.go_to_pose_goal(align_pose, move_lin=True, speed=1.0, retime=True)
    for _ in range(5):
      self.b_bot.gripper.close(force=100)
      self.b_bot.gripper.open()
    self.b_bot.go_to_pose_goal(approach_pose, move_lin=True, speed=1.0, retime=True)
    return True

  @lock_impedance
  def insert_motor(self, target_link, attempts=1):
    inclination = radians(28)
    target_pose  = conversions.to_pose_stamped(target_link, [-0.028, -0.004, -0.0155, -tau/4, tau/4-inclination, -tau/4])
    selection_matrix = [0., 0.2, 0.2, 1, 1, 1]
    self.b_bot.linear_push(2, "+X", max_translation=0.05, timeout=15.0)
    result = self.b_bot.do_insertion(target_pose, insertion_direction="+X", force=8.0, timeout=20.0, 
                                      wiggle_direction=None, wiggle_angle=np.deg2rad(3.0), wiggle_revolutions=1.0,
                                      radius=0.003, relaxed_target_by=0.002, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA

    if not success:
      rospy.logerr("** Insertion Failed!! **")
      if attempts > 0:
        rospy.loginfo("** Insertion try **")
        self.allow_collisions_with_robot_hand("panel_bearing", "b_bot")
        self.allow_collisions_with_robot_hand("panel_motor", "b_bot")
        self.b_bot.move_lin_rel(relative_translation=[-0.03,0,0], speed=0.02)
        pre_insertion_pose = copy.deepcopy(target_pose)
        pre_insertion_pose.pose.position.x = -0.02
        if not self.b_bot.go_to_pose_goal(pre_insertion_pose, speed=0.1):
          rospy.logerr("fail to go pre_insertion")
          return False
        self.allow_collisions_with_robot_hand("panel_motor", "b_bot", False)
        self.allow_collisions_with_robot_hand("panel_bearing", "b_bot", False)
        return self.insert_motor(target_link, attempts-1)
      return False

    if self.use_real_robot:
      self.b_bot.linear_push(10, "+X", max_translation=0.01, timeout=10.0)
    push_direction = "+Z" if not self.assembly_database.assembly_info.get("motor_shaft_down", False) else "-Z"
    self.b_bot.linear_push(4, push_direction, max_translation=0.01, timeout=10.0)
    self.b_bot.gripper.forget_attached_item()
    return True

  def center_motor(self):
    """ Push grasped motor into the tray holder, and regrasp it centered.
    """
    motor_length = 0.035
    approach_centering = conversions.to_pose_stamped("tray_left_stopper", [0.0, motor_length,     0.05, tau/2., tau/4., tau/4.])
    on_centering =       conversions.to_pose_stamped("tray_left_stopper", [0.0, motor_length,    -0.004, tau/2., tau/4., tau/4.])
    p_pick =             conversions.to_pose_stamped("right_centering_link", [-0.006, -0.081, 0.043, 0, 0, 0])
    
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
      rospy.logerr("Fail to go to motor repick position")
      return False

    self.b_bot.gripper.close()
    if not self.b_bot.gripper.opening_width > 0.025:
      rospy.logerr("No motor detected in gripper. Going back up and aborting.")
      self.b_bot.go_to_pose_goal(approach_centering)
      return False
  
    return True

  def orient_motor_in_aid_edge(self, use_urscript=True):
    """ Starting with the motor picked (after center_motor), drop motor in vgroove aid's edge a number of times,
        to align the shaft near the top.
    """
    # ALTERNATIVE A:
    if use_urscript:
      # TODO: Confirm that the motor is indeed in the vgroove after urscript
      return self.orient_motor_in_aid_edge_urscript()
    # ALTERNATIVE B:
    self.center_motor()
    
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

  def align_motor_pre_insertion(self, simultaneous=False):
    """ Assuming we are in the vgroove aid drop or close
        Grasp carefully the motor
        Put in in front of the motor plate hole ready for insertion
    """
    self.allow_collisions_with_robot_hand("motor", "b_bot")
    self.b_bot.gripper.open()
    inclination = radians(28)
    inside_vgroove = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [ 0.0, 0.005, 0,  tau/2., radians(3.0), inclination])
    above_vgroove  = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.2, 0, 0,      tau/2., radians(3.0), inclination])
    midpoint1      = conversions.to_pose_stamped("vgroove_aid_drop_point_link", [-0.25, 0.1, 0.4, tau/2,  radians(3.0), inclination])
    midpoint2      = conversions.to_pose_stamped("assembled_part_02_back_hole", [-0.100, 0, 0.1, -tau/4, tau/4-inclination, -tau/4])
    pre_insertion  = conversions.to_pose_stamped("assembled_part_02_back_hole", [-0.055, -0.004, -0.0155, -tau/4, tau/4-inclination, -tau/4])
    
    if simultaneous:
      # TODO(cambel): do we need midpoint 1 here?
      self.b_bot.go_to_pose_goal(above_vgroove, speed=1.0)
    self.b_bot.go_to_pose_goal(inside_vgroove, speed=0.2, move_lin=True)
    self.confirm_to_proceed("")
    self.b_bot.gripper.close(force=70, velocity=0.01)
    if not self.simple_gripper_check("b_bot", min_opening_width=0.01):
      self.b_bot.gripper.open()
      self.b_bot.go_to_pose_goal(above_vgroove, move_lin=True)
      rospy.logerr("Fail to grasp motor, did it fall of the vgroove?")
      return False

    self.b_bot.gripper.attach_object("motor")
    self.b_bot.move_lin_rel([0,0,0.03], speed=0.2)
    # seq = []
    # seq.append(helpers.to_sequence_item(above_vgroove, speed=0.2))
    # seq.append(helpers.to_sequence_item(midpoint1, speed=1.0, linear=False))
    # seq.append(helpers.to_sequence_item(midpoint2, speed=1.0, linear=False))
    # seq.append(helpers.to_sequence_item(pre_insertion, speed=0.8))
    # return self.execute_sequence("b_bot", seq, "align motor pre insertion")
    waypoints = []
    waypoints.append((self.b_bot.compute_ik(above_vgroove, timeout=0.02, retry=True), 0, 1.0))
    waypoints.append((self.b_bot.compute_ik(midpoint1, timeout=0.02, retry=True), 0, 1.0))
    waypoints.append((self.b_bot.compute_ik(midpoint2, timeout=0.02, retry=True), 0, 1.0))
    waypoints.append((self.b_bot.compute_ik(pre_insertion, timeout=0.02, retry=True), 0, 1.0))
    return self.b_bot.move_joints_trajectory(waypoints)
    # seq.append(helpers.to_sequence_joint_trajectory([midpoint1, midpoint2, pre_insertion], speed=[1,1,1.0]))
  
  def align_motor_holes(self):
    """ Starts with the gripper holding the motor at the assembled position after insertion
    """
    # Spawn a motor at the target position
    # Attach it to the robot, disable collisions
    # Rotate motor using the subframe at the tip of the motor
    # Call vision action, get best position

    p = conversions.to_pose_stamped("assembled_part_04", [0, 0, 0, 0, 0 ,0])
    self.spawn_object(object_name="motor", object_pose=p)
    self.b_bot.gripper.attach_object("motor", with_collisions=True)
    def rotate_inserted_motor_to(target_angle):
      target_p = conversions.to_pose_stamped("assembled_part_04_tip", [0, 0, 0, target_angle, 0 ,0])
      self.b_bot.go_to_pose_goal(target_p, speed=.005, move_lin = True, end_effector_link="motor/tip")
      pass
    
    # Call vision action for each angle, record result
    
    return True

  def fasten_motor(self, robot_name="a_bot", support_robot="b_bot", simultaneous=False, part1=True, part2=True):
    """ Assumes that the tool is already equipped """
    assert robot_name != support_robot, "Same robot cannot fill both roles"
    offset = -1 if robot_name == "a_bot" else 1
    screw_order = [6,4,2,1,3,5] if not self.assembly_database.assembly_info.get("motor_shaft_down", False) else [4,6,2,3,1,5]
    screw_poses = []
    for i in screw_order: # interlock order
      if self.assembly_database.db_name == "wrs_assembly_2021":
        screw_pose = conversions.to_pose_stamped("assembled_part_02_motor_screw_hole_%s"%i, [0.007, 0, -0.0035, offset*tau/12, 0, 0])
      else:
        screw_pose = conversions.to_pose_stamped("assembled_part_02_motor_screw_hole_%s"%i, [0.007, -0.001, -0.0045, offset*tau/12, 0, 0])
      screw_poses.append(screw_pose)

    # To avoid b_bot camera with the tool cable
    intermediate_pose = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.05, -0.15, 0.006, radians(150), 0, tau/2])

    if part1:
      # Attempt first screw only
      success = self.pick_and_fasten_screw(robot_name, screw_poses[0], screw_size=3, 
                                           intermediate_pose=intermediate_pose, attempts=0, 
                                           duration=30, spiral_radius=0.002, save_plan_on_success=False)
      eef = robot_name+"_screw_tool_m3_tip_link"
      waypoints = []
      rel_pose = self.a_bot.move_lin_rel([-0.03,0,0], relative_to_tcp=True, pose_only=True, end_effector_link=eef)
      waypoints.append((self.active_robots[robot_name].compute_ik(rel_pose, timeout=0.02, retry=True, end_effector_link = eef), 0, 0.1))
      waypoints.append((self.active_robots[robot_name].compute_ik(intermediate_pose, timeout=0.02, retry=True, end_effector_link = eef), 0, 0.1))
      waypoints.append(("horizontal_screw_ready", 0, 1.0))
      waypoints.append(("screw_ready",      0, 1.0))
      if not self.active_robots[robot_name].move_joints_trajectory(waypoints):
        rospy.logerr("Fail to go to back from screwing(%s)" % robot_name)
        return False

      if not success:
        # Fallback: Return False here for b_bot to retry the motor insertion
        return False
      
      if not self.fasten_set_of_screws([screw_poses[1]], screw_size=3, robot_name=robot_name, only_retighten=False,
                                      skip_intermediate_pose=False, simultaneous=simultaneous, skip_return=True,
                                      intermediate_pose=intermediate_pose, unequip_when_done=False, attempts=0):
        rospy.logerr("Fail to fasten second screw of motor")
        if not self.active_robots[robot_name].move_joints_trajectory(waypoints):
          rospy.logerr("Fail to go to back from screwing(%s)" % robot_name)
        return False
      
      # Retighten first two screws
      self.fasten_set_of_screws(screw_poses[:2], screw_size=3, robot_name=robot_name, only_retighten=True,
                                skip_intermediate_pose=True, simultaneous=simultaneous, skip_return=False,
                                intermediate_pose=intermediate_pose, unequip_when_done=False, attempts=0)

      # Let's release b_bot from holding the motor
      seq = []
      seq.append(helpers.to_sequence_gripper('open', gripper_velocity=1.0))
      seq.append(helpers.to_sequence_item_relative([-0.02,0.1,0.1,0,0,0]))
      seq.append(helpers.to_sequence_item("home", speed=0.6))
      self.execute_sequence(support_robot, seq, "fasten_motor_b_bot_return")

    if part2:
      # To avoid b_bot camera with the tool cable
      intermediate_pose = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.05, -0.1, 0.006, radians(150), 0, tau/2])
      screw_set_center_pose = conversions.to_pose_stamped("assembled_part_05_center", [-0.02, 0, 0, offset*tau/12, 0, 0])
      # Finish remaining screws
      if not self.fasten_set_of_screws(screw_poses[2:], screw_size=3, robot_name=robot_name, only_retighten=False,
                                      skip_intermediate_pose=False, simultaneous=simultaneous, with_extra_retighten=False,
                                      intermediate_pose=intermediate_pose, screw_set_center_pose=screw_set_center_pose):
        rospy.logerr("Fail to fasten remaining screws of motor")
        return False

    return True

  def fasten_motor_fallback(self):
    """"
      Reverse insertion with B_BOT
      Re align motor, and try insertion again
      Then fasten again
    """
    seq = []
    seq.append(helpers.to_sequence_item_relative([-0.04,0,0,0,0,0], speed=0.01)) # go back very slowly / reverse insertion
    seq.append(helpers.to_sequence_item_relative([ 0.0,0.05,0.15,0,0,0], speed=0.5))
    above_centering = conversions.to_pose_stamped("right_centering_link", [-0.1, 0, 0, -tau/4., 0, 0])
    at_centering    = conversions.to_pose_stamped("right_centering_link", [-0.02, 0, 0, -tau/4., 0, 0])
    re_picking      = conversions.to_pose_stamped("right_centering_link", [-0.005, 0, 0, -tau/4., 0, 0])
    # go to centering area
    seq.append(helpers.to_sequence_item(above_centering, speed=0.8))
    seq.append(helpers.to_sequence_item(at_centering, speed=0.8))
    # drop motor
    seq.append(helpers.to_sequence_gripper('open', gripper_velocity=1.0, wait=False))
    self.execute_sequence("b_bot", seq, "fasten_motor_fallback")
    # pick it again, necessary for urscript
    self.simple_pick("b_bot", object_pose=re_picking, grasp_width=.08, axis="x", sign=-1, approach_height=0.07, lift_up_after_pick=False)
    # re orient
    if not self.orient_motor_in_aid_edge():
      rospy.logerr("Fail fallback. abort")
      return False
    if not self.align_motor_pre_insertion():
      rospy.logerr("Fail fallback, part 2")
      return False
    if not self.insert_motor("assembled_part_02_back_hole"):
      rospy.logerr("Fail fallback, part 3")
      return False
    
    return self.fasten_motor(simultaneous=False)

  def insert_motor_cables(self, cable_color="black"):
    """
      insert motor cable in termina
      cable_color: "black" or "red"
    """
    if cable_color not in ["black", "red"]:
      raise ValueError("invalid cable %s" % cable_color)
    
    switch_panels_order = self.assembly_database.assembly_info.get("switched_motor_and_bearing", False)

    if cable_color == "black":
      pin_frame_id = "assembled_part_01_cable_pin3" if not switch_panels_order else "assembled_part_01_cable_pin2"
    else:
      pin_frame_id = "assembled_part_01_cable_pin4" if not switch_panels_order else "assembled_part_01_cable_pin1"

    cable_length = -0.1

    frame_id = "assembled_part_04_black_cable_connector"
    a_bot_approach        = conversions.to_pose_stamped(frame_id, [-0.0196, 0.0654,-0.0550, 0.24386819, -0.67967013, -0.2553104,  0.64295678])
    a_bot_at_cable        = conversions.to_pose_stamped(frame_id, [-0.0138,-0.0029, 0.0210, 0.25176847, -0.65889104, -0.24766403, 0.66418203])
    a_bot_wait_for_b      = conversions.to_pose_stamped(frame_id, [-0.0450,-0.0029, 0.0210, 0.25177093, -0.65888475, -0.24766217, 0.66418803])
    a_bot_waypoint1       = conversions.to_pose_stamped(frame_id, [-0.2030, 0.0018, 0.0151, 0.22084576, -0.74102362, -0.2770966, 0.57037586])
    a_bot_high_back       = conversions.to_pose_stamped(frame_id, [-0.0940, 0.1611,-0.1636, 0.2437983283352903, -0.6796027356802858, -0.255514920422451, 0.6429732670945957])
    a_bot_above_cable_end = conversions.to_pose_stamped(frame_id, [-0.0740, 0.0367,-0.0352, -0.16547481426477773, 0.8274737495079801, 0.31185939044806815, -0.4366337141227206])
    a_bot_at_cable_end    = conversions.to_pose_stamped(frame_id, [-0.0870, 0.0113,-0.0142, -0.15961350423619117, 0.8350845585963815, 0.3149209676802494, -0.4218792402745907])
    a_bot_midpoint        = conversions.to_pose_stamped(frame_id, [-0.0260,-0.0045,-0.0028, 0.15402837312781534, -0.7082824722199389, -0.2971464525247976, 0.62154258548352])
    a_bot_above_hole2     = conversions.to_pose_stamped(pin_frame_id, [-0.0200, 0.0002,-0.0031, 0, 0, 0, 0])

    b_bot_high_back       = conversions.to_pose_stamped(frame_id, [-0.0238,-0.1282,-0.0689, 0.6899835657316222, 0.11914032194144523, 0.6950818535198647, 0.1630628088049338])
    b_bot_above_cable     = conversions.to_pose_stamped(frame_id, [-0.0138,-0.0109,-0.0400, 0.690004249738519, 0.11929506126840099, 0.6950668177408967, 0.16292618750616827])
    b_bot_at_cable        = conversions.to_pose_stamped(frame_id, [-0.0135, 0.0131, 0.0162, 0.689987773146987, 0.11931277548834202, 0.695081113732324, 0.16292200540615387])
    b_bot_hold_cable      = conversions.to_pose_stamped(frame_id, [-0.0924, 0.0101, 0.0029, 0.7071360069944063, 0.55517288386069, 0.43636119671648077,-0.036477974696698945])
    b_bot_Waypoint_1      = conversions.to_pose_stamped(frame_id, [-0.1287,-0.0918, 0.0158, 0.6403967731232784, 0.618262549049528, 0.33913201040299534,-0.3043564898866053])
    b_bot_Waypoint_2      = conversions.to_pose_stamped(frame_id, [-0.0879,-0.1159,-0.0572, 0.6403968097964423, 0.6182626350195856, 0.3391318726899746,-0.3043563915329672])
    b_bot_look_at_cable   = conversions.to_pose_stamped(frame_id, [-0.0378,-0.1581, 0.0497, 0.609497539310126, 0.6163826114272067, 0.3506917013529725,-0.3544016880705233])

    self.a_bot.gripper.open(opening_width=0.01, wait=False)
    self.b_bot.gripper.open(opening_width=0.01, wait=True)

    self.ab_bot.go_to_goal_poses(a_bot_approach, b_bot_high_back, speed=1.0)
    self.ab_bot.go_to_goal_poses(a_bot_at_cable, b_bot_above_cable, speed=1.0)
    self.a_bot.gripper.send_command(0.005)
    
    self.a_bot.go_to_pose_goal(a_bot_wait_for_b, move_lin=True, speed=0.1)
    self.b_bot.go_to_pose_goal(b_bot_at_cable, move_lin=True, speed=0.1)

    self.b_bot.gripper.send_command(0.005)

    target_pose = self.a_bot.move_lin_rel([cable_length,0,0], speed=0.05, pose_only=True)
    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", target_pose, slave_relation, speed=0.1)
    self.b_bot.gripper.close()

    self.a_bot.gripper.send_command(0.02, wait=False)
    self.a_bot.go_to_pose_goal(a_bot_waypoint1, speed=1.0)

    self.ab_bot.go_to_goal_poses(a_bot_high_back, b_bot_hold_cable, speed=1.0)

    self.a_bot.go_to_pose_goal(a_bot_above_cable_end, move_lin=True, speed=0.5)
    self.a_bot.go_to_pose_goal(a_bot_at_cable_end, move_lin=True, speed=0.5)
    self.a_bot.gripper.close(force=150)
    self.b_bot.gripper.open(opening_width=0.03)
    self.b_bot.go_to_pose_goal(b_bot_Waypoint_1, move_lin=True, speed=0.5)
    def b_bot_task():
      self.b_bot.go_to_pose_goal(b_bot_Waypoint_2, move_lin=True, speed=0.5)
      self.b_bot.go_to_pose_goal(b_bot_look_at_cable, move_lin=True, speed=0.5)
    def a_bot_task():
      self.a_bot.go_to_pose_goal(a_bot_midpoint, move_lin=True, speed=0.5)
      self.a_bot.go_to_pose_goal(a_bot_above_hole2, move_lin=True, speed=0.5)
      self.a_bot.move_lin_rel([0.021,0,0], relative_to_tcp=True, speed=0.01)
      self.a_bot.gripper.open(opening_width=0.02)
      self.a_bot.move_lin_rel([-0.021,0,0], relative_to_tcp=True, speed=0.1)
      
    self.do_tasks_simultaneous(a_bot_task, b_bot_task)
    # TODO confirm that the insertion was successful


  def equip_cable_tool(self, robot_name="a_bot"):
    pickup_pose   = conversions.to_pose_stamped("cable_holder_pickup_link", [0.02, 0, 0, 0, 0, 0])
    approach_pose = conversions.to_pose_stamped("cable_holder_pickup_link", [-0.1, 0, 0.1, 0, 0, 0])
    self.active_robots[robot_name].gripper.open(wait=False)
    if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0):
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(pickup_pose, speed=1.0, move_lin=True, retime=True):
      return False
    self.spawn_tool("cable_tool")
    self.allow_collisions_with_robot_hand("cable_tool", robot_name)
    self.active_robots[robot_name].gripper.send_command(0.05, force=40)
    self.active_robots[robot_name].gripper.attach_object("cable_tool", with_collisions=True)
    self.active_robots[robot_name].gripper.forget_attached_item()
    if not self.active_robots[robot_name].move_lin_rel([0,0,0.1], speed=1.0, retime=True):
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0):
      return False
    return True

  def unequip_cable_tool(self, robot_name="a_bot"):
    approach_pose = conversions.to_pose_stamped("cable_holder_pickup_link", [-0.1, 0, 0, 0, 0, 0])
    pickup_pose   = conversions.to_pose_stamped("cable_holder_pickup_link", [0.018, 0, 0, 0, 0, 0])
    if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0):
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(pickup_pose, speed=0.3, move_lin=True, retime=True):
      return False
    self.active_robots[robot_name].gripper.detach_object("cable_tool")
    self.despawn_tool("cable_tool")
    self.allow_collisions_with_robot_hand("cable_tool", robot_name, False)
    self.active_robots[robot_name].gripper.open()
    self.active_robots[robot_name].gripper.forget_attached_item()
    if not self.active_robots[robot_name].move_lin_rel([0,0,0.1], speed=1.0, retime=True):
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0):
      return False
    return True

  def insert_motor_cables_with_tool(self, cable_color="black"):
    """
      insert motor cable in termina
      cable_color: "black" or "red"
    """
    if cable_color not in ["black", "red"]:
      raise ValueError("invalid cable %s" % cable_color)
    
    switch_panels_order = self.assembly_database.assembly_info.get("switched_motor_and_bearing", False)

    if cable_color == "black":
      pin_frame_id = "assembled_part_01_cable_pin3" if not switch_panels_order else "assembled_part_01_cable_pin2"
      frame_id = "assembled_part_04_black_cable_connector"
    else:
      pin_frame_id = "assembled_part_01_cable_pin4" if not switch_panels_order else "assembled_part_01_cable_pin1"
      frame_id = "assembled_part_04_red_cable_connector"

    cable_length = 0.06
    self.ab_bot.go_to_named_pose("home")
    self.b_bot.gripper.send_command(0.02, wait=False)
    self.equip_cable_tool("a_bot")

    a_bot_approach     = conversions.to_pose_stamped(frame_id, [-0.0196, 0.0654,-0.0550, tau/4, -tau/8, -tau/4])
    a_bot_at_cable     = conversions.to_pose_stamped(frame_id, [-0.014, 0.0, 0.0, tau/4, -tau/8, -tau/4])

    b_bot_approach      = conversions.to_pose_stamped(frame_id, [-0.1134,-0.0630,-0.0662]+np.deg2rad([87, -67, 90]).tolist())
    b_bot_at_cable_tip  = conversions.to_pose_stamped(frame_id, [-0.1134, 0.0130, 0.0162]+np.deg2rad([87, -67, 90]).tolist())
    b_bot_hold_cable    = conversions.to_pose_stamped(frame_id, [-0.0924, 0.0101, 0.0029]+np.deg2rad([145, -41,  90]).tolist())
    b_bot_Waypoint_1    = conversions.to_pose_stamped(frame_id, [-0.1287,-0.0918, 0.0158]+np.deg2rad([177, -54,  90]).tolist())
    b_bot_Waypoint_2    = conversions.to_pose_stamped(frame_id, [-0.0879,-0.1159,-0.0572]+np.deg2rad([177, -54,  90]).tolist())
    b_bot_look_at_cable = conversions.to_pose_stamped(frame_id, [-0.0378,-0.1581, 0.0497, 0.609497539310126, 0.6163826114272067, 0.3506917013529725,-0.3544016880705233])

    a_bot_above_cable_end = conversions.to_pose_stamped(frame_id, [-0.0740, 0.0367,-0.0352]+np.deg2rad([123, -38, -136]).tolist())
    a_bot_at_cable_end    = conversions.to_pose_stamped(frame_id, [-0.0870, 0.0113,-0.0142]+np.deg2rad([124, -37, -138]).tolist())
    a_bot_midpoint        = conversions.to_pose_stamped(frame_id, [-0.0260,-0.0045,-0.0028]+np.deg2rad([ 95, -52, -107]).tolist())
    a_bot_above_hole2     = conversions.to_pose_stamped(pin_frame_id, [-0.0200, 0.0002,-0.0031, 0, 0, 0, 0])

    self.planning_scene_interface.allow_collisions("cable_tool", "motor")
    self.planning_scene_interface.allow_collisions("cable_tool", "b_bot_right_outer_knuckle")
    # self.planning_scene_interface.allow_collisions("cable_tool", "b_bot_right_outer_knuckle")
    self.allow_collisions_with_robot_hand("cable_tool", "b_bot")
    self.a_bot.go_to_pose_goal(a_bot_approach, speed=1.0,  end_effector_link="a_bot_cable_tool_tip_link")
    self.a_bot.go_to_pose_goal(a_bot_at_cable, speed=0.3, move_lin=True, retime=True,  end_effector_link="a_bot_cable_tool_tip_link")
    self.a_bot.gripper.close()
    self.a_bot.move_lin_rel(relative_translation=[-cable_length,0,0],  end_effector_link="a_bot_cable_tool_tip_link", speed=0.1, retime=True)

    self.b_bot.go_to_pose_goal(b_bot_approach, speed=1.0)
    self.confirm_to_proceed("0")
    self.b_bot.go_to_pose_goal(b_bot_at_cable_tip, speed=0.3, move_lin=True, retime=True)
    self.b_bot.gripper.close(force=100)

    self.a_bot.gripper.open(opening_width=0.05)
    self.a_bot.move_lin_rel([-0.1,0,0], speed=0.3, relative_to_tcp=True, end_effector_link="a_bot_cable_tool_tip_link", retime=True)

    self.b_bot.go_to_pose_goal(b_bot_hold_cable, speed=0.3, move_lin=True, retime=True)

    self.unequip_cable_tool("a_bot")
    self.a_bot.gripper.send_command(0.02, wait=False)

    self.a_bot.go_to_pose_goal(a_bot_above_cable_end, speed=1.0)
    self.a_bot.go_to_pose_goal(a_bot_at_cable_end, speed=0.1, move_lin=True, retime=True)

    self.a_bot.gripper.close(force=150)
    self.b_bot.gripper.open(opening_width=0.05)
    self.b_bot.go_to_pose_goal(b_bot_Waypoint_1, move_lin=True, speed=0.3, retime=True)

    def b_bot_task():
      self.b_bot.go_to_pose_goal(b_bot_Waypoint_2, move_lin=True, speed=1.0, retime=True)
      self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot")
      self.b_bot.go_to_pose_goal(b_bot_look_at_cable, move_lin=True, speed=1.0, retime=True)
    def a_bot_task():
      self.a_bot.go_to_pose_goal(a_bot_midpoint, move_lin=True, speed=0.5)
      self.a_bot.go_to_pose_goal(a_bot_above_hole2, move_lin=True, speed=0.5)
      self.a_bot.move_lin_rel([0.021,0,0], relative_to_tcp=True, speed=0.01)
      self.a_bot.gripper.open(opening_width=0.02)
      self.a_bot.move_lin_rel([-0.021,0,0], relative_to_tcp=True, speed=0.1)
      
    self.do_tasks_simultaneous(a_bot_task, b_bot_task)
    self.b_bot.move_lin_rel([0.021,0,0], relative_to_tcp=True, speed=0.01)
    self.allow_collisions_with_robot_hand("base_fixture_top", "b_bot")
    # TODO confirm that the insertion was successful


    # waypoints = []
    # waypoints.append((self.a_bot.compute_ik(a_bot_approach, timeout=0.02, end_effector_link="a_bot_cable_tool_tip_link", retry=True), 0, 1.0))
    # waypoints.append((self.a_bot.compute_ik(a_bot_approach, timeout=0.02, end_effector_link="a_bot_cable_tool_tip_link", retry=True), 0, 1.0))

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

  def fasten_screw_vertical(self, robot_name, screw_hole_pose, allow_collision_with_object="", 
                                  screw_height = .02, screw_size = 4, spiral_radius=0.0015,
                                  approach_from_front=False):
    """
    Fasten a screw on one of the L-plates.

    Starts and ends at feeder_pick_ready.
    allow_collision_with_object is the name of the object we are fastening (the screw needs to be able to move into it, because of the tool's compliance)
    """
    if allow_collision_with_object:
      self.planning_scene_interface.allow_collisions("screw_tool_m%s" % screw_size, allow_collision_with_object)
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4 but is: " + str(screw_size))
      return False

    def approach():
      waypoints = []
      waypoints.append(("feeder_pick_ready", 0, 1.0))
      if approach_from_front:
        waypoints.append(("screw_ready_passthrough", 0, 1.0))
        waypoints.append(("screw_ready_front", 0, 1.0))
      self.active_robots[robot_name].move_joints_trajectory(waypoints)
    def retreat():
      self.b_bot.move_lin_rel([-0.02,0,0], relative_to_tcp=True, speed=0.015, end_effector_link="%s_screw_tool_m%s_tip_link" % (robot_name, screw_size))
      waypoints = []
      if approach_from_front:
        waypoints.append(("screw_ready_front", 0, 1.0))
        waypoints.append(("screw_ready_passthrough", 0, 1.0))
      waypoints.append(("feeder_pick_ready", 0, 1.0))
      self.active_robots[robot_name].move_joints_trajectory(waypoints)

    approach()
    res = self.screw(robot_name, screw_hole_pose, screw_size, screw_height, 
                     stay_put_after_screwing=True, skip_final_loosen_and_retighten=False, 
                     spiral_radius=spiral_radius, attempts=0)
    # second extra tighten, may get stuck, we will go up slowly
    self.tools.set_motor("screw_tool_m%s" % screw_size, "tighten", duration = 10.0, skip_final_loosen_and_retighten=True, wait=True)
    self.confirm_to_proceed("finetune")
    retreat()
    if allow_collision_with_object:
      self.planning_scene_interface.disallow_collisions("screw_tool_m%s" % screw_size, allow_collision_with_object)
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

#### subtask e - idler pulley

  def pick_idler_spacer(self, robot_name="b_bot", attempts=3):
    options = {'check_for_close_items': True, 'declutter_with_tool': True, 'object_width': 0.005, 'grasp_width': 0.03}
    idler_spacer_pose = self.look_and_get_grasp_point("idler_spacer", robot_name=robot_name, options=options)
    
    marker_pose = copy.deepcopy(idler_spacer_pose)
    marker_pose.pose.position.z = 0
    marker_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, -tau/4, 0))
    self.markers_scene.spawn_item("idler_spacer", marker_pose)
    idler_spacer_pose.pose.position.z = 0.0 # Magic numbers

    self.vision.activate_camera(robot_name + "_inside_camera")
    if not self.simple_pick(robot_name, idler_spacer_pose, grasp_height=0.001, 
                            gripper_force=50.0, grasp_width=.04, axis="z", approach_height=0.07, gripper_command=0.03,
                            item_id_to_attach="idler_spacer", allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick --> try again")
      self.active_robots[robot_name].gripper.open(opening_width=0.05)
      if attempts > 0:
        self.pick_idler_spacer(robot_name=robot_name, attempts=attempts-1)
      return False

    if not self.simple_gripper_check(robot_name, min_opening_width=0.015):
      rospy.logerr("Gripper did not grasp the idler_pulley --> try again")
      self.active_robots[robot_name].gripper.open(opening_width=0.05)
      if attempts > 0:
        self.pick_idler_spacer(robot_name=robot_name, attempts=attempts-1)
      return False
    return True

  def orient_idler_pulley_assembly(self, robot_name, target_pose, store=False):
    """
      Orient Idler - spacer or pulley in the centering area.
      Optionally define the pose where the centering will be done.
      Optionally leave the object in the centered pose.
    """
    centering_area = "right_centering_link" if robot_name == "b_bot" else "left_centering_link"
    centering_pose = target_pose if target_pose else conversions.to_pose_stamped(centering_area, [-0.005, 0, 0, -tau/4, 0, 0])
    self.active_robots[robot_name].go_to_named_pose("centering_area")
    self.active_robots[robot_name].go_to_pose_goal(centering_pose)
    self.center_with_gripper(robot_name, opening_width=0.03, gripper_force=0, gripper_velocity=0.01, move_back_to_initial_position=True)
    if not store:
      self.active_robots[robot_name].gripper.close()
    self.active_robots[robot_name].go_to_named_pose("centering_area")
    return True

  def pick_idler_pulley_assembly(self, robot_name="b_bot", attempts=3):
    options = {'grasp_width': 0.05, 'center_on_corner': True, 'approach_height': 0.02, 'grab_and_drop': True, 'object_width': 0.03}
    idler_pulley_pose = self.look_and_get_grasp_point("idler_pulley", options=options)

    idler_pulley_pose.pose.position.z = 0.0 # Magic numbers
    
    marker_pose = copy.deepcopy(idler_pulley_pose)
    marker_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, -tau/4, 0))
    self.markers_scene.spawn_item("idler_pulley", marker_pose)

    self.vision.activate_camera(robot_name + "_inside_camera")
    if not self.simple_pick(robot_name, idler_pulley_pose, grasp_height=0.001, 
                            gripper_force=50.0, grasp_width=.05, axis="z", approach_height=0.07, gripper_command=0.03,
                            item_id_to_attach="idler_pulley", allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick --> try again")
      self.active_robots[robot_name].gripper.open(opening_width=0.05)
      if attempts > 0:
        self.pick_idler_pulley_assembly(robot_name=robot_name, attempts=attempts-1)
      return False

    if not self.simple_gripper_check(robot_name, min_opening_width=0.015):
      rospy.logerr("Gripper did not grasp the idler_pulley --> try again")
      self.active_robots[robot_name].gripper.open(opening_width=0.05)
      if attempts > 0:
        self.pick_idler_pulley_assembly(robot_name=robot_name, attempts=attempts-1)
      return False
    return True

  def pick_idler_pin(self, robot_name="b_bot"):
    options = {'check_for_close_items': True, 'declutter_with_tool': True, 'object_width': 0.005, 'grasp_width': 0.04}
    idler_pin_pose = self.look_and_get_grasp_point("idler_pin", options=options)
    
    idler_pin_pose.pose.position.z = 0.0 # Magic numbers
    
    marker_pose = copy.deepcopy(idler_pin_pose)
    marker_pose.pose.position.z = 0.005
    marker_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, 0, 0))
    self.markers_scene.spawn_item("idler_pin", marker_pose)
    

    self.vision.activate_camera(robot_name + "_inside_camera")
    if not self.simple_pick(robot_name, idler_pin_pose, grasp_height=0.001, 
                            gripper_force=50.0, grasp_width=.04, axis="z", approach_height=0.07, gripper_command=0.03,
                            item_id_to_attach="idler_pin", allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check(robot_name, min_opening_width=0.002):
      rospy.logerr("Gripper did not grasp the idler_pin --> Stop")
      return False
    return True

  def pick_washer(self, washer_number, robot_name):
    washer_name = "washer_holder%s" % washer_number
    self.allow_collisions_with_robot_hand(washer_name, robot_name, True)
    orientation = [0, 0, 0] if washer_number < 3 else [0, -tau/4, 0]
    approach_pose = conversions.to_pose_stamped(washer_name + "_collar_link", [-0.1, 0, 0] + orientation)
    at_pose = conversions.to_pose_stamped(washer_name + "_collar_link", [0, 0, 0] + orientation)
    self.active_robots[robot_name].gripper.open(opening_width=0.03, wait=False)
    self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0)
    self.active_robots[robot_name].go_to_pose_goal(at_pose, speed=1.0, move_lin=True)
    self.active_robots[robot_name].gripper.close()
    self.active_robots[robot_name].go_to_pose_goal(approach_pose, speed=1.0, move_lin=True)
    self.allow_collisions_with_robot_hand(washer_name, robot_name, False)
    return True

#### subtask d - bearing spacer / output pulley

  def pick_bearing_spacer(self, robot_name="b_bot"):
    options = {'check_for_close_items': True, 'declutter_with_tool': True, 'object_width': 0.005, 'grasp_width': 0.04}
    bearing_spacer_pose = self.look_and_get_grasp_point("bearing_spacer", robot_name=robot_name, options=options)
    
    marker_pose = copy.deepcopy(bearing_spacer_pose)
    marker_pose.pose.position.z = 0.004
    marker_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, -tau/4, 0))
    self.markers_scene.spawn_item("bearing_spacer", marker_pose)

    bearing_spacer_pose.pose.position.z = 0

    self.vision.activate_camera(robot_name + "_inside_camera")
    if not self.simple_pick(robot_name, bearing_spacer_pose, grasp_height=0.0, gripper_force=50.0,
                            grasp_width=.04, axis="z", approach_height=0.07,
                            item_id_to_attach="bearing_spacer", allow_collision_with_tray=True):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check(robot_name, min_opening_width=0.002):
      rospy.logerr("Gripper did not grasp the bearing_spacer --> Stop")
      return False
    return True

  def orient_bearing_spacer(self, robot_name="b_bot"):
    centering_frame = "left_centering_link" if robot_name == "a_bot" else "right_centering_link"
    place_pose = conversions.to_pose_stamped(centering_frame, [0, 0, 0, radians(-30), 0, 0])
    
    if not self.simple_place(robot_name, place_pose, place_height=0.0045, gripper_opening_width=0.03, lift_up_after_place=False, approach_height=0.15,
                      axis="x", sign=-1):
      return False
    
    if not self.center_with_gripper(robot_name, opening_width=0.03, gripper_force=0, gripper_velocity=0.01, move_back_to_initial_position=False):
      return False

    seq = []
    pre_grasp_pose = conversions.to_pose_stamped(centering_frame, [-0.008, 0, 0, radians(-30), 0, 0])
    seq.append(helpers.to_sequence_item(pre_grasp_pose, speed=1.0, linear=True))
    seq.append(helpers.to_sequence_item_relative(pose=[0, 0, -0.0075, 0, 35, 0], relative_to_tcp=True, speed=1.0))
    seq.append(helpers.to_sequence_gripper('close', gripper_force=60, gripper_velocity=0.05))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0.05, 0, 0, 0], speed=0.4))
    if not self.execute_sequence(robot_name, seq, "orient bearing spacer"):
      return False
    
    return True
  
  def align_bearing_spacer_pre_insertion(self, robot_name="b_bot"):
    seq = []

    rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/2, radians(-35.0), 0]

    midway        = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.15, 0, 0]      + rotation)
    pre_insertion = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.058, 0, 0.007] + rotation)

    seq.append(helpers.to_sequence_item("centering_area", speed=0.4, linear=False))
    seq.append(helpers.to_sequence_item(midway, speed=1.0, linear=False))
    seq.append(helpers.to_sequence_item(pre_insertion, speed=0.2, linear=True))

    return self.execute_sequence(robot_name, seq, "align bearing spacer pre insertion")

  def insert_bearing_spacer(self, target_link, robot_name="b_bot", attempts=1):
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.045, 0.0, 0.0, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame

    selection_matrix = [0.0, 0.3, 0.3, 0.95, 1.0, 1.0]
    result = self.active_robots[robot_name].do_insertion(target_pose_target_frame, radius=0.0005, 
                                                      insertion_direction="-X", force=5.0, timeout=15.0, 
                                                      relaxed_target_by=0.005, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA

    if not success:
      grasp_check = self.simple_insertion_check(robot_name, 0.06, min_opening_width=0.02)
      if grasp_check and attempts > 0: # try again the spacer is still there   
        return self.insert_bearing_spacer(target_link, attempts=attempts-1)
      elif not grasp_check or not attempts > 0:
        self.active_robots[robot_name].gripper.open(wait=True, opening_width=0.08)
        self.active_robots[robot_name].move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
        rospy.logerr("** Insertion Failed!! **")
        return False

    self.active_robots[robot_name].gripper.open(wait=True, opening_width=0.08)
    self.active_robots[robot_name].move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
    return success

  def pick_output_pulley(self, robot_name="b_bot"):
    options = {'grasp_width': 0.05, 'check_for_close_items': False, 'object_width': 0.0}
    output_pulley_pose = self.look_and_get_grasp_point("output_pulley", robot_name=robot_name, options=options)

    marker_pose = copy.deepcopy(output_pulley_pose)
    marker_pose.pose.position.z = 0.008
    marker_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, -tau/4, 0))
    self.markers_scene.spawn_item("output_pulley", marker_pose)

    output_pulley_pose.pose.position.z = 0.0

    self.vision.activate_camera(robot_name + "_inside_camera")
    if not self.simple_pick(robot_name, output_pulley_pose, grasp_height=0.011, gripper_force=50.0,
                            grasp_width=.05, axis="z", approach_height=0.06,
                            item_id_to_attach="output_pulley"):
      rospy.logerr("Fail to simple_pick")
      return False

    if not self.simple_gripper_check(robot_name):
      rospy.logerr("Gripper did not grasp the output_pulley --> Stop")
      return False
    return True

  def orient_output_pulley(self, robot_name="b_bot"):
    centering_frame = "left_centering_link" if robot_name == "a_bot" else "right_centering_link"
    place_pose = conversions.to_pose_stamped(centering_frame, [-0.011, 0, 0, radians(-30), 0, 0])
    
    if not self.simple_place(robot_name, place_pose, place_height=0.005, gripper_opening_width=0.03, lift_up_after_place=False, approach_height=0.15,
                      axis="x", sign=-1):
      return False
    
    if not self.center_with_gripper(robot_name, opening_width=0.05, gripper_force=40, gripper_velocity=0.1, move_back_to_initial_position=False):
      return False

    seq = []
    pre_grasp_pose = conversions.to_pose_stamped(centering_frame, [-0.005, 0, 0, radians(-30), 0, 0])
    seq.append(helpers.to_sequence_gripper('open', gripper_opening_width=0.1, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_item(pre_grasp_pose, speed=1.0, linear=True))
    seq.append(helpers.to_sequence_item_relative(pose=[0, 0, -0.01, 0, 35, 0], relative_to_tcp=True, speed=1.0))
    seq.append(helpers.to_sequence_gripper('close', gripper_force=60, gripper_velocity=0.05))
    seq.append(helpers.to_sequence_item_relative([0, 0, 0.05, 0, 0, 0], speed=0.4))
    if not self.execute_sequence(robot_name, seq, "orient output pulley"):
      return False
    return True
  
  def align_output_pulley_pre_insertion(self, robot_name="b_bot"):
    seq = []

    rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/2, radians(-35.0), 0]

    midway        = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.15, 0, 0]     + rotation)
    pre_insertion = conversions.to_pose_stamped("assembled_part_07_inserted", [-0.053, 0.001, 0.009] + rotation)

    seq.append(helpers.to_sequence_item("centering_area", speed=0.4, linear=False))
    seq.append(helpers.to_sequence_item(midway, speed=1.0, linear=False))
    seq.append(helpers.to_sequence_item(pre_insertion, speed=0.2, linear=True))

    return self.execute_sequence(robot_name, seq, "align output pulley pre insertion")

  def insert_output_pulley(self, target_link, robot_name="b_bot", attempts=1):
    rospy.loginfo("Starting insertion of output pulley")
    target_pose_target_frame = conversions.to_pose_stamped(target_link, [-0.038, 0.0, 0.0, 0.0, 0.0, 0.0]) # Manually defined target pose in object frame

    selection_matrix = [0.0, 0.3, 0.3, 0.95, 1.0, 1.0]
    result = self.active_robots[robot_name].do_insertion(target_pose_target_frame, radius=0.003, 
                                                      insertion_direction="-X", force=8.0, timeout=15.0, 
                                                      relaxed_target_by=0.005, selection_matrix=selection_matrix)
    success = result == TERMINATION_CRITERIA
    rospy.loginfo("insertion finished with status: %s" % result)

    grasp_check = self.simple_insertion_check(robot_name, 0.1, min_opening_width=0.02, velocity=1.0)
    if not success and not grasp_check:
      if grasp_check and attempts > 0: # try again the pulley is still there   
        return self.insert_output_pulley(target_link, attempts=attempts-1)
      elif not grasp_check or not attempts > 0:
        self.active_robots[robot_name].gripper.open(wait=True, opening_width=0.08)
        self.active_robots[robot_name].move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.015, speed=.03)
        rospy.logerr("** Insertion Failed!! **")
        return False

    rospy.loginfo("Preparing push")
    rotation = [0, radians(-35.0), 0] if robot_name == "b_bot" else [tau/2, radians(-35), 0]
    push_pose = conversions.to_pose_stamped(target_link, [-0.008, 0, 0.006] + rotation)

    rospy.loginfo("Starting push")
    # self.a_bot.go_to_pose_goal(target_pose, speed=0.02, move_lin=True)
    current_pose = self.listener.transformPose(target_link, self.a_bot.get_current_pose_stamped())
    tries = 0
    while current_pose.pose.position.x < push_pose.pose.position.x and tries < 5:
      self.active_robots[robot_name].gripper.close()
      success = self.active_robots[robot_name].force_controller.linear_push(force=15, direction="-X", max_translation=0.05, timeout=5.)
      self.active_robots[robot_name].gripper.open(wait=True, opening_width=0.1)
      current_pose = self.listener.transformPose(target_link, self.a_bot.get_current_pose_stamped())

    self.active_robots[robot_name].move_lin_rel(relative_translation = [0.10,0,0], acceleration = 0.05, speed=.1)
    return success

  def check_output_pulley_angle(self):
    # Look at pulley with b_bot
    approach_centering = conversions.to_pose_stamped("assembled_part_08_inserted", [-0.1, 0, -0.15,  np.deg2rad(180), np.deg2rad(-60), 0])
    self.b_bot.go_to_pose_goal(approach_centering, speed=0.1, end_effector_link="b_bot_outside_camera_link", move_lin=False)
    self.vision.activate_camera("b_bot_outside_camera")

    self.vision.activate_pulley_screw_detection()

    global pulley_screws_visible
    pulley_screws_visible = False
    def f(msg):
      global pulley_screws_visible
      if not pulley_screws_visible == msg.data:
        rospy.loginfo("=== pulley_screws_visible changed to: " + str(msg.data))
      pulley_screws_visible = msg.data
    rospy.Subscriber("/o2ac_vision_server/pulley_screws_in_view", Bool, f)
    rospy.sleep(0.5)

    # pulley_grasp_pose = conversions.to_pose_stamped("assembled_part_11_front_hole", [0.006, 0, 0.006,  0, np.deg2rad(60), 0])
    pulley_grasp_pose = conversions.to_pose_stamped("assembled_part_07_front_hole", [-0.012, -0.000, -0.006, tau/2, 0, 0])
    approach_pulley = conversions.to_pose_stamped("assembled_part_07_front_hole", [-0.05, 0, -0.006, tau/2, 0, 0])

    self.a_bot.go_to_pose_goal(approach_pulley)

    screws_upright = False
    tries = 0
    while not screws_upright and not rospy.is_shutdown() and tries < 10:
      # screws_seen_at_angle = find_pulley_screws(approach_centering)
      
      # Rotate the pulley
      self.rotate_motor_pulley(self, "assembled_part_07_front_hole", rotations=1, 
                                      offset_from_center=0.04, x_offset=0.024, skip_approach=True)
      if pulley_screws_visible:
        # Grasp with a_bot
        self.a_bot.go_to_pose_goal(approach_pulley, speed=0.3)
        self.a_bot.go_to_pose_goal(pulley_grasp_pose, speed=0.3)
        self.a_bot.gripper.close(force=100)

        # Push shaft into contact
        approach_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, -0.15] + np.deg2rad([-90,-90,-90]).tolist())
        pre_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.15, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
        at_hold_pose = conversions.to_pose_stamped("assembled_part_07_inserted", [0.043, 0.000, 0.02] + np.deg2rad([-90,-90,-90]).tolist())
        self.b_bot.gripper.close(wait=False)
        self.b_bot.go_to_pose_goal(approach_hold_pose)
        self.b_bot.go_to_pose_goal(pre_hold_pose)
        self.b_bot.go_to_pose_goal(at_hold_pose)
        self.b_bot.go_to_pose_goal(pre_hold_pose)
        self.b_bot.go_to_named_pose("screw_ready")
        
        # Fasten two screws
        self.fasten_output_pulley()
        break
    self.vision.activate_pulley_screw_detection(False)
    return True
    # TODO: Turn pulley with a_bot, stop when screws visible

  def fasten_output_pulley(self):
    self.equip_tool("b_bot", "padless_tool_m4")

    screw_poses = ["assembled_part_11_screw_head_1", "assembled_part_11_screw_head_2"]

    for screw_frame in screw_poses:
      approach_screw = conversions.to_pose_stamped(screw_frame, [-0.1, 0, 0, 0, 0, 0])
      at_screw = conversions.to_pose_stamped(screw_frame, [ 0.005, 0, 0, 0, 0, 0])
      self.b_bot.go_to_pose_goal(approach_screw, speed=0.5, move_lin=True, end_effector_link="b_bot_screw_tool_m4_tip_link")
      self.b_bot.go_to_pose_goal(at_screw, speed=0.2, move_lin=True, end_effector_link="b_bot_screw_tool_m4_tip_link")
      self.tools.set_motor("padless_tool_m4", "tighten", duration=10)
      self.b_bot.go_to_pose_goal(approach_screw, speed=0.4, move_lin=True, end_effector_link="b_bot_screw_tool_m4_tip_link")

    self.unequip_tool("b_bot", "padless_tool_m4")
    return True

#### L-plates

  def grasp_test(self, object_name="panel_bearing"):
    
    self.gazebo_scene = GazeboModels('o2ac_gazebo')

    grasp_width = 0.05
    self.activate_led("b_bot")

    print("input file name?")
    while True:
      try:
        #input_file_name=input()
        input_file_name = 'grasp_test_input.txt'
        with open(input_file_name) as input_file:
          grasp_pose_data=map(float, input_file.readline().split(' '))
          pose_data_set=[]
          for line in input_file.readlines():
            pose_data_set.append(map(float, line.split(' ')))
        break
      except Exception as e:
        print(e)
        pass
    
    print("move gripper")
    grasp_pose=conversions.to_pose_stamped("tray_center", grasp_pose_data)
    self.constrain_grasp_into_tray("b_bot", grasp_pose, grasp_width=grasp_width)
    
    model_state_getter=rospy.ServiceProxy("/gazebo/get_model_state", gazebo_msgs.srv.GetModelState)
    model_deleter=rospy.ServiceProxy("/gazebo/delete_model", gazebo_msgs.srv.DeleteModel)
    
    print("output file name?")
    #output_file_name=input()
    output_file_name = 'grasp_test_output.txt'
    with open(output_file_name ,'w') as output_file:
      for pose_data in pose_data_set:
        print("test: ", pose_data)
        object_pose = conversions.to_pose_stamped("tray_center", pose_data)
        gazebo_pose = self.listener.transformPose("world", object_pose)
        models = [Model(object_name, pose=gazebo_pose.pose, reference_frame=gazebo_pose.header.frame_id, file_type="sdf")]
        model_name = object_name+'_tmp'
        # Spawn the part in gazebo
        print("spawn to gazebo")
        name = "panel_bearing"
        print(models[0].pose)
        self.gazebo_scene.load_models(models)
        rospy.sleep(0.5)
        print("grasp")
        self.simple_pick("b_bot", object_pose=grasp_pose, grasp_width=grasp_width, approach_height=0.05, grasp_height=0.005, 
                       axis="z", item_id_to_attach=object_name, lift_up_after_pick=False, approach_with_move_lin=False,
                       speed_fast=1.0, minimum_grasp_width=0.001, attach_with_collisions=True)
        self.active_robots["b_bot"].gripper.open(opening_width=0.03)
        print("output result")
        model_state_request = gazebo_msgs.srv.GetModelStateRequest()
        model_state_request.model_name = model_name
        model_state_request.relative_entity_name = gazebo_pose.header.frame_id
        model_state_response = model_state_getter(model_state_request)
        print(model_state_response)
        if not model_state_response.success:
          output_file.write(model_state_response.status_message + '\n')
          rospy.logerr(model_state_response.status_message)
        else:
          grasped_pose = self.listener.transformPose("tray_center", model_state_response)
          print>> output_file, grasped_pose.pose
          print(grasped_pose.pose)

        delete_model_request = gazebo_msgs.srv.DeleteModelRequest()
        delete_model_request.model_name = model_name
        delete_model_response = model_deleter(delete_model_request)
        if not delete_model_response.success:
          rospy.logerr(delete_model_response.status_message)
      
        rospy.sleep(0.5)
    
  
  def pick_panel_with_handover(self, panel_name="panel_bearing", simultaneous=True, rotate_on_failure=True, rotation_retry_counter=0, pose_with_uncertainty=None):
    grasp_width = 0.05
    self.activate_led("b_bot")
    goal = self.get_large_item_position_from_top(panel_name, "b_bot")
    rospy.sleep(0.2)
    
    # Initialize the pose 
    if pose_with_uncertainty !=None:
      pose_with_uncertainty.header=goal.header
      pose_with_uncertainty.pose.pose = goal.pose
      pose_with_uncertainty.pose.covariance=[0.0001, 0.00, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.0001, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.00, 0.0000, 0.00, 0.00, 0.00,
                                              0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                              0.00, 0.00, 0.00, 0.00, 0.00, 0.01]
      collision_object = self.assembly_database.get_collision_object(panel_name)
      self.visualize_object_with_distribution(collision_object, pose_with_uncertainty, frame_locked=True)
    # self.spawn_object(panel_name, goal, goal.header.frame_id, pose_with_uncertainty = pose_with_uncertainty)
    rospy.sleep(0.5)

    if not goal:
      rospy.logerr("Abort pick of %s, plate not detected" % panel_name)
      return False
    
    grasp_pose = self.get_transformed_grasp_pose(panel_name, "default_grasp")
    grasp_pose, success = self.constrain_grasp_into_tray("b_bot", grasp_pose, grasp_width=grasp_width)
    
    if not success and self.too_close_to_border(grasp_pose, border_dist=grasp_width, verbose=True):
      self.pick_panel_fallback("b_bot", panel_name, grasp_pose, too_close_too_border=True)
      return self.pick_panel_with_handover(panel_name, simultaneous, rotate_on_failure=True)
      
    self.allow_collisions_with_robot_hand(panel_name, "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    success = self.simple_pick("b_bot", object_pose=grasp_pose, grasp_width=grasp_width, approach_height=0.05, grasp_height=0.005, 
                               axis="z", item_id_to_attach=panel_name, lift_up_after_pick=True, approach_with_move_lin=False,
                               speed_fast=1.0, minimum_grasp_width=0.001, attach_with_collisions=True, 
                               pose_with_uncertainty=pose_with_uncertainty, object_name=panel_name,
                               allow_collision_with_tray=True)
      
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)

    if not success:
      if rotate_on_failure:
        # Fallback: Try to pick all 4 possible orientations
        while rotation_retry_counter < 3:
          rospy.logwarn("Fallback: Rotating plate (" + str(rotation_retry_counter) + " out of 3 times)")
          self.rotate_plate_collision_object_in_tray(panel_name)
          rospy.sleep(.2)
          grasp_pose = self.get_transformed_grasp_pose(panel_name, "default_grasp")
          grasp_pose, success = self.constrain_grasp_into_tray("b_bot", grasp_pose, grasp_width=grasp_width)
          if not success and self.too_close_to_border(grasp_pose, border_dist=grasp_width, verbose=True):
            self.pick_panel_fallback("b_bot", panel_name, grasp_pose, too_close_too_border=True)
            return self.pick_panel_with_handover(panel_name, simultaneous, rotate_on_failure=True, rotation_retry_counter=0) # potential infinite loop...
          if self.simple_pick("b_bot", object_pose=grasp_pose, grasp_width=grasp_width, approach_height=0.05, grasp_height=0.005, 
                              axis="z", item_id_to_attach=panel_name, lift_up_after_pick=True, approach_with_move_lin=False,
                              speed_fast=1.0, minimum_grasp_width=0.001, attach_with_collisions=True, allow_collision_with_tray=True):
            break
          rotation_retry_counter += 1
      else:
        rospy.logerr("Fail to grasp with b_bot")
        return False

    handover_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.1, 0.2, -tau/4, tau/8, -tau/4])
    handover_grasp = "grasp_7" if panel_name == "panel_bearing" else "grasp_9"
    if not self.simple_handover("b_bot", "a_bot", handover_pose, panel_name, handover_grasp, simultaneous=False, pose_with_uncertainty=pose_with_uncertainty):
      rospy.logerr("Fail to handover")
      return False
    return True

  def pick_panel_fallback(self, robot_name, panel, grasp_pose, too_close_too_border=False, too_close_to_other_object=False, center_with_tool=False):
    if panel == "panel_motor":
      tool_pull_pose = conversions.to_pose_stamped("move_group/panel_motor/center", [0., 0, 0, 0, 0, 0])
    elif panel == "panel_bearing":
      tool_pull_pose = conversions.to_pose_stamped("move_group/panel_bearing/center", [0., 0, 0, 0, 0, 0])
    
    # print("tool_pull_pose", tool_pull_pose.pose.position)
    tool_pull_pose = self.listener.transformPose("tray_center", tool_pull_pose)
    tool_pull_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0.0, tau/4, 0))
    # print("tool_pull_pose tfed", tool_pull_pose.pose.position)

    # If close to border, pull towards the middle
    if too_close_too_border:
      # print("tool_pull_pose after", tool_pull_pose.pose.position)
      dx, dy = self.distances_from_tray_border(grasp_pose)
      if dx < 0.03 and dy < 0.03:
        direction = None # move towards center diagonally
      elif dx < dy: # Use the close border
        direction = 'x'
      else:
        direction = 'y'
      self.move_towards_tray_center_with_push(robot_name, tool_pull_pose, approach_height=0.01, direction=direction, go_back_halfway=False, distance=0.06)
      self.planning_scene_interface.allow_collisions(panel, "")  # Collisions are reactivated in move_towards_center_with_tool
      self.planning_scene_interface.allow_collisions(panel, "tray")
      self.planning_scene_interface.allow_collisions(panel, "tray_center")
      self.allow_collisions_with_robot_hand(panel, robot_name)
    elif too_close_to_other_object:  # If not close to border, try to hit a hole and make space around the plate
      self.declutter_with_tool(robot_name, tool_pull_pose) 
    elif center_with_tool:
      self.move_towards_center_with_tool(robot_name, target_pose=tool_pull_pose, distance=0.06, start_with_spiral=True)

  def define_panel_place_pose(self, panel_name):
    """ Define the Magic numbers for placing the L-plates on the base plate"""
    if panel_name == "panel_bearing":
      object_frame = "assembled_part_01_screw_hole_panel1_1"
      l_plate = 0.116
    else: # panel_motor
      object_frame = "assembled_part_01_screw_hole_panel2_1"
      if self.assembly_database.db_name in ["wrs_assembly_2021", "wrs_assembly_2021_flipped"]:
        l_plate = 0.06
      elif self.assembly_database.db_name in ["wrs_assembly_2020", "wrs_assembly_2019_surprise"]:
        l_plate = 0.07

    if self.assembly_database.db_name == "wrs_assembly_2021":
      if panel_name == "panel_bearing":
        offset_y = 0.015             # MAGIC NUMBER (points to the global forward (x-axis))
        offset_z = -0.011           # MAGIC NUMBER (points to the global left (negative y-axis))
      else: # panel_motor
        offset_y = 0.014            # MAGIC NUMBER
        offset_z = -0.0085           # MAGIC NUMBER (+ l_plate/2)
    elif self.assembly_database.db_name == "wrs_assembly_2020":
      if panel_name == "panel_bearing":
        offset_y = 0.01             # MAGIC NUMBER
        offset_z = -0.0065          # MAGIC NUMBER
      else: # panel_motor
        offset_y = 0.011            # MAGIC NUMBER
        offset_z = -0.006           # MAGIC NUMBER
    elif self.assembly_database.db_name == "wrs_assembly_2021_flipped":
      if panel_name == "panel_bearing":
        if self.assembly_database.assembly_info.get("panel_bearing_facing_backward", False):
          offset_y = -0.01          # MAGIC NUMBER (TODO)
          offset_z = -0.007         # MAGIC NUMBER (TODO)
        else: 
          offset_y = 0.01           # MAGIC NUMBER
          offset_z = -0.007         # MAGIC NUMBER
      else: # panel_motor
        offset_y = 0.011            # MAGIC NUMBER
        offset_z = -0.008           # MAGIC NUMBER     
    elif self.assembly_database.db_name == "wrs_assembly_2019_surprise":
      if panel_name == "panel_bearing":
        if self.assembly_database.assembly_info.get("panel_bearing_facing_backward", False):
          offset_y = -0.014          # MAGIC NUMBER (TODO)
          offset_z = -0.0065         # MAGIC NUMBER (TODO)
        else: 
          offset_y = 0.01           # MAGIC NUMBER
          offset_z = -0.007         # MAGIC NUMBER
      else: # panel_motor
        offset_y = 0.011            # MAGIC NUMBER
        offset_z = -0.008           # MAGIC NUMBER     
    else:
      raise ValueError("Unknown data %s" % self.assembly_database.db_name)
    rotation_offset = np.deg2rad([0,0,0]).tolist() # Robots are not perfectly flat in the table 
    place_pose       = conversions.to_pose_stamped(object_frame, [-0.0145, offset_y, l_plate/2 + offset_z] + rotation_offset)
    return place_pose

  def place_panel(self, robot_name, panel_name, pick_again=True, grasp_pose=None, 
                        invert_gripper=True, fake_position=False, pose_with_uncertainty=None,
                        pick_only=False):
    """ Place panel in assembly pose """
    # Set place positions.
    # The plate dimensions should be in the assembly_database, but currently we only store the subframes there.
    if fake_position:  # For debugging
      if panel_name == "panel_bearing":
        l_plate = 0.116
      else:
        if self.assembly_database.db_name in ["wrs_assembly_2021", "wrs_assembly_2021_flipped"]:
          l_plate = 0.06
        elif self.assembly_database.db_name in ["wrs_assembly_2020", "wrs_assembly_2019_surprise"]:
          l_plate = 0.07
      fingertip_width = .03
      distance_to_stopper = -0.064 # w.r.t left centering link
      distance_to_touched_geometry = distance_to_stopper + fingertip_width/2
      visual_offset_y = 0.067 if panel_name == "panel_bearing" else -0.063
      visual_offset_x = -distance_to_touched_geometry+l_plate if self.assembly_database.assembly_info.get(panel_name + "_facing_backward", False) else -distance_to_touched_geometry
      aligned_pose = [0.0, visual_offset_y, visual_offset_x]
      orientation = [-0.5, 0.5, -0.5, -0.5] if not self.assembly_database.assembly_info.get(panel_name + "_facing_backward", False) else [0, tau/4, tau/4]
      goal = conversions.to_pose_stamped("left_centering_link", aligned_pose + orientation)
      self.spawn_object(panel_name, goal, goal.header.frame_id)
      self.planning_scene_interface.allow_collisions(panel_name)
      # object dimensions
      obj_dims = self.dimensions_dataset[panel_name]
      # x,y,z pose w.r.t centering link
      y_pos = 0.065 if panel_name == "panel_bearing" else -0.065
      grasp_pose = conversions.to_pose_stamped("left_centering_link", [-0.02, y_pos, -distance_to_touched_geometry+obj_dims[1]/2, tau/2, 0, 0])
    
    print("Plate grasp pose:", panel_name, grasp_pose.pose.position)

    if pick_again:
      assert grasp_pose, "No grasp pose provided"
      if invert_gripper:
        grasp_pose.pose.orientation = conversions.to_quaternion(transformations.quaternion_from_euler(0, 0, 0))
      grasp_pose = self.listener.transformPose("world", grasp_pose)
      success = self.simple_pick(robot_name, object_pose=grasp_pose, grasp_width=0.04, approach_height=0.1, grasp_height=0.0, 
                      axis="z", item_id_to_attach=panel_name, lift_up_after_pick=False, approach_with_move_lin=False,
                                 speed_fast=1.0, attach_with_collisions=True, pose_with_uncertainty=pose_with_uncertainty, object_name=panel_name)
      self.active_robots[robot_name].move_lin_rel(relative_translation=[0,-0.01,0.15])
      if not success:
        rospy.logerr("Fail to pick from stored pose")
        return False
      if pick_only:
        return True

    place_pose = self.define_panel_place_pose(panel_name)
    above_plate_pose = copy.deepcopy(place_pose)
    above_plate_pose.pose.position.x = -0.100

    print("Plate place pose:", panel_name, grasp_pose.pose.position)
    
    self.planning_scene_interface.allow_collisions("base_fixture_top", panel_name)
    if not self.active_robots[robot_name].go_to_pose_goal(above_plate_pose, speed=1.0, move_lin=True, timeout=15):
      return False
    self.active_robots[robot_name].go_to_pose_goal(place_pose, speed=0.2, move_lin=True)
    self.planning_scene_interface.disallow_collisions("base_fixture_top", panel_name)
    self.confirm_to_proceed("finetune")
    self.active_robots[robot_name].gripper.open(opening_width=0.007, velocity=0.01)
    self.active_robots[robot_name].gripper.forget_attached_item()
    self.despawn_object(panel_name)
    if pose_with_uncertainty!=None:
      self.place_object_with_uncertainty(panel_name, pose_with_uncertainty, 0.856)
    if panel_name == "panel_bearing":
      self.assembly_status.bearing_panel_placed_outside_of_tray = False
    else:
      self.assembly_status.motor_panel_placed_outside_of_tray = False
    return True
  
  def return_l_plates(self):
    """ Place L-plates back in the tray from storage.
    """
    rospy.logerr("return_l_plates is not implemented.")
    pass 

  def place_object_with_uncertainty(self, object_name, pose_with_uncertainty, support_surface_height):
    """ Calculate the object pose with uncertainty after a PLACE action (the pose input parameter is changed in place)

        support_surface_height is the z-coordinate of the support surface in the world frame.    
    """
    now=rospy.Time.now()
    collision_object=self.assembly_database.get_collision_object(object_name)
    update_goal=o2ac_msgs.msg.updateDistributionGoal()
    update_goal.observation_type=update_goal.PLACE_OBSERVATION
    update_goal.gripped_object=collision_object
    update_goal.place_observation.support_surface=support_surface_height
    self.listener.waitForTransform("world", pose_with_uncertainty.header.frame_id, now, rospy.Duration(1.0))
    update_goal.gripper_pose.pose= tf_conversions.posemath.toMsg(tf_conversions.posemath.fromTf(self.listener.lookupTransform("world", pose_with_uncertainty.header.frame_id, now)))
    update_goal.gripper_pose.header.frame_id="world"
    update_goal.gripper_pose.header.stamp= now
    update_goal.distribution_type=1
    update_goal.distribution=pose_with_uncertainty
    self.update_distribution_client.send_goal(update_goal)
    self.update_distribution_client.wait_for_result()
    update_result = self.update_distribution_client.get_result()
    transformed_pose=self.transform_pose_with_uncertainty("world", update_result.distribution, now=now)
    pose_with_uncertainty.header=transformed_pose.header
    pose_with_uncertainty.pose=transformed_pose.pose

    self.visualize_object_with_distribution(collision_object, pose_with_uncertainty, frame_locked=True)
    
  def push_object_with_uncertainty(self, object_name, gripper_pose, pose_with_uncertainty, y_shift=0.0):
    """ Calculate the object pose with uncertainty after a PUSH action (the pose input parameter is changed in place).
        This call of push action is particular: it simulates pushing the object into another object, while letting it slide through the gripper.

        y_shift is the amount by which the gripper moved (?)
    """
    collision_object=self.assembly_database.get_collision_object(object_name)
    update_goal=o2ac_msgs.msg.updateDistributionGoal()
    update_goal.observation_type=update_goal.PUSH_OBSERVATION
    update_goal.gripped_object=collision_object
    update_goal.gripper_pose = self.listener.transformPose("world", gripper_pose)
    gripper_transform = tf_conversions.posemath.fromMsg(update_goal.gripper_pose.pose)
    update_goal.distribution_type=1
    self.transform_uncertainty(gripper_transform.Inverse(), pose_with_uncertainty.pose, update_goal.distribution.pose)
    update_goal.distribution.header.frame_id="fake"
    update_goal.distribution.header.stamp=rospy.Time(0)
    self.update_distribution_client.send_goal(update_goal)
    self.update_distribution_client.wait_for_result()
    update_result = self.update_distribution_client.get_result()
    self.transform_uncertainty(gripper_transform, update_result.distribution.pose, pose_with_uncertainty.pose)

    self.visualize_object_with_distribution(collision_object, pose_with_uncertainty, frame_locked=True)
  
  def center_panel_with_uncertainty(self, panel_name, robot_name="a_bot", speed=1.0, store=True, pose_with_uncertainty=None):
    """ Places the plate next to the tray in a well-defined position, by pushing it into a stopper.
    """
    self.planning_scene_interface.allow_collisions("a_bot_base_smfl", panel_name)
    centering_frame = "left_centering_link"
    # object dimensions
    obj_dims = self.dimensions_dataset[panel_name]
        
    # Magic numbers
    gripper_at_stopper = -0.065
    # x,y,z pose w.r.t centering link
    x_pos = -0.065 if panel_name == "panel_bearing" else -0.045
    y_pos = 0.065 if panel_name == "panel_bearing" else -0.065
    z_pos = gripper_at_stopper + obj_dims[1]

    above_centering_pose = conversions.to_pose_stamped(centering_frame, [-0.15, y_pos, z_pos, 0, 0, 0])
    at_centering_pose    = conversions.to_pose_stamped(centering_frame, [x_pos, y_pos, z_pos, 0, 0, 0])
    pre_push_pose        = conversions.to_pose_stamped(centering_frame, [-0.01, y_pos, z_pos, 0, 0, 0])
    push_pose            = conversions.to_pose_stamped(centering_frame, [-0.011, y_pos, gripper_at_stopper, 0, 0, 0])
    above_centering_joint_pose = [0.48, -2.05, 2.05, -1.55, -1.58, -1.09-(tau/2)]
    
    self.active_robots[robot_name].move_joints(above_centering_joint_pose, speed=speed)
    self.active_robots[robot_name].go_to_pose_goal(above_centering_pose, speed=speed)
    self.active_robots[robot_name].go_to_pose_goal(at_centering_pose, speed=speed)
    self.active_robots[robot_name].gripper.open(opening_width=0.01)
    if pose_with_uncertainty:
      self.place_object_with_uncertainty(panel_name, pose_with_uncertainty, 0.7501)
    self.active_robots[robot_name].go_to_pose_goal(pre_push_pose, speed=speed)
    self.active_robots[robot_name].gripper.send_command(0.005, force=0, velocity=0.03)
    if not self.active_robots[robot_name].go_to_pose_goal(push_pose, speed=0.5):
      rospy.logerr("fail to push: %s" % panel_name)
      return False
    self.active_robots[robot_name].gripper.forget_attached_item()
    
    aligned_pose = [0.0, 0.067, -0.080] if panel_name == "panel_bearing" else [0.0, -0.063, -0.080]
    self.update_collision_item_pose(panel_name, conversions.to_pose_stamped(centering_frame, aligned_pose + [-0.500, 0.500, -0.500, -0.500]))
    if pose_with_uncertainty!=None:
      push_gripper_pose=conversions.to_pose_stamped(centering_frame, [0.0, aligned_pose[1], obj_dims[1]-0.07, pi, 0., 0.])
      self.push_object_with_uncertainty(panel_name, push_gripper_pose, pose_with_uncertainty)

    at_panel_center = conversions.to_pose_stamped(centering_frame, [-0.02, y_pos, -0.08+obj_dims[1]/2, 0, 0, 0])

    self.active_robots[robot_name].gripper.open(opening_width=0.03)
    if store:
      self.active_robots[robot_name].move_lin_rel(relative_translation=[0,-0.1,0.15])
    else:
      self.active_robots[robot_name].go_to_pose_goal(at_panel_center, speed=0.2)
      self.active_robots[robot_name].gripper.close()
      self.active_robots[robot_name].move_lin_rel(relative_translation=[0,-0.1,0.15])

    # If store, return pose
    return at_panel_center if store else True

  def center_panel(self, panel_name, robot_name="a_bot", speed=1.0, store=True):
    """ Places the plate next to the tray in a well-defined position, by pushing it into a stopper.
    """
    self.planning_scene_interface.allow_collisions("a_bot_base_smfl", panel_name)
    centering_frame = "left_centering_link"
    # object dimensions
    obj_dims = self.dimensions_dataset[panel_name]
        
    # Magic numbers
    gripper_at_stopper = -0.064
    # x,y,z pose w.r.t centering link
    x_pos = -0.065 if panel_name == "panel_bearing" else -0.045
    y_pos = 0.065 if panel_name == "panel_bearing" else -0.065 # Arbitrary position
    z_pos = gripper_at_stopper + obj_dims[1]

    above_centering_pose = conversions.to_pose_stamped(centering_frame, [-0.15, y_pos, z_pos, 0, 0, 0])
    at_centering_pose    = conversions.to_pose_stamped(centering_frame, [x_pos, y_pos, z_pos, 0, 0, 0])
    pre_push_pose        = conversions.to_pose_stamped(centering_frame, [-0.01, y_pos, z_pos + 0.01, 0, 0, 0])
    push_pose            = conversions.to_pose_stamped(centering_frame, [-0.0125, y_pos, gripper_at_stopper, 0, 0, 0])
    above_centering_joint_pose = [0.48, -2.05, 2.05, -1.55, -1.58, -1.09-(tau/2)]

    seq = []
    seq.append(helpers.to_sequence_item(above_centering_joint_pose, speed=speed, linear=False))
    seq.append(helpers.to_sequence_item(above_centering_pose, speed=speed))
    seq.append(helpers.to_sequence_item(at_centering_pose, speed=speed))
    seq.append(helpers.to_sequence_gripper(0.01, gripper_velocity=1.0, wait=False))
    seq.append(helpers.to_sequence_item(pre_push_pose, speed=speed))
    seq.append(helpers.to_sequence_gripper(0.0048, gripper_force=0, gripper_velocity=0.1))
    seq.append(helpers.to_sequence_item(push_pose, speed=0.1))

    def post_callback():
      self.active_robots[robot_name].gripper.forget_attached_item()
      aligned_pose = [0.0, 0.067, -0.080] if panel_name == "panel_bearing" else [0.0, -0.063, -0.080]
      self.update_collision_item_pose(panel_name, conversions.to_pose_stamped(centering_frame, aligned_pose + [-0.500, 0.500, -0.500, -0.500]))

    seq.append(helpers.to_sequence_gripper("open", gripper_opening_width=0.03, gripper_velocity=1.0, post_callback=post_callback))
  
    at_panel_center = conversions.to_pose_stamped(centering_frame, [-0.02, y_pos, -0.08+obj_dims[1]/2, 0, 0, 0])

    if store:
      seq.append(helpers.to_sequence_item_relative(pose=[0,-0.1,0.15,0,0,0]))
    else:
      seq.append(helpers.to_sequence_item(at_panel_center, speed=0.2))
      seq.append(helpers.to_sequence_gripper("close", gripper_velocity=1.0))
      seq.append(helpers.to_sequence_item_relative(pose=[0,-0.1,0.15,0,0,0]))
    
    if not self.execute_sequence("a_bot", seq, "center_panel_full"):
      rospy.logerr("fail to center panel: %s" % panel_name)
      return False

    # If store, return pose
    return at_panel_center if store else True

  def orient_panel2(self, panel_name, robot_name="a_bot", speed=0.5):
    object_frame = "assembled_part_01_screw_hole_panel1_1" if panel_name == "panel_bearing" else "assembled_part_01_screw_hole_panel2_1"
    object_length = 0.11 if panel_name == "paneactive_robots[robot_name].gripper.openl_bearing" else 0.06
    
    above_plate_pose = conversions.to_pose_stamped(object_frame, [-0.15, 0.012, (object_length/2 + 0.06), 0, 0, 0])
    place_pose       = conversions.to_pose_stamped(object_frame, [-0.06, 0.012, (object_length/2 + 0.06), 0, 0, 0])
    pre_push_pose    = conversions.to_pose_stamped(object_frame, [-0.02, 0.012, (object_length * 2),      0, 0, 0])
    push_pose        = conversions.to_pose_stamped(object_frame, [-0.02, 0.012, (object_length + 0.015),  0, 0, 0]) # TODO: FINETUNE

    if not self.active_robots[robot_name].move_joints([1.465, -1.933, 1.733, -1.349, -1.5707, -3.1415], speed=1.0):
      rospy.logerr("fail to go to intermediate pose: %s" % panel_name)
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(above_plate_pose, speed=speed, move_lin=False):
      rospy.logerr("fail to go to above_plate_pose: %s" % panel_name)
      return False
    if not self.active_robots[robot_name].go_to_pose_goal(place_pose, speed=speed):
      rospy.logerr("fail to go to place_pose: %s" % panel_name)
      return False
    self.active_robots[robot_name].gripper.open(opening_width=0.03)
    self.active_robots[robot_name].gripper.forget_attached_item()
    if not self.active_robots[robot_name].go_to_pose_goal(pre_push_pose, speed=speed):
      return False
    self.active_robots[robot_name].gripper.close()
    if not self.active_robots[robot_name].go_to_pose_goal(push_pose, speed=0.1):
      rospy.logerr("fail to push: %s" % panel_name)
      return False

    self.publish_part_in_assembled_position(panel_name)
    return True
    # at_hole    = conversions.to_pose_stamped("assembled_part_01_screw_hole_panel1_1", [-0.06, 0.0125, 0.052, -tau/2, 0, 0])
    # self.active_robots[robot_name].go_to_pose_goal(at_hole, speed=0.2)

  def simple_handover(self, from_robot_name, to_robot_name, handover_pose, object_name, grasp_pose, speed=1.0, simultaneous=True, pose_with_uncertainty=None):
    self.vision.activate_camera(to_robot_name+"_inside_camera")
    self.a_bot_success = False
    self.b_bot_success = False
    def b_task():
      self.b_bot_success = self.active_robots[from_robot_name].go_to_pose_goal(handover_pose, speed=speed, retry_non_linear=True)
      self.active_robots[from_robot_name].gripper.detach_object(object_name)
      self.active_robots[from_robot_name].gripper.forget_attached_item()
      return True
    def a_task():
      rospy.sleep(0.5)
      self.active_robots[to_robot_name].gripper.send_command(0.05, velocity=1.0, wait=False)
      pre_hand_over_pose = conversions.to_pose_stamped("tray_center", [0.0, 0.0, 0.3, tau/4, 0, tau/4])
      self.a_bot_success = self.active_robots[to_robot_name].go_to_pose_goal(pre_hand_over_pose, speed=0.5, move_lin=False, timeout=10.)
      return True
    if simultaneous:
      self.do_tasks_simultaneous(a_task, b_task, timeout=30)
      if not (self.a_bot_success and self.b_bot_success):
        rospy.logerr("Fail to execute simultaneous motions in handover")
        return False
    else:
      if not b_task():
        return False

    handover_grasp_pose = self.assembly_database.get_grasp_pose(object_name, grasp_pose)
    handover_grasp_pose.header.frame_id = "move_group/" + object_name
    handover_grasp_pose.header.stamp = rospy.Time.now()
    self.allow_collisions_with_robot_hand(object_name, to_robot_name, allow=True)
    if self.assembly_database.assembly_info.get(object_name + "_facing_backward", False):
      handover_grasp_pose = helpers.rotatePoseByRPY(tau/2, 0, 0, handover_grasp_pose)
    if not self.simple_pick(to_robot_name, object_pose=handover_grasp_pose, grasp_width=0.06, approach_height=0.05, grasp_height=0.0, 
                     axis="y", item_id_to_attach=object_name, lift_up_after_pick=False, approach_with_move_lin=False,
                            speed_fast=1.0, speed_slow=0.2, attach_with_collisions=True, pose_with_uncertainty=pose_with_uncertainty, object_name=object_name, pick_from_ground=False):
      rospy.logerr("Fail to execute handover's grasp")
      return False

    self.active_robots[from_robot_name].gripper.open(opening_width=0.03, velocity=1.0, wait=False)
    success = self.active_robots[from_robot_name].move_lin_rel(relative_translation=[-0.1, 0, 0], relative_to_tcp=True)
    if not success:
      rospy.logerr("Fail to move lin back b_bot")
    self.allow_collisions_with_robot_hand(object_name, from_robot_name, allow=False)
    return success

  def center_panel_on_base_plate(self, panel_name, calibration=False):
    self.allow_collisions_with_robot_hand("panel_motor", robot_name="a_bot")
    self.allow_collisions_with_robot_hand("panel_bearing", robot_name="a_bot")
    switch_panels_order = self.assembly_database.assembly_info.get("switched_motor_and_bearing", False)
    facing_backwards = self.assembly_database.assembly_info.get(panel_name + "_facing_backward", False)
    obj_dims = self.dimensions_dataset[panel_name]
    if panel_name == "panel_bearing":
      pre_push_pose_frame_id = "assembled_part_03_center"
      push_pose_frame_id = "assembled_part_03_bottom_corner_1" if not switch_panels_order else "assembled_part_02_bottom_corner_2"
    elif panel_name == "panel_motor":
      pre_push_pose_frame_id = "assembled_part_02_center"
      push_pose_frame_id = "assembled_part_03_bottom_corner_2" if not switch_panels_order else "assembled_part_02_bottom_corner_1"
    else:
      raise ValueError("Invalid panel name %s" % panel_name)

    self.a_bot.gripper.open(opening_width=0.04, wait=False)
    height_offset = -obj_dims[0]/2 + 0.01 # 1cm above panel bottom w.r.t to panel center
    pre_push_pose = conversions.to_pose_stamped(pre_push_pose_frame_id, [0.004, 0.002, height_offset, -0.500, 0.500, 0.500, 0.500])
    if facing_backwards:
      pre_push_pose = helpers.rotatePoseByRPY(tau/2,0,0,pre_push_pose)

    # Pose w.r.t to left panel in case it is different 
    if switch_panels_order:
      orientation = [0,0,0] if not self.assembly_database.assembly_info.get("panel_motor_facing_backward", False) else [tau/2,0,0]
    else:
      orientation = [0,0,0] if not self.assembly_database.assembly_info.get("panel_bearing_facing_backward", False) else [tau/2,0,0]
    push_pose     = conversions.to_pose_stamped(push_pose_frame_id, [-0.01, -0.0025, 0.015] + orientation)
    
    place_pose = self.define_panel_place_pose(panel_name)
    hold_pose = copy.deepcopy(place_pose)
    hold_pose.pose.position.x = - 0.034
    hold_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(40), 0))

    seq = []
    seq.append(helpers.to_sequence_gripper("open", gripper_opening_width=0.04, wait=False))
    seq.append(helpers.to_sequence_item(pre_push_pose, speed=0.5, linear=True))
    seq.append(helpers.to_sequence_gripper(0.005, gripper_force=0, gripper_velocity=1.0))
    seq.append(helpers.to_sequence_item(push_pose, speed=0.1, linear=True))
    seq.append(helpers.to_sequence_gripper("open", gripper_opening_width=0.02, wait=True))
    seq.append(helpers.to_sequence_item_relative(pose=[0, -obj_dims[1]/2+0.015, 0, 0, 0, 0]))
    seq.append(helpers.to_sequence_gripper("close", gripper_velocity=0.1, gripper_force=40))
    seq.append(helpers.to_sequence_item(place_pose, speed=0.5, linear=True))
    seq.append(helpers.to_sequence_gripper("open", gripper_opening_width=0.02, gripper_velocity=0.01, wait=True))
    seq.append(helpers.to_sequence_item(hold_pose, speed=0.5, linear=True))
    seq.append(helpers.to_sequence_gripper("close", gripper_force=80, gripper_velocity=0.01, wait=False))
    
    return self.execute_sequence("a_bot", seq, "center_panel_on_base_plate", plan_while_moving=calibration)

  def hold_panel_for_fastening(self, panel_name):
    self.a_bot.gripper.send_command(0.06, wait=False)
    self.a_bot.gripper.forget_attached_item()
    self.despawn_object(panel_name)
    
    place_pose_inclined = self.define_panel_place_pose(panel_name)
    place_pose_inclined.pose.position.x = - 0.034
    place_pose_inclined.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(40), 0))
    if not self.a_bot.go_to_pose_goal(place_pose_inclined, move_lin=True):
      return False
    self.a_bot.gripper.close(velocity=0.01, force=40, wait=False)
    
    # Open gripper slightly
    self.a_bot.gripper.send_command(0.0045, velocity=0.01, wait=False)

    return True

  def fasten_panel(self, panel_name, simultaneous=False, a_bot_task_2nd_screw=None, unequip_tool_on_success=False, b_bot_2nd_task=None):
    approach_from_front = self.assembly_database.assembly_info.get(panel_name + "_facing_backward", False)
    if panel_name == "panel_bearing":
      part_name = "assembled_part_03_"
    elif panel_name == "panel_motor":
      part_name = "assembled_part_02_"
    
    self.confirm_to_proceed("Plate in the correct position?")

    if not simultaneous:
      self.hold_panel_for_fastening(panel_name)

    self.confirm_to_proceed("Holes are aligned?")

    self.do_change_tool_action("b_bot", equip=True, screw_size = 4)
    self.vision.activate_camera(camera_name="b_bot_outside_camera")
    if not self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
      rospy.logerr("Failed to pick screw from feeder, could not fix the issue. Abort.")
      self.a_bot.gripper.open()
      self.a_bot.go_to_named_pose("home")
      self.b_bot.go_to_named_pose("home")
      return False

    screw_order = [part_name + "bottom_screw_hole_1", part_name + "bottom_screw_hole_2"]
    if approach_from_front:
      screw_order = screw_order[::-1]

    orientation = [tau/2+radians(25), 0, 0] if approach_from_front else [radians(-25), 0, 0]
    print("rot",orientation)
    screw_target_pose = conversions.to_pose_stamped(screw_order[0], [0, 0, 0] + orientation)
    print("pose",screw_target_pose.pose)

    if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel_name, approach_from_front=approach_from_front):
      # Fallback for screw 1
      rospy.logerr("Failed to fasten panel screw 1, trying to realign tool and retry.")
      def b_bot_task():
        self.realign_tool("b_bot", "screw_tool_m4")
        self.b_bot.go_to_named_pose("feeder_pick_ready")
        self.pick_screw_from_feeder("b_bot", screw_size = 4)
      def a_bot_task():
        self.center_panel_on_base_plate(panel_name)
      if simultaneous:
        self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
      else:
        b_bot_task()
        a_bot_task()
      # Retry fastening
      if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel_name):
        rospy.logerr("Failed to fasten panel screw 2, trying to realign tool and retry.")
        if simultaneous:
          self.do_tasks_simultaneous(a_bot_task, b_bot_task, timeout=120)
        else:
          b_bot_task()
          a_bot_task()
        if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel_name):
          rospy.logerr("Failed to fasten panel screw 3 abort.")
          self.unequip_tool("b_bot")
          self.a_bot.gripper.open()
          return False
    rospy.loginfo("Successfully fastened screw 1")
    self.set_base_lock(closed=False)
    self.set_base_lock(closed=True)

    ##### Second screw #####
    orientation = [tau/2+radians(30), 0, 0] if approach_from_front else [radians(-45), 0, 0]
    screw_target_pose = conversions.to_pose_stamped(screw_order[1], [0, 0, 0] + orientation)
    if panel_name == "panel_bearing":
      screw_target_pose.pose.position.y -= 0.0015
    self.a_bot_success = False
    self.b_bot_success = False
    def a_bot_task():
      self.a_bot.gripper.open(opening_width=0.03, wait=False)
      self.a_bot_success = self.a_bot.move_lin_rel(relative_translation=[-0.2,0,0], relative_to_tcp=True)
      if a_bot_task_2nd_screw:
        rospy.loginfo("Attempting a_bot extra task while b_bot fasten 2nd screw")
        self.a_bot_success = a_bot_task_2nd_screw()
        if not self.a_bot_success:
          rospy.logerr("Fail to do a_bot extra task")
    def b_bot_task():
      if not self.pick_screw_from_feeder("b_bot", screw_size = 4, realign_tool_upon_failure=True):
        self.b_bot_success = False
        return False
      self.b_bot_success = self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel_name,
                                                       spiral_radius=0.003, approach_from_front=approach_from_front)
      if self.b_bot_success and unequip_tool_on_success:
        self.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      if self.b_bot_success and b_bot_2nd_task:
        b_bot_2nd_task()
    
    if simultaneous:
      self.do_tasks_simultaneous(a_bot_task, b_bot_task, 300)
    else:
      a_bot_task()
      b_bot_task()

    if not self.a_bot_success or not self.b_bot_success:
      rospy.logerr("Failed to fasten panel. simultaneous=%s"%simultaneous)

    if not self.b_bot_success:
      # Fallback for screw 2: Realign tool, recenter plate, try again
      rospy.logerr("Failed to fasten panel screw 2, trying to realign tool and retrying.")
      self.realign_tool("b_bot", "screw_tool_m4")
      self.b_bot.go_to_named_pose("feeder_pick_ready")
      self.pick_screw_from_feeder("b_bot", screw_size = 4)
      self.hold_panel_for_fastening(panel_name)
      self.a_bot.gripper.open(opening_width=0.03, wait=False)
      self.a_bot_success = self.a_bot.move_lin_rel(relative_translation=[-0.2,0,0], relative_to_tcp=True)
      
      # Recenter plate
      center_plate_pose = geometry_msgs.msg.PoseStamped()
      if panel_name == "panel_bearing":
        center_plate_pose.header.frame_id = part_name + "pulley_ridge_middle"
      else:  # motor panel
        center_plate_pose.header.frame_id = part_name + "motor_screw_hole_5"
      center_plate_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, radians(60), -tau/4))
      center_plate_pose.pose.position.x = 0.0025
      if not simultaneous: # TODO: add a fallback for simultaneous motions
        self.a_bot.gripper.open(opening_width=0.04, wait=False)
        self.a_bot.move_lin_rel([0.2,0,0], relative_to_tcp=True, speed=.3)
        self.a_bot.go_to_pose_goal(center_plate_pose, move_lin=True)
        self.a_bot.gripper.close(force = 100)
        self.a_bot.gripper.open()
        self.a_bot.move_lin_rel([-0.2,0,0], relative_to_tcp=True, speed=.3)
      if not self.fasten_screw_vertical('b_bot', screw_target_pose, allow_collision_with_object=panel_name, approach_from_front=approach_from_front):
        rospy.logerr("Failed to fasten panel screw 2 again. Aborting.")
        return False
    
    self.set_base_lock(closed=False)
    self.set_base_lock(closed=True)
    self.publish_part_in_assembled_position(panel_name, disable_collisions=False)
    return True

#### Tray manipulation

  def check_motor_pulley_angle(self):
    # Check bearing orientation
    approach_centering = conversions.to_pose_stamped("assembled_part_04_tip", [0.0, 0, -0.15,  0, -tau/4., 0])
    self.b_bot.go_to_pose_goal(approach_centering, speed=0.1, end_effector_link="b_bot_outside_camera_link",move_lin=False)
    self.vision.activate_camera("b_bot_outside_camera")

    # TODO(cambel): compute angle

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

  def center_tray_stack(self, orientation_parallel=True, spawn_single_tray=False):
    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)
    short_side = 0.255/2.
    long_side = 0.375/2.

    self.spawn_tray_stack(orientation_parallel=orientation_parallel)

    tray_pose = None
    tries = 0
    while not tray_pose and tries < 10:
      try:
        self.listener.waitForTransform("agv_tray_center", "move_group/tray1/center", rospy.Time(0), rospy.Duration(5))
        tray_pose = self.listener.transformPose("agv_tray_center", conversions.to_pose_stamped("move_group/tray1/center",[0,0,0,0,0,0]))
      except:
        rospy.sleep(1)
      tries += 1
      if tray_pose:
        break

    if spawn_single_tray:
      self.despawn_object("tray1")

    tray_point = conversions.from_point(tray_pose.pose.position)
    tray_x, tray_y, tray1_z = tray_point
    # Push the tray from the side
    self.a_bot.gripper.open(wait=False)
    self.b_bot.gripper.open(wait=False)
    self.ab_bot.go_to_named_pose("centering_area", speed=1.0)

    # Gripper needs to be open for these poses
    offset = long_side + 0.1 if orientation_parallel else short_side + 0.1
    a_bot_point = [tray_x + 0.01, tray_y - offset, tray1_z + 0.07]
    b_bot_point = [tray_x + 0.004, tray_y + offset, tray1_z + 0.08]
    a_bot_goal = tray_y-long_side-0.025 if orientation_parallel else tray_y-short_side-0.025
    b_bot_goal = tray_y+long_side+0.025 if orientation_parallel else tray_y+short_side+0.025
    a_bot_centering_height = tray1_z - 0.065
    b_bot_centering_height = tray1_z - 0.07
    frame = "agv_tray_center"
    a_bot_push_tray_side_start_high = conversions.to_pose_stamped(frame, [a_bot_point[0], a_bot_point[1], a_bot_point[2], tau/4, tau/4, 0])
    b_bot_push_tray_side_start_high = conversions.to_pose_stamped(frame, [b_bot_point[0], b_bot_point[1], b_bot_point[2], -tau/4, tau/4, 0])
    a_bot_push_tray_side_start      = conversions.to_pose_stamped(frame, [a_bot_point[0], a_bot_point[1], a_bot_centering_height, tau/4, tau/4, 0])
    b_bot_push_tray_side_start      = conversions.to_pose_stamped(frame, [b_bot_point[0], b_bot_point[1], b_bot_centering_height, -tau/4, tau/4, 0])
    a_bot_push_tray_side_goal       = conversions.to_pose_stamped(frame, [a_bot_point[0], a_bot_goal, a_bot_centering_height, tau/4, tau/4, 0])
    b_bot_push_tray_side_goal       = conversions.to_pose_stamped(frame, [b_bot_point[0], b_bot_goal, b_bot_centering_height, -tau/4, tau/4, 0]) #.277
    a_bot_push_tray_side_retreat    = a_bot_push_tray_side_start_high
    b_bot_push_tray_side_retreat    = b_bot_push_tray_side_start_high if orientation_parallel else conversions.to_pose_stamped(frame, [-0.004, 0.48, 0.14, -tau/4, tau/4, 0])

    # 
    a_bot_start = [tray_x - short_side - 0.10, tray_y] if orientation_parallel else [tray_x - long_side - 0.1, tray_y]
    a_bot_goal  = [tray_x - short_side - 0.025, tray_y] if orientation_parallel else [tray_x - long_side, tray_y]
    a_bot_push_tray_front_start_high = conversions.to_pose_stamped(frame, [a_bot_start[0], a_bot_start[1], a_bot_point[2], tau/2, tau/4, 0])
    a_bot_push_tray_front_start      = conversions.to_pose_stamped(frame, [a_bot_start[0], a_bot_start[1], a_bot_centering_height, tau/2, tau/4, 0])
    a_bot_push_tray_front_goal       = conversions.to_pose_stamped(frame, [a_bot_goal[0] , a_bot_goal[1] , a_bot_centering_height, tau/2, tau/4, 0])
    a_bot_push_tray_front_retreat    = conversions.to_pose_stamped(frame, [a_bot_start[0], 0.0, a_bot_point[2], tau/2, tau/4, 0])
    
    b_bot_goal  = [tray_x + short_side + 0.03, tray_y + 0.05] if orientation_parallel else [tray_x + long_side, tray_y]
    b_bot_start = [tray_x + short_side + 0.13, tray_y + 0.05] if orientation_parallel else [tray_x + long_side + 0.1, tray_y]
    b_bot_push_tray_front_start_high = conversions.to_pose_stamped(frame, [b_bot_start[0], b_bot_start[1], b_bot_point[2], 0, tau/4, 0])
    b_bot_push_tray_front_start      = conversions.to_pose_stamped(frame, [b_bot_start[0], b_bot_start[1], a_bot_centering_height, 0, tau/4, 0])
    b_bot_push_tray_front_goal       = conversions.to_pose_stamped(frame, [b_bot_goal[0] , b_bot_goal[1] , a_bot_centering_height, 0, tau/4, 0])
    b_bot_push_tray_front_retreat    = conversions.to_pose_stamped(frame, [b_bot_start[0], b_bot_goal[1], b_bot_point[2], 0, tau/4, 0])

    offset = 0.0 # w.r.t to the tray's center, to avoid grasping the center with both robots
    long_side = 0.375/2. + 0.004
    a_bot_point = [tray_point[0] - long_side, tray_point[1] - offset]
    b_bot_point = [tray_point[0] + long_side, tray_point[1] + offset]
    if orientation_parallel:
      a_bot_point = [tray_point[0] - offset, tray_point[1] - long_side]
      b_bot_point = [tray_point[0] + offset, tray_point[1] + long_side]
    a_bot_z_high = tray_point[2] + 0.10
    pick_orientation  = [tau/4, tau/4, 0] if not orientation_parallel else [ 0, tau/4, 0]
    a_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_high] + pick_orientation)
    b_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [b_bot_point[0], b_bot_point[1], a_bot_z_high] + pick_orientation)

    # Push the tray from the side
    self.a_bot.gripper.open(wait=False)
    self.b_bot.gripper.open(wait=False)

    plan_name = "center_trays" + ("_parallel" if orientation_parallel else "")
    saved_plan = helpers.load_single_plan(plan_name)
    if not saved_plan:
      rospy.loginfo("Computing center trays's plan")
      # push side
      waypoints = []
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_side_start_high, b_bot_push_tray_side_start_high, timeout=0.02, retry=True), 0, 1.0))
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_side_start, b_bot_push_tray_side_start, timeout=0.02, retry=True), 0, 1.0))
      res1 = self.ab_bot.move_joints_trajectory(waypoints, plan_only=True)
      if not res1:
        rospy.logerr("Fail to plan center trays")
        return False
      plan1, _ = res1
      plan2, _ = self.ab_bot.go_to_goal_poses(a_bot_push_tray_side_goal, b_bot_push_tray_side_goal, planner="OMPL", speed=0.05, plan_only=True, initial_joints=helpers.get_trajectory_joint_goal(plan1))

      # push front
      waypoints = []
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_side_retreat, b_bot_push_tray_side_retreat, timeout=0.02, retry=True), 0, 1.0))
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_front_start_high, b_bot_push_tray_front_start_high, timeout=0.02, retry=True), 0, 1.0))
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_front_start, b_bot_push_tray_front_start, timeout=0.02, retry=True), 0, 1.0))
      res3 = self.ab_bot.move_joints_trajectory(waypoints, plan_only=True, initial_joints=helpers.get_trajectory_joint_goal(plan2))
      if not res3:
        rospy.logerr("Fail to plan center trays")
        return False
      plan3, _ = res3

      plan4, _ = self.ab_bot.go_to_goal_poses(a_bot_push_tray_front_goal, b_bot_push_tray_front_goal, planner="OMPL", speed=0.05, plan_only=True, initial_joints=helpers.get_trajectory_joint_goal(plan3))
      
      # prepare for pick up
      waypoints = []
      waypoints.append((self.ab_bot.compute_ik(a_bot_push_tray_front_retreat, b_bot_push_tray_front_retreat, timeout=0.02, retry=True), 0, 1.0))
      waypoints.append((self.ab_bot.compute_ik(a_bot_above_tray_agv, b_bot_above_tray_agv, timeout=0.02, retry=True), 0, 1.0))

      res5 = self.ab_bot.move_joints_trajectory(waypoints, plan_only=True, initial_joints=helpers.get_trajectory_joint_goal(plan4))
      if not res5:
        rospy.logerr("Fail to plan center trays")
        return False
      plan5, _ = res5

      saved_plan = helpers.stack_plans([plan1, plan2, plan3, plan4, plan5])
      helpers.save_single_plan(plan_name, saved_plan)
    else:
      rospy.loginfo("Using trays's saved plan")

    if not self.ab_bot.execute_plan(saved_plan):
      rospy.logerr("Fail to execute center trays")
      return False

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)

  def spawn_tray_stack(self, stack_center=[-0.03,0], tray_heights=[0.03,-0.02], orientation_parallel=False, spawn_single_tray=False):
    orientation = [0, 0, 0] if orientation_parallel else [0, 0, tau/4] # tray's long side parallel to the table
    self.trays = {"tray%s"%(i+1): (stack_center+[tray_height], orientation_parallel) for i, tray_height in enumerate(tray_heights)}
    self.trays_return = {"tray%s"%(i+1): (stack_center+[tray_height], orientation_parallel) for i, tray_height in enumerate(tray_heights[::-1])}
    for i, (name, pose) in enumerate(self.trays.items()):
      pose = conversions.to_pose(pose[0]+orientation)
      if (spawn_single_tray and i == len(self.trays)-1) or not spawn_single_tray:
        self.planning_scene_interface.add_object(helpers.create_tray_collision_object(name, pose, "agv_tray_center"))
      self.planning_scene_interface.allow_collisions(name, "")

  def pick_tray_from_agv_stack_calibration_short_side(self, tray_name):
    try:
      self.listener.waitForTransform("agv_tray_center", "move_group/"+tray_name+"/center", rospy.Time(0), rospy.Duration(5))
    except:
      pass
    tray_pose = self.listener.transformPose("agv_tray_center", conversions.to_pose_stamped("move_group/"+tray_name+"/center",[0,0,0,0,0,0]))
    tray_point = conversions.from_point(tray_pose.pose.position)
    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)

    offset = 0.05
    short_side = 0.255/2.
    a_bot_x = tray_point[0] - offset
    b_bot_x = tray_point[0] + offset
    a_bot_y = tray_point[1] - short_side
    b_bot_y = tray_point[1] + short_side
    a_bot_z_high = tray_point[2] + 0.10
    a_bot_z_low  = tray_point[2] + 0.01
    a_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [a_bot_x, a_bot_y, a_bot_z_high, 0, tau/4, 0])
    b_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [b_bot_x, b_bot_y, a_bot_z_high, 0, tau/4, 0])
    b_bot_at_tray_agv    = conversions.to_pose_stamped("agv_tray_center", [b_bot_x, b_bot_y, a_bot_z_low, 0, tau/4, 0])
    
    a_bot_x +=  short_side + offset
    b_bot_x += -short_side - offset
    a_bot_y += -short_side + offset
    b_bot_y +=  short_side - offset
    a_bot_at_tray_table    = conversions.to_pose_stamped("tray_center", [short_side, -offset, 0.02, -tau/4, tau/4, 0])
    a_bot_above_table      = conversions.to_pose_stamped("tray_center", [short_side, -offset, 0.15, -tau/4, tau/4, 0])

    # Go to tray
    self.ab_bot.go_to_goal_poses(a_bot_above_tray_agv, b_bot_above_tray_agv, planner="OMPL")
    slave_relation = self.ab_bot.get_relative_pose_of_slave("b_bot", "a_bot")
    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_agv, slave_relation)
    
    # Grasp
    self.b_bot.gripper.attach_object("tray1", with_collisions=True)
    self.a_bot.gripper.close()
    self.b_bot.gripper.close()

    # move to tray center
    self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_agv, slave_relation)

    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_table, slave_relation)
    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_table, slave_relation)
    
    self.b_bot.gripper.detach_object("tray1")
    self.b_bot.gripper.forget_attached_item()
    self.b_bot.gripper.open(wait=False)
    self.a_bot.gripper.open()

    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_table, slave_relation)
    self.b_bot.go_to_named_pose("home")
    self.a_bot.go_to_named_pose("home")
    self.despawn_object("tray1")

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)
    return True

  def pick_tray_from_agv_stack_calibration_long_side(self, tray_name, use_saved_trajectory=True):
    self.ab_bot.go_to_named_pose("pickup_tray", speed=1.0)
    rospy.sleep(0.3)
    try:
      self.listener.waitForTransform("agv_tray_center", "move_group/"+tray_name+"/center", rospy.Time(0), rospy.Duration(5))
      tray_pose = self.listener.transformPose("agv_tray_center", conversions.to_pose_stamped("move_group/"+tray_name+"/center",[0,0,0,0,0,0]))
    except:
      pass
    tray_point = conversions.from_point(tray_pose.pose.position)
    tray_parallel = self.trays[tray_name][1]
    pick_orientation  = [tau/4, tau/4, 0] if not tray_parallel else [ 0, tau/4, 0]
    place_orientation = [    0, tau/4, 0]
    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)
    self.a_bot.gripper.open(opening_width=0.08, wait=False)
    self.b_bot.gripper.open(opening_width=0.08, wait=False)

    offset = 0.0 # w.r.t to the tray's center, to avoid grasping the center with both robots
    long_side = 0.375/2. + 0.004
    a_bot_point = [tray_point[0] - long_side, tray_point[1] - offset]
    b_bot_point = [tray_point[0] + long_side, tray_point[1] + offset]
    if tray_parallel:
      a_bot_point = [tray_point[0] - offset, tray_point[1] - long_side]
      b_bot_point = [tray_point[0] + offset, tray_point[1] + long_side]
    a_bot_z_high = tray_point[2] + 0.10
    a_bot_z_low  = tray_point[2] + 0.01
    a_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_high] + pick_orientation)
    b_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [b_bot_point[0], b_bot_point[1], a_bot_z_high] + pick_orientation)
    a_bot_at_tray_agv    = conversions.to_pose_stamped("agv_tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_low]  + pick_orientation)
    
    a_bot_point = [-offset-0.017, -long_side-0.003] if tray_parallel else [offset, -long_side]

    a_bot_at_tray_table    = conversions.to_pose_stamped("tray_center", [a_bot_point[0], a_bot_point[1], 0.03] + place_orientation)
    a_bot_above_table      = conversions.to_pose_stamped("tray_center", [a_bot_point[0], a_bot_point[1], 0.15] + place_orientation)
    a_bot_above_low_table  = conversions.to_pose_stamped("tray_center", [a_bot_point[0], a_bot_point[1], 0.04] + place_orientation)

    # Go to tray
    self.ab_bot.go_to_goal_poses(a_bot_above_tray_agv, b_bot_above_tray_agv, planner="OMPL", speed=1.0)
    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_agv, slave_relation)
    
    # Grasp
    self.b_bot.gripper.attach_object(tray_name, with_collisions=True)
    self.b_bot.gripper.close(force=100, wait=False)
    self.a_bot.gripper.close(force=100)
    self.b_bot.set_payload(2.5, center_of_gravity=[0.001, -0.018, 0.049])
    self.a_bot.set_payload(2.5, center_of_gravity=[0.0, -0.017, 0.053])
    rospy.sleep(0.5)

    # move to tray center
    if tray_parallel:
      seq = []
      seq.append(helpers.to_sequence_item_master_slave("a_bot", "b_bot", a_bot_above_tray_agv, slave_relation, speed=0.6))
      seq.append(helpers.to_sequence_item_master_slave("a_bot", "b_bot", a_bot_above_table, slave_relation, speed=0.6))
      seq.append(helpers.to_sequence_item_master_slave("a_bot", "b_bot", a_bot_above_low_table, slave_relation, speed=0.3))
      seq.append(helpers.to_sequence_item_master_slave("a_bot", "b_bot", a_bot_at_tray_table, slave_relation, speed=0.015))
      self.execute_sequence("ab_bot", seq, "pick_tray_long_side", save_on_success=True, use_saved_plans=use_saved_trajectory)
    else:
      self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_tray_agv, slave_relation)
      slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot") # necessary in case of rotations
      self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_table, slave_relation)
      slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot") # necessary in case of rotations
      self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_table, slave_relation)

    is_protective_stop = False    
    if self.a_bot.is_protective_stopped():
      rospy.logfatal("Fatal error. attempting to unlock a_bot")
      self.a_bot.unlock_protective_stop()
      is_protective_stop = True
    if self.b_bot.is_protective_stopped():
      rospy.logfatal("Fatal error. attempting to unlock b_bot")
      self.b_bot.unlock_protective_stop()
      is_protective_stop = True

    self.b_bot.gripper.detach_object(tray_name)
    self.b_bot.gripper.forget_attached_item()
    self.b_bot.gripper.open(wait=False)
    self.a_bot.gripper.open(wait=False)
    self.b_bot.set_payload(1.2, center_of_gravity=[0.001, -0.018, 0.049])
    self.a_bot.set_payload(1.27, center_of_gravity=[0.0, -0.017, 0.053])
    rospy.sleep(0.5)

    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_table, slave_relation)

    if is_protective_stop:
      # fallback
      self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_table, slave_relation)
      self.a_bot.gripper.close(wait=False)
      self.b_bot.gripper.close()
      self.b_bot.gripper.open(opening_width=0.08, wait=False)
      self.a_bot.gripper.open(opening_width=0.08, wait=False)
      rospy.sleep(0.5)
      self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_table, slave_relation)
      # a_bot_at_tray_table_rotated    = conversions.to_pose_stamped("tray_center", [+0.225/2., 0, 0.015, -tau/4, tau/4, 0])
      # a_bot_above_tray_table_rotated    = conversions.to_pose_stamped("tray_center", [+0.225/2., 0, 0.15, -tau/4, tau/4, 0])
      # b_bot_at_tray_table_rotated    = conversions.to_pose_stamped("tray_center", [-0.225/2., 0, 0.015, -tau/4, tau/4, 0])
      # b_bot_above_tray_table_rotated    = conversions.to_pose_stamped("tray_center", [-0.225/2., 0, 0.15, -tau/4, tau/4, 0])
      # self.ab_bot.go_to_goal_poses(a_bot_above_tray_table_rotated, b_bot_above_tray_table_rotated, planner="OMPL", speed=1.0)
      # self.ab_bot.go_to_goal_poses(a_bot_at_tray_table_rotated, b_bot_at_tray_table_rotated, planner="OMPL", speed=0.3)
      # self.a_bot.gripper.close(wait=False)
      # self.b_bot.gripper.close()
      # self.b_bot.gripper.open(wait=False)
      # self.a_bot.gripper.open(wait=False)
      # rospy.sleep(0.5)
      # self.ab_bot.go_to_goal_poses(a_bot_above_tray_table_rotated, b_bot_above_tray_table_rotated, planner="OMPL", speed=1.0)

    if tray_parallel:
      self.a_bot.go_to_named_pose("home", speed=1.0)
      # self.ab_bot.go_to_named_pose("home")
    else:
      self.a_bot.go_to_named_pose("home", speed=1.0)
      self.b_bot.go_to_named_pose("home", speed=1.0)
    self.despawn_object(tray_name)

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)

    return True

  def return_tray_to_agv_stack_calibration_long_side(self, tray_name):
    tray_point = self.trays_return[tray_name][0]
    tray_parallel = self.trays[tray_name][1]
    pose = conversions.to_pose_stamped("tray_center", [0, 0, 0.01, 0, 0, 0])
    self.planning_scene_interface.add_box(tray_name, pose, [.255, .375, 0.05])
    self.planning_scene_interface.allow_collisions(tray_name, "")

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=True)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=True)
    self.a_bot.gripper.send_command(0.05, wait=False)
    self.b_bot.gripper.send_command(0.05, wait=False)

    offset = 0.0 # w.r.t to the tray's center, to avoid grasping the center with both robots
    long_side = 0.375/2.
    short_side = 0.255
    a_bot_point = [-offset, -long_side]
    b_bot_point = [+offset-0.01, +long_side]
    a_bot_z_high = 0.10
    a_bot_z_low  = 0.025
    a_bot_above_tray_table = conversions.to_pose_stamped("tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_high, 0, tau/4, 0])
    b_bot_above_tray_table = conversions.to_pose_stamped("tray_center", [b_bot_point[0], b_bot_point[1], a_bot_z_high, 0, tau/4, 0])
    a_bot_at_tray_table    = conversions.to_pose_stamped("tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_low,  0, tau/4, 0])
    
    # Go to tray
    self.ab_bot.go_to_goal_poses(a_bot_above_tray_table, b_bot_above_tray_table, planner="OMPL")
    self.confirm_to_proceed("0")
    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_table, slave_relation)
    
    self.confirm_to_proceed("1")

    # Grasp
    self.b_bot.gripper.attach_object(tray_name, with_collisions=True)
    self.b_bot.gripper.close(force=100, wait=False)
    self.a_bot.gripper.close(force=100)
    self.b_bot.set_payload(2.5, center_of_gravity=[0.001, -0.018, 0.049])
    self.a_bot.set_payload(2.5, center_of_gravity=[0.0, -0.017, 0.053])
    self.confirm_to_proceed("2")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_tray_table, slave_relation, speed=0.1)
    self.confirm_to_proceed("3")

    if tray_name == "tray1":
      a_bot_point = [tray_point[0] + long_side, -tray_point[1] - offset]
      if tray_parallel:
        a_bot_point = [tray_point[0] + (short_side + 0.04) - offset, (tray_point[1] - long_side)]
      a_bot_z_high = tray_point[2] + 0.07
      a_bot_z_low  = tray_point[2] + 0.01
    elif tray_name == "tray2":
      a_bot_point = [tray_point[0], -tray_point[1] - offset]
      if tray_parallel:
        a_bot_point = [tray_point[0] - offset, (tray_point[1] - long_side)]
      a_bot_z_high = 0.06
      a_bot_z_low  = 0.01
    else:
      self.a_bot.set_payload(1.27, center_of_gravity=[0.0, -0.017, 0.053])
      self.b_bot.set_payload(1.2, center_of_gravity=[0.001, -0.018, 0.049]) # Default

      raise ValueError("More than 2 trays are not supported yet")
    orientation = [0, tau/4, 0] if tray_parallel else [-tau/4, tau/4, 0]

    a_bot_above_tray_agv = conversions.to_pose_stamped("agv_tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_high] + orientation)
    a_bot_at_tray_agv    = conversions.to_pose_stamped("agv_tray_center", [a_bot_point[0], a_bot_point[1], a_bot_z_low]  + orientation)

    # move to agv tray center
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_tray_agv, slave_relation, speed=0.1)

    slave_relation = self.ab_bot.get_relative_pose_of_slave("a_bot", "b_bot")
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_at_tray_agv, slave_relation, speed=0.1)
    
    self.b_bot.gripper.detach_object(tray_name)
    self.b_bot.gripper.forget_attached_item()
    self.b_bot.gripper.open(wait=False)
    self.a_bot.gripper.open(wait=False)
    rospy.sleep(0.5)
    self.a_bot.set_payload(1.27, center_of_gravity=[0.0, -0.017, 0.053])
    self.b_bot.set_payload(1.2, center_of_gravity=[0.001, -0.018, 0.049]) # Default
    self.ab_bot.master_slave_control("a_bot", "b_bot", a_bot_above_tray_agv, slave_relation)

    # self.despawn_object(tray_name)

    self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray", "b_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
    self.allow_collisions_with_robot_hand("tray_center", "b_bot", allow=False)

    return True

  def unload_drive_unit(self):
    """ Pick the drive unit from the fixation and place it in the tray.
    """
    a_bot_above_drive_unit = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.0025, -0.068, 0.060, 0, 0.891, tau/4])
    b_bot_above_drive_unit = conversions.to_pose_stamped("assembled_part_03_front_hole", [0.0025, -0.067, 0.078, 0, 0.883, tau/4])
    a_bot_at_drive_unit = conversions.to_pose_stamped("assembled_part_02_back_hole", [0.0025, -0.018, 0.008, 0, 0.891, tau/4])
    b_bot_at_drive_unit = conversions.to_pose_stamped("assembled_part_03_front_hole", [0.0025, -0.017, 0.028, 0, 0.883, tau/4])
    
    b_bot_drive_unit_loosened = self.listener.transformPose("tray_center", b_bot_at_drive_unit)
    b_bot_drive_unit_loosened.pose.position.x -= 0.008
    b_bot_drive_unit_loosened.pose.position.y -= 0.012
    b_bot_drive_unit_up = copy.deepcopy(b_bot_drive_unit_loosened)
    b_bot_drive_unit_up.pose.position.z += 0.1

    b_bot_above_tray_target = conversions.to_pose_stamped("tray_center", [0.03, 0.1, 0.2, 0, 0.883, -tau/4])
    b_bot_at_tray_target = conversions.to_pose_stamped("tray_center", [0.03, 0.1, 0.078, 0, 0.883, -tau/4])

    self.publish_status_text("Target: Unload product")
    self.unlock_base_plate()

    # Grasp the drive unit
    self.a_bot.gripper.open(opening_width=0.05, wait=False)
    self.b_bot.gripper.open(opening_width=0.05, wait=False)

    if not self.ab_bot.go_to_goal_poses(a_bot_above_drive_unit, b_bot_above_drive_unit, planner="OMPL"):
      return False
    if not self.ab_bot.go_to_goal_poses(a_bot_at_drive_unit, b_bot_at_drive_unit, planner="OMPL"):
      return False

    self.b_bot.gripper.close(force=100, wait=False)
    self.a_bot.gripper.close(force=100, wait=True)

    # Move product to the tray

    # TODO(felixvd): Adjust placement height if unit incomplete. Avoid protective stops.
    # if self.drive_unit_completed:
    #   b_bot_at_tray_target

    slave_relation = self.ab_bot.get_relative_pose_of_slave(master_name="b_bot", slave_name="a_bot")
    if not self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_drive_unit_loosened, slave_relation, speed=0.02):
      return False
    if not self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_drive_unit_up, slave_relation, speed=0.05):
      return False
    if not self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_above_tray_target, slave_relation, speed=0.2):
      return False
    if not self.ab_bot.master_slave_control("b_bot", "a_bot", b_bot_at_tray_target, slave_relation, speed=0.1):
      return False

    self.a_bot.gripper.open(opening_width=0.07, wait=False)
    self.b_bot.gripper.open(opening_width=0.07)

    self.ab_bot.go_to_named_pose("home")
    self.publish_status_text("SUCCESS: Unload product")
    return True

  def do_tasks_simultaneous(self, function_a_bot, function_b_bot, timeout=60.0):
    """ Execute two threads simultaneously. Break out after a time.

        TODO: Add usage example
    """
    a_thread = ThreadTrace(target=function_a_bot)
    a_thread.daemon = True
    b_thread = ThreadTrace(target=function_b_bot)
    b_thread.daemon = True
    rospy.set_param("/o2ac/simultaneous", True) # Inform of simultaneous to param server
    a_thread.start()
    b_thread.start()
    a_thread.join()
    rospy.loginfo("a_bot DONE")
    b_thread.join()
    rospy.loginfo("b_bot DONE")
    
    timeout = 1500 # almost ignore

    start_time = rospy.Time.now()
    a_thread.join(timeout)
    if a_thread.is_alive():
      rospy.logerr("a_bot not done yet, breaking out")
      a_thread.kill()
      b_thread.kill()
      rospy.set_param("/o2ac/simultaneous", False)
      return False
    rospy.loginfo("a_bot DONE")
    
    time_passed = (rospy.Time.now() - start_time).secs
    time_left_until_timeout = timeout - time_passed
    if time_left_until_timeout < 0.0:
      rospy.logerr("simultaneous task timed out!")
      b_thread.kill()
      rospy.set_param("/o2ac/simultaneous", False)
      return False

    b_thread.join(time_left_until_timeout)
    if b_thread.is_alive():
      rospy.logerr("b_bot not done yet, abort")
      rospy.logerr("Started at " + str(start_time) + " and timeout was " + str(timeout))
      b_thread.kill()
      rospy.set_param("/o2ac/simultaneous", False)
      return False
    rospy.set_param("/o2ac/simultaneous", False)
    rospy.loginfo("b_bot DONE")
    return True

  def orient_base_panel(self, grasp_pose):
    dx, dy = self.distances_from_tray_border(grasp_pose)
    print("Base distance from border dx:", dx, "dy:", dy)
    
    if self.assembly_database.db_name == "wrs_assembly_2020" or (dx < 0.03 or dy < 0.03):
      rospy.loginfo("moving towards center")
      # Move plate into the middle a bit to avoid collisions with tray wall or other parts during centering
      self.a_bot.gripper.close()
      self.a_bot.gripper.attach_object("base", with_collisions=True)
      self.planning_scene_interface.allow_collisions("base", "")
      direction = 'x' if dy > dx else 'y'
      self.move_towards_tray_center("a_bot", distance=0.05, go_back_halfway=True, one_direction=direction, go_back_ratio=0.4)

  def pick_base_panel(self, grasp_name='default_grasp', skip_initial_perception=False, 
                            use_b_bot_camera=False, retry_with_rotated_orientation=True, 
                            retry_counter=0):
    # TODO(cambel): Add fallback grasp names, if nothing else, use terminal for grasp
    # Find base panel 
    if use_b_bot_camera:
      robot_name = "b_bot"
    else:
      robot_name = "a_bot"
    if self.use_real_robot:
      if not skip_initial_perception:
        self.activate_led(robot_name)
        rospy.loginfo("Looking for base plate")
        base_pose = self.get_large_item_position_from_top("base", robot_name)
        if not base_pose:
          if use_b_bot_camera:
            self.active_robots[robot_name].go_to_named_pose("home")
            rospy.logerr("Cannot find base plate in tray. Return False.")
            self.active_robots[robot_name].go_to_named_pose("home")
            return False
          if retry_counter < 8:
            self.despawn_object("base")
            return self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=False, use_b_bot_camera=False, 
                                        retry_with_rotated_orientation=True, retry_counter=retry_counter+1)
        # FIXME: silly workaround to the base consistently being predicted in the wrong orientation
        self.rotate_plate_collision_object_in_tray("base")
    else:
      self.active_robots[robot_name].go_to_named_pose("above_tray", speed=1.0)
      goal = conversions.to_pose_stamped("tray_center", [-0.08, -0.08, 0.001, tau/4, 0, tau/4])
      self.spawn_object("base", goal, goal.header.frame_id)
      rospy.sleep(0.5)
    
    # Load grasp pose and allow collisions to prepare for grasp
    rospy.sleep(0.3)
    self.allow_collisions_with_robot_hand("base", "a_bot", allow=True)
    object_name = "base"
    target_frame = "tray_center"
    centering_pose = self.get_transformed_grasp_pose(object_name, "terminal_grasp", target_frame)
    grasp_pose     = self.get_transformed_grasp_pose(object_name, grasp_name, target_frame)
    
    if not centering_pose or not grasp_pose:
      rospy.logerr("Could not load grasp poses for object " + "base" + ". Aborting pick.")
      return False
    centering_pose.pose.position.z += .006

    p = helpers.interpolate_between_poses(centering_pose.pose, grasp_pose.pose, 0.1)
    centering_pose_closer_to_part_center = conversions.to_pose_stamped(target_frame, conversions.from_pose_to_list(p))
    centering_pose_closer_to_part_center.pose.position.z = centering_pose.pose.position.z # do not offset anything
    
    above_centering_pose = copy.deepcopy(centering_pose)
    above_centering_pose.pose.position.z += .08
    
    self.a_bot.gripper.open(opening_width=0.08, wait=False)
    self.planning_scene_interface.allow_collisions("base", "")  # Allow collisions with all other objects
    self.planning_scene_interface.allow_collisions("base", "tray")
    self.planning_scene_interface.allow_collisions("base", "tray_center")
    if not self.a_bot.go_to_pose_goal(above_centering_pose, speed=0.5, move_lin=False):
      return False
    if not self.a_bot.go_to_pose_goal(centering_pose, speed=0.5, move_lin=True):
      return False

    self.allow_collisions_with_robot_hand("tray", "a_bot")
    self.allow_collisions_with_robot_hand("tray_center", "a_bot")
    # Grasp terminal to check for orientation, assume the perception
    self.a_bot.gripper.close(force=0)
    if self.a_bot.gripper.opening_width < 0.008 and self.use_real_robot:
      self.despawn_object("base")
      if retry_with_rotated_orientation:
        # Assume the plate was perceived rotated by 180 degrees and retry
        rospy.logwarn("Centering unsuccessful. Assuming vision failed. Retry with other orientation.")
        self.rotate_plate_collision_object_in_tray("base")
        rospy.sleep(1.5)
        return self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=True, use_b_bot_camera=False, retry_with_rotated_orientation=False, retry_counter=retry_counter)
      elif retry_counter < 8:
        return self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=False, use_b_bot_camera=False, retry_with_rotated_orientation=True, retry_counter=retry_counter+1)
      else:  # If retry has also failed
        rospy.logerr("Plate was not perceived by gripper. Breaking out.")
        self.a_bot.gripper.open(wait=False)
        return False
    print("gripper opening after terminal confirmation:", round(self.a_bot.gripper.opening_width, 4))

    self.orient_base_panel(centering_pose)
    
    # move_towards_tray_center disables collisions with the tray, so we have to reallow them here
    self.allow_collisions_with_robot_hand("tray", "a_bot")
    self.allow_collisions_with_robot_hand("tray_center", "a_bot")
    # self.a_bot.gripper.detach_object("base")
    self.a_bot.gripper.open(opening_width=0.07)

    centering_pose = self.get_transformed_grasp_pose(object_name, "terminal_grasp", target_frame)
    centering_pose.pose.position.z += .007
    centering_pose, _ = self.constrain_grasp_into_tray("a_bot", centering_pose, grasp_width=0.06)

    if not self.a_bot.go_to_pose_goal(centering_pose, speed=0.5, move_lin=True):
      self.despawn_object("base")
      return False

    self.center_with_gripper("a_bot", opening_width=0.06, gripper_force=150, 
                                      required_width_when_closed=0.008, move_back_to_initial_position=False,
                                      gripper_velocity=0.2)
    self.update_base_plate_pose_from_grasp_pose(centering_pose)

    if not self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.1]):
      self.despawn_object("base")
      return False
    
    grasp_pose = self.get_transformed_grasp_pose(object_name, grasp_name, target_frame)

    grasp_width         = 0.0425 if grasp_name == "big_holes_grasp" else 0.125
    minimum_grasp_width = 0.025  if grasp_name == "big_holes_grasp" else 0.03
    grasp_height        = 0.015  if grasp_name == "big_holes_grasp" else 0.0
    self.allow_collisions_with_robot_hand("workplate", "a_bot", allow=True)
    success = self.simple_pick("a_bot", grasp_pose, axis="z", approach_height=0.05, retreat_height=0.15, grasp_height=grasp_height,
                               grasp_width=grasp_width, gripper_force=100.0, minimum_grasp_width=minimum_grasp_width, lift_up_after_pick=False,
                               item_id_to_attach="base", attach_with_collisions=True)

    success &= self.simple_gripper_check("a_bot", min_opening_width=minimum_grasp_width)
    
    if success:
      self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.15])
    
    success = self.simple_gripper_check("a_bot", min_opening_width=minimum_grasp_width)

    if not success:
      self.a_bot.gripper.open()
      self.a_bot.move_lin_rel(relative_translation=[0, 0, 0.1])
      self.allow_collisions_with_robot_hand("tray", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("tray_center", "a_bot", allow=False)
      self.allow_collisions_with_robot_hand("workplate", "a_bot", allow=False)
      self.despawn_object("base")
      if retry_counter < 3:
        return self.pick_base_panel(grasp_name=grasp_name, skip_initial_perception=False, use_b_bot_camera=False, retry_with_rotated_orientation=True, retry_counter=retry_counter+1)
      else:
        rospy.logerr("Fail to grasp base plate")
        return False

    return True # return the name of the grasp used

  def update_base_plate_pose_from_grasp_pose(self, new_pose, grasp_name="terminal_grasp"):
    grasp_pose =self.get_transformed_grasp_pose("base", grasp_name, target_frame="tray_center")
    base_pose =self.get_transformed_collision_object_pose("base", target_frame="tray_center")
    base_pose.pose.position.x += new_pose.pose.position.x - grasp_pose.pose.position.x
    base_pose.pose.position.y += new_pose.pose.position.y - grasp_pose.pose.position.y
    self.update_collision_item_pose("base", base_pose)

  def rotate_plate_collision_object_in_tray(self, plate_name):
    """ Rotates the collision object in the scene.
        Used as a fallback for failing vision.

        plate_name can be "base", "panel_motor", "panel_bearing". The angle is tau/2 for base, tau/4 for the panels.
    """
    if plate_name == "base":
      new_pose = conversions.to_pose_stamped("move_group/base", [0.2, 0.0, 0.12, 0, tau/2, 0])
    elif plate_name == "panel_motor":
      new_pose = conversions.to_pose_stamped("move_group/panel_motor", [0.06, 0.0, 0.0, 0, 0, tau/4])
    elif plate_name == "panel_bearing":
      new_pose = conversions.to_pose_stamped("move_group/panel_bearing", [0.101, 0.0, 0.0, 0, 0, tau/4]) # The sides are 0.09 and 0.112, but we accept the noise.
    new_pose = self.listener.transformPose("tray_center", new_pose)
    obj = self.assembly_database.get_collision_object(plate_name)
    obj.header.frame_id = new_pose.header.frame_id
    obj.pose = new_pose.pose
    self.planning_scene_interface.add_object(obj)
    rospy.sleep(0.1)
    self.constrain_into_tray(plate_name)

## Belt

  def define_belt_threading_waypoints(self, pulley_center_frame, radius, d_buffer, q_start, q_end):
    """ Return a list of 6D poses that contour the pulley with the belt hook tool.
        The end effector link is assumed to be around the curve of the hook, where the belt would be.
        The pulley_center_frame is assumed to at the center of the groove.
    """
    def pulley_zy(pulley_radius, d_buffer, theta):
      """ Return a tuple (z, y) of the target position around the pulley.
          Theta needs to be [0, tau].
      """
      z = sin(theta)*(pulley_radius + d_buffer)
      y = cos(theta)*(pulley_radius + d_buffer)
      return (z, y)
    
    def orientation(theta, theta_min, theta_max, q_start, q_end):
      ratio = (theta - theta_min)/(theta_max-theta_min)
      return tf_conversions.transformations.quaternion_slerp(q_start, q_end, ratio)

    waypoints = []
    theta_min = tau/4.0
    theta_max = tau*3/4.0
    step = (theta_max-theta_min)/20
    q_start = tf_conversions.transformations.quaternion_from_euler(tau/2 + radians(5), 0, 0)
    q_start = tf_conversions.transformations.quaternion_from_euler(tau/2 - radians(5), 0, 0)
    for theta in range(theta_min, theta_max+step, step):  # To include last element
      pz, py = pulley_zy()
      p = conversions.to_pose_stamped(pulley_center_frame, [0, py, pz, 0, 0, 0])
      q = orientation(theta, theta_min, theta_max, q_start, q_end)
      p.pose.orientation = geometry_msgs.msg.Quaternion(*q)
      waypoints.append(p)
    return waypoints

  def thread_belt_with_b_bot(self):
    # Start pose from UR script:
    # global pulley_east_p=p[.055997818734, -.418531223747, .290523106530, -1.827798982973, -1.874228477717, .614956800003]
    p_start = conversions.to_pose_stamped("b_bot_base", [.055997, -.418531, .290523, 0, 0, 0])
    q = helpers.ur_axis_angle_to_quat([-1.827798, -1.874228, .614956])
    p_start.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    
    # End pose from UR script:
    # global after_pulley_p=p[.044329768444, -.474814986661, .308597350181, -1.711792093336, -2.015147036377, .657820892175]
    p_end = conversions.to_pose_stamped("b_bot_base", [.044329, -.474814, .308597, 0, 0, 0])
    q = helpers.ur_axis_angle_to_quat([-1.71179, -2.01514, .657820])
    p_end.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    
    # FIXME: What was the frame name?
    waypoints = self.define_belt_threading_waypoints("taskboard_output_pulley_frame", radius=0.03, d_buffer=0.01)

    self.b_bot.go_to_pose_goal(p_start, end_effector_link="b_bot_belt_hook_curve_link")
    # TODO: Set end effector link
    (plan, fraction) = self.b_bot.compute_cartesian_path(waypoints,
                                      0.004,        # eef_step
                                      0.1)         # jump_threshold
    if fraction > 0.99:
      self.b_bot.execute_plan(plan)
      self.b_bot.go_to_pose_goal(p_end, end_effector_link="b_bot_belt_hook_curve_link")
      return True
    else:
      rospy.logerror("compute_cartesian_path failed: fraction = " +str(fraction))
      return False
    
  def belt_fallback(self, pick_goal):
    rospy.logerr("Belt pick has failed. Return tool and abort.")
    self.b_bot.load_and_execute_program(program_name="wrs2020/taskboard_place_hook.urp")
    rospy.sleep(2)
    pick_goal.pose.position.x = 0  # In tray_center
    pick_goal.pose.position.y = 0
    pick_goal.pose.position.z += 0.06
    self.a_bot.go_to_pose_goal(pick_goal, speed=1.0)
    self.a_bot.gripper.open(opening_width=0.07, wait=False)
    self.a_bot.go_to_named_pose("home")
    wait_for_UR_program("/b_bot", rospy.Duration.from_sec(20))
    return True
