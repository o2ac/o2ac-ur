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

import sys
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import controller_manager_msgs.srv as cm_srv
from franka_control.msg import ErrorRecoveryActionGoal
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
import franka_gripper.msg
import tf
import numpy as np

import geometry_msgs.msg
from franka_control.srv import SetJointImpedance
from franka_control.srv import SetCartesianImpedance
from franka_control.srv import SetLoad
from franka_control.srv import SetFullCollisionBehavior


class ControlSwitcher:
    # Class to switch between controllers in ROS
    def __init__(self, controllers, controller_manager_node='/controller_manager'):
        self.controllers = controllers
        rospy.wait_for_service(controller_manager_node + "/switch_controller")
        rospy.wait_for_service(controller_manager_node + "/list_controllers")

        self.switcher_srv = rospy.ServiceProxy(controller_manager_node + "/switch_controller", cm_srv.SwitchController)
        self.lister_srv = rospy.ServiceProxy(controller_manager_node + "/list_controller", cm_srv.ListControllers)

    def switch_controllers(self, start_controller_names):
        rospy.sleep(0.5)
        # Get list of controller full names to start and stop
        start_controllers = [self.controllers[start_controller] for start_controller in start_controller_names]
        stop_controllers = [self.controllers[n] for n in self.controllers if n not in start_controller_names]

        controller_switch_msg = cm_srv.SwitchControllerRequest()
        controller_switch_msg.strictness = 1
        controller_switch_msg.start_controllers = start_controllers
        controller_switch_msg.stop_controllers = stop_controllers

        result = self.switcher_srv(controller_switch_msg).ok
        if result:
            rospy.logdebug('Successfully switched to controllers {} ({})'.format(start_controllers, start_controller_names))
            return result
        else:
            rospy.logdebug("Failed switching controllers")
            return False

    def stop_controllers(self):
        self.switch_controllers([])


class Panda:
    def __init__(self):
        # Setup Moveit
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('panda_control', anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.ERROR = False

        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()

        # group_name = "a_bot"
        # self.group = moveit_commander.MoveGroupCommander(group_name)
        # self.group.set_max_velocity_scaling_factor(0.1)
        # self.group.set_end_effector_link("panda_hand")

        # Publishers and Subscribers
        #  Cartesian Velocity Pub
        self.velocity_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', geometry_msgs.msg.Twist, queue_size=1)
        #  Error Recovery Pub
        self.recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)
        #  Joint Impedance Service Proxy
        self.impedance_serv = rospy.ServiceProxy('/franka_control/set_joint_impedance', SetJointImpedance)
        #  EE Load Service Proxy
        self.load_serv = rospy.ServiceProxy('/franka_control/set_load', SetLoad)
        #  Collision Service
        self.collision_serv = rospy.ServiceProxy('/franka_control/set_full_collision_behavior', SetFullCollisionBehavior)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.robot_state_callback, queue_size=1)

        # Create the controller switcher
        self.cs = ControlSwitcher({'moveit':   'position_joint_trajectory_controller',
                                   'velocity': 'cartesian_velocity_node_controller'})

        rospy.sleep(1)

    def robot_state_callback(self, msg):
        for s in FrankaErrors.__slots__:
            error = getattr(msg.current_errors, s)
            if error:
                print("Error!!!")
                print(s)
                self.ERROR = True

    def set_joint_impedance(self, impedance):
        # Set impedance
        self.cs.stop_controllers()
        joint_impedance = [impedance for i in range(7)]
        self.impedance_serv(joint_impedance)

    def set_ee_load(self, mass):
        self.cs.stop_controllers()
        self.load_serv(mass, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def set_collision_behaviour_low(self):
        lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
        upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
        lower_torque_thresholds_acceleration = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
        upper_torque_thresholds_acceleration = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]

        lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
        lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

        self.collision_serv(lower_torque_thresholds_acceleration,
                            upper_torque_thresholds_acceleration,
                            lower_torque_thresholds_nominal,
                            upper_torque_thresholds_nominal,
                            lower_force_thresholds_acceleration,
                            upper_force_thresholds_acceleration,
                            lower_force_thresholds_nominal,
                            upper_force_thresholds_nominal)

    def set_gripper(self, width, speed=0.1, wait=True):
        client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()
        client.send_goal(franka_gripper.msg.MoveGoal(width, speed))
        return client.wait_for_result()

    def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
        client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
                speed,
                force
            )
        )
        return client.wait_for_result()

    def publish_cartesian_vel(self, vel):
        # Control the robot for 5 seconds w/ velocity control
        rate = rospy.Rate(100)
        v = geometry_msgs.msg.Twist()
        v.linear.x = vel[0]
        v.linear.y = vel[1]
        v.linear.z = vel[2]
        v.angular.x = vel[3]
        v.angular.y = vel[4]
        v.angular.z = vel[5]

        self.velocity_pub.publish(v)

    def spiral_motion(self, vel):
        self.cs.switch_controllers(["velocity"])
        rate = rospy.Rate(100)
        # Archimides Spiral
        a = 0.2
        for i in range(3000):
            t = 2*np.pi*float(i)/500
            x = 0.005*(a*t*np.sin(t))
            z = 0.005*(a*t*np.cos(t))
            self.publish_cartesian_vel([x, vel, z, 0.0, 0.0, 0.0])
            rate.sleep()

    def move_till_contact(self, vel):
        self.cs.switch_controllers(["velocity"])
        rate = rospy.Rate(100)
        while True:
            self.publish_cartesian_vel([0.0, vel, 0.0, 0.0, 0.0, 0.0])
            if self.ERROR:
                self.recovery_pub.publish(ErrorRecoveryActionGoal())
                return
            rate.sleep()

    def helix_motion(self, vel):
        self.cs.switch_controllers(["velocity"])
        rate = rospy.Rate(100)
        # Archimides Spiral
        a = 0.2
        for i in range(3000):
            s = np.clip(i, 0, 80)
            t = 2*np.pi*float(i)/500
            s = 2*np.pi*float(s)/500
            print(s)
            x = 0.005*(a*s*np.sin(t))
            z = 0.005*(a*s*np.cos(t))
            self.publish_cartesian_vel([x, vel, z, 0.0, 0.0, 0.0])
            rate.sleep()

    def move_to_pose(self, pose, velocity=0.1):
        # Switch to moveit control
        self.cs.switch_controllers(['moveit'])

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.pose.position.x = pose[0]
        pose_goal.pose.position.y = pose[1]
        pose_goal.pose.position.z = pose[2]
        pose_goal.pose.orientation.x = pose[3]
        pose_goal.pose.orientation.y = pose[4]
        pose_goal.pose.orientation.z = pose[5]
        pose_goal.pose.orientation.w = pose[6]
        pose_goal.header.frame_id = "panda_link0"

        pose_goal_world = self.tf_listener.transformPose("world", pose_goal).pose

        (plan, fraction) = self.group.compute_cartesian_path(
            [pose_goal_world],
            0.005,
            0.0)
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, velocity)
        if fraction != 1.0:
            raise ValueError("Unable to plan path")

        success = self.group.execute(plan, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        print("Done!")
        return success


# panda = Panda()

# panda.set_joint_impedance(2000.0)
# panda.set_ee_load(0.1) #0.0447
# panda.cs.switch_controllers(['moveit'])

# home_pose = [0.36, 0.0, 0.58, 1.0, 0.0, 0.0, 0.0]
# panda.move_to_pose(home_pose, velocity=0.5)
# panda.set_gripper(0.3)
# panda.grasp(speed=0.5)


# # Above grasp pose
# before_grasp_pose = [0.49036, -0.24, 0.17302, -0.70944, -0.0041187, 0.0043205, 0.70474]
# #above_grasp_pose = [0.49, -0.255, 0.165, -0.7, 0.0, 0.0, 0.7]
# panda.move_to_pose(before_grasp_pose, velocity=0.1)

# # closer to grasp pose
# near_grasp_pose = [0.49036, -0.18, 0.17302, -0.70944, -0.0041187, 0.0043205, 0.70474]
# #above_grasp_pose = [0.49, -0.255, 0.165, -0.7, 0.0, 0.0, 0.7]
# panda.move_to_pose(near_grasp_pose, velocity=0.1)


# panda.set_joint_impedance(1000.0)
# panda.cs.switch_controllers(['moveit'])
# # Grasp Pose
# grasp_pose = [0.49036, -0.15189, 0.17302, -0.70944, -0.0041187, 0.0043205, 0.70474]
# #grasp_pose = [0.49, -0.155, 0.165, -0.7, 0.0, 0.0, 0.7]
# panda.move_to_pose(grasp_pose, velocity=0.01)
# exit()

# # Above grasp pose

# #panda.cs.switch_controllers(['velocity'])

# rate = rospy.Rate(100)
# #ERROR = False

# #for i in range(1000):
# #    panda.publish_cartesian_vel([0.0,-0.005,0.0,0.0,0.0,0.0])
# #    rate.sleep()

# panda.move_to_pose(above_grasp_pose, velocity=0.01)


# panda.cs.switch_controllers(['velocity'])


# panda.cs.stop_controllers()
# panda.set_joint_impedance(100.0)
# panda.cs.switch_controllers(['velocity'])

# #ERROR = False
# while True:
#     panda.publish_cartesian_vel([0.0,0.005,0.0,0.0,0.0,0.0])
#     rate.sleep()
