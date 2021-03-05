#!/usr/bin/env python
import rospy
import moveit_commander
import geometry_msgs.msg
import sys

rospy.init_node('jointflips', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("manipulator")
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

start_joint_state = [1.7817931175231934, -1.5258987706950684, 1.684814755116598, -1.5867115459837855, -2.046345059071676, 1.8501019477844238]
group.set_joint_value_target(start_joint_state)
group.go(wait=True)

print("Enter for next movement")
i = raw_input()
if rospy.is_shutdown():
    raise

# Attach obstacle (will hit the wrist)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "ee_link"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position = geometry_msgs.msg.Point(.014, -.085, 0.07)
scene.add_box("box", box_pose, size=(.05, .03, .055))
scene.attach_box("ee_link", "box")

print("Enter for next movement")
i = raw_input()
if rospy.is_shutdown():
    raise

goal = geometry_msgs.msg.PoseStamped()
goal.header.frame_id = "base"
# goal.pose.position = geometry_msgs.msg.Point(0.03318, -0.55052, 0.31026)
# goal.pose.orientation = geometry_msgs.msg.Quaternion(-0.5, 0.00014274, 0.86603, 0.000267)

goal.pose.position = geometry_msgs.msg.Point(-0.03318, 0.55052, 0.31026)
goal.pose.orientation = geometry_msgs.msg.Quaternion(0.0042461, 0.51239, -0.018782, 0.85854)

waypoints = []
waypoints.append(goal.pose)
(plan, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
rospy.loginfo("Compute cartesian path succeeded " + str(fraction*100) + "%")
plan = group.retime_trajectory(robot.get_current_state(), plan, 1.0)
group.execute(plan, wait=True)
