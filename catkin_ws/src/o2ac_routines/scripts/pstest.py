#!/usr/bin/env python

import rospy

import moveit_commander
import moveit_msgs.msg
import copy

if __name__ == '__main__':
    rospy.init_node('ps_test_client')

    # Disable collisions
    robot_group = moveit_commander.MoveGroupCommander("panda_arm")
    robot_group

    pub_ = rospy.Publisher("/planning_scene_cleaned", moveit_msgs.msg.PlanningScene)

    def clean_and_publish(in_msg):
        # Clean message
        new_msg = copy.deepcopy(in_msg)
        new_msg.link_padding = []
        new_msg.link_scale = []
        # Publish to separate topic
        pub_.publish(new_msg)
        print("t")

    sub_ = rospy.Subscriber("/move_group/monitored_planning_scene", moveit_msgs.msg.PlanningScene, clean_and_publish)

    rospy.spin()
