#!/usr/bin/env python

import rospy
import actionlib

from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceGoal

if __name__ == '__main__':
    rospy.init_node('test_client')
    client = actionlib.SimpleActionClient('playback_sequence', PlayBackSequenceAction)
    client.wait_for_server()

    goal = PlayBackSequenceGoal()
    goal.sequence_name = 'bearing_orient_down'
    # Fill in the goal here
    client.send_goal(goal)
    res = client.wait_for_result(rospy.Duration.from_sec(20.0))
    print("this is the result:", res)
