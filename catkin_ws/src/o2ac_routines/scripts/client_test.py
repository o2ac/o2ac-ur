#!/usr/bin/env python

import rospy
import actionlib

from o2ac_msgs.msg import PlayBackSequenceAction, PlayBackSequenceGoal
from o2ac_msgs.msg import PickUpAction, PickUpGoal
from o2ac_msgs.msg import InsertAction, InsertGoal

if __name__ == '__main__':
    rospy.init_node('test_client')
    playback_client = actionlib.SimpleActionClient('playback_sequence', PlayBackSequenceAction)
    playback_client.wait_for_server()

    pickup_client = actionlib.SimpleActionClient('pickup_object', PickUpAction)
    pickup_client.wait_for_server()

    insertion_client = actionlib.SimpleActionClient('force_insertion', InsertAction)
    insertion_client.wait_for_server()

    # goal = PlayBackSequenceGoal()
    # goal.sequence_name = 'bearing_orient_down'
    # playback_client.send_goal(goal)
    # res = playback_client.wait_for_result(rospy.Duration.from_sec(20.0))
    # print("playback result:", res)

    goal = PickUpGoal()
    goal.object_name = 'bearing'
    pickup_client.send_goal(goal)
    res = pickup_client.wait_for_result()
    print("pickup result:", res)

    # goal = InsertGoal()
    # goal.object_name = 'bearing'
    # goal.task_name = 'taskboard'
    # insertion_client.send_goal(goal)
    # res = insertion_client.wait_for_result()
    # print("insert result:", res)
