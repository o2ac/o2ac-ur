#! /usr/bin/env python

import rospy
import argparse
import actionlib
import o2ac_msgs.msg

def test_client():
    client = actionlib.SimpleActionClient('poseEstimationTest', o2ac_msgs.msg.poseEstimationTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationTestGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_client_py')

        results = test_client()
        #print 'ssd result'
        #for j in range(len(results.ssd_result_list)):
        #    print '--------------------------------'
        #    print 'bbox: ', results.ssd_result_list[j].bbox
        #    print 'class label: ', results.ssd_result_list[j].label
        #    print 'confidence: ', results.ssd_result_list[j].confidence
    except rospy.ROSInterruptException:
        pass