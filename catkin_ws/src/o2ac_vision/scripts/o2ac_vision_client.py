#! /usr/bin/env python

import rospy
import argparse
import actionlib
import o2ac_msgs.msg

def pose_estimation_client():
    client = actionlib.SimpleActionClient('poseEstimationTest', o2ac_msgs.msg.poseEstimationTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationTestGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

def belt_detection_client():
    client = actionlib.SimpleActionClient('beltDetectionTest', o2ac_msgs.msg.beltDetectionTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationTestGoal(object_id="1", camera_id="1")
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_client_py')

        results = pose_estimation_client()
        print 'pose estimation result'
        for j in range(len(results.pose_estimation_result_list)):
            print '--------------------------------'
            print "confidence: ", results.pose_estimation_result_list[j].confidence
            print "rotation [deg(ccw)]: ", results.pose_estimation_result_list[j].rotation
            print "center [j,i]: ", results.pose_estimation_result_list[j].center
            #print "3d pose: ", results.pose_estimation_result_list[j].pose
            print "class id: ", results.pose_estimation_result_list[j].class_id
            #print "grasp points: ", results.pose_estimation_result_list[j].grasp_points

        results = belt_detection_client()
        print '\nbelt detection result'
        print "grasp points: ", results.grasp_points

    except rospy.ROSInterruptException:
        pass