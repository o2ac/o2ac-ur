#! /usr/bin/env python

import rospy
import argparse
from o2ac_vision import PoseEstimationClient
from o2ac_vision import BeltDetectionClient

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

    rospy.init_node('test_client_py')

    pose_estimator = PoseEstimationClient()
    belt_detector  = BeltDetectionClient()

    while not rospy.is_shutdown():
        if raw_input('Hit return key >> ') == 'q':
            break

        pose_estimator,trigger()
        results = pose_estimator.get_results()

        print 'pose estimation result'
        for result in results:
            print '--------------------------------'
            print "confidence: ", result.confidence
            print "rotation [deg(ccw)]: ", result.rotation
            print "center [j,i]: ", result.center
            #print "3d pose: ", results.pose_estimation_result_list[j].pose
            print "class id: ", result.class_id
            print "bbox: ", result.bbox
            #print "grasp points: ", results.pose_estimation_result_list[j].grasp_points

        belt_detector.trigger()
        grasp_points = belt_detector.get_grasp_points()
        print '\nbelt detection result'
        print "grasp points: ", grasp_points
