#! /usr/bin/env python

import rospy
import actionlib
import o2ac_msgs.msg

class TestServer(object):
    def __init__(self):
        rospy.init_node('test_server_py')

        self.test_action_server = actionlib.SimpleActionServer("Test", o2ac_msgs.msg.TestAction, 
            execute_cb = self.test_callback, auto_start = True)

        rospy.loginfo("test_server has started up!")

    def test_callback(self, goal):
        action_result = o2ac_msgs.msg.TestResult()

        test_id = goal.id
        print 'test id: ', test_id

        # Call SSDTest.action
        ssd_result = ssd_test_client(test_id)
        print 'ssd result'
        for j in range(len(ssd_result.ssd_result_list)):
            print '--------------------------------'
            print 'bbox: ', ssd_result.ssd_result_list[j].bbox
            print 'class label: ', ssd_result.ssd_result_list[j].label
            print 'confidence: ', ssd_result.ssd_result_list[j].confidence

        # Call poseEstimationTest.action
        pose_estimation_result = pose_estimation_test_client(test_id, goal.tdir, ssd_result)
        print ''
        print 'pose estimation result'
        for j in range(len(pose_estimation_result.pose_estimation_result_list)):
            print '--------------------------------'
            print 'rotation [deg(ccw)]: ', pose_estimation_result.pose_estimation_result_list[j].rotation
            print 'class label: ', pose_estimation_result.pose_estimation_result_list[j].label
            print 'center [j,i]: ', pose_estimation_result.pose_estimation_result_list[j].center

        # Call beltDetectionTest.action
        belt_detection_result = belt_detection_test_client(test_id)
        print ''
        print 'belt detection result'
        print 'Candidate index: ', belt_detection_result.candidate_idx 

        # Sending result to action
        action_result.pose_estimation_result_list = pose_estimation_result.pose_estimation_result_list
        action_result.belt_detection_result = belt_detection_result.candidate_idx

        self.test_action_server.set_succeeded(action_result)

def ssd_test_client(test_id):
    client = actionlib.SimpleActionClient('SSD', o2ac_msgs.msg.SSDTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.SSDTestGoal(id=test_id)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

def pose_estimation_test_client(test_id, tdir, ssd_result):
    client = actionlib.SimpleActionClient('poseEstimation', o2ac_msgs.msg.poseEstimationTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationTestGoal(id=test_id, tdir=tdir, ssd_result_list=ssd_result.ssd_result_list)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

def belt_detection_test_client(test_id):
    client = actionlib.SimpleActionClient('beltDetectionTest', o2ac_msgs.msg.beltDetectionTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.beltDetectionTestGoal(id=test_id)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        server = TestServer()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass