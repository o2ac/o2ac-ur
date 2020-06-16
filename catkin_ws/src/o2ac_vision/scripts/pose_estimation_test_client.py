#! /usr/bin/env python

import rospy
import actionlib
import o2ac_msgs.msg

def pose_estimation_test_client():
    client = actionlib.SimpleActionClient('poseEstimationTest', o2ac_msgs.msg.poseEstimationTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationTestGoal()
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('pose_estimation_test_client_py')

        # Call poseEstimationTest.action
        result = pose_estimation_test_client()
        for j in range(len(result.pose_estimation_result_list)):
            print "Result: rotation [deg(ccw)], center [j,i]", result.pose_estimation_result_list[j].rotation, result.pose_estimation_result_list[j].center

    except rospy.ROSInterruptException:
        pass
