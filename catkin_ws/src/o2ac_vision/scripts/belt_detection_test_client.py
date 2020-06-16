#! /usr/bin/env python

import rospy
import argparse
import actionlib
import o2ac_msgs.msg

def get_arguments():
    """
        Parse arguments from command line
    """

    parser = argparse.ArgumentParser( description='RGBDImage2PCD')
    parser.add_argument('--id', type=int, default=0,
                        help='dataset id of input image')

    return parser.parse_args()

def belt_detection_test_client():
    args = get_arguments()

    client = actionlib.SimpleActionClient('beltDetectionTest', o2ac_msgs.msg.beltDetectionTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.beltDetectionTestGoal(id=args.id)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('belt_detection_test_client_py')

        # Call beltDetectionTest.action
        result = belt_detection_test_client()
        print( "Candidate index: ", result.candidate_idx )

    except rospy.ROSInterruptException:
        pass
