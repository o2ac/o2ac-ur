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

def ssd_test_once_client():
    args = get_arguments()

    client = actionlib.SimpleActionClient('SSD', o2ac_msgs.msg.SSDTestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.SSDTestGoal(id=args.id)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('ssd_test_client_py')

        # Call SSDTest.action
        result = ssd_test_once_client()
        for j in range(len(result.SSD_result.SSD_result)):
            print '================================'
            print 'bbox: ', result.SSD_result.SSD_result[j].bbox
            print 'class label: ', result.SSD_result.SSD_result[j].label
            print 'confidence: ', result.SSD_result.SSD_result[j].confidence

    except rospy.ROSInterruptException:
        pass
