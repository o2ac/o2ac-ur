#! /usr/bin/env python

import rospy
import rospkg
import argparse
import actionlib
import o2ac_msgs.msg

def get_arguments():
    """
        Parse arguments from command line
    """

    rospack = rospkg.RosPack()
    parser = argparse.ArgumentParser( description='RGBDImage2PCD')
    parser.add_argument('--id', type=int, default=0,
                        help='dataset id of input image')
    parser.add_argument('--tdir', type=str, default=rospack.get_path("WRS_Dataset")+'/data/templates',
                        help='directory name of the template image.')

    return parser.parse_args()

def test_client():
    args = get_arguments()

    client = actionlib.SimpleActionClient('Test', o2ac_msgs.msg.TestAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.TestGoal(id=args.id, tdir=args.tdir)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_client_py')

        # Call Test.action
        result = test_client()
        print 'pose estimation result'
        for j in range(len(result.pose_estimation_result_list)):
            print '--------------------------------'
            print "rotation [deg(ccw)]: ", result.pose_estimation_result_list[j].rotation
            print 'class label: ', result.pose_estimation_result_list[j].label
            print "center [j,i]: ", result.pose_estimation_result_list[j].center

        print ''
        print 'belt detection result'
        print 'Candidate index: ', result.belt_detection_result

    except rospy.ROSInterruptException:
        pass
