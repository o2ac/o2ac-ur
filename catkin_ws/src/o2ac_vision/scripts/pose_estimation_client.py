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
    parser.add_argument('--cimg', type=str, default='data/rgb.png',
                        help='file name of the RGB image of the input scene.')
    parser.add_argument('--tdir', type=str, default='../data/templates',
                        help='directory name of the template image.')

    return parser.parse_args()

def pose_estimation_client():
    args = get_arguments()

    client = actionlib.SimpleActionClient('poseEstimation', o2ac_msgs.msg.poseEstimationAction)
    client.wait_for_server()

    goal = o2ac_msgs.msg.poseEstimationGoal(cimg=args.cimg, tdir=args.tdir)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

def talker(result):
    # Publisher of pose
    pub = rospy.Publisher('pose', PoseWithCovarianceStamped, queue_size=10)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
    	pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
    	pose.pose.pose.position.x = 1
    	pose.pose.pose.position.y = 1
    	pose.pose.pose.position.z = 1
    	pub.publish(pose)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('pose_estimation_client_py')

        # Call poseEstimation.action
        result = pose_estimation_client()
        print "Result: rotation [deg(ccw)], center [j,i]", result.rotation, result.center

    except rospy.ROSInterruptException:
        pass
