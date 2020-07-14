#!/usr/bin/env python

import rospy, argparse
from aist_model_spawner import ModelSpawnerClient
from geometry_msgs      import msg as gmsg

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test localizeAction')
    parser.add_argument('-m',
                        '--model',
                        action='store',
                        nargs='?',
                        default='04_37D-GEARMOTOR-50-70',
                        type=str,
                        choices=None,
                        help='name of model to be matched',
                        metavar=None)
    args = parser.parse_args()

    rospy.init_node('model_spawner_client')

    spawner = ModelSpawnerClient()
    pose = gmsg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose = gmsg.Pose(gmsg.Point(0, 0, 0), gmsg.Quaternion(0, 0, 0, 1))

    spawner.add(args.model, pose)

    print(spawner.get_list())
