#!/usr/bin/env python

import rospy, argparse
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

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
    parser.add_argument('-s',
                        '--scene',
                        action='store',
                        nargs='?',
                        default='scene.ply',
                        type=str,
                        choices=None,
                        help='path to scene PLY file',
                        metavar=None)
    parser.add_argument('-f',
                        '--frame',
                        action='store',
                        nargs='?',
                        default='world',
                        type=str,
                        choices=None,
                        help='camera frame',
                        metavar=None)
    parser.add_argument('-n',
                        '--nposes',
                        action='store',
                        nargs='?',
                        default=2,
                        type=int,
                        choices=None,
                        help='the number of candidate poses',
                        metavar=None)
    parser.add_argument('-t',
                        '--timeout',
                        action='store',
                        nargs='?',
                        default=10,
                        type=float,
                        choices=None,
                        help='timeout value',
                        metavar=None)
    args = parser.parse_args()

    rospy.init_node('localization_client')

    localize = LocalizationClient()
    localize.load_scene(args.scene, args.frame)
    models  = rospy.get_param('aist_localization', [])
    spawner = ModelSpawnerClient()


    for model in models:
        localize.send_goal(model, args.nposes)
        (poses, overlaps, success) \
            = localize.wait_for_result(rospy.Duration(args.timeout))

        if poses:
            print('{}\noverlap: {}'.format(poses[0], overlaps[0]))
            spawner.add(model, poses[0])

        # for pose, overlap in zip(poses, overlaps):
        #     print('{}\noverlap: {}'.format(pose, overlap))
        #     spawner.add(model, pose)

    print(spawner.get_list())
