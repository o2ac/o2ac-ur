#!/usr/bin/env python

import re
import rospy, argparse
from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Localize through depth filter')
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default="a_bot_camera",
                        type=str,
                        choices=None,
                        help='camera name',
                        metavar=None)
    parser.add_argument('-n',
                        '--number_of_poses',
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

    dfilter   = DepthFilterClient(args.camera_name + "/depth_filter")
    dfilter.set_window_radius(2)
    localizer = LocalizationClient(args.camera_name + "/localization")
    spawner   = ModelSpawnerClient()
    models    = rospy.get_param("aist_localization", [])

    while not rospy.is_shutdown():
        print("\nmodels: {}\n".format(models))
        try:
            key = raw_input("Model # >> ")
            if key == "q":
                break
            num = int(key)

            spawner.delete_all()

            for model in models:
                if int(re.split("[_-]", model)[0]) == num:
                    dfilter.savePly()
                    localizer.send_goal(model, args.number_of_poses)
                    (poses, overlaps, success) \
                        = localizer.wait_for_result(rospy.Duration(args.timeout))
                    print("{} poses found. Overlaps are {}."
                          .format(len(poses), overlaps))

                    for pose in reversed(poses):
                        spawner.add(model, pose)
                        rospy.sleep(3)
                    break
        except ValueError:
            print("Please specify model number.")
