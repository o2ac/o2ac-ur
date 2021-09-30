#!/usr/bin/env python

import rospy, sys, re
from aist_depth_filter     import DepthFilterClient
from aist_new_localization import LocalizationClient
from aist_model_spawner    import ModelSpawnerClient

if __name__ == '__main__':

    rospy.init_node('~')
    nposes    = rospy.get_param('~nposes',  2)
    timeout   = rospy.get_param('~timeout', 10)
    models    = rospy.get_param('~models',  [])

    dfilter   = DepthFilterClient('depth_filter')
    dfilter.window_radius = 2
    localizer = LocalizationClient('localization')
    spawner   = ModelSpawnerClient('/model_spawner')

    while not rospy.is_shutdown():
        print('\nmodels: {}\n'.format(models))
        try:
            key = raw_input('Model # >> ')
            if key == 'q':
                break
            num = int(key)
            model = [m for m in models if int(re.split('[_-]', m)[0]) == num][0]

            spawner.delete_all()
            dfilter.capture()                   # Load PLY to the localizer
            localizer.send_goal(model, nposes)
            (poses, overlaps) \
                = localizer.wait_for_result(rospy.Duration(timeout))

            print('{} poses found. Overlaps are {}.'
                  .format(len(poses), overlaps))

            for pose in reversed(poses):
                spawner.add(model, pose)
                rospy.sleep(1)

        except ValueError:
            print('Please specify model number.')
        except KeyboardInterrupt:
            sys.exit()
