import rospy
import dynamic_reconfigure.client
import actionlib
import numpy as np
from geometry_msgs         import msg as gmsg
from aist_new_localization import msg as lmsg
from aist_depth_filter     import msg as dmsg
from operator              import itemgetter
from actionlib_msgs.msg    import GoalStatus

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):
    _DefaultParams = {'axes':             [1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0],
                      'offset':           [0.0, 0.0, 0.0],
                      'rotation_range':   [0.0, 0.0, 0,0],
                      'refine_transform': False,
                      'transform_2d':     False}

    def __init__(self, server='/localization'):
        super(LocalizationClient, self).__init__()

        self._params     = rospy.get_param('~localization_parameters',  {})
        self._dyn_reconf = dynamic_reconfigure.client.Client(server,
                                                             timeout=5.0)
        self._localize   = actionlib.SimpleActionClient(server + '/localize',
                                                        lmsg.LocalizeAction)

        for part, params in self._params.items():
            for key, value in LocalizationClient._DefaultParams.items():
                if key not in params:
                    self._params[part][key] = value
        print(self._params)
        print('start localizer...')
        self._localize.wait_for_server()
        print('OK')

    def set_setting(self, name, value):
        self.set_settings({name : value})
        return self.get_setting(name)

    def get_setting(self, name):
        return self.get_settings()[name]

    def set_settings(self, settings):
        self._dyn_reconf.update_configuration(settings)

    def get_settings(self):
        return self._dyn_reconf.get_configuration()

    def send_goal(self, object_name, plane,
                  poses2d=[gmsg.Pose2D(0.0, 0.0, 0.0)], dx=0.0, dy=0.0):
        params = self._params[object_name]
        offset = params['offset']
        offset[0] += dx
        offset[1] += dy

        goal = lmsg.LocalizeGoal()
        rotation_range = params['rotation_range']
        if rotation_range[0] < rotation_range[1] and \
           rotation_range[2] > 0:
            goal.poses2d = [ gmsg.Pose2D(0.0, 0.0, 180.0/np.pi*theta)
                             for theta in np.arange(rotation_range[0],
                                                    rotation_range[1],
                                                    rotation_range[2]) ]
        else:
            goal.poses2d = poses2d
        goal.object_name      = object_name
        goal.plane            = plane
        goal.axes             = params['axes']
        goal.offset           = offset
        goal.refine_transform = params['refine_transform']
        goal.transform_2d     = params['transform_2d']
        self._localize.send_goal(goal)

    def send_goal_with_target_frame(self, object_name, frame_id, stamp,
                                    poses2d=[gmsg.Pose2D(0.0, 0.0, 0.0)],
                                    dx=0.0, dy=0.0):
        plane = dmsg.PlaneStamped()
        plane.header.frame_id = frame_id
        plane.header.stamp    = stamp
        plane.plane.normal.x  = 0
        plane.plane.normal.y  = 0
        plane.plane.normal.z  = 1
        plane.plane.distance  = 0
        return self.send_goal(object_name, plane, poses2d, dx, dy)

    def wait_for_result(self, timeout=0):
        if not self._localize.wait_for_result(rospy.Duration(timeout)):
            self._localize.cancel_goal()  # Cancel goal if timeout expired
            return None
        elif self._localize.get_state() != GoalStatus.SUCCEEDED:
            return None
        return self._localize.get_result().poses
