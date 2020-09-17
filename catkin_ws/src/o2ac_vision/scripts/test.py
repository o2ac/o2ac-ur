#!/usr/bin/env python

import os, sys, rospy, re, skimage, cv2
from cv_bridge          import CvBridge
from sensor_msgs        import msg as smsg
from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

import actionlib
from o2ac_msgs import msg as omsg
#from o2ac_vision import PoseEstimationClient, BeltDetectionClient

#########################################################################
#  class PoseEstimationClient                                           #
#########################################################################
class PoseEstimationClient(object):
    def __init__(self, server=''):
        super(PoseEstimationClient, self).__init__()
        self._client \
            = actionlib.SimpleActionClient(server + '/poseEstimationTest',
                                           omsg.poseEstimationTestAction)
        self._client.wait_for_server()

    def trigger(self, object_id='', camera_id=''):
        goal = omsg.poseEstimationTestGoal(object_id=object_id,
                                           camera_id=camera_id)
        self._client.send_goal(goal)

    def get_results(self, timeout=rospy.Duration()):
        if (not self._client.wait_for_result(timeout)):
            self._client.cancel_goal()  # Cancel if timeout expired

        return self._client.get_result().pose_estimation_result_list

#########################################################################
#  class BeltDetectionClient                                            #
#########################################################################
class BeltDetectionClient(object):
    def __init__(self, server=''):
        super(BeltDetectionClient, self).__init__()
        self._client \
            = actionlib.SimpleActionClient(server + '/beltDetectionTest',
                                           omsg.beltDetectionTestAction)
        self._client.wait_for_server()

    def trigger(self, object_id='', camera_id=''):
        goal = omsg.beltDetectionTestGoal(object_id=object_id,
                                          camera_id=camera_id)
        self._client.send_goal(goal)

    def get_grasp_points(self, timeout=rospy.Duration()):
        if (not self._client.wait_for_result(timeout)):
            self._client.cancel_goal()  # Cancel if timeout expired

        return self._client.get_result().grasp_points

#########################################################################
#  class Recognizer                                                    #
#########################################################################
class Recognizer(object):
    _Models = ('00-UNKNOWN',                    # 0
               '01-BASE',                       # 1
               '02-PANEL',                      # 2
               '03-PANEL2',                     # 3
               '04_37D-GEARMOTOR-50-70',        # 4
               '05_MBRFA30-2-P6',               # 5
               '06_MBT4-400',                   # 6
               '07_SBARB6200ZZ_30',             # 7
               '08_KZAF1075NA4WA55GA20AA0',     # 8
               '09_EDCS10',                     # 9
               '10_CLBPS10_17_4'                # 10
               '11_MBRAC60-2-10',               # 11
               '12_CLBUS6-9-9.5',               # 12
               '13_MBGA30-2',                   # 13
               '14_BGPSL6-9-L30-F8',            # 14
              )
    _Colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0),
               (255, 255, 0), (255, 0, 255), (0, 255, 255))

    def __init__(self):
        super(Recognizer, self).__init__()

        self._nposes  = rospy.get_param('~nposes',  2)
        self._timeout = rospy.get_param('~timeout', 10)

        # Load camera intrinsics
        self._pose_estimator = PoseEstimationClient()
        self._belt_detector  = BeltDetectionClient()
        # self._dfilter        = DepthFilterClient('~depth_filter')
        # self._dfilter.window_radius = 2
        # self._localizer      = LocalizationClient('~localization')
        # self._spawner        = ModelSpawnerClient()

    def detect_and_localize(self):
        #self._spawner.delete_all()

        self._pose_estimator.trigger()
        results = self._pose_estimator.get_results()

        for result in results:
            print '--------------------------------'
            print "confidence: ", result.confidence
            print "rotation [deg(ccw)]: ", result.rotation
            print "center [j,i]: ", result.center
            #print "3d pose: ", results.pose_estimation_result_list[j].pose
            print "class id: ", result.class_id
            print "bbox: ", result.bbox

    def detect_belt(self):
        self._belt_detector.trigger()
        grasp_points = self._belt_detector.get_grasp_points()

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('~')

    recognizer = Recognizer()

    while not rospy.is_shutdown():
        if raw_input('Hit return key >> ') == 'q':
            break
        recognizer.detect_and_localize()

        if raw_input('Hit return key >> ') == 'q':
            break
        recognizer.detect_belt()
