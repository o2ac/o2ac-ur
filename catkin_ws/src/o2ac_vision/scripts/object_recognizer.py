#!/usr/bin/env python

import rospy, actionlib
from math               import radians
from o2ac_msgs          import msg as omsg
from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

#########################################################################
#  class ObjectRecognizer                                               #
#########################################################################
class ObjectRecognizer(object):
    _Models = ('00-ALL',                        # 0
               '01-BASE',                       # 1
               '02-PANEL',                      # 2
               '03-PANEL2',                     # 3
               '04_37D-GEARMOTOR-50-70',        # 4
               '05_MBRFA30-2-P6',               # 5
               '06_MBT4-400',                   # 6
               '07_SBARB6200ZZ_30',             # 7
               '08_KZAF1075NA4WA55GA20AA0',     # 8
               '09_EDCS10',                     # 9
               '10_CLBPS10_17_4',               # 10
               '11_MBRAC60-2-10',               # 11
               '12_CLBUS6-9-9.5',               # 12
               '13_MBGA30-2',                   # 13
               '14_BGPSL6-9-L30-F8',            # 14
              )

    def __init__(self):
        super(ObjectRecognizer, self).__init__()

        self._nposes  = rospy.get_param('~nposes',  2)
        self._timeout = rospy.get_param('~timeout', 10)

        # Setup action server for recognition
        self._recognition_server = \
            actionlib.SimpleActionServer("~recognize_object",
                                         omsg.recognizeObjectAction,
                                         execute_cb=self.recognition_callback,
                                         auto_start = False)
        self._recognition_server \
            .register_preempt_callback(self.recognition_preempt_callback)
        self._recognition_server.start()

        # Setup action client for pose estimation
        self._object_detector \
            = actionlib.SimpleActionClient('poseEstimationTest',
                                           omsg.poseEstimationTestAction)
        self._object_detector.wait_for_server()

        # Setup action client for pose estimation
        self._belt_detector \
            = actionlib.SimpleActionClient('beltDetectionTest',
                                           omsg.beltDetectionTestAction)
        self._belt_detector.wait_for_server()

        self._dfilter = DepthFilterClient('depth_filter')
        self._dfilter.window_radius = 2
        self._localizer = LocalizationClient('localization')
        self._spawner = ModelSpawnerClient()

    def recognition_callback(self, goal):
        self._object_detector.send_goal(omsg.poseEstimationTestGoal())
        self._object_detector.wait_for_result()
        detection_results = self._object_detector.get_result()

        self._spawner.delete_all()

        recognition_result = omsg.recognizeObjectResult()
        recognition_result.succeeded = False

        for result in detection_results.pose_estimation_result_list:
            if goal.class_id == result.class_id:
                pose2d = [result.center[1] - result.bbox[0],
                          result.center[0] - result.bbox[1],
                          radians(result.rotation)] if result.center else []
                poses, overlaps = self.localize(goal.class_id,
                                                result.bbox, pose2d)
                if len(poses) > 0:
                    recognition_result.succeeded = True
                    recognition_result.detected_pose = poses[0]
                    recognition_result.confidence    = overlaps[0]
                    self._spawner.add(self.model(goal.class_id), poses[0])
                    self._recognition_server.set_succeeded(recognition_result)
                    return
                break
        self._recognition_server.set_aborted(recognition_result)

    def recognition_preempt_callback(self):
        rospy.loginfo("o2ac_msgs.msg.recognizeObjectAction preempted")
        self._recognition_server.set_preempted()

    def model(self, class_id):
        return ObjectRecognizer._Models[class_id]

    def localize(self, class_id, bbox, pose2d):
        self._dfilter.roi = (bbox[0],           bbox[1],
                             bbox[0] + bbox[2], bbox[1] + bbox[3])
        while not self._dfilter.capture():  # Load PLY data to the localizer
            pass
        self._localizer.send_goal(self.model(class_id), self._nposes, pose2d)
        return self._localizer.wait_for_result(rospy.Duration(self._timeout))

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognizer = ObjectRecognizer()
    rospy.spin()
