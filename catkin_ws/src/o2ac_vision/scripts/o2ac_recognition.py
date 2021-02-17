#!/usr/bin/env python

import rospy, actionlib
from math               import radians
from o2ac_msgs          import msg as omsg
from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient

#########################################################################
#  class ObjectRecognition                                               #
#########################################################################
class ObjectRecognition(object):
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
        super(ObjectRecognition, self).__init__()

        self._nposes  = rospy.get_param('~nposes',  2)
        self._timeout = rospy.get_param('~timeout', 10)

        # Setup action server for recognition
        self._recognition_server = \
            actionlib.SimpleActionServer("~recognize_object",
                                         omsg.detectObjectAction,
                                         execute_cb=self.recognition_callback,
                                         auto_start = False)
        self._recognition_server \
            .register_preempt_callback(self.recognition_preempt_callback)
        self._recognition_server.start()
        rospy.loginfo("Started recognition server")

        # Setup action client for object detection and 2D pose estimation
        self._object_detector \
            = actionlib.SimpleActionClient('poseEstimation',
                                           omsg.poseEstimationAction)
        self._object_detector.wait_for_server()

        # Setup clients for depth filtering and localization
        self._dfilter = DepthFilterClient('depth_filter')
        self._dfilter.window_radius = 2
        self._localizer = LocalizationClient('localization')
        rospy.loginfo("Recognition node started up")

    def recognition_callback(self, goal):
        self._object_detector.send_goal(omsg.poseEstimationGoal())
        self._object_detector.wait_for_result()
        detection_results = self._object_detector.get_result()

        recognition_result = omsg.detectObjectResult()
        recognition_result.succeeded = False

        for result in detection_results.results:
            if goal.item_id == self.item_id(result.class_id):
                recognition_result.detected_poses, \
                recognition_result.confidences     \
                    = self.localize(goal.item_id, result.bbox, result.poses)

                if recognition_result.detected_poses:
                    recognition_result.succeeded = True
                    self._recognition_server.set_succeeded(recognition_result)
                    return
                break
        self._recognition_server.set_aborted(recognition_result)

    def recognition_preempt_callback(self):
        rospy.loginfo("o2ac_msgs.msg.detectObjectAction preempted")
        self._recognition_server.set_preempted()

    def item_id(self, class_id):
        return ObjectRecognition._Models[class_id]

    def localize(self, item_id, bbox, poses2d):
        self._dfilter.roi = (bbox[0],           bbox[1],
                             bbox[0] + bbox[2], bbox[1] + bbox[3])
        while not self._dfilter.capture():  # Load PLY data to the localizer
            pass
        self._localizer.send_goal(item_id, self._nposes, poses2d)
        return self._localizer.wait_for_result(rospy.Duration(self._timeout))

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('o2ac_recognition')
    recognizer = ObjectRecognition()
    rospy.spin()
