#!/usr/bin/env python

import rospy, actionlib
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
               '10_CLBPS10_17_4'                # 10
               '11_MBRAC60-2-10',               # 11
               '12_CLBUS6-9-9.5',               # 12
               '13_MBGA30-2',                   # 13
               '14_BGPSL6-9-L30-F8',            # 14
              )

    def __init__(self):
        super(ObjectRecognizer, self).__init__()

        self._nposes  = rospy.get_param('~nposes',  2)
        self._timeout = rospy.get_param('~timeout', 10)

        # Setup subscriber for object detection results
        self._detection_sub = rospy.Subscriber('/detection_results',
                                               omsg.ObjectDetectionResults,
                                               self.detection_callback)

        # Setup action server for recognition
        self._recognition_server = \
            actionlib.SimpleActionServer("~recognize_object",
                                         omsg.recognizeObjectAction,
                                         auto_start = False)
        self._recognition_server \
            .register_goal_callback(self.recognition_goal_callback)
        self._recognition_server \
            .register_preempt_callback(self.recognition_preempt_callback)
        self._recognition_server.start()

        self._dfilter = DepthFilterClient('depth_filter')
        self._dfilter.window_radius = 2
        self._localizer = LocalizationClient('localization')
        self._spawner = ModelSpawnerClient()

        self._class_id = 0

    def detection_callback(self, detection_results):
        if not self._recognition_server.is_active():
            return

        self._spawner.delete_all()

        recognition_result = omsg.recognizeObjectResult()
        recognition_result.succeeded = False

        for result in detection_results.pose_estimation_results:
            if self._class_id == result.class_id:
                poses, overlaps = self.localize(result.bbox)
                if len(poses) > 0:
                    recognition_result.succeeded = True
                    recognition_result.detected_pose = poses[0]
                    recognition_result.confidence    = overlaps[0]
                    self._spawner.add(self.model, poses[0])
                    self._recognition_server.set_succeeded(recognition_result)
                    return
                break
        self._recognition_server.set_aborted(recognition_result)

    def localize(self, bbox):
        self._dfilter.roi = (bbox[0],           bbox[1],
                             bbox[0] + bbox[2], bbox[1] + bbox[3])
        self._dfilter.capture()  # Load PLY data to the localizer
        self._localizer.send_goal(self.model, self._nposes)
        return self._localizer.wait_for_result(rospy.Duration(self._timeout))

    def recognition_goal_callback(self):
        self._class_id = self._recognition_server.accept_new_goal().class_id
        rospy.loginfo('Received a request to recognize object[' + self.model + ']')

    def recognition_preempt_callback(self):
        rospy.loginfo("o2ac_msgs.msg.recognizeObjectAction preempted")
        self._recognition_server.set_preempted()

    @property
    def model(self):
        return ObjectRecognizer._Models[self._class_id]

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    recognizer = ObjectRecognizer()
    rospy.spin()
