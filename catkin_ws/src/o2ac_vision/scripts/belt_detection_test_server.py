#!/usr/bin/env python

import rospy
import rospkg
import cv2
import numpy as np
import argparse
import math
import time
import json
import os, glob
import actionlib
import o2ac_msgs.msg
from pose_estimation_func import *

rad2deg = 180.0 / math.pi
deg2rad = math.pi/180.0

rospack = rospkg.RosPack()
annotation_root = rospack.get_path("o2ac_vision") + "/dataset/Annotations/Far/Image-wise/*.json"
annotations = glob.glob(annotation_root)

class BeltDetectionTest(object):
    def __init__(self):
        rospy.init_node('belt_detection_test_server_py')

        self.belt_detection_test_action_server = actionlib.SimpleActionServer("beltDetectionTest", o2ac_msgs.msg.beltDetectionTestAction, 
            execute_cb = self.belt_detection_test_callback, auto_start = True)

        rospy.loginfo("belt_detection_test_server has started up!")

    def belt_detection_test_callback(self, goal):
        action_result = o2ac_msgs.msg.beltDetectionTestResult()

        """Get arguments"""
        test_id = goal.id

        count = 0
        # print annotations
        for anno in annotations:
            f = open( anno )
            json_data = json.load( f )
            # print(json_data)
            bboxes = json_data["bbox"]
            class_id = json_data["class_id"]

            input_image = cv2.imread( json_data["rgb_path"], 1 )
            image_bbox = input_image.copy()
            for bbox, cid in zip(bboxes, class_id):
                image_bbox = cv2.rectangle( image_bbox, ( bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,255,0), 3 ) 

            count+=1
            if test_id < count:
                break

        # Hand template
        image_hand = np.zeros( (60,60), np.float )
        image_hand[0:16,20:40] = 1
        image_hand[44:60,20:40] = 1

        # Colision template
        image_colision = np.zeros( (60,60), np.float )
        image_colision[16:44,20:40] = 1

        param_FGE = {"ds_rate": 0.5, "target_lower":[0, 150, 100], "target_upper":[15, 255, 255],
                            "fg_lower": [0, 0, 50], "fg_upper": [179, 255, 255], "hand_rotation":[0,45,90,135], "threshold": 0.01}

        start = time.time()

        FGE = FastGraspabilityEvaluation( input_image, image_hand, image_colision, param_FGE )
        results = FGE.main_proc()

        elapsed_time = time.time() - start

        image_result = FGE.visualization()
        print( "Processing time: ", int(1000*elapsed_time), "[msec]" )
        cv2.imwrite( "im_belt_detection.png", image_result )

        #Sending result to action
        action_result.candidate_idx = results
        self.belt_detection_test_action_server.set_succeeded(action_result)

if __name__ == '__main__':
    try:
        server = BeltDetectionTest()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
