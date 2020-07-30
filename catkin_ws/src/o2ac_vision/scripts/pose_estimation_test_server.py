#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Roatation estimator
# Shuichi Akizuki, Chukyo Univ.
# Email: s-akizuki@sist.chukyo-u.ac.jp
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

rospack = rospkg.RosPack()
anno_root = rospack.get_path("WRS_Dataset") + "/Annotations/Far/Image-wise/*.json"
# アノテーションファイルの取得
annos = glob.glob(anno_root)

# Name of template infomation
temp_info_name = "template_info.json"
# downsampling rate
ds_rate = 1.0/2.0

class PoseEstimationTest(object):
    def __init__(self):
        rospy.init_node('pose_estimation_test_server_py')

        self.pose_estimation_action_server = actionlib.SimpleActionServer("poseEstimation", o2ac_msgs.msg.poseEstimationTestAction, 
            execute_cb = self.pose_estimation_callback, auto_start = True)

        rospy.loginfo("pose_estimation_test_server has started up!")

    def pose_estimation_callback(self, goal):
        action_result = o2ac_msgs.msg.poseEstimationTestResult()

        """Get arguments"""
        temp_root = goal.tdir
        test_id = goal.id

        """Load input image"""
        cnt = 0
        for anno in annos:
            f = open( anno )
            json_data = json.load( f )
            # print(json_data)
            # bboxes = json_data["bbox"]
            # class_id = json_data["class_id"]
            im_c = cv2.imread( json_data["rgb_path"], cv2.IMREAD_GRAYSCALE )

            cnt+=1
            if test_id < cnt:
                break
        
        bboxes = []
        class_id = []
        confidence = []
        for ssd_result in goal.ssd_result_list:
            bbox = []
            for n in range(4):
                bbox.append(ssd_result.bbox[n])
            bboxes.append(bbox)
            class_id.append(ssd_result.label)
            confidence.append(ssd_result.confidence)

        has_templates = [8,9,10,14]

        """Get result of object detection """
        """ set DUMMY DATA """
        results = list()
        for bbox, cid, conf in zip(bboxes, class_id, confidence):
            result = {"bbox": bbox, "class": cid, "confidence": conf}

            results.append( result )
            # print( result )

        TM = template_matching( im_c, ds_rate, temp_root, temp_info_name )

        # Sending result to action
        for result in results:
            poseEstimationResult_msg = o2ac_msgs.msg.poseEstimationResult()
            class_id = result["class"]
            poseEstimationResult_msg.label = class_id
            if class_id in has_templates:
                start = time.time()

                # template matching
                center, ori = TM.compute( result )
                poseEstimationResult_msg.rotation = ori
                poseEstimationResult_msg.center = center
                print( "Result: center [j, i], rotation [deg(ccw)]: ", center, ori )

                elapsed_time = time.time() - start
                print( "Processing time[msec]: ", 1000*elapsed_time )

                # Generate detection result
                res_name = "pose_estimation_"+str(class_id)+".png"
                im_res = TM.get_result_image( result, ori, center )
                print( "Save", res_name )
                cv2.imwrite( res_name, im_res )
            action_result.pose_estimation_result_list.append(poseEstimationResult_msg)
        
        self.pose_estimation_action_server.set_succeeded(action_result)
    
if __name__ == '__main__':
    try:
        server = PoseEstimationTest()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
