# -*- coding: utf-8 -*-
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

class pose_estimation():

    def pose_estimation(self, im_in, ssd_results):
        rospack = rospkg.RosPack()
        temp_root = rospack.get_path("o2ac_vision") + "/dataset/data/templates"
        # Name of template infomation
        temp_info_name = "template_info.json"
        # downsampling rate
        ds_rate = 1.0/2.0

        class_id = []
        for result in ssd_results:
            class_id.append(result["class"])
        print(class_id)

        has_templates = [8,9,10,14]
        TM = template_matching( im_in, ds_rate, temp_root, temp_info_name )

        # Sending result to action
        center_list = []
        orientation_list = []
        results = list()
        for result in ssd_results:
            if class_id in has_templates:
                start = time.time()

                # template matching
                center, ori = TM.compute( result )
                center_list.append(center)
                orientation_list.append(ori)
                # print( "Result: center [j, i], rotation [deg(ccw)]: ", center, ori )

                elapsed_time = time.time() - start
                # print( "Processing time[msec]: ", 1000*elapsed_time )

                # Generate detection result
                res_name = "pose_estimation_"+str(class_id)+".png"
                im_res = TM.get_result_image( result, ori, center )
                # print( "Save", res_name )
                cv2.imwrite( res_name, im_res )
        
        results.append(center_list)
        results.append(orientation_list)
        return results