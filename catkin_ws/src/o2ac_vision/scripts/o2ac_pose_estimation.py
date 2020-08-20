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

    def pose_estimation(self, im_in, ssd_result):
        rospack = rospkg.RosPack()
        temp_root = rospack.get_path("o2ac_vision") + "/dataset/data/templates"
        # Name of template infomation
        temp_info_name = "template_info.json"
        # downsampling rate
        ds_rate = 1.0/2.0

        if im_in.shape[2] == 3:
            im_in = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)

        class_id = ssd_result["class"]

        has_templates = [8,9,10,14]
        TM = template_matching( im_in, ds_rate, temp_root, temp_info_name )

        center = 0.0
        ori = 0.0
        if class_id in has_templates:
            start = time.time()

            # template matching
            center, ori = TM.compute( ssd_result )
            print( "Result: center [j, i], rotation [deg(ccw)]: ", center, ori )

            elapsed_time = time.time() - start
            # print( "Processing time[msec]: ", 1000*elapsed_time )

            # Generate detection result
            im_res = TM.get_result_image( ssd_result, ori, center )
            # print( "Save", res_name )
            #cv2.imwrite( res_name, im_res )
        
        return center, ori
