#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import cv2
import numpy as np
import argparse
import math
import time
import json
import os
import glob
import actionlib
import o2ac_msgs.msg
from o2ac_vision.pose_estimation_func import *

# This file uses the SSD to estimate the 2D poses of objects in the tray
# when viewing it from above.


class pose_estimation:
    def __init__(self, im_in, ssd_result):
        rospack = rospkg.RosPack()
        temp_root = rospack.get_path("wrs_dataset") + "/data/templates"
        # Name of template infomation
        temp_info_name = "template_info.json"
        # downsampling rate
        ds_rate = 1.0 / 2.0

        if im_in.shape[2] == 3:
            im_in = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)

        class_id = ssd_result["class"]

        has_templates = [8, 9, 10, 14]
        self.TM = template_matching(im_in, ds_rate, temp_root, temp_info_name)

        center = 0.0
        ori = 0.0
        if class_id in has_templates:
            start = time.time()

            # template matching
            center, ori = self.TM.compute(ssd_result)

            elapsed_time = time.time() - start
            print("Processing time[msec]: ", 1000 * elapsed_time)

            self.save_result_image("result.png", ssd_result, ori, center)

        else:
            print("!!ERROR!! o2ac_pose_estimation")
            print("class_id =", class_id, "does not have the template file.")

        return center, ori

    def save_result_image(self, name, ssd_result, ori, center):

        # Generate detection result
        im_res = self.TM.get_result_image(ssd_result, ori, center)
        print("Save", name)
        cv2.imwrite(name, im_res)
