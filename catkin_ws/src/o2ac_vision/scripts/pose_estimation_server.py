#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import time
import json
import os
import actionlib
import o2ac_msgs.msg
from pose_estimation_func import *

# Name of template infomation
temp_info_name = "template_info.json"
# downsampling rate
ds_rate = 1.0/2.0 

class PoseEstimation(object):
    def __init__(self):
        rospy.init_node('pose_estimation_server_py')

        self.pose_estimation_action_server = actionlib.SimpleActionServer("poseEstimation", o2ac_msgs.msg.poseEstimationAction, 
            execute_cb = self.pose_estimation_callback, auto_start = True)

        rospy.loginfo("pose_estimation_server has started up!")

    def pose_estimation_callback(self, goal):
        action_result = o2ac_msgs.msg.poseEstimationResult()

        """Get arguments"""
        input_image = cv2.imread( goal.cimg, 0 )
        temp_root = goal.tdir
        bbox = [492, 328, 68, 87]

        """ Load template infomation """
        temp_info_fullpath = os.path.join( temp_root, temp_info_name )
        if os.path.isfile( temp_info_fullpath ):
            json_open = open( temp_info_fullpath, 'r' )
            temp_info = json.load( json_open )
        else:
            print("ERROR!!")
            print(temp_info_fullpath, " couldn't read.")
            exit()

        start = time.time()

        """Get result of object detection """
        """ DUMMY DATA """
        result = {"bbox": bbox, "class": 10, "confidence": 0.8}
        results = list()
        for i in range(10):
            results.append( result )
        print( results )

        """Compute orientation of the target"""
        image_c = cv2.GaussianBlur( input_image, (5,5), 0 )
        re = RotationEstimation( image_c, bbox )
        input_orientation = re.get_orientation()
        image_edge = re.get_im_edge()
        image_edge_ds = downsampling_binary( image_edge, _fx=ds_rate, _fy=ds_rate )

        """Compute orientation of the template"""
        info_id = -1
        for n, info in enumerate( temp_info ):
            if result["class"] in info.values():
                info_id = n

        if info_id is -1:
            print("ERROR!!")
            print("Template info does not include id ",  result["class"] )

        image_temp_edge = cv2.imread( os.path.join(temp_root, temp_info[info_id]["name_edge"]), 0 )
        temp_orientation = temp_info[info_id]["orientation"]


        # Rotation of template image
        rows, cols = image_temp_edge.shape
        res_orientations = list() # orientation in degree, ccw
        for o_in in input_orientation:
            for o_temp in temp_orientation:
                res_orientations.append( o_in - o_temp )

        print("difference of orientations")
        print( res_orientations )

        """ Create rotated templates """
        image_temp_edge_rots = list()
        image_temp_eds = list()
        for res_ori in res_orientations:
            # getRotationMatrix2D( center, angle(ccw), scale )
            rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-res_ori,1 )
            image_rot = cv2.warpAffine( image_temp_edge, rot_mat, (cols, rows) )
            image_temp_edge_rots.append( image_rot )
            image_temp_eds.append( downsampling_binary( image_rot, ds_rate, ds_rate) )

        """ Binary Template Matching """
        scores = list() # score list of each template
        offsets = list() # offset list of each template
        for temp in image_temp_eds: # Apply template matching
            scene = image_edge_ds
            btm = BinaryTemplateMatching( temp, scene, (20,20) )
            _, offset, score = btm.get_result()
            scores.append( score )
            offsets.append( offset )
        print(scores)

        # Get ID of the most similar template 
        res_idx = np.argmin( np.asarray( scores ) )

        # Up scale
        offset_original = offsets[res_idx]* 1.0/ds_rate

        # compute center coordinate
        ltop = np.asarray( [ bbox[1]+offset_original[0], bbox[0]+offset_original[1] ], np.int )
        temp_center = np.asarray( image_temp_edge.shape, np.int )/2
        center = ltop + temp_center

        elapsed_time = time.time() - start

        # DEBUG save result
        print( "Result: rotation [deg(ccw)], center [j,i]: ", res_orientations[res_idx], center )
        print( "Processing time[msec]: ", 1000*elapsed_time )
        image_res_on_original = visualize_result( image_temp_edge, input_image, ltop, res_orientations[res_idx] )
        image_e = cv2.Canny( image_c, 100, 200 )
        image_res_on_edge = visualize_result( image_temp_edge, image_e, ltop, res_orientations[res_idx] )
        cv2.imwrite( "pose_estimation.png", image_res_on_original )
        cv2.imwrite( "pose_estimation_edge.png", image_res_on_edge )

        action_result.rotation = float(res_orientations[res_idx])
        action_result.center = center.tolist()
        self.pose_estimation_action_server.set_succeeded(action_result)

if __name__ == '__main__':
    try:
        server = PoseEstimation()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
