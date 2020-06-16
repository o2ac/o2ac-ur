#!/usr/bin/env python

import rospy
import rospkg
import cv2
import numpy as np
import argparse
import math
import time
import json
import os, sys
import actionlib
import o2ac_msgs.msg
from pose_estimation_func import *
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("M2Det"))
sys.path.append(rospack.get_path("M2Det") + '/utils')
from get_wrs_dataset import load_dataset, Trans_Dataset
from utils.core import *

#import matplotlib.pyplot as plt
#get_ipython().magic(u'matplotlib inline')
rad2deg = 180.0 / math.pi
deg2rad = math.pi/180.0

out_root = "../../out/"

global cfg
args_config = '../M2Det/configs/m2det320_vgg.py'
cfg = Config.fromfile( args_config )

#dataset = load_dataset( args.annotations_path, args.images_path, cfg, args.cross_val, args.K_fold, args.K_num)
args_annotations_path = [
                                [
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close0-249.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close250-499.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close500-749.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close750-999.json'
                                ],
                                [
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far0-249.json', 
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far250-499+900-949.json', 
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far500-749+950-999.json', 
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far750-799.json', 
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far800-849.json', 
                                '../../WRS_Dataset/Annotations/Far/Object-Detection/Far850-899.json', 
                                ]
                                
                            ]
args_annotations_path = [
                                [
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close0-249.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close250-499.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close500-749.json', 
                                '../../WRS_Dataset/Annotations/Close/Object-Detection/Close750-999.json'
                                ]
                                
                            ]
#args_images_path = ['../../WRS_Dataset/Images/Close/RGB/', '../../WRS_Dataset/Images/Far/RGB/']
args_images_path = ['../../WRS_Dataset/Images/Close/RGB/']
dataset = load_dataset( args_annotations_path, args_images_path, cfg, True, 5, 1)
dataset = Trans_Dataset(dataset, train=False)

args_tdir = "../../data/templates"
temp_root = args_tdir

# Name of template infomation
temp_info_name = "template_info.json"
# downsampling rate
ds_rate = 1.0/2.0 

class PoseEstimationTest(object):
    def __init__(self):
        rospy.init_node('pose_estimation_test_server_py')

        self.pose_estimation_test_action_server = actionlib.SimpleActionServer("poseEstimationTest", o2ac_msgs.msg.poseEstimationTestAction, 
            execute_cb = self.pose_estimation_test_callback, auto_start = True)

        rospy.loginfo("pose_estimation_test_server has started up!")


    def pose_estimation_test_callback(self, goal):
        action_result = o2ac_msgs.msg.poseEstimationTestResult()

        """ Load template infomation """
        temp_info_fullpath = os.path.join( temp_root, temp_info_name )
        if os.path.isfile( temp_info_fullpath ):
            json_open = open( temp_info_fullpath, 'r' )
            temp_info = json.load( json_open )
        else:
            print("ERROR!!")
            print(temp_info_fullpath, " couldn't read.")
            exit()

        # a list of index that has template image which can be used to BTM
        has_templates = [10,11,13,16]
        has_templates = [10,11,13,16]
        json_path = "../../WRS_Dataset/Annotations/Close/Pose"
        orientation_error = list()
        position_error = list()
        for count, data in enumerate(dataset):

            image, target, img_path = data
            print(img_path)
    
            # reshape image
            image = image.numpy()
            img = image.transpose( 1, 2, 0 )
            img = np.asarray( img*255, np.uint8 )
            img_resize = cv2.resize( img, dsize=(640,480) )

            # reshape annotations
            bboxes = np.asarray(target[:,0:4] ) * np.array([640.,480.,640.,480.])
            bboxes = np.asarray( bboxes, np.int )
            label = np.asarray(target[:,4], np.int)

            # create data "results" that is output of M2Det
            results = list()
            for i in range( len(bboxes) ):
                bbox = [bboxes[i][0], bboxes[i][1], bboxes[i][2]-bboxes[i][0], bboxes[i][3]-bboxes[i][1] ]
                result = {"bbox": bbox, "class": label[i], "confidence": 1.0}
                results.append( result )
    
            image_c_org = cv2.cvtColor( img_resize, cv2.COLOR_RGB2GRAY )
            image_c = cv2.GaussianBlur( image_c_org, (5,5), 0 )
            rotation_list = []
            objectCenter_msg_list = []
            for result in results:
                class_id = result["class"]+1
                if class_id in has_templates:
                    start = time.time()
                    print( class_id, " has template.")

                    """Compute orientation of the target"""
                    bbox = result["bbox"]
                    re = RotationEstimation( image_c, bbox )
                    in_ori = re.get_orientation()
                    image_edge = re.get_im_edge()
                    image_edge_ds = downsampling_binary( image_edge, _fx=ds_rate, _fy=ds_rate )
                    print( bbox )

                    """Compute orientation of the template"""
                    info_id = -1
                    for n, info in enumerate( temp_info ):
                        if class_id in info.values():
                            info_id = n

                    if info_id is -1:
                        print("ERROR!!")
                        print("Template info does not include id ",  class_id )

                    print(os.path.join(temp_root, temp_info[info_id]["name_edge"]))
                    image_temp_edge = cv2.imread( os.path.join(temp_root, temp_info[info_id]["name_edge"]), 0 )
                    temp_ori = temp_info[info_id]["orientation"]


                    # Rotation of template image
                    rows, cols = image_temp_edge.shape
                    res_orientations = list() # orientation in degree, ccw
                    for o_in in in_ori:
                        for o_temp in temp_ori:
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
                        btm = BinaryTemplateMatching( temp, image_edge_ds, (20,20) )
                        _, offset, score = btm.get_result()
                        scores.append( score )
                        offsets.append( offset )

                    # Get ID of the most similar template
                    res_idx = np.argmin( np.asarray( scores ) )

                    # Up scale
                    offset_original = offsets[res_idx]* 1.0/ds_rate

                    # compute center coordinate
                    ltop = np.asarray( [ bbox[1]+offset_original[0], bbox[0]+offset_original[1] ], np.int )
                    temp_center = np.asarray( image_temp_edge.shape, np.int )/2
                    center = ltop + temp_center

                    elapsed_time = time.time() - start

                    # DEBUG
                    print( "Result: rotation [deg(ccw)], center [j,i]: ", res_orientations[res_idx], center )
                    print( "Processing time[msec]: ", 1000*elapsed_time )            

                    # Load ground truth
                    json_fullname = os.path.join( json_path, img_path.rsplit("/")[-1]+".json")
                    json_data = {}
                    # if json annotation is exist, load it
                    if os.path.exists(json_fullname):
                        f = open( json_fullname )
                        json_data = json.load( f )
                        #print(json_fullname, " is exist")
                
                    else:
                        print("no json annotation ", json_fullname )

                    if str(class_id) in json_data:
                        gt_ori = json_data[str(class_id)]
                        # calc orientation error
                        diff_ori = abs(gt_ori - res_orientations[res_idx] )
                        orientation_error.append( diff_ori ) #difference
            

                    # calc position error
                    gt_pos = np.asarray( [bbox[1]+(bbox[3]/2), bbox[0]+(bbox[2]/2)] )
                    diff_pos = gt_pos - center
                    distance = np.linalg.norm(diff_pos)
                    position_error.append( distance )

            
                    # Save result
                    image_res_on_original = visualize_result( image_temp_edge, image_c_org, ltop, res_orientations[res_idx] )
                    image_c_org = image_res_on_original
                    name = os.path.join(out_root,"pose_estimation" + str(count)+"_"+str(result["class"]+1)+ ".png")
                    if 7.0 < distance:
                        name = os.path.join(out_root,"Failure","pose_estimation" + str(count)+"_"+str(result["class"]+1)+ ".png")
                
                    cv2.imwrite( name, image_res_on_original )


                    image_bb = cv2.rectangle( image_c.copy(), tuple(bbox[0:2]), (bbox[0]+bbox[2],bbox[1]+bbox[3]), (255), 2 )
                    name = "image_bb" + str(result["class"]+1)+ ".png"
                    cv2.imwrite( name, image_bb )            
                    image_btm = btm.get_result_image( offsets[res_idx] )


                    # Sending result to action
                    rotation_list.append(float(res_orientations[res_idx]))
                    objectCenter_msg = o2ac_msgs.msg.objectCenters()
                    objectCenter_msg.center = center.tolist()
                    objectCenter_msg_list.append(objectCenter_msg)

                else:
                    print( result["class"], " no template.")

            # Sending result to action
            poseEstimationResult_msg = o2ac_msgs.msg.poseEstimationResult()
            poseEstimationResult_msg.rotation = rotation_list
            poseEstimationResult_msg.center = objectCenter_msg_list
            action_result.pose_estimation_result_list.append(poseEstimationResult_msg)
            #print poseEstimationResult_msg.rotation
            #print poseEstimationResult_msg.center

        self.pose_estimation_test_action_server.set_succeeded(action_result)

if __name__ == '__main__':
    try:
        server = PoseEstimationTest()
        while not rospy.is_shutdown():
            rospy.sleep(.1)

    except rospy.ROSInterruptException:
        pass
