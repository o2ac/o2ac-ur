#!/usr/bin/env python
#-*- encoding:utf-8 -*-

import rospy
import rospkg
import actionlib
import o2ac_msgs.msg
import sys
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("ssd_pytorch"))

import os
import glob, json, time
import argparse
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
import numpy as np
import cv2
if torch.cuda.is_available():
    torch.set_default_tensor_type('torch.cuda.FloatTensor')

from ssd import build_ssd
from data import WRS2020_Detection, WRS_ROOT, AnnotationTransform
from data import WRS2020_CLASSES as labels

anno_root = "../../WRS_Dataset/Annotations/Far/Image-wise/*.json"
fname_weight = "../ssd.pytorch/WRS_lr1e3_bs16.pth"
# アノテーションファイルの取得
annos = glob.glob(anno_root)

class SSDTestServer(object):
    def __init__(self):
        
        self.net = build_ssd('test', 300, 17)    # initialize SSD
        self.net.load_weights( fname_weight )

        rospy.init_node('ssd_test_once_server_py')

        self.ssd_test_once_action_server = actionlib.SimpleActionServer("SSDTest", o2ac_msgs.msg.SSDTestAction, 
            execute_cb = self.ssd_test_once_callback, auto_start = True)

        rospy.loginfo("ssd_test_once_server has started up!")


    def object_detection( self, input_image, threshold = 0.6 ):
        """
            Object detection by SSD
            Input: 
                input_image... input_image
                threshold... threshold of detection
            Return:
                results... a list of dict(bbox, class_id, score)
        """

        # Preproc
        x = cv2.resize( input_image, (300, 300)).astype(np.float32 )
        x -= (104.0, 117.0, 123.0)
        x = x.astype(np.float32)
        x = x[:, :, ::-1].copy()
        x = torch.from_numpy(x).permute(2, 0, 1)

        #SSD forward
        xx = Variable(x.unsqueeze(0))     # wrap tensor in Variable
        if torch.cuda.is_available():
            xx = xx.cuda()
        y = self.net(xx)


        detections = y.data
        # scale each detection back up to the image
        scale = torch.Tensor(input_image.shape[1::-1]).repeat(2)
        results = list()
        for i in range(detections.size(1)):
            j = 0
            while detections[0,i,j,0] >= threshold:
                score = detections[0,i,j,0]
                label_name = labels[i-1]
                pt = (detections[0,i,j,1:]*scale).cpu().numpy()
                coords = (pt[0], pt[1]), pt[2]-pt[0]+1, pt[3]-pt[1]+1
                j+=1

                bbox = [ int(coords[0][0]), int(coords[0][1]), int(coords[1]), int(coords[2])]
                result = {"bbox": bbox, "class": i-1, "confidence": score}
                results.append( result )
        
        return results


    def ssd_test_once_callback(self, goal):
        action_result = o2ac_msgs.msg.SSDTestResult()

        """Get arguments"""
        test_id = goal.id

        # Load image
        cnt = 0
        for anno in annos:
            f = open( anno )
            json_data = json.load( f )
            #print(json_data)
            bboxes = json_data["bbox"]
            class_id = json_data["class_id"]
            input_image = cv2.imread( json_data["rgb_path"], cv2.IMREAD_COLOR )

            cnt+=1
            if test_id < cnt:
                break

        # prepare network model
        # ssd = SSD_detection( args.weight )

        start = time.time()

        # detection
        results = self.object_detection( input_image )

        elapsed_time = time.time() - start

        # visualization
        image_vis = input_image.copy()
        for res in results:
            bbox = res["bbox"]
            image_vis = cv2.rectangle( image_vis, (bbox[0],  bbox[1]), 
                                    (bbox[0]+bbox[2], bbox[1]+bbox[3]), 
                                    (0,255,0), 3 )

        cv2.imwrite("ssd_result.png", image_vis)

        print( results )
        print( "Processing time[msec]: ", 1000*elapsed_time )

        ious = np.zeros(17, np.float )
        n_objects = np.zeros(17,np.float)
        ious, n_objects = difference( bboxes, class_id, results )

        # Sending result to action
        SSDObjectDetectionResult_msg_list = []
        for j in range(len(results)):
            SSDObjectDetectionResult_msg = o2ac_msgs.msg.SSDObjectDetectionResult()
            SSDObjectDetectionResult_msg.bbox = results[j]["bbox"]
            SSDObjectDetectionResult_msg.label = results[j]["class"]
            SSDObjectDetectionResult_msg.confidence = results[j]["confidence"].item()
            SSDObjectDetectionResult_msg_list.append(SSDObjectDetectionResult_msg)

        SSDResult_msg = o2ac_msgs.msg.SSDResult()
        SSDResult_msg.SSD_result = SSDObjectDetectionResult_msg_list
        action_result.SSD_result= SSDResult_msg

        self.ssd_test_once_action_server.set_succeeded(action_result)

    
def difference( gt_bboxes, gt_class_ids, preds ):

    class_wise_iou = np.zeros(17, np.float )
    class_wise_n_object = np.zeros(17,np.float)
    pred_class = list()
    pred_bbox = list()
    for p in preds:
        pred_class.append( p["class"] )
        pred_bbox.append( p["bbox"])

    for gt_bbox, gt_cls_id in zip(gt_bboxes, gt_class_ids):
        class_wise_n_object[gt_cls_id] +=1.0
        
        if gt_cls_id in pred_class:
            idx = pred_class.index(gt_cls_id)
            pred_bbox2 = pred_bbox[idx]
            pred_bbox2 = [pred_bbox2[0], pred_bbox2[1], 
                          pred_bbox2[0]+pred_bbox2[2],
                          pred_bbox2[1]+pred_bbox2[3]]
            # print(gt_cls_id) 
            # print( pred_bbox2 )
            # print( gt_bbox )

            iou = bb_intersection_over_union( pred_bbox2, gt_bbox )
            class_wise_iou[gt_cls_id]+=iou
            # print("IoU:", iou )
        else:
            class_wise_iou[gt_cls_id]+=0
    
    # print(class_wise_iou)
    # print(class_wise_n_object)
    return class_wise_iou, class_wise_n_object


def bb_intersection_over_union(boxA, boxB):
	# determine the (x, y)-coordinates of the intersection rectangle
	xA = max(boxA[0], boxB[0])
	yA = max(boxA[1], boxB[1])
	xB = min(boxA[2], boxB[2])
	yB = min(boxA[3], boxB[3])
	# compute the area of intersection rectangle
	interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
	# compute the area of both the prediction and ground-truth
	# rectangles
	boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
	boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
	# compute the intersection over union by taking the intersection
	# area and dividing it by the sum of prediction + ground-truth
	# areas - the interesection area
	iou = interArea / float(boxAArea + boxBArea - interArea)
	# return the intersection over union value
	return iou

if __name__ == "__main__":
    try:
        server = SSDTestServer()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
