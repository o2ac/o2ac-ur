#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import o2ac_msgs.msg
import sys
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("M2Det"))

import os
import cv2
import numpy as np
import time
from torch.multiprocessing import Pool
from utils.nms_wrapper import nms
from utils.timer import Timer
from configs.CC import Config
import argparse
from layers.functions import Detect, PriorBox
from m2det import build_net
from data import BaseTransform
from utils.core import *
from utils.pycocotools.coco import COCO

import get_wrs_dataset as get_wrs
from tqdm import tqdm
import matplotlib.pyplot as plt 
import os
import json

parser = argparse.ArgumentParser(description='M2Det Evaluation')
parser.add_argument('-c', '--config', default='configs/m2det320_vgg.py', type=str)
parser.add_argument('-m', '--trained_model', default=None, type=str, help='Trained state_dict file path to open')
parser.add_argument('--show', action='store_true', help='Whether to display the images')
parser.add_argument('-o', '--out', default='./trained/result/', type=str)
parser.add_argument('--cross_val', default=True, type=bool, help='If you use cross validation True')
parser.add_argument('--K_fold', default=5, type=int, help='The number of K-fold')
parser.add_argument('--K_num', default=1, type=int)
parser.add_argument('--annotations_path',
                    default=[
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
                            ],
                            help='The path of Annotatnios flolder') #2020.3.19 Miyoshi 
parser.add_argument('--images_path', default=['../../WRS_Dataset/Images/Close/RGB/', '../../WRS_Dataset/Images/Far/RGB/'], help='The path of Annotatnios flolder') #2020.3.19 Miyoshi 
parser.add_argument('--th', default=0.5, type=float, help='Threshfold')

args = parser.parse_args()

print_info(' ----------------------------------------------------------------------\n'
           '|                       M2Det Demo Program                             |\n'
           ' ----------------------------------------------------------------------', ['yellow','bold'])

global cfg
cfg = Config.fromfile(args.config)
anchor_config = anchors(cfg)
print_info('The Anchor info: \n{}'.format(anchor_config))
priorbox = PriorBox(anchor_config)
net = build_net('test',
                size = cfg.model.input_size,
                config = cfg.model.m2det_config)
init_net(net, cfg, args.trained_model)
print_info('===> Finished constructing and loading model',['yellow','bold'])
net.eval()
with torch.no_grad():
    priors = priorbox.forward()
    if cfg.test_cfg.cuda:
        net = net.cuda()
        priors = priors.cuda()
        cudnn.benchmark = True
    else:
        net = net.cpu()
detector = Detect(cfg.model.m2det_config.num_classes, cfg.loss.bkg_label, anchor_config)

class M2DetTestServer(object):
    def __init__(self):
        rospy.init_node('M2Det_test_server_py')

        self.M2Det_test_action_server = actionlib.SimpleActionServer("M2DetTest", o2ac_msgs.msg.M2DetTestAction, 
            execute_cb = self.M2Det_test_callback, auto_start = True)

        rospy.loginfo("M2Det_test_server has started up!")

    def _to_color(self, indx, base):
        """ return (b, r, g) tuple"""
        base2 = base * base
        b = 2 - indx / base2
        r = 2 - (indx % base2) / base
        g = 2 - (indx % base2) % base
        return int(b * 127), int(r * 127), int(g * 127)

    def draw_detection(self, im, bboxes, scores, cls_inds):
        imgcv = np.copy(im)
        h, w, _ = imgcv.shape
        for i, box in enumerate(bboxes):
            if scores[i] < args.th:
                continue
            cls_indx = int(cls_inds[i])
            box = [int(_) for _ in box]
            thick = int((h + w) / 300)
            base = int(np.ceil(pow(cfg.model.m2det_config.num_classes, 1. / 3)))
            colors = [self._to_color(x, base) for x in range(cfg.model.m2det_config.num_classes)]
            labels = tuple([_.strip().split(',')[-1] for _ in open('../../WRS_Dataset/labels.txt','r').readlines()])
            cv2.rectangle(imgcv,
                          (box[0], box[1]), (box[2], box[3]),
                          colors[cls_indx], thick)
            mess = '%s: %.3f' % (labels[cls_indx], scores[i])
            cv2.putText(imgcv, mess, (box[0], box[1] - 7),
                        0, 1e-3 * h, colors[cls_indx], thick // 3)

        return imgcv

    def M2Det_test_callback(self, goal):
        action_result = o2ac_msgs.msg.M2DetTestResult()
        
        dataset = get_wrs.load_dataset(args.annotations_path, args.images_path, cfg, args.cross_val, args.K_fold, args.K_num)
        dataset = get_wrs.Trans_Dataset(dataset, train=False) # if train=True, dataset is train dataset.

        if os.path.exists(args.out) is False:
            os.makedirs(args.out)
        if os.path.exists(args.out+"/images/Far/") is False and args.show:
            os.makedirs(args.out+"/images/Far/")
        if os.path.exists(args.out+"/images/Close/") is False and args.show:
            os.makedirs(args.out+"/images/Close/")
        if os.path.exists(args.out+"/bbox/Far/") is False and args.show:
            os.makedirs(args.out+"/bbox/Far/")
        if os.path.exists(args.out+"/bbox/Close/") is False and args.show:
            os.makedirs(args.out+"/bbox/Close/")

        for count, data in enumerate(tqdm(dataset)):
            image, target, img_path = data

            C, H, W = image.size()

            img = image.unsqueeze(0)
            if cfg.test_cfg.cuda:
                img = img.cuda()

            out = net(img)

            scale = torch.Tensor([W,H,W,H])
            boxes, scores = detector.forward(out, priors)
            boxes = (boxes[0]*scale).cpu().numpy()
            scores = scores[0].cpu().numpy()

            allboxes = []
            results = {}

            for j in range(0, cfg.model.m2det_config.num_classes):
                inds = np.where(scores[:,j] > args.th)[0]
                if len(inds) == 0:
                    continue
                c_bboxes = boxes[inds]
                c_scores = scores[inds, j]
                result = {"bbox":c_bboxes.tolist(), "class":j, "confidence":c_scores.tolist()}
                results.update({"Item_"+str(j):result}) # This list is passed to pose estimation.

                c_dets = np.hstack((c_bboxes, c_scores[:, np.newaxis])).astype(np.float32, copy=False)
                soft_nms = cfg.test_cfg.soft_nms
                keep = nms(c_dets, cfg.test_cfg.iou, force_cpu = soft_nms) #min_thresh, device_id=0 if cfg.test_cfg.cuda else None)
                keep = keep[:cfg.test_cfg.keep_per_class]
                c_dets = c_dets[keep, :]
                allboxes.extend([_.tolist()+[j] for _ in c_dets])

            # Sending result to action
            objectDetectionResult_msg_list = []
            for j in results:
                boundingBox_msg_list = []
                for i in range(len(results[j]["bbox"])):
                    boundingBox_msg = o2ac_msgs.msg.boundingBoxes()
                    boundingBox_msg.bbox = results[j]["bbox"][i]
                    boundingBox_msg_list.append(boundingBox_msg)

                objectDetectionResult_msg = o2ac_msgs.msg.objectDetectionResult()
                objectDetectionResult_msg.bboxes = boundingBox_msg_list
                objectDetectionResult_msg.label = results[j]["class"]
                objectDetectionResult_msg.confidence = results[j]["confidence"]
                objectDetectionResult_msg_list.append(objectDetectionResult_msg)

            M2DetResult_msg = o2ac_msgs.msg.M2DetResult()
            M2DetResult_msg.M2Det_result = objectDetectionResult_msg_list
            action_result.M2Det_result_list.append(M2DetResult_msg)

            allboxes = np.array(allboxes)
            boxes = allboxes[:,:4]
            scores = allboxes[:,4]
            cls_inds = allboxes[:,5]

            img = image.cpu().numpy()
            im2show = self.draw_detection(img.transpose(1,2,0), boxes, scores, cls_inds)

            if im2show.shape[0] > 1100:
                im2show = cv2.resize(im2show,
                                 (int(1000. * float(im2show.shape[1]) / im2show.shape[0]), 1000))
            if args.show:
                cv2.imshow('test', im2show)
                if dataset._get_img_path(count).split('/')[-3] == "Far":
                    cv2.imwrite(args.out+"/images/Far/"+dataset._get_img_path(count).split('/')[-1], im2show*255)
                    f = open(args.out+"/bbox/Far/"+dataset._get_img_path(count).split('/')[-1].split('.')[0]+".json", "w")
                    json.dump(results, f)
                else:
                    cv2.imwrite(args.out+"/images/Close/"+dataset._get_img_path(count).split('/')[-1], im2show*255)
                    f = open(args.out+"/bbox/Close/"+dataset._get_img_path(count).split('/')[-1].split('.')[0]+".json", "w")
                    json.dump(results, f)

                cv2.waitKey(100)

            else:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    out_video.release()
                    capture.release()
                    break

        self.M2Det_test_action_server.set_succeeded(action_result)
            

if __name__ == '__main__':
    try:
        server = M2DetTestServer()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
    except rospy.ROSInterruptException:
        pass
