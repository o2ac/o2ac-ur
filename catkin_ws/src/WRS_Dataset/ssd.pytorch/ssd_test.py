#-*- encoding:utf-8 -*-
import os
import sys
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
# アノテーションファイルの取得
annos = glob.glob(anno_root)

HOME = os.path.expanduser("~")
droot = os.path.join(HOME,"WRS2020/WRS_Dataset")

def get_argumets():
    """
        Parse arguments from command line
    """

    parser = argparse.ArgumentParser( description='SSD test')
    parser.add_argument('--weight', type=str, default='weights/WRS.pth',
                        help='file name of weight')
    parser.add_argument('--id', type=int, default=0,
                        help='dataset id of input image')

    return parser.parse_args()

class SSD_detection():

    def __init__( self, fname_weight ):
        
        self.net = build_ssd('test', 300, 17)    # initialize SSD
        self.net.load_weights( fname_weight )


    def object_detection( self, im_in, threshold = 0.6 ):
        """
            Object detection by SSD
            Input: 
                im_in... input_image
                threshold... threshold of detection
            Return:
                results... a list of dict(bbox, class_id, score)
        """

        # Preproc
        x = cv2.resize( im_in, (300, 300)).astype(np.float32 )
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
        scale = torch.Tensor(im_in.shape[1::-1]).repeat(2)
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


def difference( gt_bboxes, gt_class_ids, preds ):

    class_wise_iou = np.zeros(16, np.float )
    class_wise_n_object = np.zeros(16,np.float)
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

            iou = bb_intersection_over_union( pred_bbox2, gt_bbox )
            class_wise_iou[gt_cls_id]+=iou
        else:
            class_wise_iou[gt_cls_id]+=0
    
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

    """Get arguments"""
    args = get_argumets()
    test_id = args.id

    # prepare network model
    ssd = SSD_detection( args.weight )

    # Load image
    cnt = 0
    ious = np.zeros(16, np.float )
    n_objects = np.zeros(16,np.float)


    
    dataset = WRS2020_Detection( root=droot, image_set = 'test' )

    for n in range( len(dataset) ):
        im_name, anno_name = dataset.pull_path(n)
        f = open( anno_name )
        json_data = json.load( f )
        #print(json_data)
        bboxes = json_data["bbox"]
        class_id = json_data["class_id"]
        im_in = cv2.imread( json_data["rgb_path"], cv2.IMREAD_COLOR )

        results = ssd.object_detection( im_in )

        iou, n_object = difference( bboxes, class_id, results )

        ious += iou
        n_objects += n_object
        cnt+=1
        if cnt%50 == 0:
            print(cnt, "/", len(dataset) )
    
    miou = ious / n_objects
    print("mIOUs:", miou )
    print("mIOU:", np.mean(miou))
    print("n_objects:", n_objects )

