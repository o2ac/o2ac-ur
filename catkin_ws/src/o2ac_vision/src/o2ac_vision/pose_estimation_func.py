#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Chukyo University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Chukyo University nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Shuichi Akizuki

import cv2
import numpy as np
import math
import copy
import os
import json
import math
import random

"""
   Downsampling for binary image
   Input:
     im: input image
     fx, fy: scaling factor along with x and y coordinate
   Return:
     im_ds: downsampled image
"""


def downsampling_binary(im, _fx=0.5, _fy=0.5):
    im_ds = cv2.resize(im, None, fx=_fx, fy=_fy, interpolation=cv2.INTER_AREA)
    im_ds = np.clip(im_ds, 0, 1)

    return im_ds.copy()


"""
 In-plane Rotation Estimation using Distribution of Line Segment Directions
"""


class RotationEstimation:
    def __init__(self, im, bbox, lseg_len=15, n_bin=90):
        # Parameter for LSDH
        self._lseg_len = lseg_len  # length of line segment
        self._n_bin = n_bin  # number of histogram's bin
        self._step = int(self._lseg_len / 2)  # sampling step of line segment

        # Parameter for image processing
        self._canny1 = 100  # parameter1 for canny edge detection
        self._canny2 = 200  # parameter2 for canny edge detection
        self._continuous_streaming_mode = cv2.RETR_EXTERNAL  # contour extraction mode

        # Variables
        self._deg2rad = math.pi / 180.0
        self._im = im  # input image
        self._bbox = bbox  # bounding box. list[x,y,w,h]
        self._im_bb = self.im_crop_bbox(self._im, self._bbox)  # image of bounding box
        self._im_edge = None
        self._contours = None  # contours

        # Do main processing
        self._iprs = self.main_proc()  # in-plane rotations (in degree)

    """
    Crop image
    """

    def im_crop_bbox(self, im, bbox):
        im_bb = im[bbox[1] : bbox[1] + bbox[3], bbox[0] : bbox[0] + bbox[2]]
        im_out = im_bb.copy()
        return im_out

    """
    Contour detection from image
    """

    def contour_detection(self):

        im_bb = self.im_crop_bbox(self._im, self._bbox)
        # Histogram normalization
        # create a CLAHE object (Arguments are optional).
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cl1 = clahe.apply(self._im_bb)

        # Edge detection
        edges = cv2.Canny(cl1, self._canny1, self._canny2)
        # cv2.imshow("cl1",cl1)
        # cv2.waitKey(0)
        # cv2.imshow("edges",edges)
        # cv2.waitKey(0)
        self._im_edge = edges

        # findContours
        # check version of cv2
        cv2_versoin = cv2.__version__
        if "4" == cv2_versoin[0]:
            contours, hierarchy = cv2.findContours(
                edges, self._continuous_streaming_mode, cv2.CHAIN_APPROX_NONE
            )
        else:
            _, contours, hierarchy = cv2.findContours(
                edges, self._continuous_streaming_mode, cv2.CHAIN_APPROX_NONE
            )

        return contours

    """
    Make a Line Segment Direction Histogram from a list of contour pixels
    """

    def make_LSDH(self, cont):

        i = 0
        hist = np.zeros(self._n_bin, np.float)
        while i < len(cont) - self._lseg_len:
            # get line segment
            lseg = cont[i + self._lseg_len][0] - cont[i][0]

            # compute direction of line segment
            if lseg[0] != 0:
                dir = 180.0 * math.atan(float(lseg[1]) / float(lseg[0])) / math.pi
            else:
                dir = 90.0
            if dir < 0.0:
                dir += 180.0
            bin = int(dir / (180.0 / self._n_bin))
            # Vote to the histogram
            hist[bin] += 1.0
            if bin == self._n_bin - 1:
                hist[0] += 0.5
                hist[bin - 1] += 0.5
            elif bin == 0:
                hist[self._n_bin - 1] += 0.5
                hist[1] += 0.5
            else:
                hist[bin - 1] += 0.5
                hist[bin + 1] += 0.5

            i += self._step

        return hist

    """
    Compute dominant orientation from histogram
    """

    def compute_orientation(self, hist):

        # Normalize histogram
        sum = np.sum(hist)
        hist = hist / (sum + 0.00001)

        # detect peak
        vote_max_bin = np.argmax(hist)

        # Convert the bin index to the orientation in degree
        in_plane_rotation = vote_max_bin * 180.0 / self._n_bin
        return [in_plane_rotation]

    """
    Main processing
    """

    def main_proc(self):
        # detect contours from the bounding box image.
        self._contours = self.contour_detection()

        # make a Line Segment Direction Histogram (LSDH)
        lsdh = np.zeros(self._n_bin, np.float)
        for cont in self._contours:
            hist = self.make_LSDH(cont)
            lsdh += hist

            im_res = self.get_im_bb()
            im_res = cv2.cvtColor(im_res, cv2.COLOR_GRAY2BGR)
        # compute domitant orientation of LSDH
        orientation = self.compute_orientation(lsdh)
        return orientation

    def get_im_bb(self):
        return self._im_bb.copy()

    def get_im_edge(self):
        return self._im_edge.copy()

    def get_im_contours(self):
        im_bb3 = cv2.cvtColor(self._im_bb.copy(), cv2.COLOR_GRAY2BGR)
        im_cont = cv2.drawContours(im_bb3, self._contours, -1, (0, 255, 0), 1)
        return im_cont

    def get_result_image(self):

        im_res = self.get_im_bb()
        im_res = cv2.cvtColor(im_res, cv2.COLOR_GRAY2BGR)
        pt1 = (int(self._bbox[2] / 2), int(self._bbox[3] / 2))

        cv2.circle(im_res, pt1, 3, (0, 255, 0), -1, cv2.LINE_AA)
        line_length = int(min(self._bbox[2:]) / 2)
        for rot in self._iprs:
            pt2 = (
                int(pt1[0] + line_length * math.cos(rot * self._deg2rad)),
                int(pt1[1] + line_length * math.sin(rot * self._deg2rad)),
            )
            cv2.arrowedLine(im_res, pt1, pt2, (0, 255, 0), 1, cv2.LINE_AA)

        return im_res

    """
      Get orientation
      Input:
        flip: if true, detected orientatios and fliped orientations are returned.
      Return;
        list of orientations
    """

    def get_orientation(self, flip=True):

        orientations = copy.deepcopy(self._iprs)
        if flip is True:
            for ori in orientations:
                self._iprs.append(ori + 180.0)

        return self._iprs


"""
 Binary Template Matching
 Input:
   temp: template, 1ch numpy array
   scene: input scene, 1 ch numpy array
   region: search region (+-search_y, +-search_x)
"""


class BinaryTemplateMatching:
    def __init__(self, temp, scene, region=(0, 0)):
        self._temp = temp.copy()  # template
        self._scene = scene.copy()  # scene
        self._region = region  # search region
        self._initial_pad = np.array([0, 0], np.int)

        self.initial_padding()
        self._offset_ini_pad = np.array(
            (self._initial_pad[0], self._initial_pad[0]), np.int
        )

        self._t_size = np.asarray(self._temp.shape)  # size of template
        self._s_size = np.asarray(self._scene.shape)  # size of scene
        # left-top pixel of an initial search position
        self._ltop_c = ((self._s_size - self._t_size) / 2).astype(np.int)

        # Matching, compute score map, _s_map, and offset list
        self._s_map, self._offset_list = self.main_proc()

    """
      If the size of scene is smaller than that of template, zero padding is applied.
    """

    def initial_padding(self):
        s_shape = self._scene.shape
        t_shape = self._temp.shape
        diff_size_y = t_shape[0] - s_shape[0]
        diff_size_x = t_shape[1] - s_shape[1]

        diff_max = max(diff_size_y, diff_size_x)
        if diff_max > 0:
            pad_size = int(diff_max / 2) + 1
            self._scene = np.pad(self._scene, (pad_size, pad_size), "minimum")
            self._initial_pad = np.array((pad_size, pad_size), np.int)

    def padding(self, im, param):
        pad_size = max(param)  # define padding size
        im_pad = np.pad(im, (pad_size, pad_size), "edge")
        return im_pad, pad_size

    def main_proc(self):

        # padding
        pad_scene, pad_size = self.padding(self._scene, self._region)
        pad_scene = pad_scene > 0
        pad_scene = np.logical_not(pad_scene)
        pad_scene = np.asarray(pad_scene, np.uint8)
        pad_scene = cv2.distanceTransform(pad_scene, cv2.DIST_L2, 5)

        # score map
        s_map = np.zeros([self._region[0] * 2 + 1, self._region[1] * 2 + 1])
        offset_list = list()  # offset_list[n]+t_size/2 = center pixel
        for sj, j in enumerate(range(-self._region[0], self._region[0] + 1)):
            for si, i in enumerate(range(-self._region[1], self._region[1] + 1)):
                offset = self._ltop_c + np.array([j, i]) + pad_size
                s_map[sj, si] = np.sum(
                    self._temp
                    * pad_scene[
                        offset[0] : offset[0] + self._t_size[0],
                        offset[1] : offset[1] + self._t_size[1],
                    ]
                )
                offset_list.append(offset - pad_size)

        return s_map, offset_list

    # return the location in scene coordinate system and its score
    def get_result(self):
        res = np.argmin(self._s_map)
        res_offset = self._offset_list[res] - self._initial_pad
        center_pixel = res_offset + self._t_size / 2
        return center_pixel, res_offset, np.min(self._s_map)

    def get_result_image(self, _offset):
        offset = _offset.copy()
        pad_scene, pad_size = self.padding(self._scene, self._region)
        im_res = np.zeros(pad_scene.shape)
        offset += pad_size + self._initial_pad
        im_res[
            offset[0] : offset[0] + self._t_size[0],
            offset[1] : offset[1] + self._t_size[1],
        ] = self._temp
        im_res += pad_scene
        return im_res

    def get_score_map(self):

        return self._s_map


class TemplateMatching:
    def __init__(self, im_c, ds_rate, temp_root, temp_info_name="template_info.json"):
        """
        im_c: an image of input scene
        ds_rate: downsampling rate
        temp_root: path to templates
        temp_info_name: temp_info_name
        """

        # downsampling rate
        self.ds_rate = ds_rate
        # input image
        self.im_c = im_c
        #
        self.temp_root = temp_root

        """ Load template infomation """
        temp_info_fullpath = os.path.join(temp_root, temp_info_name)
        if os.path.isfile(temp_info_fullpath):
            json_open = open(temp_info_fullpath, "r")
            self.temp_info = json.load(json_open)
        else:
            print("ERROR!!")
            print(temp_info_fullpath, " couldn't read.")
            exit()

    def read_template(self, result):
        """
        Input:
            result: Output of the object detection(SSD). list of {bbox, class, confidence}
        Output:
            im_temp_edge: edge_template
            temp_ori: the dominant orientations of template
        """
        class_id = result["class"]
        info_id = -1
        for n, info in enumerate(self.temp_info):
            if result["class"] in info.values():
                info_id = n
                break

        if info_id is -1:
            print("ERROR!!")
            print("Template info does not include id ", class_id)

        # print(os.path.join( self.temp_root, self.temp_info[info_id]["name_edge"]))
        im_temp_edge = cv2.imread(
            os.path.join(self.temp_root, self.temp_info[info_id]["name_edge"]), 0
        )
        temp_ori = self.temp_info[info_id]["orientation"]

        return im_temp_edge, temp_ori

    def compute(self, result, search=(20, 20)):
        """
        Input:
            result: Output of the object detection(SSD). list of {bbox, class, confidence}
        Output:
            center: center coordinate of target. array(j,i)
            orientation: in-plane rotation of target. float
        """

        class_id = result["class"]
        bbox = result["bbox"]
        re = RotationEstimation(self.im_c, bbox, lseg_len=20, n_bin=120)
        in_ori = re.get_orientation()
        im_edge = re.get_im_edge()
        im_edge_ds = downsampling_binary(im_edge, _fx=self.ds_rate, _fy=self.ds_rate)

        """Read orientation of the template"""
        im_temp_edge, temp_ori = self.read_template(result)

        # Rotation of template image
        rows, cols = im_temp_edge.shape
        res_orientations = list()  # orientation in degree, ccw
        for o_in in in_ori:
            for o_temp in temp_ori:
                res_orientations.append(o_in - o_temp)

        # print("difference of orientations")
        # print( res_orientations )

        """ Create rotated templates """
        im_temp_edge_rots = list()
        im_temp_eds = list()
        for res_ori in res_orientations:
            # getRotationMatrix2D( center, angle(ccw), scale )
            rot_mat = cv2.getRotationMatrix2D((cols / 2, rows / 2), 360 - res_ori, 1)
            im_rot = cv2.warpAffine(im_temp_edge, rot_mat, (cols, rows))
            im_temp_edge_rots.append(im_rot)
            im_temp_eds.append(downsampling_binary(im_rot, self.ds_rate, self.ds_rate))

        """ Binary Template Matching """
        scores = list()  # score list of each template
        offsets = list()  # offset list of each template
        for temp in im_temp_eds:  # Apply template matching
            btm = BinaryTemplateMatching(temp, im_edge_ds, search)
            _, offset, score = btm.get_result()
            scores.append(score)
            offsets.append(offset)

        # Get ID of the most similar template
        res_idx = np.argmin(np.asarray(scores))

        # Up scale
        offset_original = offsets[res_idx] * 1.0 / self.ds_rate

        # compute center coordinate
        ltop = np.asarray(
            [bbox[1] + offset_original[0], bbox[0] + offset_original[1]], np.int
        )
        temp_center = np.asarray(im_temp_edge.shape, np.int) / 2
        center = ltop + temp_center
        orientation = res_orientations[res_idx]

        return center, orientation  # array[j, i], float

    def get_result_image(self, result, res_ori, res_center, im_scene=None):
        """
        Optional. for visualizatoin
        Input:
            result: object detection result
            res_ori: orientation ( output of self.compute() )
            res_center: center coordinate ( output of self.compute() )
        Output:
            im_res_on_original: output image.
        """
        if im_scene is None:
            im_scene = self.im_c

        im_temp_edge, temp_ori = self.read_template(result)
        temp_center = np.asarray(im_temp_edge.shape, np.int) / 2
        ltop = res_center - temp_center
        im_res_on_original = visualize_result(im_temp_edge, im_scene, ltop, res_ori)
        return im_res_on_original


"""
Visualization function.

Input:
  im_temp: template image
  im_scene: input image
  ltop: left-top pixel of target object in scene (np.array)
  orientation: orientation in degree (CCW)
Return:
  im_res_on_original: output image
"""


def visualize_result(im_temp, im_scene, ltop, orientation):

    # dilation for good visualization
    kernel = np.ones((3, 3), np.uint8)
    im_temp = cv2.dilate(im_temp, kernel, iterations=1)

    # rotate template
    rows, cols = im_temp.shape
    # getRotationMatrix2D() takes clock-wise orientation, so we need convert
    # ccw -> cw
    rot_mat = cv2.getRotationMatrix2D((cols / 2, rows / 2), 360 - orientation, 1)
    res_im_temp = cv2.warpAffine(im_temp, rot_mat, (cols, rows))

    # padding input scene
    res_pad = int(res_im_temp.shape[0])  # padding of result image
    if im_scene.ndim == 2:
        im_res_on_original = np.pad(im_scene.copy(), (res_pad, res_pad), "constant")
    else:
        im_res_on_original = np.pad(
            im_scene.copy(),
            ((res_pad, res_pad), (res_pad, res_pad), (0, 0)),
            "constant",
        )
    ltop_pad = (ltop + res_pad).astype(np.int)

    bbox_size = np.asarray(im_temp.shape, np.int)
    bb_center = (ltop_pad + bbox_size / 2).astype(np.int)

    # mapping edge pixels of template
    im_temp_edge_rot_vis = cv2.cvtColor(res_im_temp, cv2.COLOR_GRAY2BGR)
    im_temp_edge_rot_vis[:, :, 1:3] = 0
    if im_res_on_original.ndim != 3:
        im_res_on_original = cv2.cvtColor(im_res_on_original, cv2.COLOR_GRAY2BGR)

    im_temp_edge_rot_vis = np.asarray(im_temp_edge_rot_vis, np.int)
    im_res_on_original = np.asarray(im_res_on_original, np.int)
    im_res_on_original[
        ltop_pad[0] : ltop_pad[0] + bbox_size[0],
        ltop_pad[1] : ltop_pad[1] + bbox_size[1],
    ] += im_temp_edge_rot_vis
    im_res_on_original = np.clip(im_res_on_original, 0, 255)
    im_res_on_original = np.asarray(im_res_on_original, np.uint8)

    # draw bounding box
    im_res_on_original = cv2.rectangle(
        im_res_on_original,
        (ltop_pad[1], ltop_pad[0]),
        (ltop_pad[1] + bbox_size[1], ltop_pad[0] + bbox_size[0]),
        (0, 255, 255),
        3,
    )
    # draw center position
    im_res_on_original = cv2.circle(
        im_res_on_original,
        (bb_center[1], bb_center[0]),
        5,
        (0, 255, 255),
        -1,
        cv2.LINE_AA,
    )
    # draw orientation
    line_length = int(bbox_size[0] / 2)
    deg2rad = math.pi / 180.0
    pt2 = (
        int(bb_center[1] + line_length * math.cos(orientation * deg2rad)),
        int(bb_center[0] + line_length * math.sin(orientation * deg2rad)),
    )
    im_res_on_original = cv2.arrowedLine(
        im_res_on_original,
        (bb_center[1], bb_center[0]),
        pt2,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    im_res_on_original = im_res_on_original[res_pad:-res_pad, res_pad:-res_pad]

    return im_res_on_original


"""
  Fast Graspability Evaluation on RGB image


"""


class FastGraspabilityEvaluation:
    def __init__(self, _im_in, _im_hand, _param):
        self.im_in = _im_in.copy()
        self.im_hand = _im_hand.copy()
        self.im_in_org = _im_in

        self.im_belt = None  # belt = 1, other = 0
        self.fg_mask = None  # foreground = 1, background = 0

        self.im_hands = None

        self.candidate_idx = list()
        self.pos_list = list()
        self.score_list = list()

        self.gp_result = list()  # grasp points y, x, theta[deg]

        # parameter
        self.ds_rate = _param["ds_rate"]
        self.n_grasp_point = _param["n_grasp_point"]
        self.threshold = _param["threshold"]

        # Down sampling
        self.im_in = cv2.resize(
            self.im_in,
            None,
            fx=self.ds_rate,
            fy=self.ds_rate,
            interpolation=cv2.INTER_NEAREST,
        )
        self.im_hand = cv2.resize(
            self.im_hand,
            None,
            fx=self.ds_rate,
            fy=self.ds_rate,
            interpolation=cv2.INTER_NEAREST,
        )

        # Masking out the belt and foreground
        self.im_belt = self.detect_belt(self.im_in)
        self.fg_mask = self.detect_foreground(self.im_in)

    def detect_belt(self, im_in):
        """Detect belt mask
        Input:
            im_in(numpy array BGR, 8bits): input image
        Return:
            binarized image [0 or 1] np.uint8
        """
        # detect foreground
        im_hsv = cv2.cvtColor(im_in, cv2.COLOR_BGR2HSV)
        im_s = im_hsv[:, :, 1]  # get s image
        th, im_b = cv2.threshold(im_s, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # detect the belt as circle
        circles = cv2.HoughCircles(
            im_s,
            cv2.HOUGH_GRADIENT,
            dp=2,
            minDist=10,  # 検出される円の中心同士の最小距離．
            param1=100,  # Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値
            param2=100,  # 円の中心を検出する際の投票数の閾値
            minRadius=10,  # 円の半径の最小値
            maxRadius=100,
        )  # 円の半径の最大値

        im_circle = np.zeros(im_in.shape)
        cv2.circle(
            im_circle,
            (circles[0, 0, 0], circles[0, 0, 1]),
            circles[0, 0, 2],
            (1, 1, 1),
            5,
        )
        circle_mask = im_circle[:, :, 0]
        return circle_mask * im_b

    def detect_foreground(self, im_in, factor=1.5):
        """Detect foreground mask
        Input:
            im_in(numpy array BGR, 8bits): input image
            factor(float): しきい値．最瀕値のfactor倍の画素値までを背景とする．
                           大きくすると，検出領域が小さくなる
        Return:
            binarized image [0 or 1] np.uint8
        """
        im_hsv = cv2.cvtColor(im_in, cv2.COLOR_BGR2HSV)
        im_v = im_hsv[:, :, 2]  # get v image
        hist, bins = np.histogram(im_v, 20)
        threshold = bins[np.argmax(hist)]  # ヒストグラムの最瀕値の画素値を取得
        im_th = np.where(im_v < threshold * 1.5, 0, 1)  # 最瀕値の1.5倍の明るさまでを背景とする．

        return im_th

    def RotationTemplate(self, im_temp, deg):

        cols, rows = im_temp.shape[1], im_temp.shape[0]

        rot_mat = cv2.getRotationMatrix2D((cols / 2, rows / 2), 360 - deg, 1)
        im_rot = cv2.warpAffine(im_temp, rot_mat, (cols, rows))

        return im_rot, np.sum(im_rot)

    def RotationColorTemplate(self, im_temp, rotation):
        # convert binary template to color template.
        # thumb is colored by red
        temp_c = copy.deepcopy(im_temp)
        cols, rows = im_temp.shape[1], im_temp.shape[0]

        temp_c = np.clip(temp_c * 100.0, 0, 255)
        temp_c = np.asarray(temp_c, np.uint8)
        temp_c = cv2.cvtColor(temp_c, cv2.COLOR_GRAY2BGR)
        temp_c[:, : int(cols / 2), 0] = temp_c[:, : int(cols / 2), 2] = 0
        temp_c[:, int(cols / 2) :, 0] = temp_c[:, int(cols / 2) :, 1] = 0

        rot_mat = cv2.getRotationMatrix2D((cols / 2, rows / 2), 360 - rotation, 1)
        im_rot = cv2.warpAffine(temp_c, rot_mat, (cols, rows))

        return im_rot

    def main_proc(self):
        """compute grasp points
        Return:
            list: grasp point in image coordinate system (y, x, theta(degree,CCW))
        """

        # Generate search point
        points = np.where(self.im_belt == 1)
        n_search_point = self.n_grasp_point
        search_points = list()
        for n in range(len(points[0])):
            search_points.append((points[0][n], points[1][n]))

        if n_search_point < len(search_points):  # random sampling
            search_points = random.sample(search_points, n_search_point)

        search_points = np.asarray(search_points, np.int)

        # Hand pattern matching
        cols, rows = self.im_hand.shape[1], self.im_hand.shape[0]
        s_cols, s_rows = self.im_belt.shape[1], self.im_belt.shape[0]
        offset = np.array((cols / 2, rows / 2), np.int)
        score_list = list()
        pos_list = list()
        cx, cy = self.im_in.shape[1] / 2, self.im_in.shape[0] / 2
        for sp in search_points:
            ltop = sp - offset  # left-top corner of search point
            deg = np.degrees(np.arctan2((sp[0] - cy), (sp[1] - cx)))

            hand, hand_area = self.RotationTemplate(self.im_hand, deg)

            # check border
            if (
                ltop[0] < 0
                or ltop[1] < 0
                or s_rows <= ltop[0] + rows
                or s_cols <= ltop[1] + cols
            ):
                # print("skip")
                continue

            conv_hand = (
                self.fg_mask[ltop[0] : ltop[0] + rows, ltop[1] : ltop[1] + cols] * hand
            )
            score_hand = np.sum(conv_hand) / hand_area

            self.score_list.append(score_hand)
            # position and orientation
            self.pos_list.append(
                (int(sp[0] / self.ds_rate), int(sp[1] / self.ds_rate), deg)
            )

        self.score_list = np.asarray(self.score_list)
        # Candidate selection
        self.candidate_idx = np.where(self.score_list < self.threshold)[0]

        # Make final result
        for n in self.candidate_idx:
            grasp_point = np.array([self.pos_list[n][0], self.pos_list[n][1]], np.float)
            rotation = self.pos_list[n][2]
            self.gp_result.append((int(grasp_point[0]), int(grasp_point[1]), rotation))

        return self.gp_result

    def get_foreground_mask(self):

        im_fg = self.fg_mask.copy()
        im_fg = np.asarray(im_fg * 255, np.uint8)

        return im_fg

    def get_im_belt(self):

        im_out = self.im_belt.copy()
        im_out = np.asarray(im_out * 255, np.uint8)

        return im_out

    def visualization(self, im_result=None):
        if im_result is None:
            im_result = self.im_in_org.copy()

        # Overlay hand templates
        im_result = np.asarray(im_result, np.int)
        cols, rows = self.im_hand.shape[1], self.im_hand.shape[0]
        offset = np.array((cols / 2, rows / 2), np.int)
        for n in self.gp_result:
            pos = np.array((n[0], n[1]))
            im_hand_c = self.RotationColorTemplate(self.im_hand, n[2])
            ltop = pos - offset
            im_result[ltop[0] : ltop[0] + rows, ltop[1] : ltop[1] + cols] += im_hand_c

        im_result = np.clip(im_result, 0, 255)
        im_result = np.asarray(im_result, np.uint8)
        for n in self.gp_result:
            im_result = cv2.circle(
                im_result, (n[1], n[0]), 3, (0, 255, 0), -1, cv2.LINE_AA
            )

        return im_result


#############################################################################
#
# Verify picking
#
# sample code
#  class_id = 6 # target label. "6" means the belt.
#  ssd_detection = o2ac_ssd.ssd_detection()
#  pc = PickCheck( ssd_detection )
#  flag = pc.check( im_in, class_id )
#
# If flag is True, grasp was successful.
#############################################################################


class PickCheck:
    """Verify that the grasp was successful

    Args:
        ssd_detection(list): result of ssd_detection

    """

    def __init__(self, ssd_detection):
        self.ssd_detection = ssd_detection
        self.ssd_results = None
        self.target = None
        self.im_vis = None

    def check(self, im_in, class_id, ssd_treshold=0.6):
        """verify grasp

        Args:
            im_in(np.array): input image
            class_id(int): target classs to be verified
            ssd_threshold(float, optional): threshold of ssd confidence
        Return:
            bool: If True, grasp was successful.
        """
        flag = False
        im_tmp = im_in.copy()
        self.im_vis = im_in.copy()
        self.ssd_results, _ = self.ssd_detection.object_detection(
            im_in, im_tmp, ssd_treshold, 0.8
        )

        for res in self.ssd_results:
            if res["class"] == class_id:
                flag = True
                self.target = res

        return flag

    def get_ssd_results(self):
        return self.ssd_results

    def draw_result(self):

        if self.target is not None:

            bbox = self.target["bbox"]
            bb_color = (0, 255, 0)
            im_vis = cv2.rectangle(
                self.im_vis,
                (bbox[0], bbox[1]),
                (bbox[0] + bbox[2], bbox[1] + bbox[3]),
                bb_color,
                3,
            )
            text = self.target["name"] + " picked!"
            cv2.putText(
                self.im_vis, text, (50, 80), 1, 3, (255, 255, 255), 2, cv2.LINE_AA
            )
            cv2.putText(self.im_vis, text, (50, 80), 1, 3, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            text = "Pick failed!"
            cv2.putText(
                self.im_vis, text, (50, 80), 1, 3, (255, 255, 255), 2, cv2.LINE_AA
            )
            cv2.putText(self.im_vis, text, (50, 80), 1, 3, (0, 0, 255), 1, cv2.LINE_AA)

    def get_im_result(self):
        self.draw_result()
        return self.im_vis


#############################################################################
#
# Notch part detection
#
# sample code:
#
#  bbox = [360, 140, 90, 150] # (x,y,w,h) ... bbox of upper half of shaft.
#  sa = ShaftAnalysis( im1, im2, bbox )
#  flag = sa.main_proc()
#
# plt.imshow(sa.get_result_image()) # for visualize
#
# # im1 and im2 are first and second view.
# im2 is current image of "after" rotated shaft
# im1 is previous image of "before" rotated shaft
# if flag is True, notch has faced front.
#############################################################################


class ShaftAnalysis:
    def __init__(self, im_in1, im_in2, bbox):
        """
        Input:
          im_in1(np.array): previous image as background(1ch gray scale)
          im_in2(np.array): current image(1ch gray scale)
          bbox(tuple): tuple that represents bbox (x,y,w,h)
        """
        # parameters
        self.threshold = 30  # thethold for image subtraction
        self.threshold_area = 20  # threshold for the number of specular pixel

        # crop ROI
        self.im_crop1 = im_in1[bbox[1] : bbox[1] + bbox[3], bbox[0] : bbox[0] + bbox[2]]
        self.im_crop2 = im_in2[bbox[1] : bbox[1] + bbox[3], bbox[0] : bbox[0] + bbox[2]]

        # variables to be computed
        self.max_blob_bbox = None  # bbox surrounding the edge of notch
        self.max_blob_area = 0  # total pixel of the edge of notch

    def set_threshold(self, n):
        """set threshold for image subtraction
        Input:
          n(uchar): threshold value
        """
        self.threshold = n

    def set_threshold_area(self, n):
        """set threshold for specular area
        Input:
          n(uchar): threshold value
        """
        self.threshold_area = n

    def main_proc(self):
        # background subtraction
        diff = np.abs(self.im_crop1.astype(float) - self.im_crop2.astype(float))
        self.mask = np.where(self.threshold < diff, 1, 0)

        # apply y-axis sobel
        self.im_sobel = cv2.Sobel(self.im_crop2, cv2.CV_8UC1, dx=0, dy=1, ksize=3)

        # detect notch candidate
        notch_prob = self.mask * self.im_sobel
        self.notch_cand = np.where(self.threshold < notch_prob, 1, 0).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        self.notch_cand = cv2.morphologyEx(self.notch_cand, cv2.MORPH_CLOSE, kernel)

        # blob analysis
        n_label, labels, stats, centroids = cv2.connectedComponentsWithStats(
            self.notch_cand
        )

        if n_label == 1:  # no candidate detected
            print("no blob")
            return 1
        stats = stats[1:, :]  # remove background
        max_blob_id = np.argmax(stats[:, 4])
        self.max_blob_bbox = stats[max_blob_id, 0:4]  # blob bbox
        self.max_blob_area = stats[max_blob_id, 4]  # blob area

        if self.threshold_area < self.max_blob_area:
            return 2
        else:
            return 3

    def get_result_image(self):
        im_vis = cv2.cvtColor(self.im_crop2, cv2.COLOR_GRAY2BGR)
        im_vis[:, :, 0] += 100 * self.notch_cand
        if self.max_blob_bbox is not None:
            pt1 = (self.max_blob_bbox[0], self.max_blob_bbox[1])
            pt2 = self.max_blob_bbox[0:2] + self.max_blob_bbox[2:4]
            pt2 = (pt2[0], pt2[1])
            im_vis = cv2.rectangle(im_vis, pt1, pt2, (0, 255, 0), 1)
            text = "area: " + str(self.max_blob_area)
            im_vis = cv2.putText(
                im_vis, text, (5, 20), 0, 0.5, (255, 255, 255), 2, cv2.LINE_AA
            )
            im_vis = cv2.putText(
                im_vis, text, (5, 20), 0, 0.5, (255, 0, 0), 1, cv2.LINE_AA
            )
        return im_vis


##########################################################################
#  sample code:
#    bbox = [300,180,200,120]   # (x,y,w,h) ... bbox of search area
#    im_in = cv2.imread( im_name, 0 )      # input image
#    im_temp = cv2.imread( temp_name, 0 )  # template image
#    psd = PulleyScrewDetection( im_in, im_temp, bbox )  # main class
#    score, flag = psd.main_proc()  # If flag is True, screw is observed.
##########################################################################
class PulleyScrewDetection:
    def __init__(self, im_in, im_temp, bbox=(0, 0, 640, 480)):
        """Pulley screw detection
        Input:
            im_in(np.array): input image
            im_temp(np.array): template image
            bbox(tuple): bbox consits of (x,y,w,h)
        """
        # set default parameters
        ds_rate = 0.5
        self.threshold = 0.8  # score of template matching

        # crop image
        self.im_in = im_in.copy()
        self.im_in = self.im_in[
            bbox[1] : bbox[1] + bbox[3], bbox[0] : bbox[0] + bbox[2]
        ]

        # read template
        self.im_temp = im_temp.copy()

        # resize image
        self.im_in = cv2.resize(self.im_in, None, fx=ds_rate, fy=ds_rate)
        self.im_temp = cv2.resize(self.im_temp, None, fx=ds_rate, fy=ds_rate)
        self.im_vis = self.im_in.copy()

    def set_threshold(self, th):
        """set threshold of template matching's score
        Input:
            th(float): threshold [0.0-1.0]
        """
        self.threshold = th

    def get_im_in_bbox(self):
        """get image within the bounding box
        Input:
        Return:
            np.array: image in bbox (resized)
        """
        return self.im_in

    def main_proc(self):
        """
        Input:
        Return:
            float: score if template matching [0.0-1.0]
            bool: If true, screw is observed in current view point
        """

        # do template matching
        method = cv2.TM_CCOEFF_NORMED
        res = cv2.matchTemplate(self.im_in, self.im_temp, method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        flag = True
        if max_val < self.threshold:
            flag = False

        return max_val, flag


"""
 Chamfer Matching
 Input:
   temp: template, 1ch numpy array
   scene: input scene, 1 ch numpy array
"""


class ChamferMatching:
    def __init__(self, temp, scene):
        self.im_temp = temp.copy()  # template
        self.im_in = scene.copy()  # scene

        self.temp_size = np.asarray(self.im_temp.shape)  # size of template
        self.in_size = np.asarray(self.im_in.shape)  # size of scene
        self.s_map = np.zeros(self.in_size - self.temp_size + 1)

        # apply edge detector
        self.im_temp_edge = cv2.Canny(self.im_temp, 50, 100)
        self.im_in_edge = cv2.Canny(self.im_in, 50, 100)

        # apply distance transform
        in_edge_not = np.logical_not(self.im_in_edge)
        in_edge_not = np.asarray(in_edge_not, np.uint8)
        self.im_in_dt = cv2.distanceTransform(in_edge_not, cv2.DIST_L2, 5)

        temp_edge_not = np.logical_not(self.im_temp_edge)
        temp_edge_not = np.asarray(temp_edge_not, np.uint8)
        self.im_temp_dt = cv2.distanceTransform(temp_edge_not, cv2.DIST_L2, 5)

        self.top_left = (0, 0)  # left top pixel of detected target (x,y)

    def main_proc(self):

        # match template
        for j in range(self.s_map.shape[0]):
            for i in range(self.s_map.shape[1]):
                im_in_dt_crop = self.im_in_dt[
                    j : j + self.temp_size[0], i : i + self.temp_size[1]
                ]
                im_in_edge_crop = self.im_in_edge[
                    j : j + self.temp_size[0], i : i + self.temp_size[1]
                ]
                score1 = self.edge_similarity(self.im_temp_edge, im_in_dt_crop)
                score2 = self.edge_similarity(im_in_edge_crop, self.im_temp_dt)
                self.s_map[j, i] = (score1 + score2) / 2

        idx = np.unravel_index(np.argmin(self.s_map), self.s_map.shape)
        self.top_left = (idx[1], idx[0])

        return self.s_map

    def edge_similarity(self, im1, im2):
        """compute per pixel similaity"""
        size = im1.shape[0] * im1.shape[1]
        score = np.sum(im1 * im2) / size
        return score

    def get_result(self):

        return self.top_left

    def get_result_image(self):

        bottom_right = (
            self.top_left[0] + self.temp_size[1],
            self.top_left[1] + self.temp_size[0],
        )
        im_res = self.im_in.copy()
        im_res = cv2.rectangle(im_res, self.top_left, bottom_right, 255, 2)
        return im_res


##########################################################################
#  sample code:
#    bbox = [300,50,200,250]   # (x,y,w,h) ... bbox of search area
#    im_in = cv2.imread( im_name, 0 )      # input image
#    im_t1 = cv2.imread( "shaft_w_hole.png", 0 )  # template with hole
#    im_t2 = cv2.imread( "shaft_wo_hole.png", 0 ) # template without hole
#    im_temps = [im_t1, im_t2]
#    shd = ShaftHoleDetection( im_in, im_temps, bbox )
#    flag = shd.main_proc()  # if True hole is observed in im_in
##########################################################################
class ShaftHoleDetection:
    def __init__(self, shaft_w_hole_file, shaft_wo_hole_file, bbox=(0, 0, 640, 480)):
        """shaft hole detection
        Input:
            im_in(np.array): input image
            im_temp(list): a list of template images.
                           first one is w hole, second one is wo hole.
            bbox(tuple): bbox consits of (x,y,w,h)
        """
        # set default parameters
        self.ds_rate = 0.5

        self.bbox = bbox

        # read template
        self.im_t_w_hole = cv2.imread(shaft_w_hole_file, 0)
        self.im_t_wo_hole = cv2.imread(shaft_wo_hole_file, 0)

        # resize image
        self.im_t_w_hole = cv2.resize(
            self.im_t_w_hole, None, fx=self.ds_rate, fy=self.ds_rate
        )
        self.im_t_wo_hole = cv2.resize(
            self.im_t_wo_hole, None, fx=self.ds_rate, fy=self.ds_rate
        )

        # scores
        self.score_w_hole = 0.0
        self.score_wo_hole = 0.0

    def main_proc(self, im_in):
        """Chamfer Matching based approach
        Input:
        Return:
            float: score if template matching [0.0-1.0]
            bool: If true, hole is observed in current view point
        """

        im_vis = im_in.copy()

        # Crop image
        im_vis = im_vis[
            self.bbox[1] : self.bbox[1] + self.bbox[3],
            self.bbox[0] : self.bbox[0] + self.bbox[2],
        ]
        im_vis = cv2.resize(im_vis, None, fx=self.ds_rate, fy=self.ds_rate)

        # Evaluate image
        self.cm_w = ChamferMatching(self.im_t_w_hole, im_vis)
        self.cm_wo = ChamferMatching(self.im_t_wo_hole, im_vis)
        smap_w = self.cm_w.main_proc()
        smap_wo = self.cm_wo.main_proc()
        self.score_w_hole = np.min(smap_w)
        self.score_wo_hole = np.min(smap_wo)

        visible_hole = False
        if self.score_w_hole < self.score_wo_hole:
            visible_hole = True

        im_vis = self.cm_w.get_result_image()
        im_vis = cv2.cvtColor(im_vis, cv2.COLOR_GRAY2RGB)
        # Draw result
        if visible_hole:
            # text = "Shaft hole seen! (" + str(self.score_w_hole) + " < " + str(self.score_wo_hole) + ")"
            text = "OK!"
            im_vis = cv2.putText(
                im_vis, text, (1, 13), 0, 0.5, (255, 255, 255), 2, cv2.LINE_AA
            )
            im_vis = cv2.putText(
                im_vis, text, (1, 13), 0, 0.5, (0, 255, 0), 1, cv2.LINE_AA
            )
        else:
            # text = "Hole on other side! (" + str(self.score_w_hole) + " > " + str(self.score_wo_hole) + ")"
            text = "X!"
            im_vis = cv2.putText(
                im_vis, text, (1, 13), 0, 0.5, (255, 255, 255), 2, cv2.LINE_AA
            )
            im_vis = cv2.putText(
                im_vis, text, (1, 13), 0, 0.5, (0, 0, 255), 1, cv2.LINE_AA
            )

        return visible_hole, im_vis


#############################################################
# MotorOrientation class: sample code
#
# # Object detection
# ssd_results, im_vis = ssd_detection.object_detection(im_in, im_vis, 0.3, 0.8)
#
# # Orientation analysis
# mo = MotorOrientation()
# orientation = mo.main_proc(im_in, ssd_results)
#
# orientation is cable direction in degree
# # False: motor was not detected
#
# if orientation is not False:
#    print(orientation)
#    plt.imshow(mo.get_im_vis(im_vis))
# else:
#    print("no motor")
#############################################################


class MotorOrientation:
    def __init__(self):
        """Calculation of motor's orientation code"""
        self.cable_orientation = 0.0
        self.im_roi = list()
        self.th = 0.5

    def main_proc(self, im_in, ssd_results):
        """Detect motor orientation code
        Input:
            im_in(np.array): input image. 3ch
            ssd_results(list): list of ssd_result
        """
        # get motor bbox
        class_list = [d.get("class") for d in ssd_results]
        if (4 in class_list) is False:
            return False
        motor_idx = class_list.index(4)  # motor id is 4.
        bbox = ssd_results[motor_idx]["bbox"]
        self.bb_x, self.bb_y, self.bb_w, self.bb_h = bbox[0], bbox[1], bbox[2], bbox[3]

        im_red = self.get_red_mask(im_in, self.th)

        im_w = im_red.shape[1]
        im_h = im_red.shape[0]
        # x and y coordinate of roi surrounding bbox.
        bb_h2 = int(self.bb_h / 2)
        bb_w2 = int(self.bb_w / 2)
        cx, cy = self.bb_x + bb_w2, self.bb_y + bb_h2
        scale = 2.0
        vbb_w, vbb_h = scale * self.bb_w, scale * self.bb_h
        vbb_step_x, vbb_step_y = int(vbb_w / 3), int(vbb_h / 3)
        vbb_w2, vbb_h2 = int(vbb_w / 2), int(vbb_h / 2)

        rx = np.array(
            [
                cx - vbb_w2,
                cx - vbb_w2 + vbb_step_x,
                cx - vbb_w2 + (vbb_step_x) * 2,
                cx - vbb_w2 + (vbb_step_x) * 3,
            ]
        )
        ry = np.array(
            [
                cy - vbb_h2,
                cy - vbb_h2 + vbb_step_y,
                cy - vbb_h2 + (vbb_step_y) * 2,
                cy - vbb_h2 + (vbb_step_y) * 3,
            ]
        )

        self.vboxes = [
            (rx[0], ry[0]),
            (rx[1], ry[1]),
            (rx[1], ry[0]),
            (rx[2], ry[1]),
            (rx[2], ry[0]),
            (rx[3], ry[1]),
            (rx[0], ry[1]),
            (rx[1], ry[2]),
            (rx[2], ry[1]),
            (rx[3], ry[2]),
            (rx[0], ry[2]),
            (rx[1], ry[3]),
            (rx[1], ry[2]),
            (rx[2], ry[3]),
            (rx[2], ry[2]),
            (rx[3], ry[3]),
        ]

        max_dist = np.linalg.norm((cx - self.vboxes[0][0], cy - self.vboxes[0][1]))
        self.im_dist = self.calc_dist_map(im_red, cx, cy, max_dist)
        im_red = im_red * self.im_dist
        self.dist_map = im_red

        # clip
        rx = np.clip(rx, 0, im_w - 1)
        ry = np.clip(ry, 0, im_h - 1)

        # Crop RoI
        self.im_roi.append(im_red[ry[1] : ry[2], rx[2] : rx[3]].copy())  # 0
        self.im_roi.append(im_red[ry[2] : ry[3], rx[2] : rx[3]].copy())  # 1
        self.im_roi.append(im_red[ry[2] : ry[3], rx[1] : rx[2]].copy())  # 2
        self.im_roi.append(im_red[ry[2] : ry[3], rx[0] : rx[1]].copy())  # 3
        self.im_roi.append(im_red[ry[1] : ry[2], rx[0] : rx[1]].copy())  # 4
        self.im_roi.append(im_red[ry[0] : ry[1], rx[0] : rx[1]].copy())  # 5
        self.im_roi.append(im_red[ry[0] : ry[1], rx[1] : rx[2]].copy())  # 6
        self.im_roi.append(im_red[ry[0] : ry[1], rx[2] : rx[3]].copy())  # 7

        self.n_red = list()
        for roi in self.im_roi:
            # Get red mask
            self.n_red.append(np.sum(roi))

        orientation_code = np.argmax(self.n_red)
        self.cable_orientation = orientation_code * 45.0

        return self.cable_orientation

    def split_hsv(self, im_in):
        im_hsv = cv2.cvtColor(im_in, cv2.COLOR_BGR2HSV)
        im_h = im_hsv[:, :, 0].copy()
        im_s = im_hsv[:, :, 1].copy()
        im_v = im_hsv[:, :, 2].copy()
        return im_h, im_s, im_v

    def set_th(self, th):
        self.th = th

    def get_red_mask(self, im_in, th=0.5):
        """redが優勢なチャンネルを取り出す
        redとgreen, redとblueのチャンネルを比較して，
        redが際立って大きいかどうかを調べる．
        """
        im_r = im_in[:, :, 2].astype(np.float)
        im_g = im_in[:, :, 1].astype(np.float)
        im_b = im_in[:, :, 0].astype(np.float)

        e = 0.1  # epsilon
        rb = im_b / (im_r + e)
        rg = im_g / (im_r + e)

        rb2 = np.where(rb < th, 1, 0)
        rg2 = np.where(rg < th, 1, 0)
        im_red_mask = np.logical_and(rb2, rg2)

        return im_red_mask  # 1 means red pixel

    def calc_dist_map(self, img, cx, cy, max_dist):
        im_dist = np.zeros(img.shape)
        for j in range(img.shape[0]):
            for i in range(img.shape[1]):
                if img[j, i]:
                    dist = np.linalg.norm((cx - i, cy - j))
                    im_dist[j, i] = (max_dist - dist) / max_dist
        return im_dist

    def get_im_vis(self, im_in):
        center = (self.bb_x + int(self.bb_w / 2), self.bb_y + int(self.bb_h / 2))
        off = np.array([[50], [0]])
        radian = np.radians(self.cable_orientation)
        rot = np.array(
            [[np.cos(radian), -np.sin(radian)], [np.sin(radian), np.cos(radian)]]
        )
        rot_off = np.dot(rot, off).astype(np.int)

        im_vis = im_in.copy()
        for i in range(8):
            im_vis = cv2.rectangle(
                im_vis, self.vboxes[2 * i], self.vboxes[2 * i + 1], (255, 255, 255), 2
            )
            im_vis = cv2.rectangle(
                im_vis, self.vboxes[2 * i], self.vboxes[2 * i + 1], (255, 0, 0), 1
            )
        im_vis = cv2.arrowedLine(
            im_vis,
            center,
            (center[0] + rot_off[0], center[1] + rot_off[1]),
            (0, 255, 0),
            3,
            cv2.LINE_AA,
            tipLength=0.3,
        )

        text = "Orientation: " + str(self.cable_orientation)
        im_vis = cv2.putText(
            im_vis, text, (20, 20), 0, 0.5, (255, 255, 255), 2, cv2.LINE_AA
        )
        im_vis = cv2.putText(
            im_vis, text, (20, 20), 0, 0.5, (255, 0, 0), 1, cv2.LINE_AA
        )

        return im_vis


#####################################################
# Sample code
#
# # generate hole mask
# hole_positions = np.array([[274,164],
#                           [320,164],
#                           [342,204],
#                           [320,243],
#                           [274,243],
#                           [250,204] ])
#
# shc = ScrewHoleCheck( hole_positions )
# idx = shc.main_proc( imgs )
#
# Note:
#  you can check hole posisions by call
#    img_vis = shc.get_visualization()
#  also you can get result image of specific image id
#    img_vis = shc.get_visualization_idx( id )
#####################################################


class ScrewHoleCheck_:
    def __init__(self, hole_positions, radius=5):
        """ScrewHoleCheck
        Args:
          hole_positions(np.araay): hole positions in image coordinate (nx2)
                                    [[x,y],[x,y],...,[x,y]]
          radius(int): radius of hole in pixel
        """
        self.hp = hole_positions
        self.radius = radius  # hole radius in pixel

        self.min_idx = 0  # image index

    def create_hole_mask(self, size):
        """create binary hole mask"""
        im_mask = np.zeros(size)
        for p in self.hp:
            im_mask = cv2.circle(im_mask, (p[0], p[1]), self.radius, 1, -1)

        return im_mask

    def main_proc(self, imgs):
        """
        Args:
          imgs(list): a list of images whlie rotating parts
        """
        # Convert RGB2Gray
        self.imgs = list()
        for im in imgs:
            if im.ndim == 3:
                im = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
            self.imgs.append(im)

        # Create hole mask
        self.im_hole_mask = self.create_hole_mask(self.imgs[0].shape)

        # Check intensity value of holes
        self.pixel_values = list()
        for i, img in enumerate(self.imgs):
            iv = np.sum(img * self.im_hole_mask)
            self.pixel_values.append(iv)

        # detect image index of min intensity values
        self.pixel_values = np.asarray(self.pixel_values)
        self.min_idx = np.argmin(self.pixel_values)

        return self.min_idx

    def get_visualization(self):
        """get result image"""

        im_vis = self.get_visualization_idx(self.min_idx)
        return im_vis

    def get_visualization_idx(self, idx):
        """get result image for specific id"""

        im_vis = cv2.cvtColor(self.imgs[idx], cv2.cv2.COLOR_GRAY2BGR)
        im_vis = im_vis.astype(np.float)
        im_vis[:, :, 1] += +self.im_hole_mask * 100
        im_vis = np.clip(im_vis, 0, 255)
        im_vis = im_vis.astype(np.uint8)

        # visualize hole positions
        for i, p in enumerate(self.hp):
            # close to hole
            text = "id:" + str(i)
            cv2.putText(
                im_vis, text, (p[0] + 10, p[1]), 1, 1.0, (255, 255, 255), 2, cv2.LINE_AA
            )
            cv2.putText(
                im_vis, text, (p[0] + 10, p[1]), 1, 1.0, (255, 0, 0), 1, cv2.LINE_AA
            )
            # left top in image
            text = "id:" + str(i) + "(" + str(p[0]) + "," + str(p[1]) + ")"
            cv2.putText(
                im_vis, text, (10, 60 + 25 * i), 1, 1.0, (255, 255, 255), 2, cv2.LINE_AA
            )
            cv2.putText(
                im_vis, text, (10, 60 + 25 * i), 1, 1.0, (255, 0, 0), 1, cv2.LINE_AA
            )

        if idx == self.min_idx:
            text = "image id:" + str(idx) + "...OK"
            cv2.putText(im_vis, text, (10, 30), 1, 1.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(im_vis, text, (10, 30), 1, 1.5, (0, 150, 0), 1, cv2.LINE_AA)
        else:
            text = "image id:" + str(idx) + "...X"
            cv2.putText(im_vis, text, (10, 30), 1, 1.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(im_vis, text, (10, 30), 1, 1.5, (0, 0, 255), 1, cv2.LINE_AA)

        return im_vis


#####################################################
# Sample code
#
# radius = 46
# scd = ScrewHoleDetector( img, radius )
# hole_centers = scd.main_proc()
#
# plt.imshow( scd.get_visualization() )
#####################################################


class ScrewHoleDetector:
    def __init__(self, im_in, c_radius, hole_min_radius=3, hole_max_radius=10):
        """Screw holes detector
        This class assumes that screw holes arranged in a circle
        Args:
            im_in(np.array): input image (3ch)
            c_radius(int): circle radius (not a hole radius ) in pixel
            hole_min_radius(int): hole min radius to be detected
            hole_max_radius(int): hole max radius to be detected

        """
        if im_in.ndim == 3:
            im_in = cv2.cvtColor(im_in, cv2.COLOR_RGB2GRAY)
        self.im_in = im_in.copy()

        self.c_radius = c_radius
        self.hole_min_radius = hole_min_radius
        self.hole_max_radius = hole_max_radius

        self.cand_circles = cv2.HoughCircles(
            self.im_in,
            cv2.HOUGH_GRADIENT,
            dp=2,
            minDist=2 * self.hole_max_radius,  # 検出される円の中心同士の最小距離．
            param1=200,  # Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値
            param2=20,  # 円の中心を検出する際の投票数の閾値
            minRadius=self.hole_min_radius,  # 円の半径の最小値
            maxRadius=self.hole_max_radius,  # 円の半径の最大値
        )

        self.im_score = np.zeros(self.im_in.shape)
        self.cand_centers = np.asarray(self.cand_circles[0, :, :2])
        self.valid_centers = None  # final result. circle positions(x,y)

    def main_proc(self):
        """
        Args:
        Return:
          n screw hole position in pixels (n,2).
          position is (x,y)
        """
        # circle image shape of (n_circles, imy, imx, n_circles)
        im_circles = np.zeros(
            (self.cand_centers.shape[0], self.im_in.shape[0], self.im_in.shape[1])
        )

        for i, c in enumerate(self.cand_centers):
            cv2.circle(im_circles[i], (c[0], c[1]), self.c_radius, 1, 2)

        for im in im_circles:
            self.im_score += im

        # get maximum coordinate
        loc = np.unravel_index(self.im_score.argmax(), self.im_score.shape)

        valid_circles = list()
        for i in range(self.cand_centers.shape[0]):
            if im_circles[i, loc[0], loc[1]] == 1:
                valid_circles.append(self.cand_centers[i])
        self.valid_circles = np.asarray(valid_circles)

        return self.valid_circles.astype(np.int)

    def get_visualization(self):

        im_vis = cv2.cvtColor(self.im_in, cv2.COLOR_GRAY2BGR)
        # draw_candidate circles
        circles = np.uint16(np.around(self.cand_circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(im_vis, (i[0], i[1]), i[2], (255, 0, 0), 1)
            # draw the center of the circle
            cv2.circle(im_vis, (i[0], i[1]), 2, (255, 0, 0), 2)

        # draw valid circles
        for c in self.valid_circles:
            cv2.circle(im_vis, (c[0], c[1]), 5, (0, 255, 0), 2)

        return im_vis
