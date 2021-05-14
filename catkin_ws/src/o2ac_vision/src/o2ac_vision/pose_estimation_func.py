#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import copy
import os, json
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
def downsampling_binary( im, _fx=0.5, _fy=0.5 ):
    im_ds = cv2.resize( im, None, fx=_fx, fy=_fy, interpolation=cv2.INTER_AREA )
    im_ds = np.clip(im_ds,0,1)

    return im_ds.copy()

"""
 In-plane Rotation Estimation using Distribution of Line Segment Directions
"""
class RotationEstimation():
    def __init__( self, im, bbox, lseg_len=15, n_bin=90 ):
        # Parameter for LSDH
        self._lseg_len = lseg_len # length of line segment
        self._n_bin = n_bin # number of histogram's bin
        self._step = int(self._lseg_len/2) # sampling step of line segment

        # Parameter for image processing
        self._canny1 = 100 #parameter1 for canny edge detection
        self._canny2 = 200 #parameter2 for canny edge detection
        self._continuous_streaming_mode = cv2.RETR_EXTERNAL # contour extraction mode

        # Variables
        self._deg2rad = math.pi/180.0
        self._im =  im # input image
        self._bbox = bbox # bounding box. list[x,y,w,h]
        self._im_bb = self.im_crop_bbox( self._im, self._bbox ) # image of bounding box
        self._im_edge = None
        self._contours = None # contours


        # Do main processing
        self._iprs = self.main_proc() # in-plane rotations (in degree)

    """
    Crop image
    """
    def im_crop_bbox( self, im, bbox ):
        im_bb = im[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
        im_out = im_bb.copy()
        return im_out

    """
    Contour detection from image
    """
    def contour_detection( self ):

        im_bb = self.im_crop_bbox( self._im, self._bbox )
        # Histogram normalization
        # create a CLAHE object (Arguments are optional).
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply( self._im_bb )

        # Edge detection
        edges = cv2.Canny( cl1, self._canny1, self._canny2 )
        #cv2.imshow("cl1",cl1)
        #cv2.waitKey(0)
        #cv2.imshow("edges",edges)
        #cv2.waitKey(0)
        self._im_edge = edges

        # findContours
        # check version of cv2
        cv2_versoin = cv2.__version__
        if '4' == cv2_versoin[0]:
            contours, hierarchy = cv2.findContours(edges, self._continuous_streaming_mode, cv2.CHAIN_APPROX_NONE)
        else:
            _, contours, hierarchy = cv2.findContours(edges, self._continuous_streaming_mode, cv2.CHAIN_APPROX_NONE)


        return contours


    """
    Make a Line Segment Direction Histogram from a list of contour pixels
    """
    def make_LSDH( self, cont ):

        i = 0
        hist = np.zeros(self._n_bin, np.float)
        while( i < len(cont)-self._lseg_len):
            # get line segment
            lseg = cont[i+self._lseg_len][0] - cont[i][0]

            # compute direction of line segment
            if lseg[0] != 0:
                dir = (180.0*math.atan( float(lseg[1])/float(lseg[0]) )/math.pi)
            else:
                dir = 90.0
            if dir < 0.0:
                dir += 180.0
            bin = int(dir / (180.0/self._n_bin))
            # Vote to the histogram
            hist[bin]+=1.0
            if bin == self._n_bin-1:
                hist[0]+=0.5
                hist[bin-1]+=0.5
            elif bin == 0:
                hist[self._n_bin-1]+=0.5
                hist[1]+=0.5
            else:
                hist[bin-1]+=0.5
                hist[bin+1]+=0.5

            i+= self._step

        return hist

    """
    Compute dominant orientation from histogram
    """
    def compute_orientation( self, hist ):

        # Normalize histogram
        sum = np.sum(hist)
        hist = hist/(sum+0.00001)

        # detect peak
        vote_max_bin = np.argmax(hist)

        # Convert the bin index to the orientation in degree
        in_plane_rotation = vote_max_bin*180.0/self._n_bin
        return [in_plane_rotation]

    """
    Main processing
    """
    def main_proc( self ):
        # detect contours from the bounding box image.
        self._contours = self.contour_detection()

        # make a Line Segment Direction Histogram (LSDH)
        lsdh = np.zeros( self._n_bin, np.float )
        for cont in self._contours:
            hist = self.make_LSDH( cont )
            lsdh += hist

            im_res = self.get_im_bb()
            im_res = cv2.cvtColor(im_res, cv2.COLOR_GRAY2BGR )
        # compute domitant orientation of LSDH
        orientation = self.compute_orientation( lsdh )
        return orientation

    def get_im_bb( self ):
        return self._im_bb.copy()

    def get_im_edge( self ):
        return self._im_edge.copy()

    def get_im_contours( self ):
        im_bb3 = cv2.cvtColor( self._im_bb.copy(), cv2.COLOR_GRAY2BGR )
        im_cont = cv2.drawContours( im_bb3, self._contours, -1, (0,255,0), 1)
        return im_cont

    def get_result_image( self ):

        im_res = self.get_im_bb()
        im_res = cv2.cvtColor(im_res, cv2.COLOR_GRAY2BGR )
        pt1 = ( int(self._bbox[2]/2), int(self._bbox[3]/2) )

        cv2.circle( im_res, pt1, 3, (0,255,0), -1, cv2.LINE_AA )
        line_length = int( min(self._bbox[2:])/2 )
        for rot in self._iprs:
            pt2 = (int(pt1[0]+line_length*math.cos(rot*self._deg2rad)), int(pt1[1]+line_length*math.sin(rot*self._deg2rad)) )
            cv2.arrowedLine( im_res, pt1, pt2, (0,255,0), 1, cv2.LINE_AA )

        return im_res

    """
      Get orientation
      Input:
        flip: if true, detected orientatios and fliped orientations are returned.
      Return;
        list of orientations
    """
    def get_orientation( self, flip=True ):

        orientations = copy.deepcopy( self._iprs )
        if flip is True:
            for ori in orientations:
                self._iprs.append( ori+180.0 )

        return self._iprs



"""
 Binary Template Matching
 Input:
   temp: template, 1ch numpy array
   scene: input scene, 1 ch numpy array
   region: search region (+-search_y, +-search_x)
"""
class BinaryTemplateMatching():
    def __init__( self, temp, scene, region=(0,0) ):
        self._temp = temp.copy() # template
        self._scene = scene.copy() # scene
        self._region = region # search region
        self._initial_pad = np.array( [0,0], np.int )

        self.initial_padding()
        self._offset_ini_pad = np.array( (self._initial_pad[0], self._initial_pad[0]), np.int )

        self._t_size = np.asarray( self._temp.shape ) # size of template
        self._s_size = np.asarray( self._scene.shape ) # size of scene
        self._ltop_c = ((self._s_size-self._t_size)/2).astype(np.int) # left-top pixel of an initial search position

        # Matching, compute score map, _s_map, and offset list
        self._s_map, self._offset_list = self.main_proc()

    """
      If the size of scene is smaller than that of template, zero padding is applied.
    """
    def initial_padding( self ):
        s_shape = self._scene.shape
        t_shape = self._temp.shape
        diff_size_y = t_shape[0] - s_shape[0]
        diff_size_x = t_shape[1] - s_shape[1]

        diff_max = max(diff_size_y, diff_size_x)
        if diff_max > 0:
            pad_size = int(diff_max/2)+1
            self._scene = np.pad( self._scene, (pad_size, pad_size), 'minimum')
            self._initial_pad = np.array( (pad_size, pad_size), np.int )


    def padding( self, im, param ):
        pad_size = max( param ) # define padding size
        im_pad = np.pad( im, (pad_size, pad_size), 'edge')
        return im_pad, pad_size

    def main_proc( self ):

        # padding
        pad_scene, pad_size = self.padding( self._scene, self._region )
        pad_scene = (pad_scene >0 )
        pad_scene = np.logical_not(pad_scene)
        pad_scene = np.asarray( pad_scene, np.uint8 )
        pad_scene = cv2.distanceTransform( pad_scene, cv2.DIST_L2, 5 )

        # score map
        s_map = np.zeros( [self._region[0]*2+1, self._region[1]*2+1] )
        offset_list = list() # offset_list[n]+t_size/2 = center pixel
        for sj, j in enumerate(range( -self._region[0], self._region[0]+1 )):
            for si, i in enumerate( range( -self._region[1], self._region[1]+1 ) ):
                offset = self._ltop_c + np.array( [  j, i ] ) + pad_size
                s_map[ sj, si ] = np.sum( self._temp * pad_scene[ offset[0]:offset[0]+self._t_size[0], offset[1]:offset[1]+self._t_size[1] ] )
                offset_list.append( offset-pad_size )

        return s_map, offset_list


    # return the location in scene coordinate system and its score
    def get_result( self ):
        res = np.argmin( self._s_map )
        res_offset = self._offset_list[res] - self._initial_pad
        center_pixel = res_offset+self._t_size/2
        return center_pixel, res_offset, np.min(self._s_map)

    def get_result_image( self, _offset ):
        offset = _offset.copy()
        pad_scene, pad_size = self.padding( self._scene, self._region )
        im_res =  np.zeros( pad_scene.shape )
        offset += pad_size + self._initial_pad
        im_res[ offset[0]:offset[0]+self._t_size[0], offset[1]:offset[1]+self._t_size[1] ] = self._temp
        im_res += pad_scene
        return im_res

    def get_score_map( self ):

        return self._s_map

class TemplateMatching():
    def __init__( self, im_c, ds_rate, temp_root, temp_info_name="template_info.json" ):
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
        temp_info_fullpath = os.path.join( temp_root, temp_info_name )
        if os.path.isfile( temp_info_fullpath ):
            json_open = open( temp_info_fullpath, 'r' )
            self.temp_info = json.load( json_open )
        else:
            print("ERROR!!")
            print( temp_info_fullpath, " couldn't read.")
            exit()

    def read_template( self, result ):
        """
        Input:
            result: Output of the object detection(SSD). list of {bbox, class, confidence}
        Output:
            im_temp_edge: edge_template
            temp_ori: the dominant orientations of template
        """
        class_id = result["class"]
        info_id = -1
        for n, info in enumerate( self.temp_info ):
            if result["class"] in info.values():
                info_id = n
                break

        if info_id is -1:
            print("ERROR!!")
            print("Template info does not include id ",  class_id )

        #print(os.path.join( self.temp_root, self.temp_info[info_id]["name_edge"]))
        im_temp_edge = cv2.imread( os.path.join( self.temp_root, self.temp_info[info_id]["name_edge"]), 0 )
        temp_ori = self.temp_info[info_id]["orientation"]

        return im_temp_edge, temp_ori

    def compute( self, result, search=(20,20) ):
        """
        Input:
            result: Output of the object detection(SSD). list of {bbox, class, confidence}
        Output:
            center: center coordinate of target. array(j,i)
            orientation: in-plane rotation of target. float
        """

        class_id = result["class"]
        bbox = result["bbox"]
        re = RotationEstimation( self.im_c, bbox, lseg_len=20, n_bin=120 )
        in_ori = re.get_orientation()
        im_edge = re.get_im_edge()
        im_edge_ds = downsampling_binary( im_edge, _fx=self.ds_rate, _fy=self.ds_rate )

        """Read orientation of the template"""
        im_temp_edge, temp_ori = self.read_template( result )

        # Rotation of template image
        rows, cols = im_temp_edge.shape
        res_orientations = list() # orientation in degree, ccw
        for o_in in in_ori:
            for o_temp in temp_ori:
                res_orientations.append( o_in - o_temp )

        # print("difference of orientations")
        # print( res_orientations )


        """ Create rotated templates """
        im_temp_edge_rots = list()
        im_temp_eds = list()
        for res_ori in res_orientations:
            # getRotationMatrix2D( center, angle(ccw), scale )
            rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-res_ori,1 )
            im_rot = cv2.warpAffine( im_temp_edge, rot_mat, (cols, rows) )
            im_temp_edge_rots.append( im_rot )
            im_temp_eds.append( downsampling_binary( im_rot, self.ds_rate, self.ds_rate) )

        """ Binary Template Matching """
        scores = list() # score list of each template
        offsets = list() # offset list of each template
        for temp in im_temp_eds: # Apply template matching
            btm = BinaryTemplateMatching( temp, im_edge_ds, search )
            _, offset, score = btm.get_result()
            scores.append( score )
            offsets.append( offset )

        # Get ID of the most similar template
        res_idx = np.argmin( np.asarray( scores ) )

        # Up scale
        offset_original = offsets[res_idx]* 1.0/self.ds_rate

        # compute center coordinate
        ltop = np.asarray( [ bbox[1]+offset_original[0], bbox[0]+offset_original[1] ], np.int )
        temp_center = np.asarray( im_temp_edge.shape, np.int )/2
        center = ltop + temp_center
        orientation = res_orientations[res_idx]

        return center, orientation # array[j, i], float

    def get_result_image( self, result, res_ori, res_center, im_scene=None ):
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

        im_temp_edge, temp_ori = self.read_template( result )
        temp_center = np.asarray( im_temp_edge.shape, np.int )/2
        ltop = res_center - temp_center
        im_res_on_original = visualize_result( im_temp_edge, im_scene, ltop, res_ori )
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
def visualize_result( im_temp, im_scene, ltop, orientation ):

    # dilation for good visualization
    kernel = np.ones((3,3),np.uint8)
    im_temp = cv2.dilate( im_temp, kernel, iterations = 1 )

    # rotate template
    rows, cols = im_temp.shape
    # getRotationMatrix2D() takes clock-wise orientation, so we need convert ccw -> cw
    rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-orientation,1 )
    res_im_temp =  cv2.warpAffine( im_temp, rot_mat, (cols, rows) )

    # padding input scene
    res_pad = int(res_im_temp.shape[0]) # padding of result image
    if im_scene.ndim == 2:
        im_res_on_original = np.pad( im_scene.copy(), (res_pad,res_pad), "constant")
    else:
        im_res_on_original = np.pad( im_scene.copy(), ((res_pad,res_pad),(res_pad,res_pad),(0,0)), "constant")
    ltop_pad = (ltop + res_pad).astype(np.int)

    bbox_size = np.asarray( im_temp.shape, np.int )
    bb_center = (ltop_pad + bbox_size/2).astype(np.int)
    
    # mapping edge pixels of template
    im_temp_edge_rot_vis = cv2.cvtColor(res_im_temp, cv2.COLOR_GRAY2BGR )
    im_temp_edge_rot_vis[:,:,1:3] = 0
    if im_res_on_original.ndim != 3:
        im_res_on_original = cv2.cvtColor( im_res_on_original, cv2.COLOR_GRAY2BGR )

    im_temp_edge_rot_vis = np.asarray( im_temp_edge_rot_vis, np.int )
    im_res_on_original = np.asarray( im_res_on_original, np.int )
    im_res_on_original[ ltop_pad[0]:ltop_pad[0]+bbox_size[0],  ltop_pad[1]: ltop_pad[1]+bbox_size[1] ] += im_temp_edge_rot_vis
    im_res_on_original = np.clip( im_res_on_original, 0, 255 )
    im_res_on_original = np.asarray( im_res_on_original, np.uint8 )

    # draw bounding box
    im_res_on_original = cv2.rectangle( im_res_on_original, ( ltop_pad[1],  ltop_pad[0]), (ltop_pad[1]+bbox_size[1], ltop_pad[0]+bbox_size[0]), (0,255,255), 3 )
    # draw center position
    im_res_on_original = cv2.circle( im_res_on_original, (bb_center[1], bb_center[0] ), 5, (0,255,255), -1, cv2.LINE_AA )
    # draw orientation
    line_length = int( bbox_size[0]/2 )
    deg2rad = math.pi/180.0
    pt2 = (int(bb_center[1]+line_length*math.cos(orientation*deg2rad)), int(bb_center[0]+line_length*math.sin(orientation*deg2rad)) )
    im_res_on_original = cv2.arrowedLine( im_res_on_original, (bb_center[1], bb_center[0] ), pt2, (0,255,255), 2, cv2.LINE_AA )

    im_res_on_original = im_res_on_original[res_pad:-res_pad, res_pad:-res_pad]

    return im_res_on_original

"""
  Fast Graspability Evaluation on RGB image


"""
class FastGraspabilityEvaluation():

    def __init__( self, _im_in, _im_hand, _param ):
        self.im_in = _im_in.copy()
        self.im_hand = _im_hand.copy()
        self.im_in_org = _im_in

        self.im_belt = None    # belt = 1, other = 0
        self.fg_mask = None  # foreground = 1, background = 0

        self.im_hands = None

        self.candidate_idx = list()
        self.pos_list = list()
        self.score_list = list()
        
        self.gp_result = list() # grasp points y, x, theta[deg]

        #parameter
        self.ds_rate = _param["ds_rate"]
        self.n_grasp_point = _param["n_grasp_point"]
        self.threshold = _param["threshold"]

        # Down sampling
        self.im_in = cv2.resize( self.im_in, None, fx=self.ds_rate, fy=self.ds_rate, interpolation=cv2.INTER_NEAREST )
        self.im_hand = cv2.resize( self.im_hand, None, fx=self.ds_rate, fy=self.ds_rate, interpolation=cv2.INTER_NEAREST )

        # Masking out the belt and foreground
        self.im_belt = self.detect_belt( self.im_in )
        self.fg_mask = self.detect_foreground( self.im_in )


    def detect_belt( self, im_in ):
        """ Detect belt mask
        Input:
            im_in(numpy array BGR, 8bits): input image
        Return:
            binarized image [0 or 1] np.uint8
        """
        # detect foreground
        im_hsv = cv2.cvtColor( im_in, cv2.COLOR_BGR2HSV )
        im_s = im_hsv[:,:,1] # get s image
        th, im_b = cv2.threshold(im_s,0,1,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # detect the belt as circle
        circles = cv2.HoughCircles( im_s, 
                            cv2.HOUGH_GRADIENT,
                            dp=2,  
                            minDist=10, #検出される円の中心同士の最小距離．
                            param1=100, #Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値
                            param2=100, #円の中心を検出する際の投票数の閾値
                            minRadius=10,  #円の半径の最小値
                            maxRadius=100) #円の半径の最大値
        
        im_circle = np.zeros(im_in.shape)
        cv2.circle(im_circle,(circles[0,0,0],circles[0,0,1]),circles[0,0,2],(1,1,1),5)
        circle_mask = im_circle[:,:,0]
        return circle_mask*im_b

    def detect_foreground( self, im_in, factor=1.5 ):
        """ Detect foreground mask
        Input:
            im_in(numpy array BGR, 8bits): input image
            factor(float): しきい値．最瀕値のfactor倍の画素値までを背景とする．
                           大きくすると，検出領域が小さくなる
        Return:
            binarized image [0 or 1] np.uint8
        """
        im_hsv = cv2.cvtColor( im_in, cv2.COLOR_BGR2HSV )
        im_v = im_hsv[:,:,2] # get v image
        hist, bins = np.histogram(im_v, 20 )
        threshold = bins[np.argmax(hist)] # ヒストグラムの最瀕値の画素値を取得
        im_th = np.where( im_v< threshold*1.5, 0, 1 ) # 最瀕値の1.5倍の明るさまでを背景とする．

        return im_th

    def RotationTemplate( self, im_temp, deg ):

        cols, rows = im_temp.shape[1], im_temp.shape[0]

        rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2),  360-deg, 1 )
        im_rot = cv2.warpAffine( im_temp, rot_mat, (cols, rows) )

        return im_rot, np.sum(im_rot)
    
    def RotationColorTemplate( self, im_temp, rotation ):
        # convert binary template to color template.
        # thumb is colored by red
        temp_c = copy.deepcopy(im_temp)
        cols, rows = im_temp.shape[1], im_temp.shape[0]
        
        temp_c = np.clip( temp_c*100.0, 0, 255 )
        temp_c = np.asarray( temp_c, np.uint8 )
        temp_c = cv2.cvtColor( temp_c, cv2.COLOR_GRAY2BGR)
        temp_c[:,:int(cols/2),0] = temp_c[:,:int(cols/2),2] = 0
        temp_c[:,int(cols/2):,0] = temp_c[:,int(cols/2):,1] = 0
            
        rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2),  360-rotation, 1 )
        im_rot = cv2.warpAffine( temp_c, rot_mat, (cols, rows) )

        return im_rot


    def main_proc( self ):
        """ compute grasp points
        Return:
            list: grasp point in image coordinate system (y, x, theta(degree,CCW))
        """

        # Generate search point
        points = np.where( self.im_belt == 1 )
        n_search_point = self.n_grasp_point
        search_points = list()
        for n in range(len(points[0])):
            search_points.append( (points[0][n], points[1][n]) )

        if n_search_point < len(search_points): # random sampling
            search_points = random.sample(search_points, n_search_point )
        
        search_points = np.asarray( search_points, np.int )

        # Hand pattern matching
        cols, rows = self.im_hand.shape[1], self.im_hand.shape[0]
        s_cols, s_rows = self.im_belt.shape[1], self.im_belt.shape[0]
        offset = np.array( (cols/2, rows/2), np.int )
        score_list = list()
        pos_list = list()
        cx, cy = self.im_in.shape[1]/2, self.im_in.shape[0]/2
        for sp in search_points:
            ltop = sp - offset # left-top corner of search point
            deg = np.degrees(np.arctan2((sp[0]-cy),(sp[1]-cx)))

            hand, hand_area = self.RotationTemplate(self.im_hand, deg )
            
            # check border
            if ltop[0] < 0 or ltop[1] < 0 or s_rows <= ltop[0]+rows or s_cols <= ltop[1]+cols:
                # print("skip")
                continue

            conv_hand = self.fg_mask[ ltop[0]:ltop[0]+rows, ltop[1]:ltop[1]+cols ] * hand
            score_hand = np.sum(conv_hand) / hand_area

            self.score_list.append( score_hand )
            self.pos_list.append( (int(sp[0]/self.ds_rate), int(sp[1]/self.ds_rate), deg) ) # position and orientation 

        self.score_list = np.asarray( self.score_list )
        # Candidate selection
        self.candidate_idx = np.where( self.score_list < self.threshold )[0]

        # Make final result
        for n in self.candidate_idx:
            grasp_point = np.array( [self.pos_list[n][0], self.pos_list[n][1]], np.float )
            rotation = self.pos_list[n][2]
            self.gp_result.append( (int(grasp_point[0]), int(grasp_point[1]), rotation ) )

        return self.gp_result


    def get_foreground_mask( self ):

        im_fg = self.fg_mask.copy()
        im_fg = np.asarray( im_fg*255, np.uint8 )

        return im_fg


    def get_im_belt( self ):

        im_out = self.im_belt.copy()
        im_out = np.asarray( im_out*255, np.uint8 )

        return im_out

    def visualization( self, im_result=None ):
        if im_result is None:
            im_result = self.im_in_org.copy()

        # Overlay hand templates
        im_result = np.asarray( im_result, np.int )
        cols, rows = self.im_hand.shape[1], self.im_hand.shape[0]
        offset = np.array( (cols/2, rows/2), np.int )
        for n in self.gp_result:
            pos = np.array( (n[0], n[1]) )
            im_hand_c = self.RotationColorTemplate(self.im_hand, n[2])
            ltop = pos - offset
            im_result[ ltop[0]:ltop[0]+rows, ltop[1]:ltop[1]+cols ] += im_hand_c

        im_result = np.clip( im_result, 0, 255 )
        im_result = np.asarray( im_result, np.uint8 )
        for n in self.gp_result:
            im_result = cv2.circle( im_result, ( n[1], n[0] ), 3, (0,255,0), -1, cv2.LINE_AA )

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
class PickCheck():
    """ Verify that the grasp was successful

    Args:
        ssd_detection: ssd_detection
      
    """
    def __init__( self, ssd_detection ):
        self.ssd_detection = ssd_detection
        self.ssd_results = None
        self.im_vis = None

    def check( self, im_in, class_id, ssd_treshold=0.6 ):
        """ verify grasp

        Args:
            im_in(np.array): input image
            class_id(int): target classs to be verified
            ssd_threshold(float, optional): threshold of ssd confidence
        Return:
            bool: If True, grasp was successful.
        """
        flag = False
        im_vis = im_in.copy()
        self.ssd_results, self.im_vis = self.ssd_detection.object_detection(im_in, 
                                                                  im_vis, 
                                                                  ssd_treshold, 
                                                                  0.8)

        for res in self.ssd_results:
            if res["class"] == class_id:
                flag = True
        
        return flag

    def get_ssd_results( self ):
        return self.ssd_results

    
    def get_im_result( self ):
        return self.im_vis
        
#############################################################################
#
# Notch part detection
#
# sample code:
#  img = input image (1ch gray scale)
#  bbox = [350,100,100,400] # ROI(x,y,w,h) of shaft area
#  temp_root = rospack.get_path("wrs_dataset") + "/data/templates_shaft"
#  sa = ShaftAnalysis( imgs, bbox, temp_root )
#  (front, back) = sa.main_proc( 0.5 ) # threshold.
#
#  If notch is seen in image, front or back are true (front is at the top).
#
# for visualization:
#  ## visualization of template matching
#  plt.imshow( sa.get_tm_result_image() )
#  ## visualization of shaft intensity difference for top and bottom region
#  diff_top, diff_bottom = sa.get_diff_list()
#  plt.plot(diff_bottom)
#  plt.plot(diff_top)
#############################################################################

class ShaftAnalysis():
    # img: input image
    # bbox: bounding box [x,y,w,h] of shaft region
    # temp_root: path to templates_shaft
    def __init__( self, img, bbox, temp_root ):
        
        self.info = {"class":8, "bbox":bbox}
        temp_info_name = "template_info.json"
        
        # downsampling rate
        ds_rate = 1.0/2.0
        tm = TemplateMatching( img, ds_rate, temp_root, temp_info_name  )
        im_temp = cv2.imread( temp_root+"/8.png",0)
        

        # template matching
        self.center, self.ori = tm.compute(self.info )
        self.im_tm_result = tm.get_result_image(self.info, self.ori, 
                                                np.array(self.center, np.int))
        
        rows, cols = im_temp.shape
        rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-self.ori,1 )
        res_im_temp =  cv2.warpAffine( im_temp, rot_mat, (cols, rows) )
        res_im_temp = np.clip(res_im_temp, 0, 1)


        ltop = np.array(self.center - np.array([rows,cols])/2, np.int)
        im_scene_mask = np.zeros(img.shape)
        im_scene_mask[ ltop[0]:ltop[0]+rows,  ltop[1]:ltop[1]+cols ] = res_im_temp

        self.im_crop = copy.deepcopy( (img*im_scene_mask)[ ltop[0]:ltop[0]+rows, ltop[1]:ltop[1]+cols ] )
        self.im_crop = np.array( self.im_crop, np.uint8)
        rot_mat_rev = cv2.getRotationMatrix2D( (cols/2, rows/2), self.ori,1 )
        self.im_crop =  cv2.warpAffine( self.im_crop, rot_mat_rev, (cols, rows) )
        self.invalid_value = 255
        
        # variables for debuging
        ## a list of intensity difference
        self.diff_list = list()
        ## mean intensity difference
        self.diff_top = None  
        self.diff_bottom = None
    
    # make a list consists of line[d+1]-line[d]
    def difference( self, line ):
        line = np.array(line, np.float)
        diff = list()
        for d in range(len(line)-1):
            if line[d]==0:
                diff.append(self.invalid_value)
            else:
                diff.append(line[d+1]-line[d])

        diff = np.abs(diff)
        return diff

    # Counts the values in the list that are below the threshold. 
    def count_constant( self, diff_list, th=10 ):

        max_constant = 0
        for i in range( len(diff_list)-1 ):
            l = 1
            n_constant = 0
            while(i+l <len(diff_list)-1):
                if diff_list[i+l] < th:
                    n_constant += 1
                else:
                    break
                l+=1
            if max_constant < n_constant:
                max_constant = n_constant

        return max_constant
    
    def count_valid_elements( self, diff_list ):
        
        ve = 0
        for d in diff_list:
            if d != self.invalid_value:
                ve += 1
        return ve
    
    # Input:
    #  th: Percentage of notch area in level slice of shaft area．(default: 0.5)
    #  top_range: 　　y axis range to be checked（top area） 
    #  bottom_range:　y axis range to be checked（bottom area） 
    # Output:
    #  notch_seen_at_top: True if notch is detected at front (top).
    #  notch_seen_at_bottom: True if notch is detected at back (bottom).
    def main_proc(self, th, top_range=[30,90], bottom_range=[270,330]):
        
        nmc_list = list()
        for i in range(self.im_crop.shape[0]):
            ft_img = np.array(self.im_crop, np.float)
            diff = self.difference( self.im_crop[i,:] )
            self.diff_list.append(diff)
            nmc = self.count_constant(diff, 10)
            ve = self.count_valid_elements( diff )
            nmc_list.append(nmc/(ve+0.0001))
        
        
        dt = self.diff_list[int((top_range[0]+top_range[1])/2)]
        db = self.diff_list[int((bottom_range[0]+bottom_range[1])/2)]
        self.diff_top = dt[dt < 255]
        self.diff_bottom = db[db < 255]
        
        top_mean = np.mean(nmc_list[top_range[0]:top_range[1]] )
        bottom_mean = np.mean(nmc_list[bottom_range[0]:bottom_range[1]] )
        #print(top_mean, bottom_mean)
        
        return th<top_mean, th<bottom_mean
    
    
    def get_frontized_image(self):
        
        return copy.deepcopy(self.im_crop)
    
    def get_orientation(self):
        return self.ori
        
    def get_tm_result_image(self, front_found=False, back_found=False):
        """
        Draws the areas that were evaluated with the color indicating the result
        """
        top_color = (0,0,255)
        bottom_color = (0,0,255)
        if front_found:
            top_color = (0,255,50)
        if back_found:
            bottom_color = (0,255,50)

        x_offset = self.info["bbox"][0]
        width = self.info["bbox"][3]
        y_offset = self.info["bbox"][1]

        top_range=[30,90]
        bottom_range=[270,330]
        dt = self.diff_list[int((top_range[0]+top_range[1])/2)]
        db = self.diff_list[int((bottom_range[0]+bottom_range[1])/2)]
        im_res = copy.deepcopy(self.im_tm_result)
        im_res = cv2.rectangle( im_res, ( 10+x_offset,  top_range[0]+y_offset), (width+x_offset-20, top_range[1]+y_offset), top_color, 3 )
        im_res = cv2.rectangle( im_res, ( 10+x_offset,  bottom_range[0]+y_offset), (width+x_offset-20, bottom_range[1]+y_offset), bottom_color, 3 )
        return im_res
    
    def get_diff_list( self ):
        
        return self.diff_top, self.diff_bottom
