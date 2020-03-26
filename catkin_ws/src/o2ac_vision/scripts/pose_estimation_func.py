import cv2
import numpy as np
import math


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
    im_ds = (im_ds > 0) ==1
    
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
        self._cont_mode = cv2.RETR_EXTERNAL # contour extraction mode
        
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
        im_bb = im[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2] ]
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
        # findContours
        # for opencv 3
        im, contours, hierarchy = cv2.findContours(edges, self._cont_mode, cv2.CHAIN_APPROX_NONE)
        # for opencv 4
        # contours, hierarchy = cv2.findContours(edges, self._cont_mode, cv2.CHAIN_APPROX_NONE)
        self._im_edge = edges
        
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
        hist = hist/sum
        
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
    
    def get_orientation( self ):
        
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
        self._ltop_c = (self._s_size-self._t_size)/2 # left-top pixel of an initial search position
        
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
            pad_size = diff_max/2+1
            self._scene = np.pad( self._scene, (pad_size, pad_size), 'minimum')
            self._initial_pad = np.array( (pad_size, pad_size), np.int )
    
    
    def padding( self, im, param ):
        pad_size = max( param ) # define padding size
        im_pad = np.pad( im, (pad_size, pad_size), 'edge')
        return im_pad, pad_size
    
    def main_proc( self ):
        
        # padding
        pad_scene, pad_size = self.padding( self._scene, self._region )

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
        res = np.argmax( self._s_map )
        res_offset = self._offset_list[res] - self._initial_pad
        center_pixel = res_offset+self._t_size/2
        return center_pixel, res_offset, np.max(self._s_map)
    
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

    # rotate template
    rows, cols = im_temp.shape
    # getRotationMatrix2D() takes clock-wise orientation, so we need convert ccw -> cw
    rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-orientation,1 )
    res_im_temp =  cv2.warpAffine( im_temp, rot_mat, (cols, rows) )
    
    # padding input scene
    res_pad = int(res_im_temp.shape[0]/2) # padding of result image
    im_res_on_original = np.pad( im_scene.copy(), (res_pad,res_pad), "constant")
    ltop_pad = ltop + res_pad


    bbox_size = np.asarray( im_temp.shape, np.int )
    bb_center = ltop_pad + bbox_size/2
    

    # mapping edge pixels of template
    im_temp_edge_rot_vis = cv2.cvtColor(res_im_temp, cv2.COLOR_GRAY2BGR )
    im_temp_edge_rot_vis[:,:,1] = 0
    im_temp_edge_rot_vis[:,:,2] = 0
    im_res_on_original = cv2.cvtColor( im_res_on_original, cv2.COLOR_GRAY2BGR )
    im_res_on_original[ ltop_pad[0]:ltop_pad[0]+bbox_size[0],  ltop_pad[1]: ltop_pad[1]+bbox_size[1] ] += im_temp_edge_rot_vis

    # draw bounding box
    im_res_on_original = cv2.rectangle( im_res_on_original, ( ltop_pad[1],  ltop_pad[0]), (ltop_pad[1]+bbox_size[1], ltop_pad[0]+bbox_size[0]), (0,255,0), 3 )
    # draw center position
    im_res_on_original = cv2.circle( im_res_on_original, (bb_center[1], bb_center[0] ), 5, (0,255,0), -1, cv2.LINE_AA )
    # draw orientation
    line_length = int( bbox_size[0]/2 )
    deg2rad = math.pi/180.0
    pt2 = (int(bb_center[1]+line_length*math.cos(orientation*deg2rad)), int(bb_center[0]+line_length*math.sin(orientation*deg2rad)) )
    im_res_on_original = cv2.arrowedLine( im_res_on_original, (bb_center[1], bb_center[0] ), pt2, (0,255,0), 2, cv2.LINE_AA )
    
    return im_res_on_original