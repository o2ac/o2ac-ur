# Roatation estimator
# Shuichi Akizuki, Chukyo Univ.
# Email: s-akizuki@sist.chukyo-u.ac.jp
import cv2
import numpy as np
import argparse
import math
from pose_estimation_func import *

def get_argumets():
    """
        Parse arguments from command line
    """

    parser = argparse.ArgumentParser( description='RGBDImage2PCD')
    parser.add_argument('--cimg', type=str, default='data/rgb.png',
                        help='file name of the RGB image of the input scene.')
    parser.add_argument('--timg', type=str, default='data/temp.png',
                        help='file name of the template image.')

    return parser.parse_args()



if __name__ == "__main__":

    """Get arguments"""
    args = get_argumets()
    im_c = cv2.imread( args.cimg, 0 )
    im_temp = cv2.imread( args.timg, 0 )
    bbox = [101, 382, 223, 73]

    """Compute orientation of the target"""
    re = RotationEstimation( im_c, bbox )
    ori = re.get_orientation()
    im_edge = re.get_im_edge()
    im_edge_ds = downsampling_binary( im_edge )

    """Compute orientation of the template"""
    re2 = RotationEstimation( im_temp, [0,0,im_temp.shape[1], im_temp.shape[1]] )
    im_res2 = re2.get_im_edge()
    im_temp_edge = np.zeros([220,220], np.uint8 )
    im_temp_edge[95:125, 0:220] =  im_res2
    temp_ori = re2.get_orientation()

    # Rotation of template image
    rows, cols = im_temp_edge.shape
    res_orientations = list() # orientation in degree, ccw
    for o in ori:
        res_orientation = ori[0]-temp_ori[0]
        res_orientation_flip = res_orientation + 180
        res_orientations.append( res_orientation )
        res_orientations.append( res_orientation_flip )

    im_temp_edge_rots = list()
    for res_ori in res_orientations:
        # getRotationMatrix2D( center, angle(ccw), scale )
        rot_mat = cv2.getRotationMatrix2D( (cols/2, rows/2), 360-res_ori,1 )
        im_temp_edge_rots.append( cv2.warpAffine( im_temp_edge, rot_mat, (cols, rows) ) )

    im_temp_eds = list()
    for im in im_temp_edge_rots:
        im_temp_eds.append( downsampling_binary( im ) )

    """ Binary Template Matching """
    scores = list() # score list of each template
    offsets = list() # offset list of each template
    for temp in im_temp_eds: # Apply template matching
        scene = im_edge_ds
        btm = BinaryTemplateMatching( temp, scene, (10,10) )
        _, offset, score = btm.get_result()
        scores.append( score )
        offsets.append( offset )

    # Get ID of the most similar template 
    res_idx = np.argmax( np.asarray( scores ) )

    # Up scale
    offset_original = offsets[res_idx]*2

    # compute center coordinate
    ltop = np.asarray( [ bbox[1]+offset_original[0], bbox[0]+offset_original[1] ], np.int )
    temp_center = np.asarray( im_temp_edge.shape, np.int )/2
    center = ltop + temp_center

    # DEBUG save result
    print "Result: rotation [deg(ccw)], center [j,i]", res_orientations[res_idx], center
    im_res_on_original = visualize_result( im_temp_edge, im_c, ltop, res_orientations[res_idx] )
    cv2.imwrite( "pose_estimation.png", im_res_on_original )