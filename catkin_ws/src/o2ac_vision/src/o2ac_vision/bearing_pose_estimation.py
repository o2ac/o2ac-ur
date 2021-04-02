#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import numpy.linalg as LA
import math
import copy
import os, json
import open3d as o3d
import math
from math import tau  # = 2*pi = one turn. https://tauday.com/tau-manifesto
from o2ac_vision.common_3d_func import centering, rpy2mat, mat2rpy


#############################################################################
#
# Bearing pose estimation
#
# Sample code
#    be = BearingPoseEstimator( im_temp, img, bbox )
#    d_rotate, translation = be.main_proc( threshold=3.0, ds=3.0 )
#    im_vis = be.get_result_image()
#
#############################################################################

class ICPRegistration:
    """Full scratch implementation of ICP algorithm."""
    
    def __init__( self, pcd_s, pcd_t ):
        self.pcd_s = copy.deepcopy(pcd_s) # source
        self.pcd_t = copy.deepcopy(pcd_t) # target
        self.n_points = len(self.pcd_s.points)
        
        self.pcds = []
        self.d = []
        self.final_trans = np.identity(4)
        
        # Initialization
        ## Registration vector
        self.q = np.array([1.,0.,0.,0.,0.,0.,0.])
        
        # parameters
        self.distance_tolerance = 0.01
        self.max_iterations = 500
        
        # Final result
        self.final_transformation = np.identity(4)
        
        self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd_t)
        self.np_pcd_t = np.asarray(self.pcd_t.points)
    
    def set_distance_tolerance( self, d ):
        self.distance_tolerance = d
        
    def set_max_iterations( self, n ):
        self.max_iterations = n
        
    def rho( self, x ):
        delta = 5
        return x**2/(delta+x**2)
        
    def closest_points( self ):
        idx_list = []
        distance = []
        for i in range(self.n_points):
            [k, idx, d] = self.pcd_tree.search_knn_vector_3d(self.pcd_s.points[i], 1)
            idx_list.append(idx[0])
            distance.append(d[0])
            # M-estimater
            #distance.append( d[0]*self.rho(np.sqrt(d[0])) )

        np_pcd_y = self.np_pcd_t[idx_list]
        self.d.append( np.sqrt(np.mean(np.array(distance))) )
        return np_pcd_y.copy()
    
    def compute_registration_param( self, np_pcd_y ):
        # get center
        mu_s = self.pcd_s.get_center()
        mu_y = np.mean(np_pcd_y, axis=0)

        # compute cross-covariance matrix
        np_pcd_s = np.asarray(self.pcd_s.points)
        covar = np.zeros( (3,3) )
        n_points = np_pcd_s.shape[0]
        rho_sum = 0
        for i in range(n_points):
            d = LA.norm( np_pcd_s[i]-np_pcd_y[i] )
            # M-estimater
            #rho = self.rho(d)
            #rho_sum+=rho
            #covar += rho*np.dot( np_pcd_s[i].reshape(-1, 1), np_pcd_y[i].reshape(1, -1) )
            covar += np.dot( np_pcd_s[i].reshape(-1, 1), np_pcd_y[i].reshape(1, -1) )
        #covar /=rho_sum
        covar /= n_points
        covar -= np.dot( mu_s.reshape(-1,1), mu_y.reshape(1,-1) )
        
        ## anti-symmetric matrix
        A = covar - covar.T
        delta = np.array([A[1,2],A[2,0],A[0,1]])
        tr_covar = np.trace(covar)
        i3d = np.identity(3)
        
        # symmetric matrix
        Q = np.zeros((4,4))
        Q[0,0] = tr_covar
        Q[0,1:4] = delta
        Q[1:4,0] = delta
        Q[1:4,1:4] = covar + covar.T - tr_covar*i3d
        
        w, v = LA.eig(Q)
        rot = self.quaternion2rotation(v[:,np.argmax(w)])
        trans = mu_y - np.dot(rot,mu_s)
        
        self.q = np.concatenate((v[np.argmax(w)],trans))
        transform = np.identity(4)
        transform[0:3,0:3] = rot.copy()
        transform[0:3,3] = trans.copy()
        return transform
     
    def registration( self ):
        for i in range(self.max_iterations):
            np_pcd_y = self.closest_points()
            transform = self.compute_registration_param( np_pcd_y )
            self.pcd_s.transform(transform)
            
            self.final_transformation = np.dot( transform, self.final_transformation )
            self.pcds.append(copy.deepcopy(self.pcd_s))
            if (2<i) and ( 0.999 < self.d[-1]/self.d[-2] ):
                break
            if self.d[-1] < self.distance_tolerance:
                break
                
        return self.d[-1], self.final_transformation
        
    # quaternion to rotation matrix
    def quaternion2rotation( self, q ):
        rot = np.array([[q[0]**2+q[1]**2-q[2]**2-q[3]**2, 
                         2.0*(q[1]*q[2]-q[0]*q[3]), 
                         2.0*(q[1]*q[3]+q[0]*q[2])],

                        [2.0*(q[1]*q[2]+q[0]*q[3]),
                        q[0]**2+q[2]**2-q[1]**2-q[3]**2,
                         2.0*(q[2]*q[3]-q[0]*q[1])],

                        [2.0*(q[1]*q[3]-q[0]*q[2]),
                         2.0*(q[2]*q[3]+q[0]*q[1]),
                        q[0]**2+q[3]**2-q[1]**2-q[2]**2]]
                      )
        return rot
    
    
class BearingPoseEstimator:
    def __init__( self, im_s, img, bbox ):
        """
        im_temp: template image (Grayscale)
        im_t: input image (Grayscale)
        bbox: bounding box of the bearing (tuple [x,y,w,h])
        """

        # crop target image using a bounding box
        self.im_t = img[ bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2] ].copy()
        
        # generate source point cloud
        self.im_s = im_s
        self.pcd_s = self.get_pcd( im_s )
        
        # generate target point cloud
        #im_t = cv2.GaussianBlur(im_t,(5,5),0)
        self.pcd_t = self.get_pcd( self.im_t )
        
        # data
        self.trans_final = np.identity(4)
        self.mse = 100000
        self.pcds = None
        self.d = None
        
        
    def get_pcd( self, img ):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply( img )
        _canny1 = 100 #parameter1 for canny edge detection
        _canny2 = 200 #parameter2 for canny edge detection
        # Edge detection
        edges = cv2.Canny( cl1, _canny1, _canny2 )

        tmp = np.array( np.where( edges>0 ) )
        np_edge = tmp.T.copy()

        zeros = np.zeros( (np_edge.shape[0],1) ) 
        np_edge3 = np.hstack( (np_edge, zeros) )

        pcd_edge = o3d.geometry.PointCloud()
        pcd_edge.points = o3d.utility.Vector3dVector(np_edge3.astype(np.float))

        return pcd_edge

    
    def main_proc( self, threshold, ds=5.0 ):
        """
        Main function running the pose estimation.

        Arguments:
            threshold: distance threshold of registration.
                       If MSE is lower than this value,
                       transformation is returned.
            ds: downsampling rate for points. This parameter
                should be larger than "threshold"
        Returns:
            rotation:       rotation angle (CCW)
            translation:    translation in pixels (y,x)
        If pose estimation fails, (False, False) is returned
        """

        
        # Preprocessing
        ##  downsampling edge pixels
        self.pcd_s.paint_uniform_color([0.0,0.0,1.0])
        pcd_s_ds = self.pcd_s.voxel_down_sample( voxel_size=ds )
        pcd_t_ds = self.pcd_t.voxel_down_sample( voxel_size=ds )
        pcd_t_ds, center_t = centering(pcd_t_ds)
        pcd_s_ds, center_s = centering(pcd_s_ds)
        ts_c = np.identity(4)
        ts_c[:3,3] = -center_s
        tt_c = np.identity(4)
        tt_c[:3,3] = center_t
        
        # initial rotations
        init_rotations = [
            0, np.radians(22.5), np.radians(45.0), np.radians(67.5)
        ]
        for init in init_rotations:
            ##  apply initial rotation to the source point cloud
            T = rpy2mat( 0, 0, init )
            pcd_s_ds_ini = copy.deepcopy(pcd_s_ds)
            pcd_s_ds_ini.transform(T)
            
            # Registration by ICP algorithm
            reg = ICPRegistration( pcd_s_ds_ini, pcd_t_ds )
            reg.set_distance_tolerance( ds*0.5 )
            self.mse, reg_trans = reg.registration()
            if self.mse < threshold:
                """
                # check transformation progress
                hoge = copy.deepcopy(self.pcd_s)
                mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100., origin=[0.0,0.0,0.0])
                o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
                hoge.transform( ts_c )
                o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
                hoge.transform( T )
                o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
                hoge.transform( reg_trans )
                o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
                hoge.transform( tt_c )
                o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
                """
                              
                TT = np.dot( T, ts_c )
                TT = np.dot( reg_trans,TT )
                self.trans_final = np.dot( tt_c, TT )
                
                self.pcds = reg.pcds
                self.d = reg.d
                # Get registration result
                #  translation[x,y] and rotation
                _,_,rotate = mat2rpy(self.trans_final)
                translation = self.trans_final[:2,3]

                # Choose the direction that results in the smaller rotation
                if rotate > tau/8:  
                    rotate -= tau/4
                elif rotate < -tau/8:
                    rotate += tau/4
                return rotate, translation
        return False, False
            
        
    def vis_registration3d( self ):
        pcd_final = copy.deepcopy(self.pcd_s)
        pcd_final.transform(self.trans_final)
        o3d.visualization.draw_geometries( [self.pcd_t, pcd_final], width=640, height=500)
        
    def get_pcds( self ):
        return self.pcds
        
    def get_result_image( self ):
        """
        Returns the result visualization as a 3-channel image.
        """
        
        im_result = cv2.cvtColor(self.im_t, cv2.COLOR_GRAY2BGR )
        pcd_final = copy.deepcopy(self.pcd_s)

        pcd_final.transform(self.trans_final)
        np_final = np.asarray( pcd_final.points, np.int )

        for i in range(np_final.shape[0]):
            im_result = cv2.circle( im_result, (np_final[i,1],np_final[i,0]), 2, (0,255,0), -1, cv2.LINE_AA )
        
        im_result = cv2.rectangle( im_result, (5,12), (170,38), (0,0,0), -1)
        # Draw rotation in image
        _,_,rotate = mat2rpy(self.trans_final)
        d_rotate = np.degrees(rotate)
        str_rotate = format(d_rotate,'.2f')+"[deg](CW)"
        im_result = cv2.putText( im_result, str_rotate, (5,30), 1, 1.25, (255, 255, 255), 2, cv2.LINE_AA )
        im_result = cv2.putText( im_result, str_rotate, (5,30), 1, 1.25, (0, 255, 255), 1, cv2.LINE_AA )
        return im_result

"""
################################################################################################
# Sample code for In-plane pose estimation on RGB image for the motor
# 
# make perspective transformation
## source points (corner points of vgroove)
src_pts = np.array([[287,94], [470,91], [285,270], [490,265]], dtype=np.float32)
## destination points (rectified corner points)
dst_pts = np.array([[287,94], [470,91], [287,270], [470,265]], dtype=np.float32)
mat = cv2.getPerspectiveTransform(src_pts, dst_pts)

# set bounding box(x,y,w,h)
bbox = [270,180,240,240]

# read template image (use option "0")
im_temp = cv2.imread("../../../motor_front.png",0)

# define pose estimation class
pe = PoseEstimator( im_temp, img, bbox, mat )
# do registration
d_rotate, translation = pe.main_proc( threshold=5.0, ds=10.0 )
print(d_rotate, translation) # => CCW rotation (deg) is returned
im_vis = pe.get_result_image()
################################################################################################
"""
class PoseEstimator:
    """ Pose estimation on RGB image
    """
    def __init__( self, im_s, img, bbox, mat=None ):
        """
        Args:
           im_s(ndarray): source image(template) 1ch
           img(ndarray): target image(input scene) 1ch
           bbox(list): bounding box (x,y,w,h)
           mat(ndarray,optional): perspective transformation for img
        """
        
        if mat is not None:
            img = cv2.warpPerspective(img, mat, (img.shape[1], img.shape[0]))
        # crop target image using a bounding box
        self.im_t = img[ bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2] ].copy()
        
        # generate source point cloud
        self.im_s = im_s
        self.pcd_s = self.get_pcd( im_s )
        
        # generate target point cloud
        #self.im_t = cv2.GaussianBlur(self.im_t,(5,5),0)
        self.pcd_t = self.get_pcd( self.im_t )
        
        # data
        self.trans_final = np.identity(4)
        self.mse = 100000
        self.pcds = None
        self.d = None
        self.threshold = 10000
        
        
    def get_pcd( self, img ):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply( img )
        _canny1 = 100 #parameter1 for canny edge detection
        _canny2 = 200 #parameter2 for canny edge detection
        # Edge detection
        edges = cv2.Canny( cl1, _canny1, _canny2 )

        tmp = np.array( np.where( edges>0 ) )
        np_edge = tmp.T.copy()

        zeros = np.zeros( (np_edge.shape[0],1) ) 
        np_edge3 = np.hstack( (np_edge, zeros) )

        pcd_edge = o3d.geometry.PointCloud()
        pcd_edge.points = o3d.utility.Vector3dVector(np_edge3.astype(np.float))

        return pcd_edge

    
    def main_proc( self, threshold, ds=5.0 ):
        """ do pose estimation.

        Args:
          threshold: distance threshold of registration.
                     If MSE is lower than this value,
                     transformation is returned.
          ds: downsampling rate for points. This parameter
              should be larger than "threshold"
        Return:
            float: rotation angle (CCW)
            translation:  translation in pixels (y,x)
            If pose estimation fails, (False, False) is returned
        """

        self.threshold = threshold
        
        # Preprocessing
        ##  downsampling edge pixels
        self.pcd_s.paint_uniform_color([0.0,0.0,1.0])
        pcd_s_ds = self.pcd_s.voxel_down_sample( voxel_size=ds )
        pcd_t_ds = self.pcd_t.voxel_down_sample( voxel_size=ds )
        pcd_t_ds, center_t = centering(pcd_t_ds)
        pcd_s_ds, center_s = centering(pcd_s_ds)
        ts_c = np.identity(4)
        ts_c[:3,3] = -center_s
        tt_c = np.identity(4)
        tt_c[:3,3] = center_t
        
        # initial rotations
        #init_rotations = np.radians(np.arange(0,360,5))
        init_rotations = np.radians(np.arange(0,360,10))
        #init_rotations = [np.radians(150.0)]
        mses = []
        reg_transes = []
        Ts = []
        for init in init_rotations:
            
            ##  apply initial rotation to the source point cloud
            T = rpy2mat( 0, 0, init )
            pcd_s_ds_ini = copy.deepcopy(pcd_s_ds)
            pcd_s_ds_ini.transform(T)
            
            # Registration by ICP algorithm
            reg = ICPRegistration( pcd_s_ds_ini, pcd_t_ds )
            reg.set_distance_torrelance( ds*0.5 )
            mse_tmp, reg_trans_tmp = reg.registration()
            mses.append(mse_tmp)
            reg_transes.append(reg_trans_tmp)
            Ts.append(T)
            
        mses = np.asarray(mses)
        idx = np.argmin(mses)
        self.mse = mses[idx]
        reg_trans = reg_transes[idx]
                  
        if self.mse < self.threshold:
            """
            # check transformation progress
            hoge = copy.deepcopy(self.pcd_s)
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100., origin=[0.0,0.0,0.0])
            o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
            hoge.transform( ts_c )
            o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
            hoge.transform( T )
            o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
            hoge.transform( reg_trans )
            o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
            hoge.transform( tt_c )
            o3d.visualization.draw_geometries( [mesh_frame,hoge, self.pcd_t], width=640, height=500)
            """

            TT = np.dot( Ts[idx], ts_c )
            TT = np.dot( reg_trans,TT )
            self.trans_final = np.dot( tt_c, TT )
            self.pcds = reg.pcds
            self.d = reg.d
            # Get registration result
            #  translation[x,y] and rotation
            _,_,rotate = mat2rpy(self.trans_final)
            d_rotate = np.degrees(rotate)
            translation = self.trans_final[:2,3]
            return d_rotate, translation
                
            
        
        return False, False
            
        
    def vis_registration3d( self ):
        
        pcd_final = copy.deepcopy(self.pcd_s)
        pcd_final.transform(self.trans_final)
        o3d.visualization.draw_geometries( [self.pcd_t, pcd_final], width=640, height=500)
        
    def get_pcds( self ):
        return self.pcds
        
    def get_result_image( self ):
        
        im_result = cv2.cvtColor(self.im_t, cv2.COLOR_GRAY2BGR )
        pcd_final = copy.deepcopy( self.pcd_s )

        pcd_final.transform(self.trans_final)
        np_final = np.asarray( pcd_final.points, np.int )

        if self.mse < self.threshold:
            for i in range(np_final.shape[0]):
                im_result = cv2.circle( im_result, (np_final[i,1],np_final[i,0]), 2, (0,255,0), -1, cv2.LINE_AA )
        
        # Draw rotation in image
        _,_,rotate = mat2rpy(self.trans_final)
        print("trans_final", self.trans_final)
        d_rotate = np.degrees(rotate)
        str_rotate = format(d_rotate,'.2f')+"[deg](CCW), MSE:"+format(self.mse,'.2f')
        im_result = cv2.putText( im_result, str_rotate, (10,30), 1, 0.7, (255, 255, 255), 2, cv2.LINE_AA )
        if self.mse < self.threshold:
            im_result = cv2.putText( im_result, str_rotate, (10,30), 1, 0.7, (255, 0, 0), 1, cv2.LINE_AA )
        else:
            im_result = cv2.putText( im_result, str_rotate, (10,30), 1, 0.7, (0, 0, 255), 1, cv2.LINE_AA )
        return im_result
