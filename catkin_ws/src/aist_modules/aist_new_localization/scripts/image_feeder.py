#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
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
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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
# Author: Toshio Ueshiba
#
import os, sys, glob, rospy, json, cv2
from cv_bridge              import CvBridge
from sensor_msgs            import msg as smsg
from geometry_msgs          import msg as gmsg
from aist_depth_filter      import DepthFilterClient
from aist_new_localization  import LocalizationClient
from aist_model_spawner     import ModelSpawnerClient

#########################################################################
#  class ImageFeeder                                                    #
#########################################################################
class ImageFeeder(object):
    _Models = ('01-BASE',                       # 1
               '03-PANEL2',                     # 2
               '02-PANEL',                      # 3
               '04_37D-GEARMOTOR-50-70',        # 4
               '11_MBRAC60-2-10',               # 5
               '07_SBARB6200ZZ_30',             # 6
               '13_MBGA30-2',                   # 7
               '13_MBGA30-2',                   # 8
               '05_MBRFA30-2-P6',               # 9
               '14_BGPSL6-9-L30-F8',            # 10
               '08_KZAF1075NA4WA55GA20AA0',     # 11
               '06_MBT4-400',                   # 12
               '09_EDCS10',                     # 13
               '09_EDCS10',                     # 14
               '12_CLBUS6-9-9.5',               # 15
               '10_CLBPS10_17_4'                # 16
              )
    _Colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0),
               (255, 255, 0), (255, 0, 255), (0, 255, 255))

    def __init__(self, data_dir):
        super(ImageFeeder, self).__init__()

        self._data_dir = data_dir
        self._nposes   = rospy.get_param('~nposes',  1)
        self._timeout  = rospy.get_param('~timeout', 30)

        # Load camera intrinsics
        filename = rospy.get_param('~intrinsic', 'realsense_intrinsic.json')
        with open(self._data_dir + '/' + filename) as f:
            try:
                intrinsic = json.loads(f.read())
            except Exception as e:
                rospy.logerr('(Feeder) %s', str(e))

        Kt = intrinsic['intrinsic_matrix']
        K  = [Kt[0], Kt[3], Kt[6], Kt[1], Kt[4], Kt[7], Kt[2], Kt[5], Kt[8]]
        self._cinfo                  = smsg.CameraInfo()
        self._cinfo.header.frame_id  = rospy.get_param('~camera_frame', 'map')
        self._cinfo.height           = intrinsic['height']
        self._cinfo.width            = intrinsic['width']
        self._cinfo.distortion_model = 'plumb_bob'
        self._cinfo.D         = [0, 0, 0, 0, 0]
        self._cinfo.K         = K
        self._cinfo.R         = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self._cinfo.P         = K[0:3] + [0] + K[3:6] + [0] + K[6:9] + [0]
        self._cinfo.binning_x = 0
        self._cinfo.binning_y = 0

        self._cinfo_pub = rospy.Publisher('~camera_info',
                                          smsg.CameraInfo, queue_size=1)
        self._image_pub = rospy.Publisher('~image', smsg.Image, queue_size=1)
        self._depth_pub = rospy.Publisher('~depth', smsg.Image, queue_size=1)

        self._dfilter   = DepthFilterClient('depth_filter')
        self._localizer = LocalizationClient('localization')
        self._spawner   = ModelSpawnerClient('model_spawner')

    def load_and_localize(self, annotation_filename):
        self._spawner.delete_all()
        try:
            f = open(annotation_filename)
            annotation = json.loads(f.read())
            ids    = annotation['class_id']
            bboxes = annotation['bbox']

            img   = cv2.imread(self._data_dir + '/Annotations/' +
                               annotation['img_path'], cv2.IMREAD_UNCHANGED)
            for id, bbox in zip(ids, bboxes):
                self.draw_bbox(img, id, bbox)
            image = CvBridge().cv2_to_imgmsg(img, encoding='passthrough')

            dimg  = cv2.imread(self._data_dir + '/Annotations/' +
                               annotation['depth_path'], cv2.IMREAD_UNCHANGED)
            depth = CvBridge().cv2_to_imgmsg(dimg, encoding='passthrough')

        except Exception as e:
            rospy.logerr('(Feeder) %s(%s)', str(e), annotation_filename)
            return

        # Publish images and detect plane
        self._dfilter.detect_plane_send_goal()
        self.publish_images(image, depth)
        plane = self._dfilter.detect_plane_wait_for_result()

        if plane is None:
            rospy.logerr('*** (Feeder) failed to detect plane!')
            return

        for id, bbox in zip(ids, bboxes):
            rospy.loginfo('*** (Feeder) --------------')
            rospy.loginfo('*** (Feeder) localize id=%d', id + 1)

            # Set bbox to depth_filter
            self._dfilter.roi = bbox

            # Publish images and localize
            model = ImageFeeder._Models[id]
            self._localizer.send_goal(model, plane)
            self.publish_images(image, depth)
            poses = self._localizer.wait_for_result(self._timeout)

            if poses is None:
                rospy.logerr('*** (Feeder) failed to localize!')
                return

            rospy.loginfo('*** (Feeder) %d pose(s) found', len(poses.poses))
            for pose in reversed(poses.poses):
                self._spawner.add(model, gmsg.PoseStamped(poses.header, pose))

    def publish_images(self, image, depth):
        now = rospy.Time.now()
        self._cinfo.header.stamp = now
        image.header = self._cinfo.header
        depth.header = self._cinfo.header
        self._cinfo_pub.publish(self._cinfo)
        self._image_pub.publish(image)
        self._depth_pub.publish(depth)

    def draw_bbox(self, img, id, bbox):
        idx = id % len(ImageFeeder._Colors)
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                      ImageFeeder._Colors[idx], 3)
        cv2.putText(img, str(id + 1), (bbox[0] + 5, bbox[3] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, ImageFeeder._Colors[idx], 2,
                    cv2.LINE_AA)

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('~')

    data_dir = os.path.expanduser(rospy.get_param('~data_dir',
                                                  '~/data/WRS_Dataset'))
    feeder = ImageFeeder(data_dir)

    while not rospy.is_shutdown():
        datasets = ('Close', 'Far')
        for dataset in datasets:
            annotation_filenames = glob.glob(data_dir + '/Annotations/' +
                                             dataset + '/Image-wise/*.json')
            for annotation_filename in annotation_filenames:
                rospy.loginfo('*** (Feeder) ==================')
                rospy.loginfo('*** (Feeder) annotation: %s',
                              annotation_filename)
                feeder.load_and_localize(annotation_filename)
                if raw_input('Hit return key >> '.format(id)) == 'q':
                    sys.exit()
