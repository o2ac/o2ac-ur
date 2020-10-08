#!/usr/bin/env python

import os, sys, glob, rospy, re, json, pprint, skimage, cv2, threading
from cv_bridge          import CvBridge
from sensor_msgs        import msg as smsg
from aist_depth_filter  import DepthFilterClient
from aist_localization  import LocalizationClient
from aist_model_spawner import ModelSpawnerClient

#########################################################################
#  class ImageFeeder                                                    #
#########################################################################
class ImageFeeder(object):
    def __init__(self, data_dir):
        super(ImageFeeder, self).__init__()

        self._data_dir = data_dir

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

        self._image = None
        self._depth = None
        self._run   = True
        thread = threading.Thread(target=self._stream_image)
        thread.start()

    def stream_image(self, aanotation_filename):
        try:
            f = open(annotation_filename)
            annotation = json.loads(f.read())

            image = cv2.imread(self._data_dir + '/Annotations/' +
                               annotation['img_path'], cv2.IMREAD_COLOR)
            depth = cv2.imread(self._data_dir + '/Annotations/' +
                               annotation['depth_path'], cv2.IMREAD_UNCHANGED)
            self._image = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
            self._depth = CvBridge().cv2_to_imgmsg(depth)

        except Exception as e:
            rospy.logerr('(Feeder) %s(annotation = %s)',
                         str(e), annotation_filename)

    def quit(self):
        self._run = False

    def _stream_image(self):
        rate = rospy.Rate(10)       # 10Hz
        while self._run:
            if self._image and self._depth:
                now = rospy.Time.now()
                self._cinfo.header.stamp = now
                self._image.header = self._cinfo.header
                self._depth.header = self._cinfo.header
                self._cinfo_pub.publish(self._cinfo)
                self._image_pub.publish(self._image)
                self._depth_pub.publish(self._depth)
            rate.sleep()


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
                feeder.stream_image(annotation_filename)
                if raw_input('Hit return key >> ') == 'q':
                    feeder.quit()
                    sys.exit()
