#!/usr/bin/env python

import os, sys, glob, rospy, json, pprint, skimage, cv2
from cv_bridge         import CvBridge
from sensor_msgs       import msg as smsg
from aist_depth_filter import DepthFilterClient

#########################################################################
#  class ImagePublisher                                                 #
#########################################################################
class ImageFeeder(object):
    _Colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0),
               (255, 255, 0), (255, 0, 255), (0, 255, 255))

    def __init__(self, data_dir):
        super(ImageFeeder, self).__init__()

        self._data_dir = data_dir

        filename = rospy.get_param('intrinsic', 'realsense_intrinsic.json')
        with open(data_dir + '/' + filename) as f:
            try:
                intrinsic = json.loads(f.read())
            except Exception as e:
                rospy.logerr(str(e))
                return

        Kt = intrinsic['intrinsic_matrix']
        K  = [Kt[0], Kt[3], Kt[6], Kt[1], Kt[4], Kt[7], Kt[2], Kt[5], Kt[8]]
        self._cinfo                  = smsg.CameraInfo()
        self._cinfo.header.frame_id  = rospy.get_param('frame', 'map')
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

        self._dfilter   = DepthFilterClient('~depth_filter')
        self._dfilter.window_radius = 2

    def load_and_publish_images(self, annotation_filename):
        try:
            f = open(annotation_filename)
            annotation = json.loads(f.read())
            ids    = annotation['class_id']
            bboxes = annotation['bbox']

            image  = cv2.imread(self._data_dir + '/Annotations/' +
                                annotation['img_path'], cv2.IMREAD_UNCHANGED)
            for id, bbox in zip(ids, bboxes):
                self.draw_bbox(image, id, bbox)
            imgmsg = CvBridge().cv2_to_imgmsg(image, encoding='passthrough')

            depth  = cv2.imread(self._data_dir + '/Annotations/' +
                                annotation['depth_path'], cv2.IMREAD_UNCHANGED)
            dptmsg = CvBridge().cv2_to_imgmsg(depth, encoding='passthrough')

        except Exception as e:
            rospy.logerr('%s(%s)', str(e), annotation_filename)
            return

        for bbox in bboxes:
            self._dfilter.roi = bbox
            now = rospy.Time.now()
            self._cinfo.header.stamp = now
            imgmsg.header = self._cinfo.header
            dptmsg.header = self._cinfo.header
            self._cinfo_pub.publish(self._cinfo)
            self._image_pub.publish(imgmsg)
            self._depth_pub.publish(dptmsg)
            if raw_input('  Hit return key >> ') == 'q':
                sys.exit()

    def draw_bbox(self, image, id, bbox):
        colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0),
                  (255, 255, 0), (255, 0, 255), (0, 255, 255))
        color_idx = id % len(colors)
        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]),
                      colors[color_idx], 3)
        cv2.putText(image, str(id),
                    ((bbox[0] + bbox[2])/2, (bbox[1] + bbox[3])/2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, colors[color_idx], 2,
                    cv2.LINE_AA)

#########################################################################
#  main                                                                 #
#########################################################################
if __name__ == '__main__':

    rospy.init_node('~')

    data_dir = os.path.expanduser(rospy.get_param('data_dir',
                                                  '~/data/WRS_Dataset'))
    feeder   = ImageFeeder(data_dir)

    while not rospy.is_shutdown():
        datasets = ('Close', 'Far')
        for dataset in datasets:
            annotation_filenames = glob.glob(data_dir + '/Annotations/' +
                                             dataset + '/Image-wise/*.json')
            for annotation_filename in annotation_filenames:
                rospy.loginfo(annotation_filename)
                feeder.load_and_publish_images(annotation_filename)
