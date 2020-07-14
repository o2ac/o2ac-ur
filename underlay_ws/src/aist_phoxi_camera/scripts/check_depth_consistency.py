#!/usr/bin/env python

import argparse, rospy, numpy as np, message_filters, ros_numpy, cv2
from sensor_msgs import msg as smsg

class CloudChecker(object):
    def __init__(self, camera_name):
        super(CloudChecker, self).__init__()
        rospy.init_node('CloudChecker', anonymous=True)

        camera_info_msg = rospy.wait_for_message("/" + camera_name
                                                     + "/camera_info",
                                                 smsg.CameraInfo, timeout=10.0)
        self._K = np.array(camera_info_msg.K).reshape(3, 3)
        self._D = np.array(camera_info_msg.D)
        pointcloud_sub = message_filters.Subscriber("/" + camera_name
                                                        + "/pointcloud",
                                                    smsg.PointCloud2)
        depth_map_sub  = message_filters.Subscriber("/" + camera_name
                                                        + "/depth_map",
                                                    smsg.Image)
        self._ts = message_filters.TimeSynchronizer((pointcloud_sub,
                                                     depth_map_sub), 10)
        self._ts.registerCallback(self.callback)

    def callback(self, pointcloud_msg, depth_map_msg):
        # print("\n*** pointcloud/header ***\n" + str(pointcloud_msg.header))
        # print("\n*** depth_map/header ***\n"  + str(depth_map_msg.header))
        pointcloud = self.numpify_pointcloud(pointcloud_msg)
        depth_map  = ros_numpy.numpify(depth_map_msg)
        max_err    = self.check_depth_consistency(depth_map, pointcloud)
        print("max. err = {}".format(max_err))

    def numpify_pointcloud(self, pointcloud_msg):
        pc = ros_numpy.numpify(pointcloud_msg)
        pointcloud          = np.zeros((pc.shape[0], pc.shape[1], 3))
        pointcloud[:, :, 0] = pc['x']
        pointcloud[:, :, 1] = pc['y']
        pointcloud[:, :, 2] = pc['z']
        return pointcloud

    def check_depth_consistency(self, depth_map, pointcloud):
        max_err = 0
        for v in range(depth_map.shape[0]):
            for u in range(depth_map.shape[1]):
                d = depth_map[v, u]
                p = pointcloud[v, u, :]
                if not np.isnan(p[2]):
                    if d != p[2]:
                        print("({}, {}): depth({}) != z({})".format(u, v, d,
                                                                    p[2]))
                    xyz = self.back_project_point(u, v, d)
                    err = np.linalg.norm(p - xyz)
                    if err > max_err:
                        max_err = err
        return max_err

    def back_project_point(self, u, v, d):
        xy = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float32),
                                 self._K, self._D)[0, 0]
        return np.array((xy[0]*d, xy[1]*d, d))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check consistency of data")
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default='a_phoxi_m_camera',
                        type=str,
                        choices=None,
                        help='camera_name',
                        metavar=None)
    args = parser.parse_args()

    node = CloudChecker(args.camera_name)
    rospy.spin()
