#!/usr/bin/env python
import rospy
import tf
from aruco_msgs.msg import MarkerArray

rospy.init_node("publish_aruco_as_tf")

def callback(marker_array):
    pose = marker_array.markers[0].pose.pose
    ref_frame = marker_array.markers[0].header.frame_id

    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w)
    br = tf.TransformBroadcaster()
    br.sendTransform(position, orientation,
        rospy.Time.now(), 'ar_marker', ref_frame)

sub = rospy.Subscriber(
    "/aruco_marker_publisher/markers", MarkerArray, callback
)

while not rospy.is_shutdown():
    rospy.spin()
