
import copy
import rospy
import visualization_msgs.msg

from o2ac_assembly_database.parts_reader import PartsReader
from std_msgs.msg import ColorRGBA

class MarkersScene():
    def __init__(self, listener):
        self.listener = listener
        self.marker_publisher = rospy.Publisher("o2ac_assembly_markers", visualization_msgs.msg.Marker, queue_size = 100)
        self.parts_database = PartsReader("wrs_assembly_2021", load_meshes=False) # TODO this should be a param somewhere 
        self.published_items = {}

    def spawn_item(self, item_name, pose_stamped, attach=False, color=None):
        color = color if color else ColorRGBA(0.2, 1.0, 0.2, 0.9) # GREEN
        item_marker = self.parts_database.get_visualization_marker(item_name, pose_stamped.pose, pose_stamped.header.frame_id, color, frame_locked=attach)
        self.marker_publisher.publish(item_marker)
        self.published_items.update({item_name: copy.deepcopy(pose_stamped)})

    def despawn_item(self, item_name):
        marker = visualization_msgs.msg.Marker()
        marker.id = self.parts_database.name_to_id(item_name)
        marker.action = marker.DELETE
        try:
            self.marker_publisher.publish(marker)
            del self.published_items[item_name]
        except Exception as e:
            rospy.logerr("Try to delete item: %s but it does not exist" % item_name)

    def attach_item(self, item_name, to_link):
        current_pose = self.published_items[item_name]
        current_pose.header.stamp = rospy.Time(0)
        try:
            self.listener.waitForTransform(to_link, current_pose.header.frame_id, current_pose.header.stamp, rospy.Duration(1))
        except:
            return False
        new_pose = self.listener.transformPose(to_link, current_pose)
        color = ColorRGBA(1., 0.0, 1., 0.9) # Purple
        self.spawn_item(item_name, new_pose, attach=True, color=color)

    def detach_item(self, item_name):
        current_pose = self.published_items.get(item_name, None)
        if current_pose:
            new_pose = self.listener.transformPose("world", current_pose)
            self.spawn_item(item_name, new_pose, attach=False)
        else:
            rospy.logerr("Try to detach item: %s but it does not exist" % item_name)

    def delete_all(self):
        marker = visualization_msgs.msg.Marker()
        marker.action = marker.DELETEALL
        self.marker_publisher.publish(marker)
        self.published_items = {}
