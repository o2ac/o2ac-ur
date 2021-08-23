#include "o2ac_pose_distribution_updater/test_tools.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"

void broadcast_gripper_pose(const std::string &frame_id,
                            const ros::Time &current_time,
                            const geometry_msgs::Pose &pose) {
  static tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "world";
  transform_stamped.header.stamp = current_time;
  transform_stamped.child_frame_id = frame_id;
  msg_pose_to_msg_transform(pose, transform_stamped.transform);
  broadcaster.sendTransform(transform_stamped);
}

void send_pose_belief(
    ros::ServiceClient &visualizer_client,
    const moveit_msgs::CollisionObject &object,
    const unsigned char &distribution_type, const double &lifetime,
    const geometry_msgs::PoseWithCovarianceStamped &distribution,
    const bool frame_locked) {
  o2ac_msgs::visualizePoseBelief pose_belief;
  pose_belief.request.object = object;
  pose_belief.request.distribution_type = distribution_type;
  pose_belief.request.distribution = distribution;
  pose_belief.request.lifetime = ros::Duration(lifetime);
  pose_belief.request.frame_locked = frame_locked;
  visualizer_client.call(pose_belief);
}

void load_CollisionObject_from_file(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::string &file_path) {
  object = std::shared_ptr<moveit_msgs::CollisionObject>(
      new moveit_msgs::CollisionObject());
  object->pose = to_Pose(0., 0., 0., 1., 0., 0., 0.);
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  read_stl_from_file_path(file_path, vertices, triangles);
  for (auto &vertex : vertices) {
    vertex /= 1000.0; // milimeter -> meter
  }
  add_mesh_to_CollisionObject(object, vertices, triangles,
                              Eigen::Isometry3d::Identity());
}
