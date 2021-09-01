#include "o2ac_pose_distribution_updater/estimator.hpp"
#include "o2ac_pose_distribution_updater/planner_helpers.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "print_scene");
  ros::NodeHandle nd;

  tf::TransformListener listener;
  tf::StampedTransform tf_transform, tf_gripper_transform;
  while (true) {
    try {
      listener.lookupTransform("/world", "/move_group/panel_bearing",
                               ros::Time(0), tf_transform);
      listener.lookupTransform("/world", "/a_bot_gripper_tip_link",
                               ros::Time(0), tf_gripper_transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    break;
  }
  geometry_msgs::Transform msg_transform;
  Eigen::Isometry3d initial_pose, gripper_pose;
  tf::transformTFToMsg(tf_transform, msg_transform);
  tf::transformMsgToEigen(msg_transform, initial_pose);
  tf::transformTFToMsg(tf_gripper_transform, msg_transform);
  tf::transformMsgToEigen(msg_transform, gripper_pose);
  CovarianceMatrix initial_covariance = CovarianceMatrix::Zero();
  for (int i = 0; i < 6; i++) {
    initial_covariance(i, i) = (i < 3 ? 0.0001 : 0.01);
  }

  PoseEstimator estimator;
  estimator.load_config_file(
      "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/launch/"
      "estimator_config.yaml");
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  read_stl_from_file_path("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/"
                          "config/wrs_assembly_2020/meshes/03-PANEL2.stl",
                          vertices, triangles);
  for (int i = 0; i < vertices.size(); i++) {
    vertices[i] /= 1000.;
  }

  Eigen::Isometry3d new_mean;
  CovarianceMatrix new_covariance;
  estimator.place_step_with_Lie_distribution(
      vertices, triangles, initial_pose, 0.7781, Eigen::Isometry3d::Identity(),
      initial_covariance, new_mean, new_covariance);

  new_mean = initial_pose * new_mean;
  new_covariance = transform_covariance(initial_pose, new_covariance);
  puts("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/"
       "wrs_assembly_2020/meshes/03-PANEL2.stl");
  puts("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/"
       "wrs_assembly_2020/object_metadata/panel_bearing.yaml");
  puts("1.0 1.0 1.0 1.0 1.0 0.0 0.0");
  puts("1");
  puts("0");
  print_pose(new_mean);
  print_pose(gripper_pose);
  puts("0.7781");
  puts("1");
  std::cout << new_covariance << std::endl;
  std::cout << CovarianceMatrix::Identity() << std::endl;
  puts("1e-6");
  puts("0");
  return 0;
}
