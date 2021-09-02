#include "o2ac_pose_distribution_updater/estimator.hpp"
#include "o2ac_pose_distribution_updater/planner_helpers.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include "o2ac_pose_distribution_updater/test_tools.hpp"
#include <actionlib/client/simple_action_client.h>
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
  // new_covariance = transform_covariance(initial_pose, new_covariance);
  new_covariance = CovarianceMatrix::Zero();
  new_covariance(0, 0) = new_covariance(1, 1) = 0.0001;
  new_covariance(5, 5) = 0.01;

  // create the client
  ros::ServiceClient visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");
  geometry_msgs::PoseWithCovarianceStamped current_pose;
  tf::poseEigenToMsg(new_mean, current_pose.pose.pose);
  current_pose.pose.covariance = matrix_6x6_to_array_36(new_covariance);
  current_pose.header.frame_id = "world";
  current_pose.header.stamp = ros::Time::now();
  std::shared_ptr<moveit_msgs::CollisionObject> object(
      new moveit_msgs::CollisionObject);
  object->id = "gripped_object";
  object->pose = to_Pose(0., 0., 0., 1., 0., 0., 0.);
  add_mesh_to_CollisionObject(object, vertices, triangles,
                              Eigen::Isometry3d::Identity());
  fputs("sending pose belief\n", stderr);
  send_pose_belief(visualizer_client, *object, 1, 0.0, current_pose);
  fputs("sended\n", stderr);

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
  char goal_condition[100];
  scanf("%99s", goal_condition);
  if (!strcmp(goal_condition, "any")) {
    puts("0");
  } else if (!strcmp(goal_condition, "placed")) {
    puts("2");
  } else {
    puts("1");
    printf("%s\n0.01 0.01\n", goal_condition);
  }
  return 0;
}
