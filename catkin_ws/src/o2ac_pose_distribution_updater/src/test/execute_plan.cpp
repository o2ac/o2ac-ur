#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/test_tools.hpp"
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

void scan_pose(Eigen::Isometry3d &pose) {
  double tx, ty, tz, qw, qx, qy, qz;
  scanf("%lf%lf%lf%lf%lf%lf%lf", &tx, &ty, &tz, &qw, &qx, &qy, &qz);
  Eigen::Vector3d translation;
  translation << tx, ty, tz;
  Eigen::Quaterniond rotation(qw, qx, qy, qz);
  pose = Eigen::Translation3d(translation) * rotation;
}

int main(int argc, char **argv) {
  // load stl file

  char stl_file_path[1000];
  scanf("%999s", stl_file_path);
  std::shared_ptr<moveit_msgs::CollisionObject> object(
      new moveit_msgs::CollisionObject);
  load_CollisionObject_from_file(object, std::string(stl_file_path));

  // visualize the result of action plan

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nd;

  // create the client
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client(
      "update_distribution", true);
  client.waitForServer();

  // create the visualizer client
  ros::ServiceClient visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");

  Eigen::Isometry3d initial_mean;
  CovarianceMatrix initial_covariance;
  scan_pose(initial_mean);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      scanf("%lf", &initial_covariance(i, j));
    }
  }

  int number_of_actions;
  scanf("%d", &number_of_actions);

  std::vector<UpdateAction> actions(number_of_actions);
  for (int t = 0; t < number_of_actions; t++) {
    int type;
    scanf("%d", &type);
    actions[t].type = (action_type)type;
    scan_pose(actions[t].gripper_pose);
  }
  /*std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int,3>> triangles;
  CollisionObject_to_eigen_vectors(*object, vertices, triangles);
  for(auto& vertex: vertices){
    std::cout << vertex.transpose() << std::endl;
    }*/

  auto current_time = ros::Time::now();
  for (int t = 0; t < number_of_actions; t++) {
    std::string gripper_frame_id = "gripper-" + std::to_string(t);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(actions[t].gripper_pose, pose);
    broadcast_gripper_pose(gripper_frame_id, current_time, pose);
  }

  double lifetime = 1.0;

  while (ros::ok()) {

    Eigen::Isometry3d mean = initial_mean;
    CovarianceMatrix covariance = initial_covariance;

    geometry_msgs::PoseWithCovarianceStamped current_pose;

    tf::poseEigenToMsg(mean, current_pose.pose.pose);
    current_pose.pose.covariance = matrix_6x6_to_array_36(covariance);
    current_pose.header.frame_id = "world";
    current_pose.header.stamp = current_time;
    send_pose_belief(visualizer_client, *object, 1, lifetime, current_pose);

    std::cout << mean.matrix() << std::endl;
    std::cout << covariance << std::endl;

    ros::Duration(1.0).sleep();

    for (int t = 0; t < number_of_actions; t++) {
      UpdateAction &action = actions[t];
      std::cout << action.type << std::endl;
      std::cout << action.gripper_pose.matrix() << std::endl;

      std::string gripper_frame_id = "gripper-" + std::to_string(t);

      o2ac_msgs::updateDistributionGoal goal;
      goal.observation_type =
          (action.type == place_action_type
               ? goal.PLACE_OBSERVATION
               : action.type == grasp_action_type ? goal.GRASP_OBSERVATION
                                                  : goal.PUSH_OBSERVATION);
      tf::poseEigenToMsg(action.gripper_pose, goal.gripper_pose.pose);
      goal.place_observation.support_surface = 0.0;
      goal.distribution_type = 1;
      if (action.type == place_action_type) {
        tf::poseEigenToMsg(mean, goal.distribution.pose.pose);
        goal.distribution.pose.covariance = matrix_6x6_to_array_36(covariance);
      } else {
        tf::poseEigenToMsg(action.gripper_pose.inverse() * mean,
                           goal.distribution.pose.pose);
        goal.distribution.pose.covariance = matrix_6x6_to_array_36(
            transform_covariance(action.gripper_pose.inverse(), covariance));
      }
      goal.distribution.header.frame_id = gripper_frame_id;
      goal.distribution.header.stamp = current_time;

      goal.gripped_object = *object;

      // Send a call
      client.sendGoal(goal);
      client.waitForResult();

      auto result = client.getResult();

      assert(result->success);

      send_pose_belief(visualizer_client, *object, 1, lifetime,
                       result->distribution);

      tf::poseMsgToEigen(result->distribution.pose.pose, mean);
      covariance = array_36_to_matrix_6x6(result->distribution.pose.covariance);
      if (action.type != grasp_action_type) {
        mean = action.gripper_pose * mean;
        covariance = transform_covariance(action.gripper_pose, covariance);
      }

      std::cout << mean.matrix() << std::endl;
      std::cout << covariance << std::endl;

      ros::Duration(1.0).sleep();
    }
  }
  return 0;
}
