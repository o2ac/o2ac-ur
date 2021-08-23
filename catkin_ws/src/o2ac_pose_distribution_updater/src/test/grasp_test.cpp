/*
The implementation of the place test
*/

#include "o2ac_pose_distribution_updater/test.hpp"
#include <cstring>

void grasp_test(const std::shared_ptr<Client> &client,
                const std::string &test_file_path,
                const std::string &gripped_geometry_file_path,
                const unsigned char &distribution_type, const bool &visualize,
                ros::ServiceClient &visualizer_client,
                ros::Publisher &marker_publisher,
                const bool distribution_convert) {
  /*
    This procedure is given the path of the input file.
    The input file consists of some blocks of the form:

     gripper_pose_particle
     mean
     covariance
     X
     (new_mean)       or   (error_message)
     (new_covariance)

     Each block represents a place action call and its expected response.
     'gripper_pose_particle' and 'mean' are represented by 6-dimensional vectors
     and 'covariance' is represented by 6x6 matrix.
     'gripper_pose_particle' represents the gripper pose of the place action
    call as a particle. When X=0, it is expected that the place action is not
    calculated successfully and 'error_message' means the expected error
    message. When X=1, it is expected that the place action is calculated
    successfully and 'new_mean' and 'new_covariance' are the expected responsed
    value of this call.
  */
  // Load object from stl file
  std::shared_ptr<moveit_msgs::CollisionObject> gripped_geometry;
  load_CollisionObject_from_file(gripped_geometry, gripped_geometry_file_path);

  FILE *in = fopen(test_file_path.c_str(), "r");
  ASSERT_FALSE(in == NULL);

  int number_of_cases;
  fscanf(in, "%d", &number_of_cases);
  for (int t = 0; t < number_of_cases; t++) {
    // Read the values to send
    Particle gripper_pose_particle, mean;
    for (int i = 0; i < 6; i++) {
      fscanf(in, "%lf", &(gripper_pose_particle(i)));
    }
    for (int i = 0; i < 6; i++) {
      fscanf(in, "%lf", &(mean(i)));
    }
    CovarianceMatrix covariance;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        fscanf(in, "%lf", &(covariance(i, j)));
      }
    }
    // Convert these to updateDistributiongoal
    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = goal.GRASP_OBSERVATION;
    particle_to_pose(gripper_pose_particle,
                     goal.grasp_observation.gripper_pose.pose);
    goal.distribution_type = distribution_type;
    goal.distribution.pose = to_PoseWithCovariance(mean, covariance);
    goal.gripped_object = *gripped_geometry;

    if (visualize) {
      // name the current gripper frame
      static int gripper_id = 0;
      std::string gripper_frame_id =
          "grasp-gripper-" + std::to_string(gripper_id++);
      // store the current time
      auto current_time = ros::Time::now();
      // broadcast the gripper pose
      broadcast_gripper_pose(gripper_frame_id, current_time,
                             goal.gripper_pose.pose);
      // visualize the pose belief before grasping
      goal.distribution.header.frame_id = gripper_frame_id;
      goal.distribution.header.stamp = current_time;
      send_pose_belief(visualizer_client, goal.gripped_object,
                       distribution_type, goal.distribution);
    }

    // Send a call
    client->sendGoal(goal);
    client->waitForResult();
    // Receive the result
    auto result = client->getResult();
    // Check that the result is equal to the expected one
    int success;
    fscanf(in, "%d\n", &success);
    if (success == 0) {
      // It is expected that the update is not calculated successfully
      ASSERT_FALSE(result->success);
      // Read the expected error message
      char expected_error_message[999];
      fgets(expected_error_message, 999, in);
      // erase \n if exists
      char *p = strchr(expected_error_message, '\n');
      if (p)
        *p = 0;
      ASSERT_EQ(std::string(expected_error_message),
                result->error_message.data);
    } else {
      // It is expected that the update is calculated successfully
      ASSERT_TRUE(result->success);

      // Convert messages to Eigen Matrices
      Particle new_mean = pose_to_particle(result->distribution.pose.pose);
      CovarianceMatrix new_covariance =
          array_36_to_matrix_6x6(result->distribution.pose.covariance);

      ROS_INFO_STREAM(new_mean);
      ROS_INFO_STREAM(new_covariance);

      // Read the expected values and check the equality
      const double EPS = 1e-6;
      for (int i = 0; i < 6; i++) {
        double expected_value;
        fscanf(in, "%lf", &expected_value);
        ASSERT_LT(std::abs(new_mean(i) - expected_value),
                  EPS * std::max(1.0, expected_value));
      }
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          double expected_value;
          fscanf(in, "%lf", &expected_value);
          ASSERT_LT(std::abs(new_covariance(i, j) - expected_value),
                    EPS * std::max(1.0, expected_value));
        }
      }
    }

    if (visualize) {
      // visualize the pose belief after placing

      double max_x = 0.0;

      static int result_gripper_id = 0;
      std::string result_gripper_frame_id =
          "result-grasp-gripper-" + std::to_string(result_gripper_id++);
      // store the current time
      auto current_time = ros::Time::now();
      // broadcast the gripper pose
      auto result_gripper_pose = goal.grasp_observation.gripper_pose.pose;
      result_gripper_pose.position.y += 0.1;
      broadcast_gripper_pose(result_gripper_frame_id, current_time,
                             result_gripper_pose);
      if (result->success) {
        auto result_distribution = result->distribution;

        result_distribution.header.frame_id = result_gripper_frame_id;
        result_distribution.header.stamp = current_time;
        send_pose_belief(visualizer_client, goal.gripped_object,
                         distribution_type, result_distribution);

        Eigen::Isometry3d new_mean;
        tf::poseMsgToEigen(result_distribution.pose.pose, new_mean);

        std::vector<Eigen::Vector3d> vertices;
        std::vector<boost::array<int, 3>> triangles;
        CollisionObject_to_eigen_vectors(*gripped_geometry, vertices,
                                         triangles);
        for (auto &vertex : vertices) {
          max_x = std::max(max_x, (new_mean * vertex)(0));
        }
      }

      visualization_msgs::Marker marker_plus, marker_minus;
      marker_plus.header.frame_id = result_gripper_frame_id;
      marker_plus.header.stamp = current_time;
      marker_plus.ns = "gripper_marker";

      marker_plus.type = visualization_msgs::Marker::CUBE;

      marker_plus.frame_locked = true;
      marker_plus.action = visualization_msgs::Marker::ADD;

      marker_plus.pose = to_Pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
      marker_plus.scale.x = 0.002;
      marker_plus.scale.y = marker_plus.scale.z = 0.1;
      marker_plus.color.r = 0.0;
      marker_plus.color.g = 1.0;
      marker_plus.color.b = 0.0;
      marker_plus.color.a = 0.8;

      marker_minus = marker_plus;

      marker_plus.pose.position.x = max_x + 0.001;
      marker_minus.pose.position.x = -max_x - 0.001;

      marker_plus.id = 2 * result_gripper_id;
      marker_minus.id = 2 * result_gripper_id + 1;

      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.resize(2);
      marker_array.markers[0] = marker_plus;
      marker_array.markers[1] = marker_minus;

      marker_publisher.publish(marker_array);
    }
  }
  fclose(in);
}
