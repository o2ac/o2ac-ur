/*
The implementation of the place test
*/

#include "o2ac_pose_distribution_updater/test.hpp"
#include <cstring>

Eigen::Isometry3d particle_to_eigen_transform(const Particle &p) {
  return Eigen::Translation3d(p.block(0, 0, 3, 1)) *
         Eigen::AngleAxis<double>(p(5), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxis<double>(p(4), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<double>(p(3), Eigen::Vector3d::UnitX());
}

void place_test(const std::shared_ptr<Client> &client,
                const std::string &test_file_path,
                const std::string &gripped_geometry_file_path) {
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
  while (1) {
    // Read the values to send
    Particle gripper_pose_particle, mean;
    if (fscanf(in, "%lf", &(gripper_pose_particle(0))) == EOF) {
      break;
    }
    for (int i = 1; i < 6; i++) {
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
    double support_surface;
    fscanf(in, "%lf", &support_surface);
    // Convert these to updateDistributiongoal
    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = goal.PLACE_OBSERVATION;
    goal.place_observation.gripper_pose.pose =
        to_PoseWithCovariance(gripper_pose_particle, CovarianceMatrix::Zero())
            .pose;
    goal.place_observation.support_surface = support_surface;
    goal.distribution.pose = to_PoseWithCovariance(mean, covariance);
    goal.gripped_object = *gripped_geometry;
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
  }
  fclose(in);
}
