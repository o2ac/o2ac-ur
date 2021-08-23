/*
The implementation of the touch test
*/

#include "o2ac_pose_distribution_updater/test.hpp"

void touch_test(const std::shared_ptr<Client> &client,
                const std::string &input_file_path,
                const std::string &gripped_geometry_file_path,
                const unsigned char &distribution_type) {
  /*
  This procedure reads the values representing touch action calls from a file
  whose path is given. The first line of the input file must be the form "x y z
  qx qy qz qw", where these 7 values represents the answer pose of the gripped
  object. Each line after the first line of input must be of the form "id x y z
  qx qy qz qw" where "id" is the index of the touched object (0: ground, 1: box)
  and other 7 values represents the pose of the gripper as cartesian coordinates
  and quaternion. After each action call, this procedure prints the values of
  mean and covariance matrix of the pose distribution to ROS_INFO. After all
  calls, the value of mean is expected to be close to the answer value.
  */

  // Load object from stl file

  std::shared_ptr<moveit_msgs::CollisionObject> gripped_geometry;
  load_CollisionObject_from_file(gripped_geometry, gripped_geometry_file_path);

  // Open the input file
  FILE *in = fopen(input_file_path.c_str(), "r");
  ASSERT_FALSE(in == NULL);
  // Read the answer
  double answer_x, answer_y, answer_z, answer_roll, answer_pitch, answer_yaw;
  fscanf(in, "%lf%lf%lf%lf%lf%lf", &answer_x, &answer_y, &answer_z,
         &answer_roll, &answer_pitch, &answer_yaw);

  // Initialize the distribution
  Particle mean;
  mean.setZero();
  CovarianceMatrix covariance;
  covariance.setZero();
  covariance.diagonal() << 0.001, 0.001, 0.001, 0.0087, 0.0087, 0.0087;

  while (1) {
    // Read the touched object id and the gripper pose
    int id;
    double x, y, z, qw, qx, qy, qz;
    if (fscanf(in, "%d%lf%lf%lf%lf%lf%lf%lf", &id, &x, &y, &z, &qx, &qy, &qz,
               &qw) == EOF) {
      break;
    }
    // Convert these to updateDistributionGoal
    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = goal.TOUCH_OBSERVATION;
    goal.gripper_pose.pose = to_Pose(x, y, z, qw, qx, qy, qz);
    goal.touch_observation.touched_object_id = id;
    goal.distribution_type = distribution_type;
    goal.distribution.pose = to_PoseWithCovariance(mean, covariance);
    goal.gripped_object = *gripped_geometry;
    // Send a call
    client->sendGoal(goal);
    client->waitForResult();
    // Get a result
    auto result = client->getResult();
    if (result->success) {
      // Convert messages to Eigen matrices and update 'mean' and 'covariance'
      mean = pose_to_particle(result->distribution.pose.pose);
      covariance = array_36_to_matrix_6x6(result->distribution.pose.covariance);
      ROS_INFO_STREAM(mean);
      ROS_INFO_STREAM(covariance);
    } else {
      ASSERT_TRUE(std::string("The sum of likelihoods is 0") ==
                      result->error_message.data ||
                  std::string("Only single particle has non-zero likelihood") ==
                      result->error_message.data);
    }
  }
  fclose(in);

  // Check that the estimated values are close to the answer values
  double allowable_error = 0.002;
  EXPECT_TRUE(std::abs(answer_x - mean(0)) < allowable_error);
  EXPECT_TRUE(std::abs(answer_y - mean(1)) < allowable_error);
  EXPECT_TRUE(std::abs(answer_z - mean(2)) < allowable_error);
}
