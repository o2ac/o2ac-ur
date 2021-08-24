#include "o2ac_pose_distribution_updater/ros_converted_estimator.hpp"

void ROSConvertedPoseEstimator::touched_step(
    const unsigned char &touched_object_id,
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to fcl::Transform3f
  fcl::Transform3f gripper_transform = pose_to_fcl_transform(gripper_pose);

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {

    // convert from PoseWithcovariance to Particle and CovarianceMatrix
    Particle old_mean = pose_to_particle(old_distibution.pose), new_mean;
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::touched_step(touched_object_id, vertices, triangles,
                                gripper_transform, old_mean, old_covariance,
                                new_mean, new_covariance);

    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    new_distribution = to_PoseWithCovariance(new_mean, new_covariance);
  }

  else if (distribution_type ==
           o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {

    // convert from PoseWithcovariance to Eigen::Isometry3d and CovarianceMatrix
    Eigen::Isometry3d old_mean, new_mean;
    tf::poseMsgToEigen(old_distibution.pose, old_mean);
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::touched_step_with_Lie_distribution(
        touched_object_id, vertices, triangles, gripper_transform, old_mean,
        old_covariance, new_mean, new_covariance);

    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    tf::poseEigenToMsg(new_mean, new_distribution.pose);
    new_distribution.covariance = matrix_6x6_to_array_36(new_covariance);
  }
}

void ROSConvertedPoseEstimator::place_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose, const double &support_surface,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to Eigen::Isometry3d
  Eigen::Isometry3d gripper_transform;
  tf::poseMsgToEigen(gripper_pose, gripper_transform);

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {

    // convert from PoseWithcovariance to Particle and CovarianceMatrix
    Particle old_mean = pose_to_particle(old_distibution.pose), new_mean;
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::place_step(vertices, triangles, gripper_transform,
                              support_surface, old_mean, old_covariance,
                              new_mean, new_covariance);

    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    new_distribution = to_PoseWithCovariance(new_mean, new_covariance);
  }

  else if (distribution_type ==
           o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {

    // convert from PoseWithcovariance to Eigen::Isometry3d and CovarianceMatrix
    Eigen::Isometry3d old_mean, new_mean;
    tf::poseMsgToEigen(old_distibution.pose, old_mean);
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::place_step_with_Lie_distribution(
        vertices, triangles, gripper_transform, support_surface, old_mean,
        old_covariance, new_mean, new_covariance);

    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    tf::poseEigenToMsg(new_mean, new_distribution.pose);
    new_distribution.covariance = matrix_6x6_to_array_36(new_covariance);
  }
}

void ROSConvertedPoseEstimator::grasp_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to Eigen::Isometry3d
  Eigen::Isometry3d gripper_transform;
  tf::poseMsgToEigen(gripper_pose, gripper_transform);

  Eigen::Isometry3d old_mean, new_mean;
  CovarianceMatrix old_covariance, new_covariance;

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {

    // convert from PoseWithcovariance to Particle and CovarianceMatrix
    Particle RPY_old_mean = pose_to_particle(old_distibution.pose);
    CovarianceMatrix RPY_old_covariance =
        array_36_to_matrix_6x6(old_distibution.covariance);

    eigen_distribution_RPY_to_Lie(RPY_old_mean, RPY_old_covariance, old_mean,
                                  old_covariance);
  } else if (distribution_type ==
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {
    // convert from PoseWithcovariance to Eigen::Isometry3d and CovarianceMatrix
    tf::poseMsgToEigen(old_distibution.pose, old_mean);
    old_covariance = array_36_to_matrix_6x6(old_distibution.covariance);
  }

  // execute step
  PoseEstimator::grasp_step_with_Lie_distribution(
      vertices, triangles, gripper_transform, old_mean, old_covariance,
      new_mean, new_covariance);

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {
    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    Particle RPY_new_mean;
    CovarianceMatrix RPY_new_covariance;
    eigen_distribution_Lie_to_RPY(new_mean, new_covariance, RPY_new_mean,
                                  RPY_new_covariance);
    new_distribution = to_PoseWithCovariance(RPY_new_mean, RPY_new_covariance);
  } else if (distribution_type ==
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {
    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    tf::poseEigenToMsg(new_mean, new_distribution.pose);
    new_distribution.covariance = matrix_6x6_to_array_36(new_covariance);
  }
}

void ROSConvertedPoseEstimator::push_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to Eigen::Isometry3d
  Eigen::Isometry3d gripper_transform;
  tf::poseMsgToEigen(gripper_pose, gripper_transform);

  Eigen::Isometry3d old_mean, new_mean;
  CovarianceMatrix old_covariance, new_covariance;

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {

    // convert from PoseWithcovariance to Particle and CovarianceMatrix
    Particle RPY_old_mean = pose_to_particle(old_distibution.pose);
    CovarianceMatrix RPY_old_covariance =
        array_36_to_matrix_6x6(old_distibution.covariance);

    eigen_distribution_RPY_to_Lie(RPY_old_mean, RPY_old_covariance, old_mean,
                                  old_covariance);
  } else if (distribution_type ==
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {
    // convert from PoseWithcovariance to Eigen::Isometry3d and CovarianceMatrix
    tf::poseMsgToEigen(old_distibution.pose, old_mean);
    old_covariance = array_36_to_matrix_6x6(old_distibution.covariance);
  }

  // execute step
  PoseEstimator::push_step_with_Lie_distribution(
      vertices, triangles, gripper_transform, old_mean, old_covariance,
      new_mean, new_covariance);

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {
    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    Particle RPY_new_mean;
    CovarianceMatrix RPY_new_covariance;
    eigen_distribution_Lie_to_RPY(new_mean, new_covariance, RPY_new_mean,
                                  RPY_new_covariance);
    new_distribution = to_PoseWithCovariance(RPY_new_mean, RPY_new_covariance);
  } else if (distribution_type ==
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {
    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    tf::poseEigenToMsg(new_mean, new_distribution.pose);
    new_distribution.covariance = matrix_6x6_to_array_36(new_covariance);
  }
}

void ROSConvertedPoseEstimator::look_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const sensor_msgs::Image &looked_image,
    const boost::array<unsigned int, 4> &range_of_interest,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to Eigen::Isometry3d
  Eigen::Isometry3d gripper_transform;
  tf::poseMsgToEigen(gripper_pose, gripper_transform);

  // convert from sensor_msgs::Image to cv::Mat
  cv::Mat cv_looked_image =
      cv_bridge::toCvCopy(looked_image, sensor_msgs::image_encodings::BGR8)
          ->image;

  if (distribution_type == o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE) {

    // convert from PoseWithcovariance to Particle and CovarianceMatrix
    Particle old_mean = pose_to_particle(old_distibution.pose), new_mean;
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::look_step(vertices, triangles, gripper_transform,
                             cv_looked_image, range_of_interest, old_mean,
                             old_covariance, new_mean, new_covariance);

    // convert from Particle and CovarianceMatrix to PoseWithcovariance
    new_distribution = to_PoseWithCovariance(new_mean, new_covariance);
  }

  else if (distribution_type ==
           o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE) {

    // convert from PoseWithcovariance to Eigen::Isometry3d and CovarianceMatrix
    Eigen::Isometry3d old_mean, new_mean;
    tf::poseMsgToEigen(old_distibution.pose, old_mean);
    CovarianceMatrix old_covariance =
                         array_36_to_matrix_6x6(old_distibution.covariance),
                     new_covariance;

    // execute step
    PoseEstimator::look_step_with_Lie_distribution(
        vertices, triangles, gripper_transform, cv_looked_image,
        range_of_interest, old_mean, old_covariance, new_mean, new_covariance);

    // convert from Eigen::Isometry3d and CovarianceMatrix to PoseWithcovariance
    tf::poseEigenToMsg(new_mean, new_distribution.pose);
    new_distribution.covariance = matrix_6x6_to_array_36(new_covariance);
  }
}
