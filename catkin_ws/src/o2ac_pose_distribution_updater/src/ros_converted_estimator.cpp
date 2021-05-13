#include "o2ac_pose_distribution_updater/ros_converted_estimator.hpp"

void ROSConvertedPoseEstimator::touched_step(
    const unsigned char &touched_object_id,
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const geometry_msgs::PoseWithCovariance &old_distibution,
    geometry_msgs::PoseWithCovariance &new_distribution) {
  // convert from moveit_msgs::CollisionObject to std::vector<Eigen::Vector3d>
  // and std::vector<boost::array<int, 3>>
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(gripped_object, vertices, triangles);

  // convert from geometry_msgs::Pose to fcl::Transform3f
  auto gripper_transform = pose_to_fcl_transform(gripper_pose);

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

void ROSConvertedPoseEstimator::place_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose, const double &support_surface,
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

  // convert from PoseWithcovariance to Particle and CovarianceMatrix
  Particle old_mean = pose_to_particle(old_distibution.pose), new_mean;
  CovarianceMatrix old_covariance =
                       array_36_to_matrix_6x6(old_distibution.covariance),
                   new_covariance;

  // execute step
  PoseEstimator::place_step(vertices, triangles, gripper_transform,
                            support_surface, old_mean, old_covariance, new_mean,
                            new_covariance);

  // convert from Particle and CovarianceMatrix to PoseWithcovariance
  new_distribution = to_PoseWithCovariance(new_mean, new_covariance);
}

void ROSConvertedPoseEstimator::look_step(
    const moveit_msgs::CollisionObject &gripped_object,
    const geometry_msgs::Pose &gripper_pose,
    const sensor_msgs::Image &looked_image,
    const boost::array<unsigned int, 4> &range_of_interest,
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
  auto cv_looked_image =
      cv_bridge::toCvCopy(looked_image, sensor_msgs::image_encodings::BGR8)
          ->image;

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
