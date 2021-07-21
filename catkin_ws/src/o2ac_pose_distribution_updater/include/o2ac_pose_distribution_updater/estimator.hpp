#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <ccd/vec3.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/shape/geometric_shapes.h>
#include <opencv2/opencv.hpp>

#include <boost/array.hpp>
#include <iostream>
#include <stdexcept>

#include "o2ac_pose_distribution_updater/grasp_action_helpers.hpp"
#include "o2ac_pose_distribution_updater/place_action_helpers.hpp"
#include "o2ac_pose_distribution_updater/push_action_helpers.hpp"
#include "o2ac_pose_distribution_updater/random_particle.hpp"

using object_geometry = fcl::BVHModel<fcl::OBBRSS>;
using object_geometry_ptr = std::shared_ptr<object_geometry>;

// Conversion functions associated with fcl types

fcl::Transform3f particle_to_transform(const Particle &p);

Eigen::Vector3d fcl_to_eigen_vector(const fcl::Vec3f &v);

Eigen::Isometry3d fcl_to_eigen_transform(const fcl::Transform3f &t);

fcl::Transform3f eigen_to_fcl_transform(const Eigen::Isometry3d &t);

double calculate_distance(
    const std::shared_ptr<fcl::CollisionObject> &touched_object,
    const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry,
    const fcl::Transform3f &gripped_transform);

Eigen::Vector3d
calculate_center_of_gravity(const std::vector<Eigen::Vector3d> &vertices,
                            const std::vector<boost::array<int, 3>> &triangles);

class PoseEstimator {
protected:
  // Parameters for Gaussian particle filter
  int number_of_particles;
  Particle noise_variance;
  // Variables for Gaussian particle filter
  std::vector<Particle> particles;
  std::vector<Eigen::Isometry3d> particle_transforms;
  std::vector<fcl::Transform3f> fcl_particle_transforms;
  std::vector<double> likelihoods;

  // Parameters for touch action
  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  double distance_threshold;

  // Parameters for look action
  unsigned char look_threshold;
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1), camera_dist_coeffs;
  // Variables for look action
  cv::Mat camera_r, camera_t;

  // Parameters to place, grasp and push actions
  bool use_linear_approximation;

  // Parameters for grasp and push action
  double gripper_height, gripper_width, gripper_thickness;

public:
  PoseEstimator(){};

  void set_particle_parameters(const int &number_of_particles,
                               const Particle &noise_variance);

  void set_touch_parameters(
      const std::vector<std::shared_ptr<fcl::CollisionObject>> &touched_objects,
      const double &distance_threshold);
  void set_look_parameters(
      const double &look_threshold,
      const std::vector<std::vector<double>> &calibration_object_points,
      const std::vector<std::vector<double>> &calibration_image_points,
      const double &camera_fx, const double &camera_fy, const double &camera_cx,
      const double &camera_cy);

  void set_use_linear_approximation(const bool &use_linear_approximation) {
    this->use_linear_approximation = use_linear_approximation;
  }

  void set_grasp_parameters(const double &gripper_height,
                            const double &gripper_width,
                            const double &gripper_thickness);

  void generate_particles(const Particle &old_mean,
                          const CovarianceMatrix &old_covariance);

  void calculate_touch_likelihoods(const unsigned char &touched_object_id,
                                   const object_geometry_ptr &gripped_geometry,
                                   const fcl::Transform3f &gripper_transform);

  void calculate_new_distribution(Particle &new_mean,
                                  CovarianceMatrix &new_covariance);

  void calculate_new_Lie_distribution(const Eigen::Isometry3d &old_mean,
                                      Eigen::Isometry3d &new_mean,
                                      CovarianceMatrix &new_covariance);

  void touched_step(const unsigned char &touched_object_id,
                    const std::vector<Eigen::Vector3d> &vertices,
                    const std::vector<boost::array<int, 3>> &triangles,
                    const fcl::Transform3f &gripper_transform,
                    const Particle &old_mean,
                    const CovarianceMatrix &old_covariance, Particle &new_mean,
                    CovarianceMatrix &new_covariance);

  void touched_step_with_Lie_distribution(
      const unsigned char &touched_object_id,
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const fcl::Transform3f &gripper_transform,
      const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
      Eigen::Isometry3d &new_mean, CovarianceMatrix &new_covariance);

  void place_step(const std::vector<Eigen::Vector3d> &vertices,
                  const std::vector<boost::array<int, 3>> &triangles,
                  const Eigen::Isometry3d &gripper_transform,
                  const double &support_surface, const Particle &old_mean,
                  const CovarianceMatrix &old_covariance, Particle &new_mean,
                  CovarianceMatrix &new_covariance);

  void place_step_with_Lie_distribution(
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const Eigen::Isometry3d &gripper_transform, const double &support_surface,
      const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
      Eigen::Isometry3d &new_mean, CovarianceMatrix &new_covariance,
      const bool validity_check = false);

  void grasp_step_with_Lie_distribution(
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const Eigen::Isometry3d &gripper_transform,
      const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
      Eigen::Isometry3d &new_mean, CovarianceMatrix &new_covariance,
      const bool validity_check = false);

  void push_step_with_Lie_distribution(
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const Eigen::Isometry3d &gripper_transform,
      const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
      Eigen::Isometry3d &new_mean, CovarianceMatrix &new_covariance,
      const bool validity_check = false);

  void generate_image(cv::Mat &image,
                      const std::vector<Eigen::Vector3d> &vertices,
                      const std::vector<boost::array<int, 3>> &triangles,
                      const Eigen::Isometry3d &transform,
                      const boost::array<unsigned int, 4> &ROI);

  double similarity_of_images(const cv::Mat &estimated_image,
                              const cv::Mat &binary_looked_image);

  void
  calculate_look_likelihoods(const std::vector<Eigen::Vector3d> &vertices,
                             const std::vector<boost::array<int, 3>> &triangles,
                             const Eigen::Isometry3d &gripper_transform,
                             const cv::Mat &binary_looked_image,
                             const boost::array<unsigned int, 4> &ROI);

  void to_binary_image(const cv::Mat &bgr_image, cv::Mat &binary_image);

  void look_step(const std::vector<Eigen::Vector3d> &vertices,
                 const std::vector<boost::array<int, 3>> &triangles,
                 const Eigen::Isometry3d &gripper_transform,
                 const cv::Mat &looked_image,
                 const boost::array<unsigned int, 4> &ROI,
                 const Particle &old_mean,
                 const CovarianceMatrix &old_covariance, Particle &new_mean,
                 CovarianceMatrix &new_covariance);

  void look_step_with_Lie_distribution(
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const Eigen::Isometry3d &gripper_transform, const cv::Mat &looked_image,
      const boost::array<unsigned int, 4> &ROI,
      const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
      Eigen::Isometry3d &new_mean, CovarianceMatrix &new_covariance,
      const bool already_binary = false);
};
