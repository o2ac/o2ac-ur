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
#include <random>
#include <stdexcept>

#include "o2ac_pose_distribution_updater/place_action_helpers.hpp"

using object_geometry = fcl::BVHModel<fcl::OBBRSS>;
using object_geometry_ptr = std::shared_ptr<object_geometry>;

// Conversion functions

fcl::Transform3f particle_to_transform(const Particle &p);
Eigen::Isometry3d particle_to_eigen_transform(const Particle &p);

Eigen::Vector3d to_eigen_vector(const fcl::Vec3f &v);

Eigen::Isometry3d to_eigen_transform(const fcl::Transform3f &t);

double calculate_distance(
    const std::shared_ptr<fcl::CollisionObject> &touched_object,
    const std::shared_ptr<fcl::CollisionGeometry> &gripped_geometry,
    const fcl::Transform3f &gripped_transform);

class PoseEstimator {
private:
  // Parameters for Gaussian particle filter
  int number_of_particles;
  Particle noise_variance;
  // Variables for Gaussian particle filter
  std::vector<Particle> particles;
  std::vector<double> likelihoods;

  // Parameters for touch action
  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  double distance_threshold;

  // Parameters for look action
  unsigned char look_threshold;
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1), camera_dist_coeffs;
  // Variables for look action
  cv::Mat camera_r, camera_t;

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

  void generate_particles(const Particle &old_mean,
                          const CovarianceMatrix &old_covariance);

  void calculate_touch_likelihoods(const unsigned char &touched_object_id,
                                   const object_geometry_ptr &gripped_geometry,
                                   const fcl::Transform3f &gripper_transform);

  void calculate_new_distribution(Particle &new_mean,
                                  CovarianceMatrix &new_covariance);

  void touched_step(const unsigned char &touched_object_id,
                    const std::vector<Eigen::Vector3d> &vertices,
                    const std::vector<boost::array<int, 3>> &triangles,
                    const fcl::Transform3f &gripper_transform,
                    const Particle &old_mean,
                    const CovarianceMatrix &old_covariance, Particle &new_mean,
                    CovarianceMatrix &new_covariance);
  void place_step(const std::vector<Eigen::Vector3d> &vertices,
                  const std::vector<boost::array<int, 3>> &triangles,
                  const Eigen::Isometry3d &gripper_transform,
                  const double &support_surface, const Particle &old_mean,
                  const CovarianceMatrix &old_covariance, Particle &new_mean,
                  CovarianceMatrix &new_covariance);

  cv::Mat generate_image(const std::vector<Eigen::Vector3d> &vertices,
                         const std::vector<boost::array<int, 3>> &triangles,
                         const Eigen::Isometry3d &transform,
                         const cv::Size &image_size);

  double similarity_of_images(const cv::Mat &estimated_image,
                              const cv::Mat &binary_looked_image,
                              const cv::Mat &ROI_mask);

  void calculate_look_likelihoods(
      const std::vector<Eigen::Vector3d> &vertices,
      const std::vector<boost::array<int, 3>> &triangles,
      const Eigen::Isometry3d &gripper_transform,
      const cv::Mat &binary_looked_image,
      const boost::array<unsigned int, 4> &range_of_interest);

  void to_binary_image(const cv::Mat &bgr_image, cv::Mat &binary_image);

  void look_step(const std::vector<Eigen::Vector3d> &vertices,
                 const std::vector<boost::array<int, 3>> &triangles,
                 const Eigen::Isometry3d &gripper_transform,
                 const cv::Mat &looked_image,
                 const boost::array<unsigned int, 4> &range_of_interest,
                 const Particle &old_mean,
                 const CovarianceMatrix &old_covariance, Particle &new_mean,
                 CovarianceMatrix &new_covariance);
};
