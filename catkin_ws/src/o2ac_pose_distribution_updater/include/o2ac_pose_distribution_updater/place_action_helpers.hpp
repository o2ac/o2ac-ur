/*
Helper functions for the place action
 */

#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "o2ac_pose_distribution_updater/conversions.hpp"
#include "o2ac_pose_distribution_updater/operators_for_Lie_distribution.hpp"
#include <unsupported/Eigen/AutoDiff>

void find_three_points(const std::vector<Eigen::Vector3d> &current_vertices,
                       const Eigen::Vector3d &current_center_of_gravity,
                       int &ground_touch_vertex_id_1,
                       int &ground_touch_vertex_id_2,
                       int &ground_touch_vertix_id_3,
                       Eigen::Quaterniond &rotation, bool &stability,
                       const bool balance_check = true);

void place_update_distribution(const Particle &old_mean,
                               const CovarianceMatrix &old_covariance,
                               const Eigen::Vector3d &center_of_gravity,
                               const Eigen::Vector3d &ground_touch_vertex_1,
                               const Eigen::Vector3d &ground_touch_vertex_2,
                               const Eigen::Vector3d &ground_touch_vertex_3,
                               const double &support_surface,
                               const Eigen::Isometry3d &gripper_transform,
                               Particle &new_mean,
                               CovarianceMatrix &new_covariance);

class place_calculator {
public:
  Eigen::Vector3d center_of_gravity, ground_touch_vertex_1,
      ground_touch_vertex_2,
      ground_touch_vertex_3; // the coordinates of center of gravity, first
                             // touching point, second touching point and third
                             // touching point
  double support_surface;    // the z-coordinate of the ground
  Eigen::Isometry3d gripper_transform, old_mean,
      new_mean; // The gripper transform, the mean transform before placing, the
                // mean transform after placing

  place_calculator(const Eigen::Isometry3d &old_mean,
                   const Eigen::Vector3d &center_of_gravity,
                   const std::vector<Eigen::Vector3d> &vertices,
                   const double &support_surface,
                   const Eigen::Isometry3d &gripper_transform,
                   const bool balance_check = true);
};

void place_update_Lie_distribution(const Eigen::Isometry3d &old_mean,
                                   const CovarianceMatrix &old_covariance,
                                   const Eigen::Vector3d &center_of_gravity,
                                   const std::vector<Eigen::Vector3d> &vertices,
                                   const double &_support_surface,
                                   const Eigen::Isometry3d &gripper_transform,
                                   Eigen::Isometry3d &new_mean,
                                   CovarianceMatrix &new_covariance);
