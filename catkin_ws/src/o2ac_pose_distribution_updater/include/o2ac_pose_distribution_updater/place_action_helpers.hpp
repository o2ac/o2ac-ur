/*
Helper functions for the place action
 */

#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "o2ac_pose_distribution_updater/conversions.hpp"
#include <unsupported/Eigen/AutoDiff>

void find_three_points(const std::vector<Eigen::Vector3d> &current_vertices,
                       const Eigen::Vector3d &current_center_of_gravity,
                       int &ground_touch_vertex_id_1,
                       int &ground_touch_vertex_id_2,
                       int &ground_touch_vertix_id_3,
                       Eigen::Quaterniond &rotation, bool &stability);

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
