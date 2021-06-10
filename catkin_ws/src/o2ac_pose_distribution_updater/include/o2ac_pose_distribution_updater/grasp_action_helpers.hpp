/*
Helper functions for the grasp action
 */

#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "o2ac_pose_distribution_updater/conversions.hpp"
#include "o2ac_pose_distribution_updater/operators_for_Lie_distribution.hpp"
#include <unsupported/Eigen/AutoDiff>

void cutting_object(const std::vector<Eigen::Vector3d> &vertices,
                    const std::vector<boost::array<int, 3>> &triangles,
                    const Eigen::Hyperplane<double, 3> &plane,
                    std::vector<Eigen::Vector3d> &result_vertices,
                    std::vector<boost::array<int, 3>> &result_triangles);

class grasp_calculator {
public:
  // data for calculating the pose
  // this data is calculated in constructor and used in
  // calculate_transform_after_grasping

  Eigen::Vector3d center_of_gravity;
  Eigen::Vector3d gripper_touch_vertex_1;
  Eigen::Vector3d gripper_touch_vertex_2;
  Eigen::Vector3d gripper_touch_vertex_3;
  Eigen::Vector3d gripper_touch_vertex_4;
  Eigen::Vector3d ground_touch_vertex_1;
  Eigen::Vector3d ground_touch_vertex_2;
  int first_direction, second_direction;
  int double_vertex_side;
  int fourth_vertex_side;
  Eigen::Isometry3d rotated_gripper_transform, old_mean, new_mean;

  // constructor, which calculates new_mean
  grasp_calculator(const std::vector<Eigen::Vector3d> &vertices,
                   const std::vector<Eigen::Vector3d> &all_vertices,
                   const Eigen::Isometry3d &gripper_transform,
                   const Eigen::Isometry3d &old_mean,
                   const Eigen::Vector3d &center_of_gravity,
                   const bool balance_check = true);

  // provide function to calculate the pose after grasping given a initial pose
  // in the neighborhood of old_mean
  template <typename T>
  Eigen::Transform<T, 3, Eigen::Isometry> calculate_transform_after_grasping(
      const Eigen::Transform<T, 3, Eigen::Isometry> &old_transform) const;
};

void grasp_update_Lie_distribution(
    const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<Eigen::Vector3d> &all_vertices,
    const Eigen::Vector3d &center_of_gravity,
    const Eigen::Isometry3d &gripper_transform, Eigen::Isometry3d &new_mean,
    CovarianceMatrix &new_covariance);
