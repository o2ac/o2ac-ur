#include "o2ac_pose_distribution_updater/conversions.hpp"
#include "o2ac_pose_distribution_updater/operators_for_Lie_distribution.hpp"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"

#include <unsupported/Eigen/AutoDiff>

void eigen_distribution_RPY_to_Lie(const Particle &RPY_mean,
                                   const CovarianceMatrix &RPY_covariance,
                                   Eigen::Isometry3d &Lie_mean,
                                   CovarianceMatrix &Lie_covariance);

void eigen_distribution_Lie_to_RPY(const Eigen::Isometry3d &Lie_mean,
                                   const CovarianceMatrix &Lie_covariance,
                                   Particle &RPY_mean,
                                   CovarianceMatrix &RPY_covariance);

void distribution_RPY_to_Lie(
    const geometry_msgs::PoseWithCovariance &RPY_distribution,
    geometry_msgs::PoseWithCovariance &Lie_distribution);

void distribution_Lie_to_RPY(
    const geometry_msgs::PoseWithCovariance &Lie_distribution,
    geometry_msgs::PoseWithCovariance &RPY_distribution);
