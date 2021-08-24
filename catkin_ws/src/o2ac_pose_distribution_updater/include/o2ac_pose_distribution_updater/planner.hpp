#include "o2ac_pose_distribution_updater/estimator.hpp"
#include "o2ac_pose_distribution_updater/planner_helpers.hpp"
#include <queue>

enum action_type {
  touch_action_type,
  look_action_type,
  place_action_type,
  grasp_action_type,
  push_action_type
};

struct UpdateAction {
  Eigen::Isometry3d gripper_pose;
  action_type type;
};

struct mesh_object {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
};

using ValidityChecker = std::function<bool(
    const action_type &, const Eigen::Isometry3d &, const Eigen::Isometry3d &)>;
using CostFunction = std::function<double(
    const action_type &, const Eigen::Isometry3d &, const Eigen::Isometry3d &)>;

class Planner : public PoseEstimator {
private:
  std::shared_ptr<mesh_object> gripped_geometry;
  std::shared_ptr<std::vector<Eigen::Isometry3d>> grasp_points;
  double support_surface;
  Eigen::Vector3d center_of_gravity;
  std::vector<Eigen::Hyperplane<double, 3>> place_candidates;
  std::vector<Eigen::Vector3d> convex_hull_vertices;

  std::shared_ptr<ValidityChecker> validity_checker =
      std::make_shared<ValidityChecker>(
          [](const action_type &type,
             const Eigen::Isometry3d &current_gripper_pose,
             const Eigen::Isometry3d &target_gripper_pose) -> bool {
            return true;
          });

  std::shared_ptr<CostFunction> cost_function = std::make_shared<CostFunction>(
      [](const action_type &type, const Eigen::Isometry3d &current_gripper_pose,
         const Eigen::Isometry3d &target_gripper_pose) -> double {
        return 1.0;
      });

  void apply_action(const Eigen::Isometry3d &old_mean,
                    const CovarianceMatrix &old_covariance,
                    const UpdateAction &action, Eigen::Isometry3d &new_mean,
                    CovarianceMatrix &new_covariance);

  void
  calculate_action_candidates(const Eigen::Isometry3d &current_gripper_pose,
                              const Eigen::Isometry3d &current_mean,
                              const CovarianceMatrix &current_covariance,
                              const bool &gripping,
                              std::vector<UpdateAction> &candidates);

public:
  void set_cost_function(const std::shared_ptr<CostFunction> &cost_function) {
    this->cost_function = cost_function;
  }

  void set_validity_checker(
      const std::shared_ptr<ValidityChecker> &validity_checker) {
    this->validity_checker = validity_checker;
  }

  void set_geometry(
      const std::shared_ptr<mesh_object> &gripped_geometry,
      const std::shared_ptr<std::vector<Eigen::Isometry3d>> &grasp_points,
      const double &support_surface);

  std::vector<UpdateAction> calculate_plan(
      const Eigen::Isometry3d &current_gripper_pose,
      const bool &current_gripping, const Eigen::Isometry3d &current_mean,
      const CovarianceMatrix &current_covariance,
      const CovarianceMatrix &objective_coefficients,
      const double &objective_value, const bool goal_gripping = false,
      const std::function<bool(const Eigen::Isometry3d &)> check_goal_pose =
          [](const Eigen::Isometry3d &pose) { return true; });

  std::vector<std::pair<double, std::vector<UpdateAction>>>
  best_scores_for_each_costs(const Eigen::Isometry3d &current_gripper_pose,
                             const bool &current_gripping,
                             const Eigen::Isometry3d &current_mean,
                             const CovarianceMatrix &current_covariance,
                             const CovarianceMatrix &objective_coefficients,
                             const int &max_cost);
};

std::function<bool(const Eigen::Isometry3d)>
check_near_to_goal_pose(const Eigen::Isometry3d &goal_pose,
                        const double &translation_threshold,
                        const double &rotation_threshold);
