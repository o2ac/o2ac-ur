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

class Planner : public PoseEstimator {
private:
  std::shared_ptr<mesh_object> gripped_geometry;
  std::shared_ptr<std::vector<Eigen::Isometry3d>> grasp_points;
  Eigen::Vector3d center_of_gravity;
  std::vector<Eigen::Hyperplane<double, 3>> place_candidates;

  unsigned int image_height, image_width;

  double action_cost, translation_cost, rotation_cost;

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

  double calculate_cost(const Eigen::Isometry3d &current_gripper_pose,
                        const Eigen::Isometry3d &next_gripper_pose);

public:
  void set_image_size(const unsigned int &image_height,
                      const unsigned int &image_width) {
    this->image_height;
    this->image_width;
  }

  void set_cost_coefficients(const double &action_cost,
                             const double &translation_cost,
                             const double &rotation_cost) {
    this->action_cost = action_cost;
    this->translation_cost = translation_cost;
    this->rotation_cost = rotation_cost;
  }
  std::vector<UpdateAction> calculate_plan(
      const std::shared_ptr<mesh_object> &gripped_geometry,
      const std::shared_ptr<std::vector<Eigen::Isometry3d>> &grasp_points,
      const Eigen::Isometry3d &current_gripper_pose,
      const bool &current_gripping, const Eigen::Isometry3d &current_mean,
      const CovarianceMatrix &current_covariance,
      const CovarianceMatrix &objective_coefficients,
      const double &objective_value);
};
