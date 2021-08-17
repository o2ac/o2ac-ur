#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/convex_hull.hpp"

namespace {
double LARGE_EPS = 1e-8, EPS = 1e-9, INF = 1e9;
double pi = acos(-1);
} // namespace

void Planner::apply_action(const Eigen::Isometry3d &old_mean,
                           const CovarianceMatrix &old_covariance,
                           const UpdateAction &action,
                           Eigen::Isometry3d &new_mean,
                           CovarianceMatrix &new_covariance) {
  if (action.type == place_action_type) {
    place_step_with_Lie_distribution(
        gripped_geometry->vertices, gripped_geometry->triangles,
        action.gripper_pose, support_surface, old_mean, old_covariance,
        new_mean, new_covariance, true);
    new_mean = action.gripper_pose * new_mean;
    new_covariance = transform_covariance(action.gripper_pose, new_covariance);
  } else if (action.type == grasp_action_type) {
    grasp_step_with_Lie_distribution(
        gripped_geometry->vertices, gripped_geometry->triangles,
        action.gripper_pose, action.gripper_pose.inverse() * old_mean,
        transform_covariance(action.gripper_pose.inverse(), old_covariance),
        new_mean, new_covariance, true);
  } else if (action.type == push_action_type) {
    push_step_with_Lie_distribution(
        gripped_geometry->vertices, gripped_geometry->triangles,
        action.gripper_pose, action.gripper_pose.inverse() * old_mean,
        transform_covariance(action.gripper_pose.inverse(), old_covariance),
        new_mean, new_covariance, true);
    new_mean = action.gripper_pose * new_mean;
    new_covariance = transform_covariance(action.gripper_pose, new_covariance);
  } else if (action.type == touch_action_type) {
    touched_step_with_Lie_distribution(
        0, gripped_geometry->vertices, gripped_geometry->triangles,
        eigen_to_fcl_transform(action.gripper_pose), old_mean, old_covariance,
        new_mean, new_covariance);
  } else if (action.type == look_action_type) {
    cv::Mat mean_image;
    boost::array<unsigned int, 4> ROI{0, image_height, 0, image_width};
    generate_image(mean_image, gripped_geometry->vertices,
                   gripped_geometry->triangles, action.gripper_pose * old_mean,
                   ROI);
    look_step_with_Lie_distribution(
        gripped_geometry->vertices, gripped_geometry->triangles,
        action.gripper_pose, mean_image, ROI, old_mean, old_covariance,
        new_mean, new_covariance, true);
  }
}

Eigen::AngleAxisd rotation_to_minus_Z(const Eigen::Vector3d &vector) {
  Eigen::Vector3d axis =
      vector(0) != 0.0 || vector(1) != 0.0
          ? vector.cross(-Eigen::Vector3d::UnitZ()).normalized()
          : Eigen::Vector3d::UnitX();
  double angle = atan2(vector.head<2>().norm(), -vector[2]);
  return Eigen::AngleAxisd(angle, axis);
}

void Planner::calculate_action_candidates(
    const Eigen::Isometry3d &current_gripper_pose,
    const Eigen::Isometry3d &current_mean, const CovarianceMatrix &covariance,
    const bool &gripping, std::vector<UpdateAction> &candidates) {
  if (gripping) {
    // add place action candidates
    for (auto &plane : place_candidates) {
      UpdateAction action;
      action.type = place_action_type;

      Eigen::Vector3d normal = current_gripper_pose.rotation() *
                               current_mean.rotation() * plane.normal();

      Eigen::Quaterniond gripper_rotation(rotation_to_minus_Z(normal) *
                                          current_gripper_pose.rotation());
      Eigen::Vector3d gripper_X = gripper_rotation * Eigen::Vector3d::UnitX();
      if (gripper_X(0) != 0.0 || gripper_X(1) != 0.0) {
        double angle = atan2(gripper_X(1), -gripper_X(0));
        gripper_rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
                           gripper_rotation;
      }
      Eigen::Isometry3d rotated_gripper_pose(gripper_rotation);
      rotated_gripper_pose.translation() = current_gripper_pose.translation();

      if (abs((rotated_gripper_pose.rotation() * Eigen::Vector3d::UnitY())(2)) >
          LARGE_EPS) {
        continue;
      }
      // double sigma_z = 3.0 * sqrt(covariance(2, 2));
      double sigma_z = 0.0;
      action.gripper_pose =
          Eigen::Translation3d((support_surface + sigma_z -
                                (rotated_gripper_pose * current_mean *
                                 (-plane.offset() * plane.normal()))
                                    .z()) *
                               Eigen::Vector3d::UnitZ()) *
          rotated_gripper_pose;

      if ((*validity_checker)(place_action_type, current_gripper_pose,
                              action.gripper_pose)) {
        candidates.push_back(action);
      }
    }
    // add touch action candidates
    int number_of_touch_actions = 10;
    auto random_array = std::move(
        get_random_array(number_of_touch_actions, convex_hull_vertices.size()));
    for (int t = 0; t < random_array.size(); t++) {
      auto &vertex = convex_hull_vertices[random_array[t]];
      UpdateAction action;
      action.type = touch_action_type;

      Eigen::Vector3d direction = current_gripper_pose.rotation() *
                                  current_mean.rotation() *
                                  (vertex - center_of_gravity);

      Eigen::Isometry3d rotated_gripper_pose =
          rotation_to_minus_Z(direction) * current_gripper_pose;
      action.gripper_pose =
          Eigen::Translation3d(
              (-(rotated_gripper_pose * current_mean * vertex).z()) *
              Eigen::Vector3d::UnitZ()) *
          rotated_gripper_pose;

      if ((*validity_checker)(touch_action_type, current_gripper_pose,
                              action.gripper_pose)) {
        candidates.push_back(action);
      }
    }
    // add look action candidates
    for (int t = 0; t < 4; t++) {
      UpdateAction action;
      action.type = look_action_type;
      Eigen::Isometry3d rotated_gripper_pose =
          current_gripper_pose *
          Eigen::AngleAxisd(pi / 2.0 * t, Eigen::Vector3d::UnitX());
      action.gripper_pose =
          Eigen::Translation3d(looked_point - rotated_gripper_pose *
                                                  current_mean *
                                                  center_of_gravity) *
          rotated_gripper_pose;

      if ((*validity_checker)(look_action_type, current_gripper_pose,
                              action.gripper_pose)) {
        candidates.push_back(action);
      }
    }
  } else {
    for (auto &grasp_point : *grasp_points) {
      Eigen::Isometry3d gripper_pose = current_mean * grasp_point;
      if (abs((gripper_pose.rotation() * Eigen::Vector3d::UnitY())(2)) <=
          LARGE_EPS) {
        UpdateAction action;
        action.type = grasp_action_type;
        action.gripper_pose = gripper_pose;
        if ((*validity_checker)(grasp_action_type, current_gripper_pose,
                                action.gripper_pose)) {
          candidates.push_back(action);
        }
      }
    }
    std::vector<Eigen::Vector2d> projected_points, hull;
    std::transform(gripped_geometry->vertices.begin(),
                   gripped_geometry->vertices.end(),
                   std::back_inserter(projected_points),
                   [&current_mean](const Eigen::Vector3d &vertex) {
                     return (Eigen::Vector2d)(current_mean * vertex).head<2>();
                   });

    convex_hull_for_Eigen_Vector2d(projected_points, hull);

    Eigen::Vector3d current_center = current_mean * center_of_gravity;
    Eigen::Vector2d projected_center = current_center.head<2>();
    int number_of_push_actions = 30;
    auto random_array =
        std::move(get_random_array(number_of_push_actions, hull.size()));
    for (int t = 0; t < random_array.size(); t++) {
      int i = random_array[t], j = (i + 1) % hull.size();
      Eigen::Vector2d edge = (hull[j] - hull[i]).normalized();
      if (edge.dot(projected_center - hull[i]) < -EPS ||
          edge.dot(projected_center - hull[j]) > EPS) {
        continue;
      }
      Eigen::Vector2d edge_normal;
      edge_normal << edge[1], -edge[0];
      Eigen::Vector3d translation;
      double sigma_y =
          4.0 * sqrt(pow(edge_normal(0), 2) * covariance(0, 0) +
                     2.0 * edge_normal(0) * edge_normal(1) * covariance(0, 1) +
                     pow(edge_normal(1), 2) * covariance(1, 1) +
                     pow((hull[j] - hull[i]).norm(), 2) * covariance(5, 5));
      translation << (edge_normal.dot(hull[i]) - sigma_y +
                      gripper_width / 2.0) *
                             edge_normal +
                         edge.dot(projected_center) * edge,
          support_surface;
      double angle = -atan2(-edge(0), -edge(1));

      UpdateAction action;
      action.type = push_action_type;
      action.gripper_pose =
          Eigen::Translation3d(translation) *
          Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitY());
      if ((*validity_checker)(push_action_type, current_gripper_pose,
                              action.gripper_pose)) {
        candidates.push_back(action);
      }
    }
  }
}

std::function<bool(const Eigen::Isometry3d)>
check_near_to_goal_pose(const Eigen::Isometry3d &goal_pose,
                        const double &translation_threshold,
                        const double &rotation_threshold) {
  return [goal_pose, translation_threshold,
          rotation_threshold](const Eigen::Isometry3d &pose) -> bool {
    Eigen::Isometry3d move = pose * goal_pose.inverse();
    return move.translation().norm() < translation_threshold &&
           Eigen::AngleAxisd(move.rotation()).angle() < rotation_threshold;
  };
}

void Planner::set_geometry(
    const std::shared_ptr<mesh_object> &gripped_geometry,
    const std::shared_ptr<std::vector<Eigen::Isometry3d>> &grasp_points,
    const double &support_surface)
{
  this->gripped_geometry = gripped_geometry;
  this->grasp_points = grasp_points;
  this->support_surface = support_surface;
  center_of_gravity = calculate_center_of_gravity(gripped_geometry->vertices,
                                                  gripped_geometry->triangles);
  calculate_place_candidates(gripped_geometry->vertices, center_of_gravity,
                             place_candidates, convex_hull_vertices);
  Eigen::Isometry3d camera_pose = get_camera_pose();
  looked_point = camera_pose * (-0.10 * Eigen::Vector3d::UnitZ());
}

std::vector<UpdateAction> Planner::calculate_plan(
    const Eigen::Isometry3d &current_gripper_pose, const bool &current_gripping,
    const Eigen::Isometry3d &current_mean,
    const CovarianceMatrix &current_covariance,
    const CovarianceMatrix &objective_coefficients,
    const double &objective_value, const bool goal_gripping,
    const std::function<bool(const Eigen::Isometry3d &)> check_goal_pose) {

  struct node {
    Eigen::Isometry3d mean, gripper_pose;
    CovarianceMatrix covariance;

    int previous_node_id;
    UpdateAction previous_action;
    double cost;
    bool gripping;
  };

  std::vector<node> nodes(1);
  nodes[0].mean = current_mean;
  nodes[0].covariance = current_covariance;
  nodes[0].previous_node_id = -1;
  nodes[0].gripper_pose = current_gripper_pose;
  nodes[0].cost = 0.0;
  nodes[0].gripping = current_gripping;

  std::priority_queue<std::pair<double, int>> open_nodes;

  open_nodes.push(std::make_pair(0.0, 0));
  int goal_node_id = -1;
  double optimal_score = INF;
  while (!open_nodes.empty()) {
    int id = -open_nodes.top().second;
    open_nodes.pop();

    double score = objective_coefficients
                       .cwiseProduct(transform_covariance(
                           nodes[id].mean.inverse(), nodes[id].covariance))
                       .sum();
    fprintf(stderr, "%d %d %d %lf %lf\n", id, nodes[id].previous_node_id,
            id > 0 ? nodes[id].previous_action.type : -1, nodes[id].cost,
            score);
    if (score < objective_value &&
        (!goal_gripping ||
         nodes[id].gripping && check_goal_pose(nodes[id].mean))) {
      goal_node_id = id;
      break;
    }
    optimal_score = score;

    std::vector<UpdateAction> candidates;
    calculate_action_candidates(nodes[id].gripper_pose, nodes[id].mean,
                                nodes[id].covariance, nodes[id].gripping,
                                candidates);
    for (auto &candidate : candidates) {
      node new_node;
      try {
        apply_action(nodes[id].mean, nodes[id].covariance, candidate,
                     new_node.mean, new_node.covariance);
      } catch (std::runtime_error &e) {
        std::cerr << e.what() << std::endl;
        continue;
      }
      new_node.gripper_pose = candidate.gripper_pose;
      new_node.gripping = (candidate.type != place_action_type &&
                           candidate.type != push_action_type);
      new_node.previous_node_id = id;
      new_node.previous_action = candidate;
      new_node.cost = nodes[id].cost + (*cost_function)(candidate.type,
                                                        nodes[id].gripper_pose,
                                                        candidate.gripper_pose);
      int new_id = nodes.size();
      nodes.push_back(new_node);
      open_nodes.push(std::make_pair(-new_node.cost, -new_id));
    }
  }
  if (goal_node_id == -1) {
    throw std::runtime_error("planning failed");
  }
  std::vector<UpdateAction> actions;
  while (goal_node_id != 0) {
    std::cerr << nodes[goal_node_id].mean.matrix() << std::endl;
    std::cerr << nodes[goal_node_id].covariance << std::endl;
    std::cerr << nodes[goal_node_id].previous_action.type << std::endl;
    std::cerr << nodes[goal_node_id].previous_action.gripper_pose.matrix()
              << std::endl;
    actions.push_back(nodes[goal_node_id].previous_action);
    goal_node_id = nodes[goal_node_id].previous_node_id;
  }
  std::reverse(actions.begin(), actions.end());
  return actions;
}

std::vector<std::pair<double, std::vector<UpdateAction>>> Planner::best_scores_for_each_costs(
    const Eigen::Isometry3d &current_gripper_pose, const bool &current_gripping,
    const Eigen::Isometry3d &current_mean,
    const CovarianceMatrix &current_covariance,
    const CovarianceMatrix &objective_coefficients,
    const int &max_cost) {

  struct node {
    Eigen::Isometry3d mean, gripper_pose;
    CovarianceMatrix covariance;

    int previous_node_id;
    UpdateAction previous_action;
    int cost;
    bool gripping;
  };

  std::vector<node> nodes(1);
  nodes[0].mean = current_mean;
  nodes[0].covariance = current_covariance;
  nodes[0].previous_node_id = -1;
  nodes[0].gripper_pose = current_gripper_pose;
  nodes[0].cost = 0.0;
  nodes[0].gripping = current_gripping;

  std::vector<std::pair<double,int> > best_plans(max_cost+1, std::make_pair(INF,-1));
  for(int id=0;;id++){
    if(nodes[id].cost>max_cost){
      break;
    }
    double score = objective_coefficients
                       .cwiseProduct(transform_covariance(
                           nodes[id].mean.inverse(), nodes[id].covariance))
                       .sum();
    fprintf(stderr, "%d %d %d %d %lf\n", id, nodes[id].previous_node_id,
            id > 0 ? nodes[id].previous_action.type : -1, nodes[id].cost,
            score);
    if(nodes[id].gripping && best_plans[nodes[id].cost].first > score){
      best_plans[nodes[id].cost] = std::make_pair(score, id);
    }

    std::vector<UpdateAction> candidates;
    calculate_action_candidates(nodes[id].gripper_pose, nodes[id].mean,
                                nodes[id].covariance, nodes[id].gripping,
                                candidates);
    for (auto &candidate : candidates) {
      node new_node;
      try {
        apply_action(nodes[id].mean, nodes[id].covariance, candidate,
                     new_node.mean, new_node.covariance);
      } catch (std::runtime_error &e) {
        std::cerr << e.what() << std::endl;
        continue;
      }
      new_node.gripper_pose = candidate.gripper_pose;
      new_node.gripping = (candidate.type != place_action_type &&
                           candidate.type != push_action_type);
      new_node.previous_node_id = id;
      new_node.previous_action = candidate;
      new_node.cost = nodes[id].cost + (*cost_function)(candidate.type,
                                                        nodes[id].gripper_pose,
                                                        candidate.gripper_pose);
      nodes.push_back(new_node);
    }
  }
  std::vector<std::pair<double, std::vector<UpdateAction>>> best_actions(max_cost+1);
  for(int cost=0;cost<=max_cost;cost++){
    best_actions[cost].first = best_plans[cost].first;
    if(best_plans[cost].second == -1){
      continue;
    }
    int goal_node_id = best_plans[cost].second;
    while (goal_node_id != 0) {
      best_actions[cost].second.push_back(nodes[goal_node_id].previous_action);
      goal_node_id = nodes[goal_node_id].previous_node_id;
    }
    std::reverse(best_actions[cost].second.begin(), best_actions[cost].second.end());
  }
  return best_actions;
}
