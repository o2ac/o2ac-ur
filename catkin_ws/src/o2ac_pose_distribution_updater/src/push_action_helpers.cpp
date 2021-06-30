#include "o2ac_pose_distribution_updater/push_action_helpers.hpp"

namespace {
const double INF = 1e9, EPS = 1e-9, LARGE_EPS = 1e-7;
}

push_calculator::push_calculator(const std::vector<Eigen::Vector3d> &vertices,
                                 const Eigen::Isometry3d &gripper_transform,
                                 const Eigen::Isometry3d &old_mean,
                                 const Eigen::Vector3d &center_of_gravity,
                                 const double &gripper_width,
                                 const bool balance_check) {

  // rotate the world coordinates to make the direction of the gripper y-axis
  Eigen::Vector3d gripping_direction =
      gripper_transform.rotation() * Eigen::Vector3d::UnitY();
  if (abs(gripping_direction(2)) > LARGE_EPS) {
    throw(std::runtime_error("Invalid gripper transform"));
  }
  Eigen::AngleAxisd initial_rotation(
      -std::atan2(gripping_direction(1), gripping_direction(0)),
      Eigen::Vector3d::UnitZ());
  rotated_gripper_transform = initial_rotation * gripper_transform;

  this->old_mean = old_mean;
  this->center_of_gravity = center_of_gravity;
  this->gripper_width = gripper_width;

  Eigen::Isometry3d current_transform = rotated_gripper_transform * old_mean;
  std::vector<Eigen::Vector3d> current_vertices;
  std::transform(vertices.begin(), vertices.end(),
                 std::back_inserter(current_vertices),
                 [current_transform](const Eigen::Vector3d &vertex) {
                   return current_transform * vertex;
                 });

  // calculate the convex hull of the vertices projected along the z-axis

  Eigen::Vector2d projected_center =
      (current_transform * center_of_gravity).block(0, 0, 2, 1);

  namespace bg = boost::geometry;
  bg::model::multi_point<Eigen::Vector2d> projected_points;

  for (auto &vertex : current_vertices) {
    bg::append(projected_points, (Eigen::Vector2d)vertex.block(0, 0, 2, 1));
  }

  static const bool clock_wise = false;
  bg::model::ring<Eigen::Vector2d, clock_wise> hull;
  bg::convex_hull(projected_points, hull);
  hull.resize(hull.size() - 1);

  // find the left-most and right-most vertices of the hull

  int hull_size = hull.size();
  int left_vertex_id = 0, next_left_vertex_id;
  for (int i = 1; i < hull_size; i++) {
    if (hull[i].x() < hull[left_vertex_id].x() - EPS) {
      left_vertex_id = i;
    }
  }

  // the object rotates around z-axis

  // calculate the direction

  double first_rotation_value = projected_center.y() - hull[left_vertex_id].y();
  first_direction = (first_rotation_value > 0.0 ? 1 : -1);

  // find the rotation angle
  // after rotation, at least one edge of the hull must be on the gripper
  // (parallel to y-axis) and the hull must be stable find such edge
  Eigen::Rotation2Dd rotation;
  while (1) {
    next_left_vertex_id =
        (left_vertex_id - first_direction + hull_size) % hull_size;
    Eigen::Vector2d left_edge =
        hull[next_left_vertex_id] - hull[left_vertex_id];
    double left_angle = std::atan2(
        left_edge(0),
        first_direction * left_edge(1)); // the angle such that the left edge is
                                         // parallel to y-axis after rotation
    rotation = Eigen::Rotation2Dd(first_direction * left_angle);
    double left_y = first_direction * (rotation * hull[left_vertex_id])(1);
    double next_left_y =
        first_direction * (rotation * hull[next_left_vertex_id])(1);
    double center_y = first_direction * (rotation * projected_center)(1);
    if (left_y - LARGE_EPS <= center_y && center_y <= next_left_y + LARGE_EPS &&
        left_y + EPS < next_left_y) {
      // the hull is stable after rotation
      break;
    }
    left_vertex_id = next_left_vertex_id;
  }
  if (balance_check && std::abs(first_rotation_value) < EPS &&
      rotation.angle() > EPS) {
    throw(std::runtime_error("Balanced at the first rotation"));
  }

  // find the vertices of the object corresponding the vertices of the hull

  // define the function to find the vertices of the object corresponding the
  // vertices of hull
  auto search_vertex = [&](int j) {
    for (int i = 0; i < current_vertices.size(); i++) {
      if ((current_vertices[i].block(0, 0, 2, 1) - hull[j]).norm() < EPS) {
        return i;
      }
    }
  };
  int gripper_touch_vertex_id_1 = search_vertex(left_vertex_id);
  int gripper_touch_vertex_id_2 = search_vertex(next_left_vertex_id);

  gripper_touch_vertex_1 = vertices[gripper_touch_vertex_id_1];
  gripper_touch_vertex_2 = vertices[gripper_touch_vertex_id_2];

  // calculate the pose after pushing

  Eigen::Vector3d total_translation;
  double x_shift = rotated_gripper_transform.translation()(0) +
                   gripper_width / 2.0 - (rotation * hull[left_vertex_id])(0);
  if (x_shift < 0.0) {
    throw std::runtime_error("gripper does not reach the object");
  }
  total_translation << x_shift,
      // the the x-coordinates of gripper should be gripper_width
      projected_center(1) -
          (rotation * projected_center)(1), // the y-coordinate of the center of
      // gravity should be not changed
      0.0;

  // calculate the pose after pushing
  new_mean = rotated_gripper_transform.inverse() *
             Eigen::Translation3d(total_translation) *
             Eigen::AngleAxisd(rotation.angle(), Eigen::Vector3d::UnitZ()) *
             current_transform;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry>
push_calculator::calculate_transform_after_pushing(
    const Eigen::Transform<T, 3, Eigen::Isometry> &old_transform) const {
  // calculate the pose after pushing when the current pose is old_transform

  // calculate the first and second rotations
  // this calculation is the same with that in the constructor except the vertex
  // is fixed and the coordinate type is template T
  using point = Eigen::Matrix<T, 3, 1>;
  Eigen::Transform<T, 3, Eigen::Isometry> current_transform =
      rotated_gripper_transform.cast<T>() * old_transform;
  point current_vertex_1 = current_transform * gripper_touch_vertex_1.cast<T>();
  point current_vertex_2 = current_transform * gripper_touch_vertex_2.cast<T>();
  point v1v2 = current_vertex_2 - current_vertex_1;
  T first_angle = atan2((T)(v1v2(0)), (T)(first_direction * v1v2(1)));
  Eigen::AngleAxis<T> rotation(first_direction * first_angle, point::UnitZ());
  point current_center = current_transform * center_of_gravity.cast<T>();
  point total_translation;
  total_translation << rotated_gripper_transform.translation()(0) +
                           gripper_width / 2.0 -
                           (rotation * current_vertex_1)(
                               0), // the x-coordinates of left gripper should
                                   // be gripper_width
      current_center(1) -
          (rotation * current_center)(1), // the y-coordinate of the center of
                                          // gravity should be not changed
      0.0;

  // calculate the pose after pushing
  return rotated_gripper_transform.inverse().cast<T>() *
         Eigen::Translation<T, 3>(total_translation) * rotation *
         current_transform;
}

// the class to use Autodiff
class calculate_perturbation_after_pushing : push_calculator {

public:
  using push_calculator::push_calculator;

  // Neede by Eigen AutoDiff
  enum { InputsAtCompileTime = 6, ValuesAtCompileTime = 6 };

  // Also needed by Eigen AutoDiff
  typedef Eigen::Matrix<double, 6, 1> InputType;
  typedef Eigen::Matrix<double, 6, 1> ValueType;

  // The Vector function from the perturbation representing the pose before
  // pushing to the perturbation representing the pose after pushing. To use
  // AutoDiff, the type of coordinates is templated by typename "T".
  template <typename T>
  void operator()(const Eigen::Matrix<T, 6, 1> &input_perturbation,
                  Eigen::Matrix<T, 6, 1> *output_perturbation) const {

    // add perturbation to old_mean
    Eigen::Transform<T, 3, Eigen::Isometry> input_transform =
        Eigen::Transform<T, 3, Eigen::Isometry>(
            Eigen::Matrix<T, 4, 4>::Identity() +
            hat_operator<T>(input_perturbation)) *
        old_mean.cast<T>(); // the first approximation of
                            // exp(hat_operator(input_perturbation)) * old_mean

    // calculate transform after pushing

    Eigen::Transform<T, 3, Eigen::Isometry> result_transform =
        calculate_transform_after_pushing(input_transform);

    // calculate perturbation in result_transform
    *output_perturbation = check_operator<T>(
        -Eigen::Matrix<T, 4, 4>::Identity() +
        (result_transform * new_mean.cast<T>().inverse())
            .matrix()); // the first approximation of
                        // check_operator(log(result_transform * new_mean^{-1}))
  }

  // the function to return new_mean
  Eigen::Isometry3d get_new_mean() { return new_mean; }
};

void push_update_Lie_distribution(const Eigen::Isometry3d &old_mean,
                                  const CovarianceMatrix &old_covariance,
                                  const std::vector<Eigen::Vector3d> &vertices,
                                  const Eigen::Vector3d &center_of_gravity,
                                  const Eigen::Isometry3d &gripper_transform,
                                  const double &gripper_width,
                                  Eigen::Isometry3d &new_mean,
                                  CovarianceMatrix &new_covariance) {

  // AutoDiff class
  Eigen::AutoDiffJacobian<calculate_perturbation_after_pushing>
      calculate_perturbation_AD(vertices, gripper_transform, old_mean,
                                center_of_gravity, gripper_width);
  // By Eigen AutoDiff, calculate_particle_AD automatically calculates the
  // operation of calculate_particle and its Jacobian
  Eigen::Matrix<double, 6, 1> mean_perturbation;
  CovarianceMatrix Jacobian;
  calculate_perturbation_AD(Eigen::Matrix<double, 6, 1>::Zero(),
                            &mean_perturbation, &Jacobian);
  assert(mean_perturbation.norm() < LARGE_EPS);

  // new_mean is calculated in the class calculate_perturbation_AD
  new_mean = calculate_perturbation_AD.get_new_mean();

  // The covariance of the function value is calculated by the covariance of the
  // argument and Jacobian.
  new_covariance = Jacobian * old_covariance * Jacobian.transpose();
}
