#include "o2ac_pose_distribution_updater/grasp_action_helpers.hpp"
#include "o2ac_pose_distribution_updater/convex_hull.hpp"

namespace {
const double INF = 1e9, EPS = 1e-9, LARGE_EPS = 1e-3;
}

// a class for calculating the pose after grasping
// the calculation has two steps:
// 1. calculate the new_mean (the pose after grasping when the initial pose is
// old_mean)
// 2. provide function to calculate the pose after grasping given a initial pose
// in the neighborhood of old_mean

void cutting_object(const std::vector<Eigen::Vector3d> &vertices,
                    const std::vector<boost::array<int, 3>> &triangles,
                    const Eigen::Hyperplane<double, 3> &plane,
                    std::vector<Eigen::Vector3d> &result_vertices,
                    std::vector<boost::array<int, 3>> &result_triangles) {
  // cut object by plane and take the positive distance side
  // result is stored in result_vertices and result_triangles
  // Faces of cut object on the cut plane is ignored, so the result mesh is
  // imcomplete. but it is not a problem

  std::vector<int> new_index(vertices.size());
  for (int i = 0; i < vertices.size(); i++) {
    if (plane.signedDistance(vertices[i]) >= 0.0) {
      new_index[i] = result_vertices.size();
      result_vertices.push_back(vertices[i]);
    } else {
      new_index[i] = -1;
    }
  }
  std::map<std::pair<int, int>, int> edge_vertex_id;
  for (auto &triangle : triangles) {
    for (int k = 0; k < 3; k++) {
      int a = triangle[k], b = triangle[(k + 1) % 3];
      if (a < b) {
        std::pair<int, int> edge(a, b);
        double distance_0 = plane.signedDistance(vertices[a]),
               distance_1 = plane.signedDistance(vertices[b]);
        if (distance_0 * distance_1 < 0.0) {
          Eigen::Vector3d intersection =
              (distance_1 * vertices[a] - distance_0 * vertices[b]) /
              (distance_1 - distance_0);
          edge_vertex_id[edge] = result_vertices.size();
          result_vertices.push_back(intersection);
        }
      }
    }
  }
  for (auto &triangle : triangles) {
    int number_of_remain = 0;
    for (int k = 0; k < 3; k++) {
      if (new_index[triangle[k]] != -1) {
        number_of_remain++;
      }
    }
    if (number_of_remain == 1) {
      int v0 = 0;
      while (new_index[triangle[v0]] == -1)
        v0++; // triangle[v0] is the only vertices to remain
      int v1 = (v0 + 1) % 3, v2 = (v0 + 2) % 3;
      boost::array<int, 3> new_triangle;
      new_triangle[v0] = new_index[triangle[v0]];
      new_triangle[v1] =
          edge_vertex_id[std::make_pair(std::min(triangle[v0], triangle[v1]),
                                        std::max(triangle[v0], triangle[v1]))];
      new_triangle[v2] =
          edge_vertex_id[std::make_pair(std::min(triangle[v0], triangle[v2]),
                                        std::max(triangle[v0], triangle[v2]))];
      result_triangles.push_back(new_triangle);
    } else if (number_of_remain == 2) {
      int v0 = 0;
      while (new_index[triangle[v0]] != -1)
        v0++; // triangle[v0] is the only vertices not to remain
      int v1 = (v0 + 1) % 3, v2 = (v0 + 2) % 3;
      boost::array<int, 3> new_triangle_0, new_triangle_1;
      new_triangle_0[v0] =
          edge_vertex_id[std::make_pair(std::min(triangle[v0], triangle[v1]),
                                        std::max(triangle[v0], triangle[v1]))];
      new_triangle_0[v1] = new_index[triangle[v1]];
      new_triangle_0[v2] = new_index[triangle[v2]];
      result_triangles.push_back(new_triangle_0);
      new_triangle_1[v0] =
          edge_vertex_id[std::make_pair(std::min(triangle[v0], triangle[v2]),
                                        std::max(triangle[v0], triangle[v2]))];
      new_triangle_1[v1] = new_triangle_0[v0];
      new_triangle_1[v2] = new_triangle_0[v2];
      result_triangles.push_back(new_triangle_1);
    } else if (number_of_remain == 3) {
      boost::array<int, 3> new_triangle;
      for (int k = 0; k < 3; k++) {
        new_triangle[k] = new_index[triangle[k]];
      }
      result_triangles.push_back(new_triangle);
    }
  }
}

grasp_calculator::grasp_calculator(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<Eigen::Vector3d> &all_vertices,
    const Eigen::Isometry3d &gripper_transform,
    const Eigen::Isometry3d &old_mean, const Eigen::Vector3d &center_of_gravity,
    const bool balance_check) {

  // rotate the world coordinates to make the direction of the gripper x-axis
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

  Eigen::Isometry3d current_transform = rotated_gripper_transform * old_mean;
  std::vector<Eigen::Vector3d> current_vertices(vertices.size());
  for (int i = 0; i < vertices.size(); i++) {
    current_vertices[i] = current_transform * vertices[i];
  }

  // find the ground touching vertex before grasping

  std::vector<Eigen::Vector3d> current_all_vertices(all_vertices.size());
  for (int i = 0; i < all_vertices.size(); i++) {
    current_all_vertices[i] = current_transform * all_vertices[i];
  }
  int ground_touch_vertex_id_1 = 0;
  for (int i = 1; i < current_all_vertices.size(); i++) {
    if (current_all_vertices[i](2) <
        current_all_vertices[ground_touch_vertex_id_1](2) - EPS) {
      ground_touch_vertex_id_1 = i;
    }
  }

  // calculate the convex hull of the vertices projected along the z-axis

  namespace bg = boost::geometry;
  std::vector<Eigen::Vector2d> projected_points(current_vertices.size()), hull;

  for (int i = 0; i < current_vertices.size(); i++) {
    projected_points[i] = current_vertices[i].head<2>();
  }

  convex_hull_for_Eigen_Vector2d(projected_points, hull);

  // find the left-most and right-most vertices of the hull

  int hull_size = hull.size();
  int left_vertex_id = 0, right_vertex_id = 0;
  for (int i = 1; i < hull_size; i++) {
    if (hull[i].x() < hull[left_vertex_id].x()) {
      left_vertex_id = i;
    }
    if (hull[i].x() > hull[right_vertex_id].x()) {
      right_vertex_id = i;
    }
  }

  // the object rotates around z-axis

  // calculate the direction

  double first_rotation_value =
      hull[right_vertex_id].y() - hull[left_vertex_id].y();
  first_direction = (first_rotation_value > 0.0 ? 1 : -1);

  // find the rotation angle
  // after rotation, at least one edge of the hull must be on the gripper
  // (parallel to y-axis) and the hull must be stable find such edge
  double rotation_angle;
  int next_vertex_id;
  while (1) {
    int next_left_vertex_id =
        (left_vertex_id - first_direction + hull_size) % hull_size;
    Eigen::Vector2d left_edge =
        hull[next_left_vertex_id] - hull[left_vertex_id];
    double left_angle = std::atan2(
        left_edge(0),
        first_direction * left_edge(1)); // the angle such that the left edge is
                                         // parallel to y-axis after rotation
    int next_right_vertex_id =
        (right_vertex_id - first_direction + hull_size) % hull_size;
    Eigen::Vector2d right_edge =
        hull[next_right_vertex_id] - hull[right_vertex_id];
    double right_angle =
        std::atan2(-right_edge(0),
                   -first_direction *
                       right_edge(1)); // the angle such that the right edge is
                                       // parallel to y-axis after rotation
    if (left_angle <= right_angle) {
      Eigen::Rotation2Dd rotation(first_direction * left_angle);
      double left_y = first_direction * (rotation * hull[left_vertex_id])(1);
      double next_left_y =
          first_direction * (rotation * hull[next_left_vertex_id])(1);
      double right_y = first_direction * (rotation * hull[right_vertex_id])(1);
      if (left_y - LARGE_EPS <= right_y && right_y <= next_left_y + LARGE_EPS &&
          left_y + EPS < next_left_y) {
        // the hull is stable after rotation
        rotation_angle = left_angle;
        next_vertex_id = next_left_vertex_id;
        double_vertex_side = -1;
        break;
      }
      left_vertex_id = next_left_vertex_id;
    } else {
      Eigen::Rotation2Dd rotation(first_direction * right_angle);
      double left_y = first_direction * (rotation * hull[left_vertex_id])(1);
      double right_y = first_direction * (rotation * hull[right_vertex_id])(1);
      double next_right_y =
          first_direction * (rotation * hull[next_right_vertex_id])(1);
      if (next_right_y - LARGE_EPS <= left_y && left_y <= right_y + LARGE_EPS &&
          next_right_y + EPS < right_y) {
        // the hull is stable after rotation
        rotation_angle = right_angle;
        next_vertex_id = next_right_vertex_id;
        double_vertex_side = 1;
        break;
      }
      right_vertex_id = next_right_vertex_id;
    }
  }
  if (balance_check && std::abs(first_rotation_value) < EPS &&
      rotation_angle > EPS) {
    throw(std::runtime_error("Balanced at the first rotation"));
  }

  // find the vertices of the object corresponding the vertices of the hull
  int gripper_touch_vertex_id_1, gripper_touch_vertex_id_2,
      gripper_touch_vertex_id_3, gripper_touch_vertex_id_4;
  // define the function to find the vertices of the object corresponding the
  // vertices of hull
  auto search_vertex = [&](int j) {
    for (int i = 0; i < current_vertices.size(); i++) {
      if ((current_vertices[i].block(0, 0, 2, 1) - hull[j]).norm() < EPS) {
        return i;
      }
    }
  };
  // gripper_touch_vertex_1 and gripper_touch_vertex_2 are on the same side,
  // which double_vertex_side means gripper_touch_vertex_3 is on the other side
  if (double_vertex_side == -1) {
    gripper_touch_vertex_id_1 = search_vertex(left_vertex_id);
    gripper_touch_vertex_id_2 = search_vertex(next_vertex_id);
    gripper_touch_vertex_id_3 = search_vertex(right_vertex_id);
  } else {
    gripper_touch_vertex_id_1 = search_vertex(right_vertex_id);
    gripper_touch_vertex_id_2 = search_vertex(next_vertex_id);
    gripper_touch_vertex_id_3 = search_vertex(left_vertex_id);
  }

  // rotated vertices
  Eigen::AngleAxisd first_rotation(first_direction * rotation_angle,
                                   Eigen::Vector3d::UnitZ());
  std::vector<Eigen::Vector3d> rotated_vertices(current_vertices.size());
  for (int i = 0; i < current_vertices.size(); i++) {
    rotated_vertices[i] = first_rotation * current_vertices[i];
  }

  // check conditions
  for (int i = 0; i < rotated_vertices.size(); i++) {
    if (double_vertex_side == -1) {
      assert(rotated_vertices[gripper_touch_vertex_id_1](0) - LARGE_EPS <=
             rotated_vertices[i](0));
      assert(rotated_vertices[gripper_touch_vertex_id_3](0) + LARGE_EPS >=
             rotated_vertices[i](0));
    } else {
      assert(rotated_vertices[gripper_touch_vertex_id_3](0) - LARGE_EPS <=
             rotated_vertices[i](0));
      assert(rotated_vertices[gripper_touch_vertex_id_1](0) + LARGE_EPS >=
             rotated_vertices[i](0));
    }
  }
  assert(double_vertex_side * first_direction *
                 rotated_vertices[gripper_touch_vertex_id_1](1) +
             LARGE_EPS >=
         double_vertex_side * first_direction *
             rotated_vertices[gripper_touch_vertex_id_3](1));
  assert(double_vertex_side * first_direction *
                 rotated_vertices[gripper_touch_vertex_id_3](1) +
             LARGE_EPS >=
         double_vertex_side * first_direction *
             rotated_vertices[gripper_touch_vertex_id_2](1));

  // after first rotation, the object rotates around the axis connecting
  // gripper_touch_vertex_1 and gripper_touch_vertex_2 till at least one other
  // vertex is on the left or right gripper

  // calculate the axis
  Eigen::Vector3d second_axis = (rotated_vertices[gripper_touch_vertex_id_2] -
                                 rotated_vertices[gripper_touch_vertex_id_1])
                                    .normalized();
  // the rotation direction depends on gripper_touch_vertex_3
  double second_direction_value =
      -double_vertex_side * (rotated_vertices[gripper_touch_vertex_id_3] -
                             rotated_vertices[gripper_touch_vertex_id_1])
                                .cross(second_axis)(0);
  second_direction = (second_direction_value > 0.0 ? 1 : -1);
  second_axis = second_direction * second_axis;

  // find the gripper_touch_vertex_4
  double second_angle = INF;
  for (int i = 0; i < rotated_vertices.size(); i++) {
    if (i == gripper_touch_vertex_id_1 || i == gripper_touch_vertex_id_2 ||
        i == gripper_touch_vertex_id_3) {
      continue;
    }
    // calculate the angle such that the vertex i is on the same side gripper
    // with gripper_touch_vertex_1 after rotation
    Eigen::Vector3d v1v4 =
        rotated_vertices[i] - rotated_vertices[gripper_touch_vertex_id_1];
    double v1_side_angle =
        abs(v1v4(0)) < EPS &&
                abs(second_axis(2) * v1v4(1) - second_axis(1) * v1v4(2)) < EPS
            ? INF
            : std::atan2(abs(v1v4(0)),
                         -double_vertex_side * (second_axis(2) * v1v4(1) -
                                                second_axis(1) * v1v4(2)));
    if (v1_side_angle < second_angle - EPS) {
      second_angle = v1_side_angle;
      fourth_vertex_side = double_vertex_side;
      gripper_touch_vertex_id_4 = i;
    }

    // calculate the angle such that the vertex i is on the same side gripper
    // with gripper_touch_vertex_3 after rotation
    Eigen::Vector3d v3v4 =
        rotated_vertices[i] - rotated_vertices[gripper_touch_vertex_id_3];
    double v3_side_angle =
        abs(v3v4(0)) < EPS &&
                abs(second_axis(2) * v3v4(1) - second_axis(1) * v3v4(2)) < EPS
            ? INF
            : std::atan2(abs(v3v4(0)),
                         double_vertex_side * (second_axis(2) * v3v4(1) -
                                               second_axis(1) * v3v4(2)));
    if (v3_side_angle < second_angle - EPS) {
      second_angle = v3_side_angle;
      fourth_vertex_side = -double_vertex_side;
      gripper_touch_vertex_id_4 = i;
    }
  }
  if (balance_check && std::abs(second_direction_value) < EPS &&
      second_angle > EPS) {
    throw std::runtime_error("Balanced at the second rotation");
  }
  // rotates the vertices
  Eigen::AngleAxisd second_rotation(second_angle, second_axis);
  std::vector<Eigen::Vector3d> final_vertices(rotated_vertices.size());
  for (int i = 0; i < rotated_vertices.size(); i++) {
    final_vertices[i] = second_rotation * rotated_vertices[i];
  }
  Eigen::Matrix3d total_rotation = (second_rotation * first_rotation).matrix();
  std::vector<Eigen::Vector3d> final_all_vertices(current_all_vertices.size());
  for (int i = 0; i < current_all_vertices.size(); i++) {
    final_all_vertices[i] = total_rotation * current_all_vertices[i];
  }

  // find the ground touching vertex after grasping
  int ground_touch_vertex_id_2 = 0;
  for (int i = 1; i < final_all_vertices.size(); i++) {
    if (final_all_vertices[i](2) <
        final_all_vertices[ground_touch_vertex_id_2](2) - EPS) {
      ground_touch_vertex_id_2 = i;
    }
  }

  gripper_touch_vertex_1 = vertices[gripper_touch_vertex_id_1];
  gripper_touch_vertex_2 = vertices[gripper_touch_vertex_id_2];
  gripper_touch_vertex_3 = vertices[gripper_touch_vertex_id_3];
  gripper_touch_vertex_4 = vertices[gripper_touch_vertex_id_4];

  ground_touch_vertex_1 = all_vertices[ground_touch_vertex_id_1];
  ground_touch_vertex_2 = all_vertices[ground_touch_vertex_id_2];

  // check the conditions
  for (int i = 0; i < rotated_vertices.size(); i++) {
    if (double_vertex_side == -1) {
      assert(final_vertices[gripper_touch_vertex_id_1](0) - LARGE_EPS <=
             final_vertices[i](0));
      assert(final_vertices[gripper_touch_vertex_id_3](0) + LARGE_EPS >=
             final_vertices[i](0));
    } else {
      assert(final_vertices[gripper_touch_vertex_id_3](0) - LARGE_EPS <=
             final_vertices[i](0));
      assert(final_vertices[gripper_touch_vertex_id_1](0) + LARGE_EPS >=
             final_vertices[i](0));
    }
  }

  // stability check
  double left_x, right_x;
  // collect the vertex on the left and right grippers
  if (double_vertex_side == -1) {
    left_x = final_vertices[gripper_touch_vertex_id_1](0);
    right_x = final_vertices[gripper_touch_vertex_id_3](0);
  } else {
    left_x = final_vertices[gripper_touch_vertex_id_3](0);
    right_x = final_vertices[gripper_touch_vertex_id_1](0);
  }
  std::vector<Eigen::Vector2d> points_on_left_gripper, points_on_right_gripper;

  for (auto &vertex : final_vertices) {
    if (vertex(0) <= left_x + LARGE_EPS) {
      points_on_left_gripper.push_back(vertex.tail<2>());
    }
    if (vertex(0) >= right_x - LARGE_EPS) {
      points_on_right_gripper.push_back(vertex.tail<2>());
    }
  }
  if (!do_intersect_convex_hulls(points_on_left_gripper,
                                 points_on_right_gripper)) {
    throw(std::runtime_error("Unstable after gripping"));
  }

  // calculate the pose after grasping

  Eigen::Vector3d current_center = current_transform * center_of_gravity;
  Eigen::Vector3d final_center =
      second_rotation * first_rotation * current_center;
  double gripper_central_x = rotated_gripper_transform.translation()(0);
  double object_middle_x = (final_vertices[gripper_touch_vertex_id_1](0) +
                            final_vertices[gripper_touch_vertex_id_3](0)) /
                           2.0;
  Eigen::Vector3d total_translation;
  total_translation
      << gripper_central_x -
             object_middle_x, // the average of the x-coordinates of left and
                              // right gripper should be the same with the
                              // center of the gripper
      current_center(1) - final_center(1), // the y-coordinate of the center of
                                           // gravity should be not changed
      current_all_vertices[ground_touch_vertex_id_1](2) -
          final_all_vertices[ground_touch_vertex_id_2](
              2); // the z-coordinate of the bottom point of the
                  // object should be not changed

  // calculate the pose after grasping
  new_mean = rotated_gripper_transform.inverse() *
             Eigen::Translation3d(total_translation) * second_rotation *
             first_rotation * current_transform;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry>
grasp_calculator::calculate_transform_after_grasping(
    const Eigen::Transform<T, 3, Eigen::Isometry> &old_transform) const {
  // calculate the pose after grasping when the current pose is old_transform

  // calculate the first and second rotations
  // this calculation is the same with that in the constructor except the vertex
  // is fixed and the coordinate type is template T
  using point = Eigen::Matrix<T, 3, 1>;
  Eigen::Transform<T, 3, Eigen::Isometry> current_transform =
      rotated_gripper_transform.cast<T>() * old_transform;
  point current_vertex_1 = current_transform * gripper_touch_vertex_1.cast<T>();
  point current_vertex_2 = current_transform * gripper_touch_vertex_2.cast<T>();
  point current_vertex_3 = current_transform * gripper_touch_vertex_3.cast<T>();
  point current_vertex_4 = current_transform * gripper_touch_vertex_4.cast<T>();
  point v1v2 = current_vertex_2 - current_vertex_1;
  T first_angle = atan2((T)(-double_vertex_side * v1v2(0)),
                        (T)(-first_direction * double_vertex_side * v1v2(1)));
  Eigen::AngleAxis<T> first_rotation(first_direction * first_angle,
                                     point::UnitZ());
  point rotated_vertex_1 = first_rotation * current_vertex_1;
  point rotated_vertex_2 = first_rotation * current_vertex_2;
  point rotated_vertex_3 = first_rotation * current_vertex_3;
  point rotated_vertex_4 = first_rotation * current_vertex_4;
  point second_axis =
      second_direction * (rotated_vertex_2 - rotated_vertex_1).normalized();
  T second_angle;
  if (fourth_vertex_side == double_vertex_side) {
    point v1v4 = rotated_vertex_4 - rotated_vertex_1;
    second_angle =
        abs(v1v4(0)) < EPS &&
                second_axis(2) * v1v4(1) - second_axis(1) * v1v4(2) < EPS
            ? (T)0.0
            : (T)atan2((T)(-double_vertex_side * v1v4(0)),
                       (T)(-double_vertex_side * (second_axis(2) * v1v4(1) -
                                                  second_axis(1) * v1v4(2))));
  } else {
    point v3v4 = rotated_vertex_4 - rotated_vertex_3;
    second_angle =
        abs(v3v4(0)) < EPS &&
                second_axis(2) * v3v4(1) - second_axis(1) * v3v4(2) < EPS
            ? (T)0.0
            : (T)atan2((T)(double_vertex_side * v3v4(0)),
                       (T)(double_vertex_side * (second_axis(2) * v3v4(1) -
                                                 second_axis(1) * v3v4(2))));
  }
  Eigen::AngleAxis<T> second_rotation(second_angle, second_axis);

  // calculate the translation occurred by grasping
  point current_center = current_transform * center_of_gravity.cast<T>();
  point final_center = second_rotation * first_rotation * current_center;
  T gripper_central_x = rotated_gripper_transform.translation()(0);
  T object_middle_x = ((second_rotation * rotated_vertex_1)(0) +
                       (second_rotation * rotated_vertex_3)(0)) /
                      2.0;
  T ground_touch_z_before =
      (current_transform * ground_touch_vertex_1.cast<T>())(2);
  T ground_touch_z_after =
      (second_rotation * first_rotation * current_transform *
       ground_touch_vertex_2.cast<T>())(2);
  point total_translation;
  total_translation
      << gripper_central_x -
             object_middle_x, // the average of the x-coordinates of left and
                              // right gripper should be the same with the
                              // center of the gripper
      current_center(1) - final_center(1), // the y-coordinate of the center of
                                           // gravity should be not changed
      ground_touch_z_before -
          ground_touch_z_after; // the z-coordinate of the bottom point of the
                                // object should be not changed

  // calculate the pose after grasping
  return rotated_gripper_transform.inverse().cast<T>() *
         Eigen::Translation<T, 3>(total_translation) * second_rotation *
         first_rotation * current_transform;
}

// the class to use Autodiff
class calculate_perturbation_after_grasping : public grasp_calculator {

public:
  using grasp_calculator::grasp_calculator;

  // Neede by Eigen AutoDiff
  enum { InputsAtCompileTime = 6, ValuesAtCompileTime = 6 };

  // Also needed by Eigen AutoDiff
  typedef Eigen::Matrix<double, 6, 1> InputType;
  typedef Eigen::Matrix<double, 6, 1> ValueType;

  // The Vector function from the perturbation representing the pose before
  // grasping to the perturbation representing the pose after grasping. To use
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

    // calculate transform after grasping

    Eigen::Transform<T, 3, Eigen::Isometry> result_transform =
        calculate_transform_after_grasping(input_transform);

    // calculate perturbation in result_transform
    *output_perturbation = check_operator<T>(
        -Eigen::Matrix<T, 4, 4>::Identity() +
        (result_transform * new_mean.cast<T>().inverse())
            .matrix()); // the first approximation of
                        // check_operator(log(result_transform * new_mean^{-1}))
  }
};

void grasp_update_Lie_distribution(
    const Eigen::Isometry3d &old_mean, const CovarianceMatrix &old_covariance,
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<Eigen::Vector3d> &all_vertices,
    const Eigen::Vector3d &center_of_gravity,
    const Eigen::Isometry3d &gripper_transform, Eigen::Isometry3d &new_mean,
    CovarianceMatrix &new_covariance) {

  // AutoDiff class
  Eigen::AutoDiffJacobian<calculate_perturbation_after_grasping>
      calculate_perturbation_AD(vertices, all_vertices, gripper_transform,
                                old_mean, center_of_gravity);
  // By Eigen AutoDiff, calculate_particle_AD automatically calculates the
  // operation of calculate_particle and its Jacobian
  Eigen::Matrix<double, 6, 1> mean_perturbation;
  CovarianceMatrix Jacobian;
  calculate_perturbation_AD(Eigen::Matrix<double, 6, 1>::Zero(),
                            &mean_perturbation, &Jacobian);
  assert(mean_perturbation.norm() < LARGE_EPS);

  // new_mean is calculated in the class calculate_perturbation_AD
  new_mean = calculate_perturbation_AD.new_mean;

  // The covariance of the function value is calculated by the covariance of the
  // argument and Jacobian.
  new_covariance = Jacobian * old_covariance * Jacobian.transpose();
}
