/*
Helper functions for the place action
 */

#include "o2ac_pose_distribution_updater/place_action_helpers.hpp"

int argmin(const std::vector<double> &vec) {
  return std::distance(vec.begin(), std::min_element(vec.begin(), vec.end()));
}

void find_three_points(const std::vector<Eigen::Vector3d> &current_vertices,
                       const Eigen::Vector3d &current_center_of_gravity,
                       int &ground_touch_vertex_id_1,
                       int &ground_touch_vertex_id_2,
                       int &ground_touch_vertix_id_3,
                       Eigen::Quaterniond &rotation, bool &stability) {
  // Given the coordinates of vertices and center of gravity, find the three
  // points touching the ground after placing The object rotation occured by
  // placing is stored to 'rotation' Stabliity after placing is checked and
  // stored to 'stability'

  const double INF = 1e9, EPS = 1e-6;

  int number_of_vertices = current_vertices.size();

  // The first point touching the ground is the vertice with the minimum z
  // coordinate
  std::vector<double> current_vertices_z(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    current_vertices_z[i] = current_vertices[i](2);
  }
  ground_touch_vertex_id_1 = argmin(current_vertices_z);

  // After the first point touched the ground, the object rotates with an axis,
  // which is orthogonal to both z axis and the line connecting the first point
  // and the center of gravity The direction of rotation is the one such that
  // the center of gravity approached the ground
  Eigen::Vector3d first_axis =
      (current_vertices[ground_touch_vertex_id_1] - current_center_of_gravity)
          .cross(Eigen::Vector3d::UnitZ());
  if (first_axis.norm() < EPS) {
    throw std::runtime_error("Balanced at the first rotation");
  }
  first_axis = first_axis.normalized();
  // When another point touches the ground, the rotation stops
  // So the second touching point is the vertices with the minimum rotation
  // angle to touch the ground
  std::vector<double> first_angles(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    auto v1v2 =
        current_vertices[i] - current_vertices[ground_touch_vertex_id_1];
    first_angles[i] = (i == ground_touch_vertex_id_1
                           ? INF
                           : std::atan2(v1v2(2), first_axis(1) * v1v2(0) -
                                                     first_axis(0) * v1v2(1)));
  }
  ground_touch_vertex_id_2 = argmin(first_angles);

  // calculate the coordinates of vertices and the center of gravity after the
  // first rotation
  auto first_rotation =
      Eigen::AngleAxisd(first_angles[ground_touch_vertex_id_2], first_axis);
  auto rotated_center_of_gravity = first_rotation * current_center_of_gravity;
  std::vector<Eigen::Vector3d> rotated_vertices(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    rotated_vertices[i] = first_rotation * current_vertices[i];
  }

  // After the second point touched the ground, the object rotates with an axis,
  // which is the line connecting the first touching point and the second
  // touching point
  auto second_axis = (rotated_vertices[ground_touch_vertex_id_2] -
                      rotated_vertices[ground_touch_vertex_id_1])
                         .normalized();
  // The direction of rotation is the one such that the center of gravity
  // approached the ground
  double direction =
      (rotated_center_of_gravity - rotated_vertices[ground_touch_vertex_id_1])
          .cross(second_axis)(2);
  if (std::abs(direction) < EPS) {
    throw std::runtime_error("Balanced at the second rotation");
  }
  if (direction < 0.0) {
    second_axis = -second_axis;
  }
  // When another point touches the ground, the rotation stops
  // So the third touching point is the vertices with the minimum rotation angle
  // to touch the ground
  std::vector<double> second_angles(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    auto v1v3 =
        rotated_vertices[i] - rotated_vertices[ground_touch_vertex_id_1];
    second_angles[i] =
        (i == ground_touch_vertex_id_1 || i == ground_touch_vertex_id_2
             ? INF
             : std::atan2(v1v3(2),
                          second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1)));
  }
  ground_touch_vertix_id_3 = argmin(second_angles);

  // calculate the total rotation
  auto second_rotation =
      Eigen::AngleAxisd(second_angles[ground_touch_vertix_id_3], second_axis);
  rotation = second_rotation * first_rotation;

  // stability check

  // calculate the coordinates after the second rotation
  auto final_center_of_gravity = second_rotation * rotated_center_of_gravity;
  std::vector<Eigen::Vector3d> final_vertices(number_of_vertices);
  for (int i = 0; i < number_of_vertices; i++) {
    final_vertices[i] = second_rotation * rotated_vertices[i];
  }

  // calculate the convex hull of the vertices touching the ground
  double min_z = final_vertices[ground_touch_vertex_id_1](2);

  namespace bg = boost::geometry;
  using bg_2d_point = bg::model::d2::point_xy<double>;
  bg::model::multi_point<bg_2d_point> points_on_ground;

  for (int i = 0; i < number_of_vertices; i++) {
    auto vertice = final_vertices[i];
    if (vertice(2) <= min_z + EPS) {
      bg::append(points_on_ground, bg_2d_point(vertice(0), vertice(1)));
    }
  }

  bg::model::polygon<bg_2d_point> hull;
  bg::convex_hull(points_on_ground, hull);

  // If the center of geometry projected to ground is in the convex hull, the
  // object is stable
  bg_2d_point projected_center_of_gravity(final_center_of_gravity(0),
                                          final_center_of_gravity(1));

  stability = bg::within(projected_center_of_gravity, hull);
}

namespace {

// Calculate the function from the pose before placing to the pose after placing
// and its Jacobian. To use AutoDiff in Eigen-unsupported, the function is
// calculated in the class "calculate_particle".

Eigen::Vector3d center_of_gravity, ground_touch_vertex_1, ground_touch_vertex_2,
    ground_touch_vertex_3; // the coordinates of center of gravity, first
                           // touching point, second touching point and third
                           // touching point
double support_surface;    // the z-coordinate of the ground
Eigen::Isometry3d gripper_transform; // The gripper transform

class calculate_particle {

public:
  // Neede by Eigen AutoDiff
  enum { InputsAtCompileTime = 6, ValuesAtCompileTime = 6 };

  // Also needed by Eigen AutoDiff
  typedef Eigen::Matrix<double, 6, 1> InputType;
  typedef Eigen::Matrix<double, 6, 1> ValueType;

  // The Vector function from the particle representing the pose before placing
  // to the particle representing the pose after placing. To use AutoDiff, the
  // type of coordinates is templated by typename "T".
  template <typename T>
  void operator()(const Eigen::Matrix<T, 6, 1> &current_particle,
                  Eigen::Matrix<T, 6, 1> *result_particle) const {
    using point = Eigen::Matrix<T, 3, 1>;

    // calculate the coordinates when the pose is represented by particle
    // 'current_particle'
    auto gripper_transform_T =
        (Eigen::Transform<T, 3, Eigen::Isometry>)gripper_transform;
    auto current_transform =
        gripper_transform_T *
        Eigen::Translation<T, 3>(current_particle.block(0, 0, 3, 1)) *
        Eigen::AngleAxis<T>(current_particle(5), point::UnitZ()) *
        Eigen::AngleAxis<T>(current_particle(4), point::UnitY()) *
        Eigen::AngleAxis<T>(current_particle(3), point::UnitX());
    point current_center_of_gravity =
        current_transform * (point)center_of_gravity;
    point current_ground_touch_vertex_1 =
        current_transform * (point)ground_touch_vertex_1;
    point current_ground_touch_vertex_2 =
        current_transform * (point)ground_touch_vertex_2;
    point current_ground_touch_vertex_3 =
        current_transform * (point)ground_touch_vertex_3;

    // calculate the first rotation
    point v1v2 = current_ground_touch_vertex_2 - current_ground_touch_vertex_1;
    point first_axis =
        (current_ground_touch_vertex_1 - current_center_of_gravity)
            .cross(point::UnitZ())
            .normalized();
    T first_angle =
        atan2(v1v2(2), first_axis(1) * v1v2(0) - first_axis(0) * v1v2(1));
    auto first_rotation = Eigen::AngleAxis<T>(first_angle, first_axis);

    // calculate the coordinates after first rotation
    point rotated_center_of_gravity =
        first_rotation * current_center_of_gravity;
    point rotated_ground_touch_vertex_1 =
        first_rotation * current_ground_touch_vertex_1;
    point rotated_ground_touch_vertex_2 =
        first_rotation * current_ground_touch_vertex_2;
    point rotated_ground_touch_vertex_3 =
        first_rotation * current_ground_touch_vertex_3;

    // calculate the second rotation
    point v1v3 = rotated_ground_touch_vertex_3 - rotated_ground_touch_vertex_1;
    point second_axis =
        (rotated_ground_touch_vertex_2 - rotated_ground_touch_vertex_1)
            .normalized();
    if (second_axis.cross(point::UnitZ())
            .dot(rotated_center_of_gravity - rotated_ground_touch_vertex_1) <
        0) {
      second_axis = -second_axis;
    }
    T second_angle =
        atan2(v1v3(2), second_axis(1) * v1v3(0) - second_axis(0) * v1v3(1));
    auto second_rotation(Eigen::AngleAxis<T>(second_angle, second_axis));

    // calculate the coordinates after second rotation
    auto final_center_of_gravity = second_rotation * rotated_center_of_gravity;
    auto final_ground_touch_vertex_1 =
        second_rotation * rotated_ground_touch_vertex_1;
    auto final_ground_touch_vertex_2 =
        second_rotation * rotated_ground_touch_vertex_2;
    auto final_ground_touch_vertex_3 =
        second_rotation * rotated_ground_touch_vertex_3;
    // The translation is occured to hold the physical restraints
    point final_translation =
        (current_center_of_gravity(0) - final_center_of_gravity(0)) *
            (point::UnitX()) // The x-coordinate of the center of gravity is not
                             // changed
        + (current_center_of_gravity(1) - final_center_of_gravity(1)) *
              (point::UnitY()) // The y-coordinate of the center of gravity is
                               // not changed
        + (support_surface - final_ground_touch_vertex_1(2)) *
              (point::UnitZ()); // The z-coordinate of the vertices touching the
                                // ground is that of the ground
    // calculate the pose after placing
    auto result_transform = gripper_transform_T.inverse() *
                            Eigen::Translation<T, 3>(final_translation) *
                            second_rotation * first_rotation *
                            current_transform;
    // convert it to Particle
    result_particle->block(0, 0, 3, 1) = result_transform.translation();
    Eigen::Quaternion<T> result_rotation(result_transform.rotation());
    T roll, pitch, yaw;
    quaternion_to_RPY(result_rotation.w(), result_rotation.x(),
                      result_rotation.y(), result_rotation.z(), roll, pitch,
                      yaw);
    result_particle->block(3, 0, 3, 1) << roll, pitch, yaw;
  }
};
} // namespace

void place_update_distribution(const Particle &old_mean,
                               const CovarianceMatrix &old_covariance,
                               const Eigen::Vector3d &_center_of_gravity,
                               const Eigen::Vector3d &_ground_touch_vertex_1,
                               const Eigen::Vector3d &_ground_touch_vertex_2,
                               const Eigen::Vector3d &_ground_touch_vertex_3,
                               const double &_support_surface,
                               const Eigen::Isometry3d _gripper_transform,
                               Particle &new_mean,
                               CovarianceMatrix &new_covariance) {
  center_of_gravity = _center_of_gravity;
  ground_touch_vertex_1 = _ground_touch_vertex_1;
  ground_touch_vertex_2 = _ground_touch_vertex_2;
  ground_touch_vertex_3 = _ground_touch_vertex_3;
  support_surface = _support_surface;
  gripper_transform = _gripper_transform;
  // Calculate the particle after placing and its Jacobian
  Eigen::AutoDiffJacobian<calculate_particle> calculate_particle_AD;
  // By Eigen AutoDiff, calculate_particle_AD automatically calculates the
  // operation of calculate_particle and its Jacobian
  CovarianceMatrix Jacobian;
  calculate_particle_AD(old_mean, &new_mean, &Jacobian);

  // The covariance of the function value is calculated by the covariance of the
  // argument and Jacobian.
  new_covariance = Jacobian * old_covariance * Jacobian.transpose();
}
