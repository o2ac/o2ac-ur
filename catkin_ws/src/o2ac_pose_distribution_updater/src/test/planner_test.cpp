#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <matheval.h>
#include <random>
#include <yaml-cpp/yaml.h>

void load_grasp_points(const std::string &yaml_file_path,
                       std::vector<Eigen::Isometry3d> &grasp_points) {
  char *variable_names[] = {(char *)"pi"};
  double variable_values[] = {acos(-1)};

  YAML::Node config = YAML::LoadFile(yaml_file_path);
  const YAML::Node &grasp_points_data = config["grasp_points"];
  grasp_points.resize(grasp_points_data.size());
  for (int i = 0; i < grasp_points.size(); i++) {
    const YAML::Node &grasp_point_pose = grasp_points_data[i]["pose_xyzrpy"];
    Particle particle;
    for (int j = 0; j < 6; j++) {
      std::string expression = grasp_point_pose[j].as<std::string>();
      void *evaluator = evaluator_create((char *)expression.c_str());
      particle[j] =
          evaluator_evaluate(evaluator, 1, variable_names, variable_values);
    }
    grasp_points[i] = particle_to_eigen_transform(particle);
  }
}

void print_pose(const Eigen::Isometry3d &pose) {
  Eigen::Vector3d translation = pose.translation();
  Eigen::Quaterniond rotation(pose.rotation());
  printf("%.15lf %.15lf %.15lf %.15lf %.15lf %.15lf %.15lf\n", translation.x(),
         translation.y(), translation.z(), rotation.w(), rotation.x(),
         rotation.y(), rotation.z());
}

int main(int argc, char **argv) {
  std::string stl_file_path(
      "/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/"
      "wrs_assembly_2020/meshes/03-PANEL2.stl");

  std::shared_ptr<mesh_object> gripped_geometry(new mesh_object);
  read_stl_from_file_path(stl_file_path, gripped_geometry->vertices,
                          gripped_geometry->triangles);
  for (auto &vertex : gripped_geometry->vertices) {
    vertex /= 1000.0; // milimeter -> meter
  }
  std::shared_ptr<std::vector<Eigen::Isometry3d>> grasp_points(
      new std::vector<Eigen::Isometry3d>);
  load_grasp_points("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/"
                    "config/wrs_assembly_2020/object_metadata/panel_motor.yaml",
                    *grasp_points);
  // create planner and set parameters
  Planner planner;
  planner.set_grasp_parameters(0.0, 0.020, 0.006);
  planner.set_cost_coefficients(1.0, 0.0, 0.0);
  Particle noise_variance;
  noise_variance.setZero();
  planner.set_particle_parameters(10, noise_variance);
  planner.set_use_linear_approximation(false);
  planner.set_image_size(1080, 1920);

  // set initial pose belief
  Eigen::Isometry3d initial_mean(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(asin(1.0), Eigen::Vector3d::UnitX()));
  // set covariance randomly
  std::random_device seed_generator;
  std::default_random_engine engine(seed_generator());
  std::uniform_real_distribution<double> uniform_distribution(-1.0, 1.0);
  CovarianceMatrix deviation, initial_covariance;
  double position_deviation = 0.01, angle_deviation = 0.1;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      deviation(i, j) = (j < 3 ? position_deviation : angle_deviation) *
                        uniform_distribution(engine);
    }
  }
  initial_covariance = deviation.transpose() * deviation;

  // make action plan
  CovarianceMatrix objective_coefficients;
  objective_coefficients.setIdentity();
  auto actions = planner.calculate_plan(
      gripped_geometry, grasp_points, Eigen::Isometry3d::Identity(), false,
      initial_mean, initial_covariance, objective_coefficients, 1e-6);

  // print action plan
  printf("%s\n", stl_file_path.c_str());
  print_pose(initial_mean);
  std::cout << initial_covariance << std::endl;

  printf("%d\n", (int)actions.size());
  for (auto &action : actions) {
    printf("%d\n", action.type);
    print_pose(action.gripper_pose);
  }
  return 0;
}
