#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <matheval.h>
#include <random>
#include <yaml-cpp/yaml.h>

void load_grasp_points(const std::string &yaml_file_path,
                       std::vector<Eigen::Isometry3d> &grasp_points,
                       std::map<std::string, int> &name_to_id) {
  char *variable_names[] = {(char *)"pi"};
  double variable_values[] = {acos(-1)};

  YAML::Node config = YAML::LoadFile(yaml_file_path);
  const YAML::Node &grasp_points_data = config["grasp_points"];
  grasp_points.resize(grasp_points_data.size());
  for (int i = 0; i < grasp_points.size(); i++) {
    std::string grasp_name =
        grasp_points_data[i]["grasp_name"].as<std::string>();
    name_to_id[grasp_name] = i;
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

int main(int argc, char **argv) {
  FILE *config_file =
      fopen("/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/test/"
            "planning_test_config.txt",
            "r");
  char stl_file_path[1000], metadata_file_path[1000];
  fscanf(config_file, "%999s%999s", stl_file_path, metadata_file_path);

  std::shared_ptr<mesh_object> gripped_geometry(new mesh_object);
  read_stl_from_file_path(std::string(stl_file_path),
                          gripped_geometry->vertices,
                          gripped_geometry->triangles);
  for (auto &vertex : gripped_geometry->vertices) {
    vertex /= 1000.0; // milimeter -> meter
  }
  std::shared_ptr<std::vector<Eigen::Isometry3d>> grasp_points(
      new std::vector<Eigen::Isometry3d>);
  std::map<std::string, int> name_to_id;
  load_grasp_points(std::string(metadata_file_path), *grasp_points, name_to_id);
  // create planner and set parameters
  Planner planner;
  planner.load_config_file(
      "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/launch/"
      "estimator_config.yaml");
  double touch_cost, look_cost, place_cost, grasp_cost, push_cost,
      translation_cost, rotation_cost;
  fscanf(config_file, "%lf%lf%lf%lf%lf%lf%lf", &touch_cost, &look_cost,
         &place_cost, &grasp_cost, &push_cost, &translation_cost,
         &rotation_cost);
  planner.set_cost_coefficients(boost::array<double, 5>{touch_cost, look_cost,
                                                        place_cost, grasp_cost,
                                                        push_cost},
                                translation_cost, rotation_cost);

  // set initial pose belief
  Eigen::Isometry3d initial_mean;
  scan_pose(initial_mean, config_file);
  double support_surface;
  fscanf(config_file, "%lf", &support_surface);
  // set covariance randomly
  std::random_device seed_generator;
  std::default_random_engine engine(0);
  std::uniform_real_distribution<double> uniform_distribution(-1.0, 1.0);
  CovarianceMatrix deviation, initial_covariance;
  double deviation_scale[6];
  for (int i = 0; i < 6; i++) {
    fscanf(config_file, "%lf", deviation_scale + i);
  }
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      deviation(i, j) = deviation_scale[j] * uniform_distribution(engine);
    }
  }
  initial_covariance = deviation.transpose() * deviation;

  // make action plan
  CovarianceMatrix objective_coefficients;
  objective_coefficients.setIdentity();
  double objective_value;
  int is_goal_pose;
  fscanf(config_file, "%lf%d", &objective_value, &is_goal_pose);

  std::vector<UpdateAction> actions;
  if (is_goal_pose) {
    char goal_grasp_name[1000];
    double translation_threshold, rotation_threshold;
    fscanf(config_file, "%999s%lf%lf", goal_grasp_name, &translation_threshold,
           &rotation_threshold);
    Eigen::Isometry3d goal_pose =
        (*grasp_points)[name_to_id[std::string(goal_grasp_name)]].inverse();
    std::cerr << goal_pose.matrix() << std::endl;
    actions = planner.calculate_plan(
        gripped_geometry, grasp_points, Eigen::Isometry3d::Identity(), false,
        support_surface, initial_mean, initial_covariance,
        objective_coefficients, objective_value, true,
        check_near_to_goal_pose(goal_pose, translation_threshold,
                                rotation_threshold));
  } else {
    actions = planner.calculate_plan(
        gripped_geometry, grasp_points, Eigen::Isometry3d::Identity(), false,
        support_surface, initial_mean, initial_covariance,
        objective_coefficients, objective_value);
  }

  fclose(config_file);

  // print action plan
  printf("%s\n", stl_file_path);
  print_pose(initial_mean);
  std::cout << initial_covariance << std::endl;
  printf("%lf\n", support_surface);

  printf("%d\n", (int)actions.size());
  for (auto &action : actions) {
    printf("%d\n", action.type);
    print_pose(action.gripper_pose);
  }
  return 0;
}
