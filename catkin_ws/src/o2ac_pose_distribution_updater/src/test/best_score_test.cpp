#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <matheval.h>
#include <random>
#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

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

  ros::init(argc, argv, "planner_test");
  ros::NodeHandle nd;

  int argi = 1;
  std::string stl_file_path(argv[argi++]), metadata_file_path(argv[argi++]), initial_grasp_name(argv[argi++]);

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

  CostFunction cost_function =
      [](const action_type &type,
	 const Eigen::Isometry3d &current_gripper_pose,
	 const Eigen::Isometry3d &target_gripper_pose) -> double {
	return 1.0;
  };

  boost::array<bool, 5> able_action;
  for (int i = 0; i < 5; i++) {
    able_action[i] = atoi(argv[argi++]);
  }
  ValidityChecker type_validity_checker =
      [&able_action](const action_type &type,
                     const Eigen::Isometry3d &current_gripper_pose,
                     const Eigen::Isometry3d &target_gripper_pose) -> bool {
    return able_action[static_cast<int>(type)];
  };

  planner.set_cost_function(std::make_shared<CostFunction>(cost_function));
  planner.set_validity_checker(std::make_shared<ValidityChecker>(
      use_moveit ? moveit_validity_checker : type_validity_checker));

  int initially_gripping = true;
  double support_surface = 0.0;
  // set covariance
  CovarianceMatrix pre_initial_covariance = CovarianceMatrix::Zero();
  for (int i = 0; i < 3; i++) {
    pre_initial_covariance(i,i) = 0.0001;
  }
  for (int i = 3; i < 6; i++) {
    pre_initial_covariance(i,i) = 0.01;
  }

  Eigen::Isometry3d initial_mean, initial_gripper_pose = (*grasp_points)[name_to_id[initial_grasp_name]];
  planner.grasp_step_with_Lie_distribution(gripped_geometry->vertices,
					   gripped_geometry->triangles,
					   initial_gripper_pose,
					   initial_gripper_pose.inverse(),
					   pre_initial_covariance,
					   initial_mean,
					   initial_covariance,
					   false);

  // make action plan
  CovarianceMatrix objective_coefficients = CovarianceMatrix::Identity();
  int max_cost = 4;

  planner.set_geometry(gripped_geometry, grasp_points, support_surface);
  
  auto result = std::move(planner::best_scores_for_each_costs(initial_gripper_pose, initially_gripping,
							      initial_mean,
							      initial_covariance,
							      objective_coefficients,
							      max_cost));
  for(int i=0;i<=max_cost;i++){
    printf("%lf ",result[i].first);
  }
  putchar('\n');
  return 0;
}
