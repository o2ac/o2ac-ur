#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <matheval.h>
#include <random>
#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

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

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string config_file_name;
  if (argc > 1) {
    config_file_name = std::string(argv[1]);
  } else {
    config_file_name =
        "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/test/"
        "planning_test_config.txt";
  }

  FILE *config_file = fopen(config_file_name.c_str(), "r");
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
  int use_moveit;
  fscanf(config_file, "%lf%lf%lf%lf%lf%lf%lf%d", &touch_cost, &look_cost,
         &place_cost, &grasp_cost, &push_cost, &translation_cost,
         &rotation_cost, &use_moveit);
  boost::array<double, 5> action_cost{touch_cost, look_cost, place_cost,
                                      grasp_cost, push_cost};

  CostFunction cost_function =
      [&action_cost, &translation_cost,
       &rotation_cost](const action_type &type,
                       const Eigen::Isometry3d &current_gripper_pose,
                       const Eigen::Isometry3d &target_gripper_pose) -> double {
    return action_cost[static_cast<int>(type)] +
           translation_cost * (target_gripper_pose.translation() -
                               current_gripper_pose.translation())
                                  .norm() +
           rotation_cost *
               Eigen::AngleAxisd(target_gripper_pose.rotation() *
                                 current_gripper_pose.rotation().inverse())
                   .angle();
  };

  moveit::planning_interface::MoveGroupInterface robot_group("a_bot");
  ValidityChecker moveit_validity_checker =
      [&robot_group](const action_type &type,
                     const Eigen::Isometry3d &current_gripper_pose,
                     const Eigen::Isometry3d &target_gripper_pose) -> bool {
    if (type == touch_action_type || type == look_action_type) {
      return false;
    }
    // fprintf(stderr, "%d\n",static_cast<int>(type));
    // print_pose(current_gripper_pose, stderr);
    // print_pose(target_gripper_pose, stderr);
    robot_group.clearPoseTargets();
    robot_group.setPoseReferenceFrame("world");
    robot_group.setStartStateToCurrentState();
    robot_group.setEndEffectorLink("a_bot_gripper_tip_link");
    robot_group.setPlanningTime(15.0);

    const double pushing_length = 0.05, retreat_height = 0.05;
    std::vector<Eigen::Isometry3d> target_poses(1, current_gripper_pose);

    if (type == place_action_type) {
      Eigen::Isometry3d high_pose = target_gripper_pose;
      high_pose.translation().z() += retreat_height;
      target_poses.push_back(high_pose);
      target_poses.push_back(target_gripper_pose);
    } else if (type == grasp_action_type) {
      target_poses.push_back(target_gripper_pose);
      /*geometry_msgs::Pose high_pose = gripper_pose;
      high_pose.position.z += retreat_height;
      waypoints.push_back(high_pose);*/
    } else if (type == push_action_type) {
      Eigen::Isometry3d pose_before_push = target_gripper_pose;
      pose_before_push.translation() -=
          pushing_length *
          (target_gripper_pose.rotation() * Eigen::Vector3d::UnitZ());
      target_poses.push_back(pose_before_push);
      target_poses.push_back(target_gripper_pose);
    }

    moveit::core::RobotState last_state(*robot_group.getCurrentState());
    for (int t = 0;; t++) {
      moveit_msgs::RobotTrajectory trajectory;
      std::vector<geometry_msgs::Pose> waypoints(1);
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      if (t <= 1) {
        robot_group.setMaxVelocityScalingFactor(0.1);
        robot_group.setMaxAccelerationScalingFactor(0.5);
      } else {
        robot_group.setMaxVelocityScalingFactor(0.01);
        robot_group.setMaxAccelerationScalingFactor(0.01);
      }
      tf::poseEigenToMsg(target_poses[t], waypoints[0]);
      double cartesian_success = robot_group.computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory);

      if (cartesian_success <= 0.95) {
        // fprintf(stderr, "NG\n");
        return false;
      }
      if (t + 1 >= target_poses.size())
        break;

      moveit::core::jointTrajPointToRobotState(
          trajectory.joint_trajectory,
          trajectory.joint_trajectory.points.size() - 1, last_state);
      robot_group.setStartState(last_state);
    }
    // fprintf(stderr, "OK\n");
    return true;
  };
  boost::array<bool, 5> able_action;
  if (!use_moveit) {
    for (int i = 0; i < 5; i++) {
      int able;
      fscanf(config_file, "%d", &able);
      able_action[i] = (able > 0);
    }
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

  int initially_gripping;
  fscanf(config_file, "%d", &initially_gripping);
  Eigen::Isometry3d initial_mean, initial_gripper_pose;
  if (initially_gripping) {
    char initial_grasp_name[999];
    fscanf(config_file, "%999s", initial_grasp_name);
    initial_mean =
        (*grasp_points)[name_to_id[std::string(initial_grasp_name)]].inverse();
    scan_pose(initial_gripper_pose, config_file);
  } else {
    // set initial pose belief
    scan_pose(initial_mean, config_file);
    scan_pose(initial_gripper_pose, config_file);
  }
  double support_surface;
  int covariance_given;
  fscanf(config_file, "%lf%d", &support_surface, &covariance_given);
  CovarianceMatrix initial_covariance;
  if (covariance_given) {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        double value;
        fscanf(config_file, "%lf", &value);
        initial_covariance(i, j) = value;
      }
    }
  } else {
    // set covariance randomly
    std::random_device seed_generator;
    std::default_random_engine engine(seed_generator());
    std::uniform_real_distribution<double> uniform_distribution(-1.0, 1.0);
    CovarianceMatrix deviation;
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
  }
  // make action plan
  CovarianceMatrix objective_coefficients;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      double value;
      fscanf(config_file, "%lf", &value);
      objective_coefficients(i, j) = value;
    }
  }
  double objective_value;
  int is_goal_pose;
  fscanf(config_file, "%lf%d", &objective_value, &is_goal_pose);

  planner.set_geometry(gripped_geometry, grasp_points, support_surface);

  std::vector<UpdateAction> actions;

  if (is_goal_pose == 1) {
    char goal_grasp_name[1000];
    double translation_threshold, rotation_threshold;
    fscanf(config_file, "%999s%lf%lf", goal_grasp_name, &translation_threshold,
           &rotation_threshold);
    Eigen::Isometry3d goal_pose =
        (*grasp_points)[name_to_id[std::string(goal_grasp_name)]].inverse();
    std::cerr << goal_pose.matrix() << std::endl;
    planner.set_goal_checker(
        std::make_shared<GoalChecker>(check_near_to_goal_pose(
            goal_pose, translation_threshold, rotation_threshold)));

  } else if (is_goal_pose == 2) {
    planner.set_goal_checker(std::make_shared<GoalChecker>(
        [](const bool &gripping, const Eigen::Isometry3d &pose) {
          return !gripping;
        }));
  }

  fclose(config_file);

  actions = planner.calculate_plan(initial_gripper_pose, initially_gripping,
                                   initial_mean, initial_covariance,
                                   objective_coefficients, objective_value);

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
  printf("%d\n", initially_gripping);
  return 0;
}
