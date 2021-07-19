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

  // set initial pose belief
  Eigen::Isometry3d initial_mean(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  // set covariance randomly
  // std::random_device seed_generator;
  std::default_random_engine engine(0);
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

  /*for(auto& vertex: gripped_geometry-> vertices){
    std::cerr << vertex.transpose() << std::endl;
    }*/
  printf("%d\n", (int)actions.size());
  for (auto &action : actions) {
    printf("%d\n", action.type);
    print_pose(action.gripper_pose);
    // std::cerr << action.gripper_pose.matrix() << std::endl;
  }

  /*
  // visualize the result of action plan

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nd;

  // create the client
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client(
      "update_distribution", true);
  client.waitForServer();

  // create the visualizer client
  ros::ServiceClient visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");

  int number_of_actions = actions.size();
  auto current_time = ros::Time::now();
  for(int t=0;t<number_of_actions;t++){
    std::string gripper_frame_id = "gripper-" + std::to_string(t);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(actions[t].gripper_pose, pose);
    broadcast_gripper_pose(gripper_frame_id, current_time,
                           pose);
  }

  double lifetime = 1.0;

  std::shared_ptr<moveit_msgs::CollisionObject> object(new
  moveit_msgs::CollisionObject); object->pose=to_Pose(0.0, 0.0, 0.0, 1.0, 0.0,
  0.0, 0.0); add_mesh_to_CollisionObject(object,gripped_geometry->
  vertices,gripped_geometry-> triangles, Eigen::Isometry3d::Identity());

  while (ros::ok()) {

    Eigen::Isometry3d mean = initial_mean;
    CovarianceMatrix covariance = initial_covariance;

    geometry_msgs::PoseWithCovarianceStamped current_pose;

    tf::poseEigenToMsg(mean, current_pose.pose.pose);
    current_pose.pose.covariance = matrix_6x6_to_array_36(covariance);
    current_pose.header.frame_id = "world";
    current_pose.header.stamp = current_time;
    send_pose_belief(visualizer_client, *object, 1, lifetime, current_pose);

    std::cout << mean.matrix() << std::endl;
    std::cout << covariance << std::endl;

    ros::Duration(1.0).sleep();

    for (int t=0;t<number_of_actions;t++) {
      UpdateAction& action = actions[t];
      std::cout << action.type << std::endl;
      std::cout << action.gripper_pose.matrix() << std::endl;

      std::string gripper_frame_id = "gripper-" + std::to_string(t);

      o2ac_msgs::updateDistributionGoal goal;
      goal.observation_type =
          (action.type == place_action_type
               ? goal.PLACE_OBSERVATION
               : action.type == grasp_action_type ? goal.GRASP_OBSERVATION
                                                  : goal.PUSH_OBSERVATION);
      tf::poseEigenToMsg(action.gripper_pose, goal.gripper_pose.pose);
      goal.place_observation.support_surface = 0.0;
      goal.distribution_type = 1;
      if (action.type == place_action_type) {
        tf::poseEigenToMsg(mean, goal.distribution.pose.pose);
        goal.distribution.pose.covariance = matrix_6x6_to_array_36(covariance);
      } else {
        tf::poseEigenToMsg(action.gripper_pose.inverse() * mean,
                           goal.distribution.pose.pose);
        goal.distribution.pose.covariance = matrix_6x6_to_array_36(
            transform_covariance(action.gripper_pose.inverse(), covariance));
      }
      goal.distribution.header.frame_id = gripper_frame_id;
      goal.distribution.header.stamp = current_time;

      goal.gripped_object = *object;

      // Send a call
      client.sendGoal(goal);
      client.waitForResult();

      auto result = client.getResult();

      assert(result->success);

      send_pose_belief(visualizer_client, *object, 1, lifetime,
                       result->distribution);

      tf::poseMsgToEigen(result->distribution.pose.pose, mean);
      covariance = array_36_to_matrix_6x6(result->distribution.pose.covariance);
      if (action.type != grasp_action_type) {
        mean = action.gripper_pose * mean;
        covariance = transform_covariance(action.gripper_pose, covariance);
      }

      std::cout << mean.matrix() << std::endl;
      std::cout << covariance << std::endl;

      ros::Duration(1.0).sleep();
    }
  }
  */
  return 0;
}
