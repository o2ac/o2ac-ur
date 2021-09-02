#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include "o2ac_pose_distribution_updater/test_tools.hpp"
#include "o2ac_skills/o2ac_skill_server.h"
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/Grasp.h>

// ################################################################

int main(int argc, char **argv) {
  // visualize the result of action plan

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nd;
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }*/

  // open file
  FILE *in;
  if (argc > 1) {
    in = fopen(argv[1], "r");
  } else {
    in = fopen("/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/"
               "test/plan.txt",
               "r");
  }

  // load stl file

  ROS_INFO("Loading stl file");
  char stl_file_path[1000];
  while (true) {
    fscanf(in, "%999s", stl_file_path);
    if (stl_file_path[0] != 27) {
      break;
    }
    char c;
    while ((c = getc(in)) != '\n')
      ;
  }
  std::shared_ptr<moveit_msgs::CollisionObject> object(
      new moveit_msgs::CollisionObject);
  load_CollisionObject_from_file(object, std::string(stl_file_path));
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(*object, vertices, triangles);

  // create the client
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client(
      "update_distribution", true);
  ROS_INFO("Waiting for Update Action Server");
  client.waitForServer();
  ROS_INFO("Connected to Update Action Server");

  // create the visualizer client
  ros::ServiceClient visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");

  ROS_INFO("Reading initial values and actions from config file");
  Eigen::Isometry3d initial_mean;
  CovarianceMatrix initial_covariance;
  scan_pose(initial_mean, in);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      fscanf(in, "%lf", &initial_covariance(i, j));
    }
  }
  double support_surface;
  fscanf(in, "%lf", &support_surface);

  /*tf::poseEigenToMsg(initial_mean, object->pose);
  object->header.frame_id = "world";
  object->header.stamp = ros::Time::now();
  object->id = "gripped_object";
  object->operation = object->ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  planning_scene_interface.applyCollisionObject(*object);*/

  PoseEstimator estimator;
  estimator.load_config_file(
      "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/launch/"
      "estimator_config.yaml");

  int number_of_actions;
  fscanf(in, "%d", &number_of_actions);

  std::vector<UpdateAction> actions(number_of_actions);
  for (int t = 0; t < number_of_actions; t++) {
    int type;
    fscanf(in, "%d", &type);
    actions[t].type = (action_type)type;
    scan_pose(actions[t].gripper_pose, in);
  }
  int initially_gripping;
  fscanf(in, "%d", &initially_gripping);
  fclose(in);

  Eigen::Isometry3d mean = initial_mean;
  CovarianceMatrix covariance = initial_covariance;

  geometry_msgs::PoseWithCovarianceStamped current_pose;

  double lifetime = 0.0;
  std::string robot_name = "a_bot", tip_link = "a_bot_gripper_tip_link";

  tf::poseEigenToMsg(mean, current_pose.pose.pose);
  current_pose.pose.covariance = matrix_6x6_to_array_36(covariance);
  current_pose.header.frame_id = (initially_gripping ? tip_link : "world");
  current_pose.header.stamp = ros::Time::now();
  object->id = "gripped_object";
  send_pose_belief(visualizer_client, *object, 1, lifetime, current_pose);

  // std::string movegroup_name;//, ee_link;
  // Dynamic parameters. Last arg is the default value. You can assign these
  // from a launch file.
  // nh.param<std::string>("move_group", movegroup_name, "a_bot");
  // nh.param<std::string>("ee_link", ee_link, "a_bot_ee_link");
  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate *loop_rate_ = new ros::Rate(ros_rate);

  ros::Duration(1.0).sleep(); // 1 second

  // Create MoveGroup
  // moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  moveit::planning_interface::MoveGroupInterface gripper_group(robot_name +
                                                               "_robotiq_85");

  // Configure planner
  // group.setPlanningTime(0.5);
  // group.setPlannerId("RRTConnectkConfigDefault");
  // group.setEndEffectorLink(ee_link);
  moveit::planning_interface::MoveItErrorCode
      success_plan = moveit_msgs::MoveItErrorCodes::FAILURE,
      motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds

  bool gripper_is_open = !initially_gripping;

  if (gripper_is_open) {
    gripper_group.setNamedTarget("open");
    if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
      ROS_ERROR("Initial opening before push failed");
      return 0;
    }
  } else {
    gripper_group.setNamedTarget("close");
    if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
      ROS_ERROR("Initial closing failed");
      return 0;
    }
  }
  loop_rate_->sleep();

  const double pushing_length = 0.05, retreat_height = 0.05;
  // const double distance_ee_and_tip = 0.246;
  // Eigen::Isometry3d
  // ee_link_to_tip_link(Eigen::Translation3d(Eigen::Vector3d(std::vector<double>{-distance_ee_and_tip,
  // 0.0, 0.0}.data())));

  SkillServer skill_server;

  ROS_INFO("Start actions");

  for (int t = 0; t < number_of_actions; t++) {
    UpdateAction &action = actions[t];
    geometry_msgs::PoseStamped gripper_pose;
    gripper_pose.header.frame_id = "world";
    gripper_pose.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(action.gripper_pose, gripper_pose.pose);

    if (action.type == push_action_type) {
      Eigen::Isometry3d pose_before_push = action.gripper_pose;
      pose_before_push.translation() -=
          pushing_length *
          (action.gripper_pose.rotation() * Eigen::Vector3d::UnitZ());
      /*group.setPoseTarget(pose_before_push * ee_link_to_tip_link);
      while(ros::ok()){
        success_plan = group.plan(myplan);
        if (!success_plan)
          continue;
        motion_done = group.execute(myplan);
        if (!motion_done)
          continue;
        break;
        }*/
      geometry_msgs::PoseStamped tip_pose;
      tip_pose.header = gripper_pose.header;
      tf::poseEigenToMsg(pose_before_push, tip_pose.pose);
      if (!skill_server.moveToCartPoseLIN(tip_pose, robot_name)) {
        ROS_ERROR("Moving before push failed");
        break;
      }
      if (gripper_is_open) {
        // if (!skill_server.closeGripper(robot_name)) {
        gripper_group.setNamedTarget("close");
        if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
          ROS_ERROR("Closing before push failed");
          break;
        }
        gripper_is_open = false;
      }
      if (!skill_server.moveToCartPoseLIN(gripper_pose, robot_name, true, "",
                                          0.01, 0.01)) {
        ROS_ERROR("Pushing object failed");
        break;
      }
    } else if (action.type == grasp_action_type) {
      if (!gripper_is_open) {
        // if (!skill_server.openGripper(robot_name)) {
        gripper_group.setNamedTarget("open");
        if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
          ROS_ERROR("Opening before grasp failed");
          break;
        }
      }
      if (!skill_server.moveToCartPoseLIN(gripper_pose, robot_name)) {
        ROS_ERROR("Moving to grasp failed");
        break;
      }
      gripper_group.setNamedTarget("close");
      // if (!skill_server.closeGripper(robot_name)) {
      if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
        ROS_ERROR("Closing to grasp failed");
        break;
      }
      gripper_is_open = false;
      /*geometry_msgs::PoseStamped high_pose = gripper_pose;
      high_pose.pose.position.z += retreat_height;
      if (!skill_server.moveToCartPoseLIN(high_pose, robot_name)) {
        ROS_ERROR("Moving after grasp failed");
        break;
        }*/
    } else if (action.type == place_action_type) {
      geometry_msgs::PoseStamped high_pose = gripper_pose;
      high_pose.pose.position.z += retreat_height;
      if (!skill_server.moveToCartPoseLIN(high_pose, robot_name)) {
        ROS_ERROR("Moving to place failed");
        break;
      }
      if (!skill_server.moveToCartPoseLIN(gripper_pose, robot_name, true, "",
                                          0.01, 0.01)) {
        ROS_ERROR("Placing down failed");
        break;
      }
      gripper_group.setNamedTarget("open");
      if (gripper_group.move() == moveit_msgs::MoveItErrorCodes::FAILURE) {
        ROS_ERROR("Opening to place failed");
        break;
      }
      gripper_is_open = true;
    }

    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = (action.type == touch_action_type
                                 ? goal.TOUCH_OBSERVATION
                                 : action.type == look_action_type
                                       ? goal.LOOK_OBSERVATION
                                       : action.type == place_action_type
                                             ? goal.PLACE_OBSERVATION
                                             : action.type == grasp_action_type
                                                   ? goal.GRASP_OBSERVATION
                                                   : goal.PUSH_OBSERVATION);
    goal.gripper_pose = gripper_pose;
    if (action.type == touch_action_type) {
      goal.touch_observation.touched_object_id = 0;
    } else if (action.type == look_action_type) {
      cv::Mat mean_image, inv_image, bgr_image;
      boost::array<unsigned int, 4> ROI{0, 1080, 0, 1920};
      estimator.generate_image(mean_image, vertices, triangles,
                               action.gripper_pose * mean, ROI);
      inv_image = cv::Mat::ones(1080, 1920, CV_8UC1) - mean_image;
      cv::imwrite(std::to_string(t) + ".jpg", 255 * inv_image);
      cv::cvtColor(255 * inv_image, bgr_image, cv::COLOR_GRAY2BGR);
      goal.look_observation.looked_image =
          *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image)
                .toImageMsg());
      goal.look_observation.ROI = ROI;
    } else if (action.type == place_action_type) {
      goal.place_observation.support_surface = support_surface;
    }
    goal.distribution_type = 1;
    if (action.type != grasp_action_type && action.type != push_action_type) {
      tf::poseEigenToMsg(mean, goal.distribution.pose.pose);
      goal.distribution.pose.covariance = matrix_6x6_to_array_36(covariance);
    } else {
      tf::poseEigenToMsg(action.gripper_pose.inverse() * mean,
                         goal.distribution.pose.pose);
      goal.distribution.pose.covariance = matrix_6x6_to_array_36(
          transform_covariance(action.gripper_pose.inverse(), covariance));
    }
    goal.distribution.header.frame_id = tip_link;
    goal.distribution.header.stamp = ros::Time::now();

    goal.gripped_object = *object;

    o2ac_msgs::updateDistributionResultConstPtr result;

    while (ros::ok()) {
      // Send a call
      client.sendGoal(goal);
      client.waitForResult();

      result = client.getResult();

      if (result->success) {
        break;
      }
    }

    bool gripping =
        (action.type != place_action_type && action.type != push_action_type);
    send_pose_belief(visualizer_client, *object, 1, lifetime,
                     result->distribution, gripping);

    tf::poseMsgToEigen(result->distribution.pose.pose, mean);
    covariance = array_36_to_matrix_6x6(result->distribution.pose.covariance);
    if (action.type == place_action_type || action.type == push_action_type) {
      mean = action.gripper_pose * mean;
      covariance = transform_covariance(action.gripper_pose, covariance);
    }

    loop_rate_->sleep();
  }
  return 0;
}
