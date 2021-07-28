/*
The implementation of the action server for updateDistribution action
 */

#include "o2ac_msgs/updateDistributionAction.h"
#include "o2ac_pose_distribution_updater/pose_belief_visualizer.hpp"
#include "o2ac_pose_distribution_updater/ros_converted_estimator.hpp"
#include <actionlib/server/simple_action_server.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf2/convert.h>

using Server =
    actionlib::SimpleActionServer<o2ac_msgs::updateDistributionAction>;

namespace {
// Action server
std::unique_ptr<Server> server;

// Distributions are actually calculated by 'estimator'
std::unique_ptr<ROSConvertedPoseEstimator> estimator;

// To obtain the part's position if not supplied in the goal request
planning_scene_monitor::PlanningSceneMonitorPtr my_planning_scene_monitor;
Eigen::Isometry3d get_object_pose_from_scene(const std::string &object_name) {
  if (my_planning_scene_monitor->getPlanningScene()->knowsFrameTransform(
          object_name)) {
    return my_planning_scene_monitor->getPlanningScene()->getFrameTransform(
        object_name);
  }
}

bool visualize_each_step;
std::shared_ptr<PoseBeliefVisualizer> pose_belief_visualizer;

void execute(const o2ac_msgs::updateDistributionGoalConstPtr &goal) {
  // Given the current distribution and the observation, this calculates the
  // updated distribution

  o2ac_msgs::updateDistributionResult result;
  auto distribution = goal->distribution;
  auto gripper_pose = goal->gripper_pose.pose;

  try {
    if (distribution.header.frame_id == "") {
      Eigen::Isometry3d t = get_object_pose_from_scene(goal->gripped_object.id);
      tf::poseEigenToMsg(t, distribution.pose.pose);
      distribution.header.frame_id = "world";
      tf::poseEigenToMsg(Eigen::Isometry3d::Identity(), gripper_pose);
    }

    // Touch Action
    if (goal->observation_type == goal->TOUCH_OBSERVATION) {
      auto &observation = goal->touch_observation;

      estimator->touched_step(
          observation.touched_object_id, goal->gripped_object, gripper_pose,
          goal->distribution_type, distribution.pose, result.distribution.pose);
    }
    // Place Action
    else if (goal->observation_type == goal->PLACE_OBSERVATION) {
      auto &observation = goal->place_observation;

      estimator->place_step(
          goal->gripped_object, gripper_pose, observation.support_surface,
          goal->distribution_type, distribution.pose, result.distribution.pose);
    }
    // Look Action
    else if (goal->observation_type == goal->LOOK_OBSERVATION) {
      auto &observation = goal->look_observation;

      estimator->look_step(goal->gripped_object, gripper_pose,
                           observation.looked_image, observation.ROI,
                           goal->distribution_type, distribution.pose,
                           result.distribution.pose);
    }
    // Grasp Action
    else if (goal->observation_type == goal->GRASP_OBSERVATION) {
      estimator->grasp_step(goal->gripped_object, gripper_pose,
                            goal->distribution_type, distribution.pose,
                            result.distribution.pose);
    }
    // Push Action
    else if (goal->observation_type == goal->PUSH_OBSERVATION) {
      estimator->push_step(goal->gripped_object, gripper_pose,
                           goal->distribution_type, distribution.pose,
                           result.distribution.pose);
    }
  } catch (std::exception &e) {
    // Update is not calculated successfully by std::exception
    auto error_message = e.what();
    ROS_ERROR_STREAM("Distribution cannot be calculated: " << error_message);
    result.success = false;
    result.error_message.data = error_message;
    server->setSucceeded(result);
    return;
  } catch (boost::exception &e) {
    // Update is not calculated successfully by boost::exception
    auto error_message = diagnostic_information(e);
    ROS_ERROR_STREAM("Distribution cannot be calculated: " << error_message);
    result.success = false;
    result.error_message.data = error_message;
    server->setSucceeded(result);
    return;
  } catch (...) {
    // Update is not calculated successfully by other exception
    std::string error_message("Unknown error");
    ROS_ERROR_STREAM("Distribution cannot be calculated: " << error_message);
    result.success = false;
    result.error_message.data = error_message;
    server->setSucceeded(result);
    return;
  }
  // Update is calculated successfully
  result.success = true;
  result.distribution.header = distribution.header;
  server->setSucceeded(result);

  if (visualize_each_step) {
    o2ac_msgs::visualizePoseBelief::Request belief;
    o2ac_msgs::visualizePoseBelief::Response response;

    belief.object = goal->gripped_object;
    belief.distribution_type = goal->distribution_type;
    belief.distribution = result.distribution;
    belief.lifetime = ros::Duration(0.0);

    pose_belief_visualizer->publish_marker_for_pose_belief(belief, response);
  }
}
} // namespace

void sigint_handler(int sig) {
  estimator.reset();
  server->shutdown();
  server.reset();
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "update_distribution_action_server");
  ros::NodeHandle nd;

  // construct 'estimator' and set parameters
  estimator = std::unique_ptr<ROSConvertedPoseEstimator>(
      new ROSConvertedPoseEstimator());

  std::string config_file_path;
  nd.getParam("config_file", config_file_path);
  estimator->load_config_file(config_file_path);

  bool use_planning_scene_monitor;
  nd.getParam("use_planning_scene_monitor", use_planning_scene_monitor);
  if (use_planning_scene_monitor) {
    // Load the planning scene monitor
    my_planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            "robot_description");
    if (!my_planning_scene_monitor->getPlanningScene()) {
      ROS_ERROR_STREAM("Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }
    my_planning_scene_monitor->startSceneMonitor();
    my_planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::
            DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::
            DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    my_planning_scene_monitor->startStateMonitor();
  }

  nd.getParam("visualize_each_step", visualize_each_step);

  // start the action server
  server = std::unique_ptr<Server>(
      new Server(nd, "update_distribution", execute, false));
  server->start();

  // start the pose belief visualization server
  std::string marker_array_topic_name;
  nd.getParam("marker_array_topic_name", marker_array_topic_name);
  pose_belief_visualizer =
      std::make_shared<PoseBeliefVisualizer>(nd, marker_array_topic_name);
  int number_of_particles_to_visualize;
  std::vector<double> visualization_scale, mean_color, variance_color;
  nd.getParam("visualization_scale", visualization_scale);
  nd.getParam("mean_color", mean_color);
  nd.getParam("variance_color", variance_color);
  nd.getParam("number_of_particles_to_visualize",
              number_of_particles_to_visualize);
  pose_belief_visualizer->set_scale(
      visualization_scale[0], visualization_scale[1], visualization_scale[2]);
  pose_belief_visualizer->set_mean_color(mean_color[0], mean_color[1],
                                         mean_color[2], mean_color[3]);
  pose_belief_visualizer->set_variance_color(
      variance_color[0], variance_color[1], variance_color[2],
      variance_color[3]);
  pose_belief_visualizer->set_number_of_particles(
      number_of_particles_to_visualize);

  ros::ServiceServer server_to_visualize_pose_belief =
      nd.advertiseService("visualize_pose_belief",
                          &PoseBeliefVisualizer::publish_marker_for_pose_belief,
                          pose_belief_visualizer.get());

  signal(SIGINT, sigint_handler);

  ros::spin();
  return 0;
}
