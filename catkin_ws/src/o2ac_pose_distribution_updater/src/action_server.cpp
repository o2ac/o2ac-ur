/*
The implementation of the action server for updateDistribution action
 */

#include "o2ac_pose_distribution_updater/ros_converted_estimator.hpp"
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <signal.h>

using Server =
    actionlib::SimpleActionServer<o2ac_msgs::updateDistributionAction>;

namespace {
// Action server
std::unique_ptr<Server> server;

// Distributions are actually calculated by 'estimator'
std::unique_ptr<ROSConvertedPoseEstimator> estimator;

void execute(const o2ac_msgs::updateDistributionGoalConstPtr &goal) {
  // Given the current distribution and the observation, this calculates the
  // updated distribution

  o2ac_msgs::updateDistributionResult result;

  try {
    // Touch Action
    if (goal->observation_type == goal->TOUCH_OBSERVATION) {
      auto &observation = goal->touch_observation;

      estimator->touched_step(
          observation.touched_object_id, goal->gripped_object,
          observation.gripper_pose.pose, goal->distribution.pose,
          result.distribution.pose);
    }
    // Place Action
    else if (goal->observation_type == goal->PLACE_OBSERVATION) {
      auto &observation = goal->place_observation;

      estimator->place_step(goal->gripped_object, observation.gripper_pose.pose,
                            observation.support_surface,
                            goal->distribution.pose, result.distribution.pose);
    }
    // Look Action
    else if (goal->observation_type == goal->LOOK_OBSERVATION) {
      auto &observation = goal->look_observation;

      estimator->look_step(goal->gripped_object, observation.gripper_pose.pose,
                           observation.looked_image, observation.ROI,
                           goal->distribution.pose, result.distribution.pose);
    }
  } catch (std::exception &e) {
    // Update is not calculated successfully by std::exceptiom
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
  result.distribution.header = goal->distribution.header;
  server->setSucceeded(result);
}
} // namespace

void sigint_handler(int sig) {
  server->shutdown();
  server.reset();
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "update_distribution_action_server");
  ros::NodeHandle nd;
  ros::NodeHandle pnd("~");

  // Read the parameters from rosparam and initialize estimator

  // Read the sizes of objects and create fcl::CollisionObject classes
  std::vector<double> ground_size, ground_position;
  nd.getParam("ground_size", ground_size);
  nd.getParam("ground_position", ground_position);
  std::shared_ptr<fcl::Box> ground_geometry(
      new fcl::Box(ground_size[0], ground_size[1], ground_size[2]));
  fcl::Vec3f ground_translation(ground_position[0], ground_position[1],
                                ground_position[2]);
  std::shared_ptr<fcl::CollisionObject> ground_object(new fcl::CollisionObject(
      ground_geometry,
      fcl::Transform3f(fcl::Quaternion3f(), ground_translation)));

  std::vector<double> box_size, box_position;
  nd.getParam("box_size", box_size);
  nd.getParam("box_position", box_position);
  std::shared_ptr<fcl::Box> box_geometry(
      new fcl::Box(box_size[0], box_size[1], box_size[2]));
  fcl::Vec3f box_translation(box_position[0], box_position[1], box_position[2]);
  std::shared_ptr<fcl::CollisionObject> box_object(new fcl::CollisionObject(
      box_geometry, fcl::Transform3f(fcl::Quaternion3f(), box_translation)));

  std::vector<std::shared_ptr<fcl::CollisionObject>> touched_objects;
  touched_objects.push_back(ground_object);
  touched_objects.push_back(box_object);

  // Read other parameters
  double distance_threshold;
  nd.getParam("distance_threshold", distance_threshold);

  std::vector<double> noise_variance_vector;
  nd.getParam("noise_variance", noise_variance_vector);
  Particle noise_variance(noise_variance_vector.data());

  int number_of_particles;
  nd.getParam("number_of_particles", number_of_particles);

  double look_threshold;
  nd.getParam("look_threshold", look_threshold);

  // calibration points are represented by the concatenated list of xyz
  // coordinates of all points
  std::vector<double> calibration_object_coordinates,
      calibration_image_coordinates;
  nd.getParam("calibration_object_points", calibration_object_coordinates);
  nd.getParam("calibration_image_points", calibration_image_coordinates);
  int number_of_calibration_points = calibration_object_coordinates.size() / 3;
  std::vector<std::vector<double>> calibration_object_points(
      number_of_calibration_points),
      calibration_image_points(number_of_calibration_points);
  for (int i = 0; i < number_of_calibration_points; i++) {
    calibration_object_points[i] =
        std::vector<double>{calibration_object_coordinates[3 * i],
                            calibration_object_coordinates[3 * i + 1],
                            calibration_object_coordinates[3 * i + 2]};
    calibration_image_points[i] =
        std::vector<double>{calibration_image_coordinates[2 * i],
                            calibration_image_coordinates[2 * i + 1]};
  }
  double camera_fx, camera_fy, camera_cx, camera_cy;
  nd.getParam("camera_fx", camera_fx);
  nd.getParam("camera_fy", camera_fy);
  nd.getParam("camera_cx", camera_cx);
  nd.getParam("camera_cy", camera_cy);

  // construct 'estimator' and set parameters
  estimator = std::unique_ptr<ROSConvertedPoseEstimator>(
      new ROSConvertedPoseEstimator());
  estimator->set_particle_parameters(number_of_particles, noise_variance);
  estimator->set_touch_parameters(touched_objects, distance_threshold);
  estimator->set_look_parameters(look_threshold, calibration_object_points,
                                 calibration_image_points, camera_fx, camera_fy,
                                 camera_cx, camera_cy);

  // start the server
  server = std::unique_ptr<Server>(
      new Server(nd, "update_distribution", execute, false));
  server->start();

  signal(SIGINT, sigint_handler);

  ros::spin();
  return 0;
}
