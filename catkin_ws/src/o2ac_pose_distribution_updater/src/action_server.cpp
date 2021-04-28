/*
The implementation of the action server for updateDistribution action
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/server/simple_action_server.h>
#include "o2ac_msgs/updateDistributionAction.h"
#include "estimator.cpp"
#include "ros_converters.cpp"
#include "tools.cpp"

using Server = actionlib::SimpleActionServer<o2ac_msgs::updateDistributionAction>;
				
namespace{
  // Action server
  std::shared_ptr<Server> server;

  // Distibutions are actually calculated by 'estimator'
  std::shared_ptr<PoseEstimator> estimator;

  void execute(const o2ac_msgs::updateDistributionGoalConstPtr& goal)
  {
    // Given the current distribution and the observation, this calculates the updated distribution

    // convert from geometry_msgs::PoseWithCovariance to Eigen Vector and Eigen Matrix.
    Particle old_mean = pose_to_particle(goal->distribution.pose), new_mean;
    CovarianceMatrix old_covariance = array_36_to_matrix_6x6(goal->distribution.covariance), new_covariance;
    o2ac_msgs::updateDistributionResult result;

    try{
      // Touch Action
      if(goal->observation_type == goal->TOUCH_OBSERVATION){
	auto& observation = goal->touch_observation;

	//convert from geometry_msgs::Pose to fcl::Transform3f
	auto gripper_transform = pose_to_fcl_transform(observation.gripper_pose);

	estimator->touched_step(observation.touched_object_id, gripper_transform, old_mean, old_covariance, new_mean, new_covariance);
      }
      // Place Action
      else if(goal->observation_type == goal->PLACE_OBSERVATION){
	auto& observation = goal->place_observation;

	//convert from geometry_msgs::Pose to Eigen::Transform<double, 3, Eigen::Isometry>
	auto gripper_transform = pose_to_eigen_transform(observation.gripper_pose);

	estimator->place_step(gripper_transform, observation.ground_z, old_mean, old_covariance, new_mean, new_covariance);
      }
      // Look Action
      else if(goal->observation_type == goal->LOOK_OBSERVATION){
	auto& observation = goal->look_observation;

	//convert from geometry_msgs::Pose to Eigen::Transform<double, 3, Eigen::Isometry>
	auto gripper_transform = pose_to_eigen_transform(observation.gripper_pose);

	//convert from sensor_msgs::Image to cv::Mat
	auto looked_image = cv_bridge::toCvCopy(observation.looked_image, sensor_msgs::image_encodings::BGR8)->image;

	// convert from MultiArray in std_msgs to std::vector
	std::vector<cv::Point2d> calibration_points(observation.calibration_points.data.size()/2);
	for(int i=0;i<calibration_points.size();i++){
	  calibration_points[i]=cv::Point2d(observation.calibration_points.data[2*i], observation.calibration_points.data[2*i+1]);
	}
	std::vector<int> ROI(4);
	for(int i=0;i<4;i++){
	  ROI[i]=observation.ROI.data[i];
	}
	
	estimator->look_step(gripper_transform, looked_image, calibration_points, ROI, old_mean, old_covariance, new_mean, new_covariance);
      }
    }
    catch(std::runtime_error &e){
      // Update is not calculated successfully
      std::cerr << "Distribution cannot be calculated: " << e.what() << std::endl;
      result.success = false;
      server->setSucceeded(result);
      return;
    } 
    // Update is calculated successfully
    result.success = true;
    // convert from Eigen Vector and Eigen Matrix to geometry_msgs::PoseWithcovariance
    result.distribution = to_PoseWithCovariance(new_mean, new_covariance);
    server->setSucceeded(result);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_distribution_action_server");
  ros::NodeHandle nd;
  ros::NodeHandle pnd("~");

  // Read the parameters from rosparam and initlialize estimator

  // Load the stl file representing the gripped object
  std::string stl_file_path;
  pnd.getParam("stl_file_path", stl_file_path);
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry(new fcl::BVHModel<fcl::OBBRSS>());
  load_BVHModel_from_stl_file(stl_file_path, gripped_geometry);

  // Read the sizes of objects and create fcl::CollisionObject classes
  std::vector<double> ground_size, ground_position;
  nd.getParam("ground_size", ground_size);
  nd.getParam("ground_position", ground_position);
  std::shared_ptr<fcl::Box> ground_geometry(new fcl::Box(ground_size[0], ground_size[1], ground_size[2]));
  fcl::Vec3f ground_translation(ground_position[0], ground_position[1], ground_position[2]);
  std::shared_ptr<fcl::CollisionObject> ground_object(new fcl::CollisionObject(ground_geometry, fcl::Transform3f(fcl::Quaternion3f(),ground_translation)));

  std::vector<double> box_size, box_position;
  nd.getParam("box_size", box_size);
  nd.getParam("box_position", box_position);
  std::shared_ptr<fcl::Box> box_geometry(new fcl::Box(box_size[0], box_size[1], box_size[2]));
  fcl::Vec3f box_translation(box_position[0], box_position[1], box_position[2]);
  std::shared_ptr<fcl::CollisionObject> box_object(new fcl::CollisionObject(box_geometry, fcl::Transform3f(fcl::Quaternion3f(),box_translation)));

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

  // calibration points are represented by the concatenated list of xyz coordinates of all points
  std::vector<double> calibration_coordinates;
  nd.getParam("calibration_points", calibration_coordinates);
  std::vector<std::vector<double> > calibration_points(calibration_coordinates.size()/3);
  for(int i=0;i<calibration_points.size();i++){
    calibration_points[i]=std::vector<double>{calibration_coordinates[3*i],calibration_coordinates[3*i+1],calibration_coordinates[3*i+2]};
  }
  double camera_fx, camera_fy, camera_cx, camera_cy;
  nd.getParam("camera_fx",camera_fx);
  nd.getParam("camera_fy",camera_fy);
  nd.getParam("camera_cx",camera_cx);
  nd.getParam("camera_cy",camera_cy);

  // construct 'estimator' and set parameters
  estimator = std::shared_ptr<PoseEstimator>(new PoseEstimator(gripped_geometry));
  estimator->set_particle_parameters(number_of_particles, noise_variance);
  estimator->set_touch_parameters(touched_objects, distance_threshold);
  estimator->set_look_parameters(look_threshold, calibration_points, camera_fx, camera_fy, camera_cx, camera_cy);

  // start the server
  server=std::shared_ptr<Server>(new Server(nd, "update_distribution", execute, false));
  server->start();
  ros::spin();
  return 0;
}
