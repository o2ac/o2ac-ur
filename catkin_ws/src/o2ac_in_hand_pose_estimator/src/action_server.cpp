#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <actionlib/server/simple_action_server.h>
#include "o2ac_msgs/updateDistributionAction.h"
#include "estimator.cpp"
#include "ros_converters.cpp"
#include "tools.cpp"

using Server = actionlib::SimpleActionServer<o2ac_msgs::updateDistributionAction>;
				
namespace{
  std::shared_ptr<Server> server;

  std::shared_ptr<PoseEstimator> estimator;

  void execute(const o2ac_msgs::updateDistributionGoalConstPtr& goal)
  {
    if(goal->observation_type == goal->TOUCH_OBSERVATION){
      auto observation = goal->touch_observation;
      auto gripper_transform = pose_to_transform(observation.gripper_pose);
      Particle old_mean = pose_to_particle(goal->distribution.pose), new_mean;
      CovarianceMatrix old_covariance = array_36_to_matrix_6x6(goal->distribution.covariance), new_covariance;
      int ret=estimator->touched_step(observation.touched_object_id, gripper_transform, old_mean, old_covariance, new_mean, new_covariance);
      o2ac_msgs::updateDistributionResult result;
      if(ret == 0){
	result.success = true;
	result.distribution = to_PoseWithCovariance(new_mean, new_covariance);
      }
      else{
	result.success = false;
      }
      server->setSucceeded(result);
    }
  }
}

int main(int argc, char **argv)
{
  // Read from rosparam and initlialize estimator

  ros::init(argc, argv, "update_distribution_action_server");
  ros::NodeHandle nd;
  ros::NodeHandle pnd("~");

  std::string stl_file_path;
  pnd.getParam("stl_file_path", stl_file_path);
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> gripped_geometry(new fcl::BVHModel<fcl::OBBRSS>());
  //fprintf(stderr, "%s\n", stl_file_path.c_str();
  load_BVHModel_from_stl_file(stl_file_path, gripped_geometry);


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

  double distance_threshold;
  nd.getParam("distance_threshold", distance_threshold);

  std::vector<double> noise_variance_vector;
  nd.getParam("noise_variance", noise_variance_vector);
  Particle noise_variance(noise_variance_vector.data());

  int number_of_particles;
  nd.getParam("number_of_particles", number_of_particles);

  estimator = std::shared_ptr<PoseEstimator>(new PoseEstimator(gripped_geometry,
							       touched_objects,
							       distance_threshold,
							       noise_variance,
							       number_of_particles));

  server=std::shared_ptr<Server>(new Server(nd, "update_distribution", execute, false));
  server->start();
  ros::spin();
  return 0;
}
