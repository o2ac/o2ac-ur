#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "../estimator.cpp"
#include "../ros_converters.cpp"
#include "o2ac_msgs/updateDistributionAction.h"

int main(int argc, char **argv)
{
  char exp_file_path[99];
  scanf("%s",exp_file_path);
  FILE *exp_file=fopen(exp_file_path, "r");

  Particle mean;
  mean.setZero();
  CovarianceMatrix covariance;
  covariance.setZero();
  covariance.diagonal() << 0.001, 0.001, 0.001, 0.0087, 0.0087, 0.0087;

  ros::init(argc,  argv, "test_client");
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client("update_distribution", true);
  client.waitForServer();

  while(1){
    int id;
    double x,y,z,qw,qx,qy,qz;
    if(fscanf(exp_file, "%d%lf%lf%lf%lf%lf%lf%lf", &id, &x, &y, &z, &qx, &qy, &qz, &qw) == EOF){
      break;
    }
    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = goal.TOUCH_OBSERVATION;
    goal.touch_observation.gripper_pose = to_Pose(x,y,z,qw,qx,qy,qz);
    goal.touch_observation.touched_object_id = id;
    goal.distribution = to_PoseWithCovariance(mean, covariance);
    client.sendGoal(goal);
    client.waitForResult();
    auto result = client.getResult();
    if(result->success){
      mean = pose_to_particle(result->distribution.pose);
      covariance = array_36_to_matrix_6x6(result->distribution.covariance);
      std::cout << mean << std::endl;
      std::cout << covariance << std::endl;
    }
    else{
      puts("not updated");
    }
  }
  return 0;
}
