/*
The implementation of the test client for touch action

This client reads the path of csv file representing the gripper pose for each time and the path of directory containing images from standard input.
Each line of csv files must be of the form "t,x,y,z,qx,qy,qz,qw" where 't' is the time and other seven values represents the pose of gripper at the time 't'.
Each file in the image directory must have a name of the from "[t].jpg" where [t] is the time when the image is looked

The client also reads the initial covariance, the coordinates of the calibration points and the range of interests from standard input.

Finally, the client reads the three integers 'beg', 'num' and 'T' from standard input.
Starting at the 'beg'-th file in the image directory, the client reads 'num' images from the 'num' image files in the image directory and finds the corresponding gripper pose from csv file for each image.
The client sends the look action call to the server for each image and repeat it 'T' times.

After each action call, client prints the values of mean and covariance matrix of the pose distribution to standard output.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include "../conversions.cpp"
#include "../ros_converters.cpp"
#include "o2ac_msgs/updateDistributionAction.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>

int main(int argc, char **argv)
{
  char csv_file_path[999], image_dir_path[999];
  scanf("%s%s",csv_file_path, image_dir_path);
  FILE *csv_file=fopen(csv_file_path, "r");
  assert(csv_file!=NULL);
  char c;
  while((c=getc(csv_file))!='\n');
  struct dirent **images;
  int num_of_images = scandir(image_dir_path, &images, NULL, versionsort);
  assert(num_of_images > 0);
  
  Particle mean;
  mean.setZero();
  CovarianceMatrix covariance;
  covariance.setZero();
  double var_xyz, var_ang;
  scanf("%lf%lf",&var_xyz, &var_ang);
  covariance.diagonal() << var_xyz, var_xyz, var_xyz, var_ang, var_ang, var_ang;
  
  ros::init(argc,  argv, "test_client");
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client("update_distribution", true);
  client.waitForServer();

  std_msgs::Float64MultiArray calibration_points;
  calibration_points.data.resize(6);
  for(int i=0;i<6;i++)
    scanf("%lf",&(calibration_points.data[i]));

  std_msgs::UInt32MultiArray ROI;
  ROI.data.resize(4);
  for(int i=0;i<4;i++)
    scanf("%d",&(ROI.data[i]));

  int beg, num, T;
  scanf("%d%d%d",&beg, &num, &T);
  std::vector<cv::Mat> img(num);
  std::vector<double> x(num),y(num),z(num),qx(num),qy(num),qz(num),qw(num);
  for(int i=0;i<num;i++){
    char *fname=images[beg+i]->d_name;
    unsigned long long time, t;
    sscanf(fname,"%llu.jpg",&time);
    while(fscanf(csv_file, "%llu,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &t, &x[i], &y[i], &z[i], &qx[i], &qy[i], &qz[i], &qw[i]) != EOF){
      if(t>=time){
	break;
      }
    }
    img[i]=cv::imread(std::string(image_dir_path)+'/'+std::string(fname));
    if(img[i].type()==0){
      cv::Mat reversed_img, bgr_image;
      cv::threshold(img[i], reversed_img, 128, 255, cv::THRESH_BINARY_INV);
      cv::cvtColor(reversed_img, bgr_image, cv::COLOR_GRAY2BGR);
      img[i]=bgr_image;
    }
  }
  fclose(csv_file);
  for(int i=0;i<num_of_images;i++){
    free(images[i]);
  }
  free(images);
  while(T--){
    for(int i=0;i<num;i++){
      o2ac_msgs::updateDistributionGoal goal;
      goal.observation_type = goal.LOOK_OBSERVATION;
      goal.distribution = to_PoseWithCovariance(mean, covariance);
      goal.look_observation.gripper_pose = to_Pose(x[i],y[i],z[i],qw[i],qx[i],qy[i],qz[i]);
      goal.look_observation.looked_image= *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", img[i]).toImageMsg());
      goal.look_observation.calibration_points=calibration_points;
      goal.look_observation.ROI=ROI;
    
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
  }
  return 0;
}
