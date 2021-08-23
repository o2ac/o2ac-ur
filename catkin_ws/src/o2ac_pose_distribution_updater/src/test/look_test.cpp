/*
The implementation of the look test
*/

#include "o2ac_pose_distribution_updater/test.hpp"

void look_test(const std::shared_ptr<Client> &client,
               const std::string &csv_file_path,
               const std::string &image_directory_path,
               const std::string &gripped_geometry_file_path,
               const std::vector<int> &ROI_values, const int &beginning,
               const int &number_of_images_to_send, const int &number_of_turns,
               const unsigned char &distribution_type) {
  /*
      This procedure reads the csv file, whose path is given, representing the
     gripper pose for each time and the directory, whose path is also give,
     containing images. Each line after the first one of the csv file must be of
     the form "t,x,y,z,qx,qy,qz,qw" where 't' is the time and other seven values
     represents the pose of gripper at the time 't'. Each file in the image
     directory must have a name of the form "[t].jpg" where [t] is the time when
     the image is looked

      Starting at the 'beginning'-th file in the image directory, this procedure
     reads 'number_of_images_to_send' images from the 'number_of_images_to_send'
     image files in the image directory and finds the corresponding gripper pose
     from csv file for each image. The client sends the look action call to the
     server for each image and repeat it 'number_of_turns' times.

      After each action call, this procedure prints the values of mean and
     covariance matrix of the pose distribution to ROS_INFO.
  */

  // Load object from stl file
  std::shared_ptr<moveit_msgs::CollisionObject> gripped_geometry;
  load_CollisionObject_from_file(gripped_geometry, gripped_geometry_file_path);

  // Open the csv files and image directory
  FILE *csv_file = fopen(csv_file_path.c_str(), "r");
  ASSERT_FALSE(csv_file == NULL);
  char c;
  while ((c = getc(csv_file)) != '\n')
    ; // ingore the first line
  struct dirent **image_files;
  int number_of_image_files =
      scandir(image_directory_path.c_str(), &image_files, NULL, versionsort);
  ASSERT_TRUE(number_of_image_files > 0);

  // Set the initial value of the distribution
  Particle mean;
  mean.setZero();
  CovarianceMatrix covariance;
  covariance.setZero();
  double variance_xyz = 0.01, variance_angle = 1.0;
  covariance.diagonal() << variance_xyz, variance_xyz, variance_xyz,
      variance_angle, variance_angle, variance_angle;
  // Read range of interests
  boost::array<unsigned int, 4> ROI;
  for (int i = 0; i < 4; i++)
    ROI[i] = ROI_values[i];

  // Read images from the image directory and gripper poses from the csv file
  std::vector<cv::Mat> image(number_of_images_to_send);
  std::vector<double> x(number_of_images_to_send), y(number_of_images_to_send),
      z(number_of_images_to_send), qx(number_of_images_to_send),
      qy(number_of_images_to_send), qz(number_of_images_to_send),
      qw(number_of_images_to_send);
  for (int i = 0; i < number_of_images_to_send; i++) {
    char *image_file_name = image_files[beginning + i]->d_name;
    unsigned long long image_time, gripper_time;
    sscanf(image_file_name, "%llu.jpg", &image_time);
    while (fscanf(csv_file, "%llu,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &gripper_time,
                  &x[i], &y[i], &z[i], &qx[i], &qy[i], &qz[i], &qw[i]) != EOF) {
      if (gripper_time >= image_time) {
        break;
      }
    }
    image[i] =
        cv::imread(image_directory_path + '/' + std::string(image_file_name));
  }

  // Free memory
  fclose(csv_file);
  for (int i = 0; i < number_of_image_files; i++) {
    free(image_files[i]);
  }
  free(image_files);

  // Send action calls
  for (int turn = 0; turn < number_of_turns;
       turn++) { // repeating in 'number_of_turns' turns
    for (int i = 0; i < number_of_images_to_send; i++) {
      ROS_INFO("Turn: %d, Image:%d", turn, i);
      o2ac_msgs::updateDistributionGoal goal;
      goal.observation_type = goal.LOOK_OBSERVATION;
      goal.distribution_type = distribution_type;
      goal.distribution.pose = to_PoseWithCovariance(mean, covariance);
      goal.gripped_object = *gripped_geometry;
      goal.gripper_pose.pose =
          to_Pose(x[i], y[i], z[i], qw[i], qx[i], qy[i], qz[i]);
      goal.look_observation.looked_image =
          *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[i])
                .toImageMsg());
      goal.look_observation.ROI = ROI;

      client->sendGoal(goal);
      client->waitForResult();
      auto result = client->getResult();
      if (result->success) {
        mean = pose_to_particle(result->distribution.pose.pose);
        covariance =
            array_36_to_matrix_6x6(result->distribution.pose.covariance);
        ROS_INFO_STREAM(mean);
        ROS_INFO_STREAM(covariance);
      } else {
        ASSERT_TRUE(
            std::string("The sum of likelihoods is 0") ==
                result->error_message.data ||
            std::string("Only single particle has non-zero likelihood") ==
                result->error_message.data);
      }
    }
  }
}
