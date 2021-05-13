#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using Client =
    actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction>;

void load_CollisionObject_from_file(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::string &file_path);

void touch_test(const std::shared_ptr<Client> &client,
                const std::string &input_file_path,
                const std::string &gripped_geometry_file_path);

void look_test(const std::shared_ptr<Client> &client,
               const std::string &csv_file_path,
               const std::string &image_direcotory_path,
               const std::string &gripped_geometry_file_path,
               const std::vector<int> &ROI_values, const int &beginning,
               const int &number_of_images_to_send, const int &number_of_turns);

void place_test(const std::shared_ptr<Client> &client,
                const std::string &test_file_path,
                const std::string &gripped_geometry_file_path);
