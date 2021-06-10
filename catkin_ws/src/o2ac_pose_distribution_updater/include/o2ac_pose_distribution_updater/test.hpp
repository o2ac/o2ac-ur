#include "o2ac_msgs/updateDistributionAction.h"
#include "o2ac_msgs/visualizePoseBelief.h"
#include "o2ac_pose_distribution_updater/distribution_conversions.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

using Client =
    actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction>;

void load_CollisionObject_from_file(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::string &file_path);

void msg_pose_to_msg_transform(const geometry_msgs::Pose &pose,
                               geometry_msgs::Transform &transform);

void broadcast_gripper_pose(const std::string &frame_id,
                            const ros::Time &current_time,
                            const geometry_msgs::Pose &pose);

void send_pose_belief(
    ros::ServiceClient &visualizer_client,
    const moveit_msgs::CollisionObject &object,
    const unsigned char &distribution_type,
    const geometry_msgs::PoseWithCovarianceStamped &distribution);

void touch_test(const std::shared_ptr<Client> &client,
                const std::string &input_file_path,
                const std::string &gripped_geometry_file_path,
                const unsigned char &distribution_type);

void look_test(const std::shared_ptr<Client> &client,
               const std::string &csv_file_path,
               const std::string &image_direcotory_path,
               const std::string &gripped_geometry_file_path,
               const std::vector<int> &ROI_values, const int &beginning,
               const int &number_of_images_to_send, const int &number_of_turns,
               const unsigned char &distribution_type);

void place_test(const std::shared_ptr<Client> &client,
                const std::string &test_file_path,
                const std::string &gripped_geometry_file_path,
                const unsigned char &distribution_type, const bool &visualize,
                ros::ServiceClient &visualizer_client,
                const bool distribution_convert = false);

void grasp_test(const std::shared_ptr<Client> &client,
                const std::string &test_file_path,
                const std::string &gripped_geometry_file_path,
                const unsigned char &distribution_type, const bool &visualize,
                ros::ServiceClient &visualizer_client,
                ros::Publisher &marker_publisher,
                const bool distribution_convert = false);
