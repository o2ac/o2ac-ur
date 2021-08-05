#include "o2ac_msgs/updateDistributionAction.h"
#include "o2ac_msgs/visualizePoseBelief.h"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

void broadcast_gripper_pose(const std::string &frame_id,
                            const ros::Time &current_time,
                            const geometry_msgs::Pose &pose);

void send_pose_belief(
    ros::ServiceClient &visualizer_client,
    const moveit_msgs::CollisionObject &object,
    const unsigned char &distribution_type, const double &lifetime,
    const geometry_msgs::PoseWithCovarianceStamped &distribution,
    const bool frame_locked = true);

void load_CollisionObject_from_file(
    std::shared_ptr<moveit_msgs::CollisionObject> &object,
    const std::string &file_path);
