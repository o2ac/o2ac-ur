#include "o2ac_pose_distribution_updater/estimator.hpp"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"
#include <cv_bridge/cv_bridge.h>

class ROSConvertedPoseEstimator : public PoseEstimator {
public:
  void touched_step(const unsigned char &touched_object_id,
                    const moveit_msgs::CollisionObject &gripped_object,
                    const geometry_msgs::Pose &gripper_pose,
                    const geometry_msgs::PoseWithCovariance &old_distibution,
                    geometry_msgs::PoseWithCovariance &new_distribution);
  void place_step(const moveit_msgs::CollisionObject &gripped_object,
                  const geometry_msgs::Pose &gripper_pose,
                  const double &support_surface,
                  const geometry_msgs::PoseWithCovariance &old_distibution,
                  geometry_msgs::PoseWithCovariance &new_distribution);

  void look_step(const moveit_msgs::CollisionObject &gripped_object,
                 const geometry_msgs::Pose &gripper_pose,
                 const sensor_msgs::Image &looked_image,
                 const boost::array<unsigned int, 4> &range_of_interest,
                 const geometry_msgs::PoseWithCovariance &old_distibution,
                 geometry_msgs::PoseWithCovariance &new_distribution);
};
