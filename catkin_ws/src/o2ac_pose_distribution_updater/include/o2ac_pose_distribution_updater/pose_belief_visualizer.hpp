#include "o2ac_msgs/visualizePoseBelief.h"
#include "o2ac_pose_distribution_updater/ros_converters.hpp"
#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class PoseBeliefVisualizer {
  // This class provides functions to visualize pose beliefs.

private:
  ros::Publisher marker_publisher;

  // object namespace data
  std::string object_namespace;
  int object_id;

  // parameters to visualize
  geometry_msgs::Vector3 scale;
  std_msgs::ColorRGBA mean_color, variance_color;

  void make_marker_from_particle(
      const std_msgs::Header &header,
      const std::vector<geometry_msgs::Point> &triangle_list,
      const Particle &particle, const std_msgs::ColorRGBA &color,
      visualization_msgs::Marker &marker);

  void inner_publish_marker_for_pose_belief(
      o2ac_msgs::visualizePoseBelief::Request &belief,
      o2ac_msgs::visualizePoseBelief::Response &response);

public:
  PoseBeliefVisualizer(ros::NodeHandle &nd, const std::string &topic_name) {
    // start the publisher
    marker_publisher =
        nd.advertise<visualization_msgs::MarkerArray>(topic_name, 1);

    // initialize the object id
    object_namespace = "pose_belief";
    object_id = 0;
  }
  // functions to set the parameters
  void set_scale(const double &x, const double &y, const double &z) {
    scale.x = x;
    scale.y = y;
    scale.z = z;
  }

  void set_mean_color(const double &r, const double &g, const double &b,
                      const double &a) {
    mean_color.r = r;
    mean_color.g = g;
    mean_color.b = b;
    mean_color.a = a;
  }

  void set_variance_color(const double &r, const double &g, const double &b,
                          const double &a) {
    variance_color.r = r;
    variance_color.g = g;
    variance_color.b = b;
    variance_color.a = a;
  }

  bool publish_marker_for_pose_belief(
      o2ac_msgs::visualizePoseBelief::Request &belief,
      o2ac_msgs::visualizePoseBelief::Response &response) {
    // actual process is done in 'inner_publish_marker_for_pose_belief'
    // procedure

    try {
      inner_publish_marker_for_pose_belief(belief, response);
    } catch (std::exception &e) {
      ROS_ERROR("%s", e.what());
      return false;
    } catch (...) {
      ROS_ERROR("Unkown Exception");
      return false;
    }
    return true;
  }
};
