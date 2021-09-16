#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h> // Includes the TF conversions

#include <termios.h> // non-blocking getchar
#include <tf/transform_datatypes.h>

// Was this for logging?
#include "ros/ros.h"
#include <ros/console.h>

#include <algorithm> //For min
#include <math.h>
#include <string>

// RPY rotations are applied in the frame of the pose.
void rotatePoseByRPY(const double roll, const double pitch, const double yaw,
                     geometry_msgs::Pose &inpose) {
  tf::Quaternion q;
  tf::Quaternion qrotate = tf::createQuaternionFromRPY(roll, pitch, yaw);

  tf::quaternionMsgToTF(inpose.orientation, q);

  q = q * qrotate;

  tf::quaternionTFToMsg(q, inpose.orientation);
}

// Returns the angle between two quaternions
double quaternionDistance(geometry_msgs::Quaternion q1,
                          geometry_msgs::Quaternion q2) {
  tf::Quaternion q1tf, q2tf;
  tf::quaternionMsgToTF(q1, q1tf);
  tf::quaternionMsgToTF(q2, q2tf);
  return 2 * q1tf.angle(q2tf);
}

double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  tf::Point tp1, tp2;
  tf::pointMsgToTF(p1, tp1);
  tf::pointMsgToTF(p2, tp2);
  return tfDistance(tp1, tp2);
}

// (c) Salvo Virga, sankyu~~
// Transforms a stamped pose from its current reference frame (as given in its
// header) to referenceFrame
geometry_msgs::PoseStamped
transform_pose_now(geometry_msgs::PoseStamped &pose,
                   const std::string &referenceFrame,
                   const tf::TransformListener &listener) {
  // Check if the frames are different
  if (pose.header.frame_id != referenceFrame) {

    bool success = false;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped result_pose;

    while (!success) {
      try {
        ros::Time t = ros::Time::now();
        pose.header.stamp = t;
        listener.waitForTransform(pose.header.frame_id, referenceFrame, t,
                                  ros::Duration(3.0));
        listener.lookupTransform(pose.header.frame_id, referenceFrame, t,
                                 transform);
        listener.transformPose(referenceFrame, pose, result_pose);
        success = true;
        return result_pose;
      } catch (tf::ExtrapolationException e) {
        ROS_ERROR("Something went wrong in transform_pose_now.");
        // ROS_ERROR(e.what());
      }
      sleep(0.1);
    }
  }
  return pose;
}

// This function is for tuning the gripper orientation when grasping into a bin.
// Checks if an orientation is permissible (within tolerance of target_rotation)
// and if not, flips it by 180 degrees
double flipGraspRotationIfNecessary(double in_rotation, double target_rotation,
                                    double tolerance) {
  // Thank you internet!!!
  // https://github.com/petercorke/toolbox-common-matlab/blob/master/angdiff.m
  double angdiff = in_rotation - target_rotation;
  angdiff = fmod(angdiff + M_PI, 2.0 * M_PI) - M_PI;

  if (abs(angdiff) > tolerance) {
    // Flip rotation
    if (in_rotation <= 0.0) {
      return (in_rotation + M_PI);
    }
    if (in_rotation > 0.0) {
      return (in_rotation - M_PI);
    }
  }
  return in_rotation;
}

// Non-blocking getchar
int getch() {
  ROS_INFO("Press any key to continue...");
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

// Used to restrict rotation values to a certain interval.
double restrictValueToInterval(double input_value, double allowed_min,
                               double allowed_max) {
  if (input_value > allowed_max) {
    input_value = allowed_max;
  } else if (input_value < allowed_min) {
    input_value = allowed_min;
  }
  return input_value;
}

// Returns how far the value is from the interval
double distOfValueToInterval(double input_value, double allowed_min,
                             double allowed_max) {
  if (input_value > allowed_max) {
    return abs(input_value - allowed_max);
  } else if (input_value < allowed_min) {
    return abs(input_value - allowed_min);
  } else {
    return input_value;
  }
}

// Below are factory functions for common geometry messages.

geometry_msgs::Point makePoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z,
                                         double w) {
  geometry_msgs::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::Pose makePose() {
  geometry_msgs::Pose pose;
  pose.position = makePoint(0.0, 0.0, 0.0);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z) {
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p) {
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z,
                             geometry_msgs::Quaternion q) {
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = q;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p, double xq, double yq,
                             double zq, double w) {
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z, double xq, double yq,
                             double zq, double w) {
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}
