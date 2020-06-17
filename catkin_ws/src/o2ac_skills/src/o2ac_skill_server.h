#ifndef o2ac_SKILL_SERVER_H
#define o2ac_SKILL_SERVER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include <tf/transform_broadcaster.h>

#include "wait_for_ur_program.h"
#include "o2ac_helper_functions.h"

#include <chrono>
#include <thread>
#include <cmath>
#include <boost/thread/mutex.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>

// For IPTP scaling
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "visualization_msgs/Marker.h"
#include "o2ac_msgs/RobotStatus.h"

// Services
#include "o2ac_msgs/goToNamedPose.h"
#include "o2ac_msgs/sendScriptToUR.h"
#include "o2ac_msgs/publishMarker.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2ac_msgs/pickScrewAction.h"
#include "o2ac_msgs/placeAction.h"
#include "o2ac_msgs/regraspAction.h"
#include "o2ac_msgs/screwAction.h"
#include "o2ac_msgs/changeToolAction.h"
#include "o2ac_msgs/FastenerGripperControlAction.h"
#include "o2ac_msgs/SuctionControlAction.h"

#include <actionlib/client/simple_action_client.h>
#include <robotiq_msgs/CModelCommandAction.h>

class SkillServer
{
public:
  //Constructor
  SkillServer();
  void initializeCollisionObjects(); // Defines tool objects

  //Helpers (convenience functions)
  bool moveToJointPose(std::vector<double> joint_positions, std::string robot_name, bool wait = true, double velocity_scaling_factor = 1.0, bool use_UR_script = false, double acceleration = 0.0);
  bool moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1);
  bool moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1, double acceleration = 0.0, bool force_UR_script = false, bool force_moveit = false);
  bool goToNamedPose(std::string pose_name, std::string robot_name, double speed = 1.0, double acceleration = 0.0, bool use_UR_script=false);
  bool stop();                  // Stops the robot at the current position
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
  std::string getEELink(std::string robot_name);
  bool toggleCollisions(bool collisions_on);
  bool updatePlanningScene();
  
  // Internal functions
  bool equipScrewTool(std::string robot_name, std::string screw_tool_id);
  bool unequipScrewTool(std::string robot_name);
  bool equipUnequipScrewTool(std::string robot_name, std::string screw_tool_id, std::string equip_or_unequip);
  bool spawnTool(std::string screw_tool_id);
  bool despawnTool(std::string screw_tool_id);
  bool attachTool(std::string screw_tool_id, std::string link_name);
  bool detachTool(std::string screw_tool_id, std::string link_name);
  bool attachDetachTool(std::string screw_tool_id, std::string link_name, std::string attach_or_detach);
  bool placeFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name = "");
  bool pickFromAbove(geometry_msgs::PoseStamped target_tip_link_pose, std::string end_effector_link_name, std::string robot_name, std::string gripper_name = "");
  bool pickScrew(geometry_msgs::PoseStamped screw_head_pose, std::string screw_tool_id, std::string robot_name, std::string screw_tool_link, std::string fastening_tool_name);
  bool publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type = "");
  bool publishPoseMarker(geometry_msgs::PoseStamped marker_pose);

  bool openGripper(std::string robot_name, std::string gripper_name = "");
  bool closeGripper(std::string robot_name, std::string gripper_name = "");
  bool sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name = "");
  bool sendFasteningToolCommand(std::string fastening_tool_name, std::string direction = "tighten", bool wait = false, double duration = 20.0, int speed = 500);
  bool setSuctionEjection(std::string fastening_tool_name, bool turn_suction_on = true, bool eject_screw = false);

  // Callback declarations
  bool goToNamedPoseCallback(o2ac_msgs::goToNamedPose::Request &req,
                        o2ac_msgs::goToNamedPose::Response &res);
  bool publishMarkerCallback(o2ac_msgs::publishMarker::Request &req,
                        o2ac_msgs::publishMarker::Response &res);
  bool toggleCollisionsCallback(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

  void runModeCallback(const std_msgs::BoolConstPtr& msg);
  void pauseModeCallback(const std_msgs::BoolConstPtr& msg);
  void testModeCallback(const std_msgs::BoolConstPtr& msg);

  // Actions
  void executePickScrew(const o2ac_msgs::pickScrewGoalConstPtr& goal);
  void executePlace(const o2ac_msgs::placeGoalConstPtr& goal);
  void executeRegrasp(const o2ac_msgs::regraspGoalConstPtr& goal);
  void executeScrew(const o2ac_msgs::screwGoalConstPtr& goal);
  void executeChangeTool(const o2ac_msgs::changeToolGoalConstPtr& goal);
  

// private:
  ros::NodeHandle n_;
  bool use_real_robot_;
  bool run_mode_, pause_mode_, test_mode_;
  double reduced_speed_limit_ = .25;
  double speed_fast = 1.5, speed_fastest = 3.0;
  double acc_fast = 1.5, acc_fastest = 2.0;
  boost::mutex mutex_;

  ros::Subscriber subRunMode_, subPauseMode_, subTestMode_;
  ros::Publisher pubMarker_;
  int marker_id_count = 0;

  // Service declarations
  ros::ServiceServer goToNamedPoseService_;
  ros::ServiceServer publishMarkerService_;
  ros::ServiceServer toggleCollisionsService_;

  // Service clients
  ros::ServiceClient sendScriptToURClient_;
  
  // Action declarations
  actionlib::SimpleActionServer<o2ac_msgs::pickScrewAction> pickScrewActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::placeAction> placeActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::regraspAction> regraspActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::screwAction> screwActionServer_;  
  actionlib::SimpleActionServer<o2ac_msgs::changeToolAction> changeToolActionServer_;

  // Action clients
  // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> a_bot_gripper_client_;
  actionlib::SimpleActionClient<robotiq_msgs::CModelCommandAction> a_bot_gripper_client_, b_bot_gripper_client_;
  actionlib::SimpleActionClient<o2ac_msgs::FastenerGripperControlAction> fastening_tool_client;
  actionlib::SimpleActionClient<o2ac_msgs::SuctionControlAction> suction_client;

  double PLANNING_TIME = 5.0, LIN_PLANNING_TIME = 15.0;
  
  // Status variables
  tf::TransformListener tflistener_;
  tf::TransformBroadcaster tfbroadcaster_;
  bool holding_object_ = false;
  // A status of the robot. This should almost definitely be rosparams or a topic instead.
  std::map<std::string, o2ac_msgs::RobotStatus> robot_statuses_;
  std::string held_object_id_ = "";
  std::string held_screw_tool_ = "";    // "m3", "m4", "m5", "nut"...

  // MoveGroup connections
  moveit_msgs::PlanningScene planning_scene_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface b_bot_group_;
  
  moveit_msgs::CollisionObject screw_tool_m6, screw_tool_m4, screw_tool_m3, suction_tool, nut_tool, set_screw_tool;

};//End of class SkillServer

#endif //o2ac_SKILL_SERVER_H
