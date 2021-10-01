#ifndef o2ac_SKILL_SERVER_H
#define o2ac_SKILL_SERVER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include <tf/transform_broadcaster.h>

#include "wait_for_ur_program.h"
#include "o2ac_helper_functions.h"

#include <chrono>
#include <thread>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <algorithm>  // max

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
#include "o2ac_msgs/sendScriptToUR.h"
#include "o2ac_msgs/publishMarker.h"

// Actions
#include <actionlib/server/simple_action_server.h>
#include "o2ac_msgs/suckScrewAction.h"
#include "o2ac_msgs/pickScrewFromFeederAction.h"
#include "o2ac_msgs/placeAction.h"
#include "o2ac_msgs/regraspAction.h"
#include "o2ac_msgs/screwAction.h"
#include "o2ac_msgs/changeToolAction.h"
#include "o2ac_msgs/ScrewToolControlAction.h"
#include "o2ac_msgs/SuctionControlAction.h"

// For the URe program activation
#include "ur_dashboard_msgs/GetLoadedProgram.h"
#include "ur_dashboard_msgs/IsProgramRunning.h"
#include "ur_dashboard_msgs/Load.h"

#include <actionlib/client/simple_action_client.h>
#include <robotiq_msgs/CModelCommandAction.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 6.283185307179586476925;

class SkillServer
{
public:
  //Constructor
  SkillServer();
  void initializeCollisionObjects(); // Defines tool objects
  void advertiseActionsAndServices();

  //Helpers (convenience functions)
  bool activateROSControlOnUR(std::string robot_name, int recursion_depth = 0);
  bool hardReactivate(std::string robot_name);
  bool moveToJointPose(std::vector<double> joint_positions, std::string robot_name, bool wait = true, double velocity_scaling_factor = 1.0, bool use_UR_script = false, double acceleration = 0.5);
  bool moveToCartPosePTP(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1);
  bool moveToCartPoseLIN(geometry_msgs::PoseStamped pose, std::string robot_name, bool wait = true, std::string end_effector_link = "", double velocity_scaling_factor = 0.1, double acceleration = 0.5, bool force_UR_script = false, bool force_moveit = false);
  bool goToNamedPose(std::string pose_name, std::string robot_name, double speed = 1.0, double acceleration = 0.5, bool use_UR_script=false);
  bool stop();                  // Stops the robot at the current position
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
  std::string getEELink(std::string robot_name);
  bool toggleCollisions(bool collisions_on);
  bool updatePlanningScene();
  
  // Internal functions
  void updateRobotStatusFromParameterServer(const std::string robot_name);
  void updateRobotStatus(std::string robot_name, bool carrying_object, bool carrying_tool, std::string held_tool_id);
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
  bool suckScrew(geometry_msgs::PoseStamped screw_head_pose, std::string screw_tool_id, std::string robot_name, std::string screw_tool_link, std::string fastening_tool_name);
  bool publishMarker(geometry_msgs::PoseStamped marker_pose, std::string marker_type = "");
  bool publishPoseMarker(geometry_msgs::PoseStamped marker_pose);

  bool openGripper(std::string robot_name, std::string gripper_name = "", bool wait = true);
  bool closeGripper(std::string robot_name, std::string gripper_name = "", bool wait = true);
  bool sendGripperCommand(std::string robot_name, double opening_width, std::string gripper_name = "", bool wait = true);
  bool sendFasteningToolCommand(std::string fastening_tool_name, std::string direction = "tighten", bool wait = false, double duration = 20.0, int speed = 0, bool skip_final_loosen_and_retighten = false);
  bool setSuctionEjection(std::string fastening_tool_name, bool turn_suction_on = true, bool eject_screw = false);

  // Callback declarations
  bool publishMarkerCallback(o2ac_msgs::publishMarker::Request &req,
                        o2ac_msgs::publishMarker::Response &res);
  bool toggleCollisionsCallback(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

  void runModeCallback(const std_msgs::BoolConstPtr& msg);
  void pauseModeCallback(const std_msgs::BoolConstPtr& msg);
  void testModeCallback(const std_msgs::BoolConstPtr& msg);
  void aBotStatusCallback(const std_msgs::BoolConstPtr& msg);
  void bBotStatusCallback(const std_msgs::BoolConstPtr& msg);
  void m3SuctionCallback(const std_msgs::BoolConstPtr& msg);
  void m4SuctionCallback(const std_msgs::BoolConstPtr& msg);

  // Actions
  void executeSuckScrew(const o2ac_msgs::suckScrewGoalConstPtr& goal);
  void executePickScrewFromFeeder(const o2ac_msgs::pickScrewFromFeederGoalConstPtr& goal);
  void executePlace(const o2ac_msgs::placeGoalConstPtr& goal);
  void executeRegrasp(const o2ac_msgs::regraspGoalConstPtr& goal);
  void executeScrew(const o2ac_msgs::screwGoalConstPtr& goal);
  void executeChangeTool(const o2ac_msgs::changeToolGoalConstPtr& goal);
  bool activateCamera(const int camera_id);
  

// private:
  ros::NodeHandle n_;
  bool use_real_robot_;
  bool a_bot_ros_control_active_, b_bot_ros_control_active_;
  bool run_mode_, pause_mode_, test_mode_;
  double reduced_speed_limit_ = .25;
  double speed_fast = 1.5, speed_fastest = 3.0;
  double acc_fast = 1.5, acc_fastest = 2.0;
  boost::mutex mutex_;

  ros::Subscriber sub_a_bot_status_, sub_b_bot_status_, sub_m3_screw_suction_, sub_m4_screw_suction_;
  ros::Subscriber subRunMode_, subPauseMode_, subTestMode_;
  ros::Publisher pubMarker_;
  int marker_id_count = 0;

  // Service declarations
  ros::ServiceServer publishMarkerService_;
  ros::ServiceServer toggleCollisionsService_;

  // Service clients
  ros::ServiceClient sendScriptToURClient_;
  ros::ServiceClient a_bot_get_loaded_program_, a_bot_program_running_, a_bot_load_program_, a_bot_play_, a_bot_stop_, b_bot_get_loaded_program_, b_bot_program_running_, b_bot_load_program_, b_bot_play_, b_bot_stop_;
  
  // Action servers
  actionlib::SimpleActionServer<o2ac_msgs::suckScrewAction> suckScrewActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::pickScrewFromFeederAction> pickScrewFromFeederActionServer_;
  o2ac_msgs::pickScrewFromFeederResult pick_screw_from_feeder_result_;
  
  actionlib::SimpleActionServer<o2ac_msgs::placeAction> placeActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::regraspAction> regraspActionServer_;
  actionlib::SimpleActionServer<o2ac_msgs::screwAction> screwActionServer_;  
  o2ac_msgs::screwResult screw_result_;
  actionlib::SimpleActionServer<o2ac_msgs::changeToolAction> changeToolActionServer_;  // Superceded by Python

  // Action clients
  actionlib::SimpleActionClient<robotiq_msgs::CModelCommandAction> a_bot_gripper_client_, b_bot_gripper_client_;
  actionlib::SimpleActionClient<o2ac_msgs::ScrewToolControlAction> fastening_tool_client;
  actionlib::SimpleActionClient<o2ac_msgs::SuctionControlAction> suction_client;

  double PLANNING_TIME = 5.0, LIN_PLANNING_TIME = 15.0;
  
  // Status variables
  tf::TransformListener tflistener_;
  tf::TransformBroadcaster tfbroadcaster_;
  bool holding_object_ = false;
  std::map<std::string, bool> screw_suctioned_;  // True/False for "screw_tool_m4" and "screw_tool_m3"
  // A status of the robot. This should almost definitely be rosparams or a topic instead.
  std::map<std::string, o2ac_msgs::RobotStatus> robot_statuses_;
  std::string held_object_id_ = "";
  std::string held_screw_tool_ = "";    // "m3", "m4", "m5", "nut"...

  // MoveGroup connections
  moveit_msgs::PlanningScene planning_scene_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface a_bot_group_, b_bot_group_;
  
  moveit_msgs::CollisionObject screw_tool_m6, screw_tool_m4, screw_tool_m3, suction_tool, nut_tool, set_screw_tool;

};//End of class SkillServer

#endif //o2ac_SKILL_SERVER_H
