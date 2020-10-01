/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Artur Istvan Karoly, Felix von Drigalski
   Desc:   Class for creating pick-place/pick-hold tasks and their building blocks (pick object, lift object, place object, release object) in a modular fashion and 
	provide these planning capabilities as ros actions.
*/

#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <stages/load_grasp_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <stages/dummy.h>
#include <stages/generate_pose.h>
#include <stages/generate_place_pose.h>
#include <stages/generate_handover_pose.h>
#include <moveit/task_constructor/container.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <o2ac_task_planning_msgs/PickObjectAction.h>
#include <o2ac_task_planning_msgs/PlaceObjectAction.h>
#include <o2ac_task_planning_msgs/PlaceObjectWithCorrectionAction.h>
#include <o2ac_task_planning_msgs/ReleaseObjectAction.h>
#include <o2ac_task_planning_msgs/PickPlaceWithRegraspAction.h>

#include <actionlib/server/simple_action_server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <cmath>
#include <tf/transform_listener.h>

namespace mtc_modules {
constexpr char LOGNAME[] = "mtc_modules";
using namespace moveit::task_constructor;

// Class for the action servers
class Modules_Planner{
	public:

		// Start action servers when calling the constructor:
		Modules_Planner(): pick_planning_server(Modules_Planner::nh, "pick_planning", boost::bind(&Modules_Planner::pick_planning_server_cb, this, _1), false),
		place_planning_server(Modules_Planner::nh, "place_planning", boost::bind(&Modules_Planner::place_planning_server_cb, this, _1), false),
		place_with_correction_planning_server(Modules_Planner::nh, "place_with_correction_planning", boost::bind(&Modules_Planner::place_with_correction_planning_server_cb, this, _1), false),
		release_planning_server(Modules_Planner::nh, "release_planning", boost::bind(&Modules_Planner::release_planning_server_cb, this, _1), false),
		pick_place_planning_server(Modules_Planner::nh, "pick_place_planning", boost::bind(&Modules_Planner::pick_place_planning_server_cb, this, _1), false),
		fastening_planning_server(Modules_Planner::nh, "fastening_planning", boost::bind(&Modules_Planner::fastening_planning_server_cb, this, _1), false),
		wrs_subtask_b_planning_server(Modules_Planner::nh, "wrs_subtask_b_planning", boost::bind(&Modules_Planner::wrs_subtask_b_planning_server_cb, this, _1), false)
    	{
			pick_place_planning_server.start();
        	pick_planning_server.start();
			place_planning_server.start();
			place_with_correction_planning_server.start();
			release_planning_server.start();
			fastening_planning_server.start();
			wrs_subtask_b_planning_server.start();
			ROS_INFO_NAMED(LOGNAME, "Starting MTC Modules action servers");
    	}

		// Initialize internal parameters of the class
		void init();

		// Functions that are called to build the tasks
		std::unique_ptr<SerialContainer> Pick_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Lift_Object(const std::string& object);
		std::unique_ptr<SerialContainer> Pick_and_Lift(const std::string& object, const std::string& arm_group_name, bool this_is_start=true);
		std::unique_ptr<SerialContainer> Place_Object(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Place_Object(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Place_Object_With_Correction(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Place_Object_With_Correction(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Release_Object_and_Retreat(const std::string& object, const std::string& pose_to_retreat_to="");

		std::unique_ptr<SerialContainer> Release_and_Retreat(const std::string& object, const std::string& arm_group_name, const std::string& pose_to_retreat_to="", bool this_is_start=true);
		std::unique_ptr<SerialContainer> Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="", bool this_is_start=true);
		std::unique_ptr<SerialContainer> Place_With_Correction(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="", bool this_is_start=true);
		std::unique_ptr<SerialContainer> Fasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place="", bool this_is_start=true, const std::string& container_name="Fasten");
		std::unique_ptr<SerialContainer> Fasten(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place="", bool this_is_start=true, const std::string& container_name="Fasten");
		std::unique_ptr<SerialContainer> Pick_Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="");
		std::unique_ptr<SerialContainer> Pick_Place_with_Regrasp(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object=true, const std::string& object_subframe_to_place="");
		
		std::unique_ptr<Alternatives> Pick_and_Lift_Alternatives(const std::string& object, bool this_is_start=true);
		std::unique_ptr<Alternatives> Release_and_Retreat_Alternatives(const std::string& object, const std::string& pose_to_retreat_to="", bool this_is_start=true);
		std::unique_ptr<Alternatives> Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="", bool this_is_start=true);
		std::unique_ptr<Alternatives> Place_With_Correction_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="", bool this_is_start=true);
		std::unique_ptr<Alternatives> Fasten_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="", bool this_is_start=true, const std::string& container_name="Fasten");
		std::unique_ptr<Alternatives> Fasten_Alternatives(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place="", bool this_is_start=true, const std::string& container_name="Fasten");
		std::unique_ptr<Alternatives> Pick_Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		
		std::unique_ptr<Fallbacks> Pick_Place_Fallback(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="", bool force_robot_order = false);

		// Task definitions
		void createPickPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="", bool force_robot_order = false);
		void createPick(const std::string& object);
		void createPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		void createPlaceWithCorrection(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");
		void createReleaseRetreat(const std::string& object, const std::string& pose_to_retreat_to="");
		void createFasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place="");
		void createWRSSubtaskMotorPlate(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object=true, const std::string& object_subframe_to_place="");

		/****************************************************
		 *                                                  *
		 *    Callback functions for the action servers     *
		 *                                                  *
		 ***************************************************/

		void pick_planning_server_cb(const o2ac_task_planning_msgs::PickObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_grasp_parameter_location = grasp_parameter_location;
			std::string temp_lift_direction_reference_frame = lift_direction_reference_frame;
			std::vector<double> temp_lift_direction = lift_direction;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->grasp_parameter_location != ""){
				grasp_parameter_location = goal->grasp_parameter_location;
			}

			if (goal->lift_direction_reference_frame != "" && !goal->lift_direction.empty()){
				lift_direction_reference_frame = goal->lift_direction_reference_frame;
				lift_direction = goal->lift_direction;
			}

			if (goal->robot_name != ""){
				arm_group_names = {goal->robot_name};
			}

			createPick(goal->object_name);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			pick_result.success = success;
			pick_result.solution = sol;

			grasp_parameter_location = temp_grasp_parameter_location;
			lift_direction_reference_frame = temp_lift_direction_reference_frame;
			lift_direction = temp_lift_direction;
			arm_group_names = temp_arm_group_names;
			pick_planning_server.setSucceeded(pick_result);
		}

		void place_planning_server_cb(const o2ac_task_planning_msgs::PlaceObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			if (goal->robot_name != ""){
				arm_group_names = {goal->robot_name};
			}

			createPlace(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			place_result.success = success;
			place_result.solution = sol;

			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			arm_group_names = temp_arm_group_names;
			place_planning_server.setSucceeded(place_result);
		}

		void place_with_correction_planning_server_cb(const o2ac_task_planning_msgs::PlaceObjectWithCorrectionGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			if (goal->robot_name != ""){
				arm_group_names = {goal->robot_name};
			}

			createPlaceWithCorrection(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			place_with_correction_result.success = success;
			place_with_correction_result.solution = sol;

			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			arm_group_names = temp_arm_group_names;
			place_with_correction_planning_server.setSucceeded(place_with_correction_result);
		}
		void release_planning_server_cb(const o2ac_task_planning_msgs::ReleaseObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->robot_name != ""){
				arm_group_names = {goal->robot_name};
			}

			createReleaseRetreat(goal->object_name, goal->pose_to_retreat_to);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			release_result.success = success;
			release_result.solution = sol;

			arm_group_names = temp_arm_group_names;
			release_planning_server.setSucceeded(release_result);
		}

		void fastening_planning_server_cb(const o2ac_task_planning_msgs::PlaceObjectGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			if (goal->robot_name != ""){
				arm_group_names = {goal->robot_name};
			}

			createFasten(goal->object_name, goal->object_target_pose, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			place_result.success = success;
			place_result.solution = sol;

			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			arm_group_names = temp_arm_group_names;
			fastening_planning_server.setSucceeded(place_result);
		}

		void pick_place_planning_server_cb(const o2ac_task_planning_msgs::PickPlaceWithRegraspGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_grasp_parameter_location = grasp_parameter_location;
			std::string temp_lift_direction_reference_frame = lift_direction_reference_frame;
			std::vector<double> temp_lift_direction = lift_direction;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;
			std::vector<std::string> temp_arm_group_names = arm_group_names;

			if (goal->grasp_parameter_location != ""){
				grasp_parameter_location = goal->grasp_parameter_location;
			}

			if (goal->lift_direction_reference_frame != "" && !goal->lift_direction.empty()){
				lift_direction_reference_frame = goal->lift_direction_reference_frame;
				lift_direction = goal->lift_direction;
			}

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			if (!goal->robot_names.empty()){
				arm_group_names = goal->robot_names;
			}

			createPickPlace(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place, goal->force_robot_order);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			pick_place_result.success = success;
			pick_place_result.solution = sol;

			grasp_parameter_location = temp_grasp_parameter_location;
			lift_direction_reference_frame = temp_lift_direction_reference_frame;
			lift_direction = temp_lift_direction;
			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			arm_group_names = temp_arm_group_names;
			pick_place_planning_server.setSucceeded(pick_place_result);
		}

		void wrs_subtask_b_planning_server_cb(const o2ac_task_planning_msgs::PickPlaceWithRegraspGoalConstPtr& goal){
			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			std::string temp_grasp_parameter_location = grasp_parameter_location;
			std::string temp_lift_direction_reference_frame = lift_direction_reference_frame;
			std::vector<double> temp_lift_direction = lift_direction;
			std::string temp_approach_place_direction_reference_frame = approach_place_direction_reference_frame;
			std::vector<double> temp_approach_place_direction = approach_place_direction;

			if (goal->grasp_parameter_location != ""){
				grasp_parameter_location = goal->grasp_parameter_location;
			}

			if (goal->lift_direction_reference_frame != "" && !goal->lift_direction.empty()){
				lift_direction_reference_frame = goal->lift_direction_reference_frame;
				lift_direction = goal->lift_direction;
			}

			if (goal->approach_place_direction_reference_frame != "" && !goal->approach_place_direction.empty()){
				approach_place_direction_reference_frame = goal->approach_place_direction_reference_frame;
				approach_place_direction = goal->approach_place_direction;
			}

			createWRSSubtaskMotorPlate(goal->object_name, goal->object_target_pose, goal->release_object_after_place, goal->object_subframe_to_place);

			try {
				success = task_->plan(10);
				if (success && task_->numSolutions() != 0){
					ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
					task_->solutions().front()->fillMessage(sol);
				} else{
					ROS_INFO_NAMED(LOGNAME, "Planning failed");
				}
			} catch (const moveit::task_constructor::InitStageException& ex) {
				std::cerr << "planning failed with exception" << std::endl << ex;
			}

			pick_place_result.success = success;
			pick_place_result.solution = sol;

			grasp_parameter_location = temp_grasp_parameter_location;
			lift_direction_reference_frame = temp_lift_direction_reference_frame;
			lift_direction = temp_lift_direction;
			approach_place_direction_reference_frame = temp_approach_place_direction_reference_frame;
			approach_place_direction = temp_approach_place_direction;
			wrs_subtask_b_planning_server.setSucceeded(pick_place_result);
		}

	private:
		ros::NodeHandle nh;

		// Action servers and result messages
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PickPlaceWithRegraspAction> pick_place_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PickPlaceWithRegraspAction> wrs_subtask_b_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PickObjectAction> pick_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PlaceObjectAction> place_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PlaceObjectAction> place_with_correction_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::ReleaseObjectAction> release_planning_server;
		actionlib::SimpleActionServer<o2ac_task_planning_msgs::PlaceObjectAction> fastening_planning_server;
		o2ac_task_planning_msgs::PickPlaceWithRegraspResult pick_place_result;
		o2ac_task_planning_msgs::PickObjectResult pick_result;
		o2ac_task_planning_msgs::PlaceObjectResult place_result;
		o2ac_task_planning_msgs::PlaceObjectWithCorrectionResult place_with_correction_result;
		o2ac_task_planning_msgs::ReleaseObjectResult release_result;

		// The task (whenever there is a call for an action server this task is reset)
		moveit::task_constructor::TaskPtr task_;

		// Planners
		moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner;
		moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner;

		// Planning Groups
		std::vector<std::string> arm_group_names;
		std::vector<std::string> hand_group_names;

		std::string group;
		std::string hand_group_name;

		// Frames
		std::vector<std::string> hand_frames;

		std::string hand_frame;

		// Robot Model
		moveit::core::RobotModelConstPtr robot_model_;

		// Internal stage pointer for hooks
		Stage* current_state_stage;
		Stage* attach_object_stage;
		Stage* lift_object_stage;

		// Grasp parameter namespace
		std::string grasp_parameter_location;

		// Lifting object
		std::string lift_direction_reference_frame;
		std::vector<double> lift_direction;

		// Placing object
		std::string approach_place_direction_reference_frame;
		std::vector<double> approach_place_direction;

		// Retreat after place
		std::string retreat_direction_reference_frame;
		std::vector<double> retreat_direction;

		// Support surfaces
		std::vector<std::string> support_surfaces;
};

void Modules_Planner::init(){
	// Initializing internal parameters of the class

	// Load required params from the param server
	ROS_INFO_NAMED(LOGNAME, "Initializing Modules Planner");
	ros::NodeHandle pnh("~");

	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_names", arm_group_names);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_names", hand_group_names);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_parameter_location", grasp_parameter_location);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_direction_reference_frame", lift_direction_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_direction", lift_direction);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_place_direction_reference_frame", approach_place_direction_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_place_direction", approach_place_direction);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "retreat_direction_reference_frame", retreat_direction_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "retreat_direction", retreat_direction);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "support_surfaces", support_surfaces);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	// Init Cartesian planner
	cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Init sampling planner
	sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);


	// Init hand frames
	for (std::string group : hand_group_names){
		std::string hand_frame = group + "_tip_link";

		hand_frames.push_back(hand_frame);
	}

	// Init current robot
	group = arm_group_names[0];
	hand_group_name = hand_group_names[0];
	hand_frame = hand_frames[0];

	// Internal stage pointer for hooks
	current_state_stage = nullptr;
	attach_object_stage = nullptr;
	lift_object_stage = nullptr;

	ROS_INFO_NAMED(LOGNAME, "Initialization finished!\nReady to start planning ...");
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Object(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Grasp '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Approach object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
		stage->properties().set("marker_ns", "approach_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);
		// stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);
		//TODO: Make this parameterizable instead of hard-coded for 0.1 and 0.15

		// Set hand forward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame;
		//TODO: Make this parameterizable instead of hard-coded for hand frame and x direction (see retreat_direction_reference_frame and retreat_direction)
		vec.vector.x = 1.0;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Generate Grasp Pose                *
	 *                                                  *
	 ***************************************************/
	{
		// Load grasp pose
		auto stage = std::make_unique<stages::LoadGraspPose>("load grasp pose");
		stage->properties().set("marker_ns", "grasp_pose");
		stage->setPreGraspPose("open");
		stage->setAssembly(grasp_parameter_location);
		stage->setObject(object);
		stage->setMonitoredStage(current_state_stage);  // Hook into current state
		stage->setEndEffector(group + "_tip");

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(8);
		wrapper->setMinSolutionDistance(1.0);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}

	/****************************************************
	 *                                                  *
	 *          Allow Collision (hand object)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
		stage->allowCollisions(
			object, robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
			true);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Close Hand                      *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("close");
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Lift_Object(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Lift '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *                 Attach Object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject(object, hand_frame);
		attach_object_stage = stage.get();
		c->insert(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *       Allow collision (object support)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
		stage->allowCollisions({ object }, support_surfaces, true);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Lift Object                     *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);
		//TODO: Make this parameterizable instead of hard-coded for 0.1 and 0.15
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "lift_object");

		// Set upward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = lift_direction_reference_frame;
		vec.vector.x = lift_direction[0];
		vec.vector.y = lift_direction[1];
		vec.vector.z = lift_direction[2];
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *        Forbid collision (object support)         *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
		stage->allowCollisions({ object }, support_surfaces, false);
		lift_object_stage = stage.get();
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_and_Lift(const std::string& object, const std::string& arm_group_name, bool this_is_start){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";
	
	auto c = std::make_unique<SerialContainer>("Pick '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("'before pick' state");
		auto _dummy_state = std::make_unique<stages::Dummy>("'before pick' state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);
	
		// Verify that object is not attached
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});

		current_state_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	// Pick
	c->insert(std::move(Modules_Planner::Pick_Object(object)));


	// Lift
	c->insert(std::move(Modules_Planner::Lift_Object(object)));

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place_Object(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Approach target pose for '" + object + "' with " + group);

	/******************************************************
	 *                                                    *
	 *          Lower Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
		stage->properties().set("marker_ns", "lower_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(.03, .13);
		//TODO: Make this parameterizable instead of hard-coded for 0.03 and 0.13

		// Set downward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = approach_place_direction[0];
		vec.vector.y = approach_place_direction[1];
		vec.vector.z = approach_place_direction[2];
		stage->setDirection(vec);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Generate Place Pose                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
		stage->properties().set("marker_ns", "place_pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setObject(object);
		stage->setSubframe(object_subframe_to_place);  // The subframe has to be named according to 'object_name/subframe_name' convention

		stage->setPose(target_pose);
		stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}
	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place_Object(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Approach target pose for '" + object + "' with " + group);

	/******************************************************
	 *                                                    *
	 *          Lower Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
		stage->properties().set("marker_ns", "lower_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(.03, .13);
		//TODO: Make this parameterizable instead of hard-coded for 0.03 and 0.13

		// Set downward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = approach_place_direction[0];
		vec.vector.y = approach_place_direction[1];
		vec.vector.z = approach_place_direction[2];
		stage->setDirection(vec);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Generate Place Pose                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
		stage->properties().set("marker_ns", "place_pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setObject(object);
		stage->setSubframe(object_subframe_to_place);  // The subframe has to be named according to 'object_name/subframe_name' convention

		stage->setPose(target_pose);
		stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}
	return c;
}

////////////////

std::unique_ptr<SerialContainer> Modules_Planner::Place_Object_With_Correction(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Approach target pose for '" + object + "' with " + group + " and correct its pose by pushing.");

	/******************************************************
	 *                                                    *
	 *          Lower Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
		stage->properties().set("marker_ns", "lower_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(.03, .13);
		//TODO: Make this parameterizable instead of hard-coded for 0.03 and 0.13

		// Set downward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = approach_place_direction[0];
		vec.vector.y = approach_place_direction[1];
		vec.vector.z = approach_place_direction[2];
		stage->setDirection(vec);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Generate Drop Pose                        *
	 *                                                    *
	 *****************************************************/
	// This is the pose *above* the actual place pose and slightly shifted towards the push prep pose
	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("generate pre-place pose");
		stage->properties().set("marker_ns", "place_pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setObject(object);
		stage->setSubframe(object_subframe_to_place);  // The subframe has to be named according to 'object_name/subframe_name' convention

		geometry_msgs::PoseStamped drop_pose = target_pose;
		// TODO: Put drop_pose at 5 mm offset from target in approach_place_direction (but the header might not be the same, UGH)
		stage->setPose(drop_pose);
		stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}
	
	/******************************************************
	 *                                                    *
	 *          Open gripper                              *
	 *                                                    *
	 *****************************************************/

	/******************************************************
	 *                                                    *
	 *          Move to pushing prep pose                 *
	 *                                                    *
	 *****************************************************/
	// This is the pose next to place pose

	/******************************************************
	 *                                                    *
	 *          Close gripper                             *
	 *                                                    *
	 *****************************************************/

	/******************************************************
	 *                                                    *
	 *          Push object into place                    *
	 *                                                    *
	 *****************************************************/
	// This moves the gripper such that the object is at the right position (in that dimension)
	// There is currently no way to obtain this in a reasonable way from the planning scene, so this will have to be hardcoded/handed in

	/******************************************************
	 *                                                    *
	 *          Open gripper                              *
	 *                                                    *
	 *****************************************************/

	/******************************************************
	 *                                                    *
	 *          Move to target pose                       *
	 *                                                    *
	 *****************************************************/

	/******************************************************
	 *                                                    *
	 *          Close gripper                             *
	 *                                                    *
	 *****************************************************/

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place_Object_With_Correction(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Approach target pose for '" + object + "' with " + group);
	// TODO(felixvd)

	// /******************************************************
	//  *                                                    *
	//  *          Lower Object                              *
	//  *                                                    *
	//  *****************************************************/
	// {
	// 	auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
	// 	stage->properties().set("marker_ns", "lower_object");
	// 	stage->properties().set("link", hand_frame);
	// 	stage->properties().set("group", group);
	// 	stage->setMinMaxDistance(.03, .13);
	// 	//TODO: Make this parameterizable instead of hard-coded for 0.03 and 0.13

	// 	// Set downward direction
	// 	geometry_msgs::Vector3Stamped vec;
	// 	vec.header.frame_id = approach_place_direction_reference_frame;
	// 	vec.vector.x = approach_place_direction[0];
	// 	vec.vector.y = approach_place_direction[1];
	// 	vec.vector.z = approach_place_direction[2];
	// 	stage->setDirection(vec);
	// 	current_state_stage = stage.get();
	// 	c->insert(std::move(stage));
	// }

	// /******************************************************
	//  *                                                    *
	//  *          Generate Place Pose                       *
	//  *                                                    *
	//  *****************************************************/
	// {
	// 	auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
	// 	stage->properties().set("marker_ns", "place_pose");
	// 	stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
	// 	stage->setObject(object);
	// 	stage->setSubframe(object_subframe_to_place);  // The subframe has to be named according to 'object_name/subframe_name' convention

	// 	stage->setPose(target_pose);
	// 	stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

	// 	// Compute IK
	// 	auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
	// 	wrapper->setMaxIKSolutions(2);
	// 	wrapper->setIKFrame(hand_frame);
	// 	wrapper->properties().set("group", group);
	// 	wrapper->setEndEffector(group + "_tip");
	// 	wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
	// 	c->insert(std::move(wrapper));
	// }
	return c;
}

////////////////



std::unique_ptr<SerialContainer> Modules_Planner::Release_Object_and_Retreat(const std::string& object, const std::string& pose_to_retreat_to){
	auto c = std::make_unique<SerialContainer>("Release '" + object + "' and retreat, " + group);

	// /******************************************************
	//  *                                                    *
	//  *                  Open Hand                         *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *         Forbid collision (hand, object)            *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
		stage->allowCollisions(
			object,
			robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), false);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                 Detach Object                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, hand_frame);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
		stage->setMinMaxDistance(.05, .1);
		//TODO: Make this parameterizable instead of hard-coded for 0.1 and 0.15
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		if (retreat_direction_reference_frame == ""){
			vec.header.frame_id = hand_frame;
			vec.vector.x = retreat_direction[0];
			vec.vector.y = retreat_direction[1];
			vec.vector.z = retreat_direction[2];
		} else {
			vec.header.frame_id = retreat_direction_reference_frame;
			vec.vector.x = retreat_direction[0];
			vec.vector.y = retreat_direction[1];
			vec.vector.z = retreat_direction[2];
		}
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	if(pose_to_retreat_to != "") {
		{
			auto stage = std::make_unique<stages::MoveTo>("move to pose '" + pose_to_retreat_to + "'", sampling_planner);
			stage->properties().set("group", group);
			stage->setGoal(pose_to_retreat_to);
			c->insert(std::move(stage));
		}
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Release_and_Retreat(const std::string& object, const std::string& arm_group_name, const std::string& pose_to_retreat_to, bool this_is_start){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Release '" + object + "' and retreat, " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		auto _dummy_state = std::make_unique<stages::Dummy>("current state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is attached to the given planning group
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached so cannot be released";
				return false;
			}
		});

		c->insert(std::move(applicability_filter));
	}

	// Release and retreat
	c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object, pose_to_retreat_to)));

	return c;

}

std::unique_ptr<SerialContainer> Modules_Planner::Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object, const std::string& object_subframe_to_place, bool this_is_start){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Place '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		auto _dummy_state = std::make_unique<stages::Dummy>("current state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is attached
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached and cannot be placed";
				return false;
			}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

		if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Place_With_Correction(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object, const std::string& object_subframe_to_place, bool this_is_start){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Place (with correction) '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		auto _dummy_state = std::make_unique<stages::Dummy>("current state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is attached
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached and cannot be placed";
				return false;
			}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object_With_Correction(object, target_pose, object_subframe_to_place)));

		if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Fasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place, bool this_is_start, const std::string& container_name){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;

	auto c = std::make_unique<SerialContainer>(container_name + " with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>(container_name + " start state");
		auto _dummy_state = std::make_unique<stages::Dummy>(container_name + " start state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is not attached
		if (this_is_start){
			applicability_filter =
			std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter =
			std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached so fastening cannot be planned";
				return false;
				}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	c->insert(std::move(std::make_unique<stages::Dummy>(container_name + " (dummy)")));

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("Retreat", cartesian_planner);
		stage->setMinMaxDistance(.05, .1);
		//TODO: Make this parameterizable instead of hard-coded for 0.05 and 0.1
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = -1*approach_place_direction[0];
		vec.vector.y = -1*approach_place_direction[1];
		vec.vector.z = -1*approach_place_direction[2];
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Fasten(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& arm_group_name, const std::string& object_subframe_to_place, bool this_is_start, const std::string& container_name){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	std::string tem = hand_frame;

	auto c = std::make_unique<SerialContainer>(container_name + " with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>(container_name + " start state");
		auto _dummy_state = std::make_unique<stages::Dummy>(container_name + " start state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is not attached
		if (this_is_start){
			applicability_filter =
			std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter =
			std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached so fastening cannot be planned";
				return false;
				}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	c->insert(std::move(std::make_unique<stages::Dummy>(container_name + " (dummy)")));

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("Retreat", cartesian_planner);
		stage->setMinMaxDistance(.05, .1);
		//TODO: Make this parameterizable instead of hard-coded for 0.05 and 0.1
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = approach_place_direction_reference_frame;
		vec.vector.x = -1*approach_place_direction[0];
		vec.vector.y = -1*approach_place_direction[1];
		vec.vector.z = -1*approach_place_direction[2];
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Place(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& arm_group_name, bool release_object, const std::string& object_subframe_to_place){
	group = arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	auto c = std::make_unique<SerialContainer>("Pick-Place " + group);

	// Pick and Lift
	c->insert(std::move(Modules_Planner::Pick_and_Lift(object, group)));

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<SerialContainer> Modules_Planner::Pick_Place_with_Regrasp(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& pick_arm_group_name, const std::string& place_arm_group_name, bool release_object, const std::string& object_subframe_to_place){
	auto c = std::make_unique<SerialContainer>("Pick-Place with Regrasp");

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	// Pick and Lift
	c->insert(std::move(Modules_Planner::Pick_and_Lift(object, group)));


	geometry_msgs::PoseStamped handover_pose; //TODO: Make this a data member of the class and load it at init from the param server
	handover_pose.header.frame_id = "workspace_center";
	handover_pose.pose.orientation.w = 1.0;
	handover_pose.pose.position.z = 0.25;


	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to handover", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateHandoverPose>("handover pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setPose(handover_pose);
		stage->setObject(object);
		stage->setMonitoredStage(lift_object_stage);

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("handover pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(group + "_tip");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}

	group = place_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	// Pick
	c->insert(std::move(Modules_Planner::Pick_Object(object)));

	group = pick_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));

	{
	auto stage = std::make_unique<stages::MoveTo>("move_out_of_the_way", sampling_planner);
		stage->setGroup(group);
		stage->setGoal("tool_pick_ready");
		c->insert(std::move(stage));
	}

	group = place_arm_group_name;
	hand_group_name = group + "_robotiq_85";
	hand_frame = hand_group_name + "_tip_link";

	/****************************************************
	 *                                                  *
	 *                 Attach Object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object to " + group);
		stage->attachObject(object, hand_frame);
		attach_object_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	c->insert(std::move(Modules_Planner::Place_Object(object, target_pose, object_subframe_to_place)));

	if (release_object){
		// Release and retreat
		c->insert(std::move(Modules_Planner::Release_Object_and_Retreat(object)));
	}

	return c;
}

std::unique_ptr<Alternatives> Modules_Planner::Pick_and_Lift_Alternatives(const std::string& object, bool this_is_start){
	auto parallel = std::make_unique<Alternatives>("Pick '" + object + "'");

	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Pick_and_Lift(object, arm_group_name, this_is_start)));
	}
	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Release_and_Retreat_Alternatives(const std::string& object, const std::string& pose_to_retreat_to, bool this_is_start){
	auto parallel = std::make_unique<Alternatives>("Release '" + object + "' and retreat");

	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Release_and_Retreat(object, arm_group_name, pose_to_retreat_to, this_is_start)));
	}
	return parallel;

}

std::unique_ptr<Alternatives> Modules_Planner::Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place, bool this_is_start){
	auto parallel = std::make_unique<Alternatives>("Place '" + object + "'");

	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Place(object, target_pose, arm_group_name, release_object, object_subframe_to_place, this_is_start)));
	}
	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Place_With_Correction_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place, bool this_is_start){
	auto parallel = std::make_unique<Alternatives>("Place with correction '" + object + "'");

	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Place_Object_With_Correction(object, target_pose, arm_group_name, release_object, object_subframe_to_place, this_is_start)));
	}
	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Fasten_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place, bool this_is_start, const std::string& container_name){
	auto parallel = std::make_unique<Alternatives>(container_name);


	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Fasten(object, target_pose, arm_group_name, object_subframe_to_place, this_is_start, container_name)));
	}
	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Fasten_Alternatives(const std::string& object, const std::vector<geometry_msgs::PoseStamped>& target_pose, const std::string& object_subframe_to_place, bool this_is_start, const std::string& container_name){
	auto parallel = std::make_unique<Alternatives>(container_name);

	for (std::string arm_group_name : arm_group_names){
		parallel->insert(std::move(Modules_Planner::Fasten(object, target_pose, arm_group_name, object_subframe_to_place, this_is_start, container_name)));
	}
	return parallel;
}

std::unique_ptr<Alternatives> Modules_Planner::Pick_Place_Alternatives(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place){
	auto parallel = std::make_unique<Alternatives>("Pick-Place");

	parallel->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "a_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, "b_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "a_bot", "b_bot", release_object, object_subframe_to_place)));
	parallel->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, "b_bot", "a_bot", release_object, object_subframe_to_place)));

	return parallel;
}

std::unique_ptr<Fallbacks> Modules_Planner::Pick_Place_Fallback(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place, bool force_robot_order){
	auto parallel = std::make_unique<Fallbacks>("Pick-Place '" + object + "'");

	auto single_robot_task_solutions = std::make_unique<Alternatives>("Pick-Place with single robot");

	for (std::string arm_group_name : arm_group_names){
		single_robot_task_solutions->insert(std::move(Modules_Planner::Pick_Place(object, target_pose, arm_group_name, release_object, object_subframe_to_place)));
	}
	
	auto regrasp_task_solutions = std::make_unique<Alternatives>("Pick-Place with regrasp");

	if (arm_group_names.size() > 1){

		if (force_robot_order && arm_group_names.size() == 2){
			regrasp_task_solutions->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, arm_group_names[0], arm_group_names[1], release_object, object_subframe_to_place)));
		} else {
			for (std::string arm_group_name_1 : arm_group_names){
				for (std::string arm_group_name_2 : arm_group_names){
					if (arm_group_name_1 != arm_group_name_2){
						regrasp_task_solutions->insert(std::move(Modules_Planner::Pick_Place_with_Regrasp(object, target_pose, arm_group_name_1, arm_group_name_2, release_object, object_subframe_to_place)));
					}
				}
			}
		}

		parallel->insert(std::move(single_robot_task_solutions));
		parallel->insert(std::move(regrasp_task_solutions));
	} else {
		parallel->insert(std::move(single_robot_task_solutions));
	}

	return parallel;
}

void Modules_Planner::createPickPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place, bool force_robot_order) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	// t.add(Modules_Planner::Pick_Place_Alternatives(object, target_pose, false, object_subframe_to_place));  // In case we want it as an alternative and not fallback
	t.add(Modules_Planner::Pick_Place_Fallback(object, target_pose, release_object, object_subframe_to_place, force_robot_order));
}

void Modules_Planner::createPick(const std::string& object) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Pick_and_Lift_Alternatives(object));
}

void Modules_Planner::createPlace(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Place_Alternatives(object, target_pose, release_object, object_subframe_to_place));
}

void Modules_Planner::createPlaceWithCorrection(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Place_With_Correction_Alternatives(object, target_pose, release_object, object_subframe_to_place));
}

void Modules_Planner::createReleaseRetreat(const std::string& object, const std::string& pose_to_retreat_to){
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Release_and_Retreat_Alternatives(object, pose_to_retreat_to));
}

void Modules_Planner::createFasten(const std::string& object, const geometry_msgs::PoseStamped& target_pose, const std::string& object_subframe_to_place) {
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Task");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	t.add(Modules_Planner::Fasten_Alternatives(object, target_pose, object_subframe_to_place));
}

void Modules_Planner::createWRSSubtaskMotorPlate(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool release_object, const std::string& object_subframe_to_place){
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());
	moveit::task_constructor::Task& t = *task_;
	t.stages()->setName("Attach motor L-plate to base plate");
	t.loadRobotModel();

	robot_model_ = t.getRobotModel();

	// pick-place object
	t.add(Modules_Planner::Pick_Place_Fallback(object, target_pose, release_object, object_subframe_to_place));

	// Try to position the object correctly
	{
		auto stage = std::make_unique<stages::Dummy>("'before adjusting plate position'");
		t.add(std::move(stage));
	}
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup("a_bot_robotiq_85");
		stage->setGoal("open");
		t.add(std::move(stage));
	}
	{  // Close Hand
		auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
		stage->setGroup("a_bot_robotiq_85");
		stage->setGoal("close");
		t.add(std::move(stage));
	}
	{
		auto stage = std::make_unique<stages::Dummy>("'after adjusting plate position'");
		t.add(std::move(stage));
	}

	grasp_parameter_location = "tools";
	std::string screw_type = "m4";

	std::string tool = "screw_tool_" + screw_type;
	lift_direction_reference_frame = tool + "_pickup_link";
	lift_direction[0] = -1.0;
	lift_direction[1] = 0.0;
	lift_direction[2] = 0.0;

	// move to screw_tool_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_tool_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("tool_pick_ready");
		t.add(std::move(stage));
	}

	// pick screw tool
	t.add(Modules_Planner::Pick_and_Lift(tool, "b_bot", false));
	

	// move back
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_tool_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("tool_pick_ready");
		t.add(std::move(stage));
	}

	geometry_msgs::PoseStamped screw_pickup_pose;
	screw_pickup_pose.header.frame_id = screw_type + "_feeder_outlet_link";
	screw_pickup_pose.pose.position.x = -0.01;
	screw_pickup_pose.pose.position.y = 0;
	screw_pickup_pose.pose.position.z = 0;
	screw_pickup_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(5*M_PI/3, 0, 0);

	std::string screw_tool_tip_frame = tool + "/" + tool + "_tip";

	// move to screw_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_screw_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	// pick up screw
	t.add(Modules_Planner::Fasten(tool, screw_pickup_pose, "b_bot", screw_tool_tip_frame, false, "Pick up screw"));

	// move back
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_screw_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	approach_place_direction_reference_frame = "panel_bearing/bottom_screw_hole_1";
	approach_place_direction[0] = 1.0;
	approach_place_direction[1] = 0.0;
	approach_place_direction[2] = 0.0;

	geometry_msgs::PoseStamped screwing_pose;
	screwing_pose.header.frame_id = "panel_bearing/bottom_screw_hole_1";
	screwing_pose.pose.position.x = -0.01;
	screwing_pose.pose.position.y = 0;
	screwing_pose.pose.position.z = 0;
	screwing_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

	// move to screw_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("fasten_screw_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	t.add(Modules_Planner::Fasten(tool, screwing_pose, "b_bot", screw_tool_tip_frame, false, "Fasten screw"));

	// move_back
	{
	auto stage = std::make_unique<stages::MoveTo>("fasten_screw_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	retreat_direction_reference_frame = "world";
	retreat_direction[0] = 0;
	retreat_direction[1] = 0;
	retreat_direction[2] = 1;

	// release object and retreat
	t.add(Modules_Planner::Release_and_Retreat(object, "a_bot", "home", false));

	retreat_direction_reference_frame = "";
	retreat_direction[0] = -1;
	retreat_direction[1] = 0;
	retreat_direction[2] = 0;

	approach_place_direction_reference_frame = "world";
	approach_place_direction[0] = 0.0;
	approach_place_direction[1] = 0.0;
	approach_place_direction[2] = -1.0;

	// move to screw_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_screw_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	// pick up screw
	t.add(Modules_Planner::Fasten(tool, screw_pickup_pose, "b_bot", screw_tool_tip_frame, false, "Pick up screw"));

	// move back
	{
	auto stage = std::make_unique<stages::MoveTo>("pick_screw_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	approach_place_direction_reference_frame = "panel_bearing/bottom_screw_hole_1";
	approach_place_direction[0] = 1.0;
	approach_place_direction[1] = 0.0;
	approach_place_direction[2] = 0.0;

	screwing_pose.header.frame_id = "panel_bearing/bottom_screw_hole_2";

	// move to screw_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("fasten_screw_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	t.add(Modules_Planner::Fasten(tool, screwing_pose, "b_bot", screw_tool_tip_frame, false, "Fasten screw"));

	// move_back
	{
	auto stage = std::make_unique<stages::MoveTo>("fasten_screw_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("feeder_pick_ready");
		t.add(std::move(stage));
	}

	geometry_msgs::PoseStamped screw_tool_place;
	screw_tool_place.header.frame_id = "screw_tool_" + screw_type + "_link";
	screw_tool_place.pose.position.x = 0;
	screw_tool_place.pose.position.y = -0.009;
	screw_tool_place.pose.position.z = 0.0275;
	screw_tool_place.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

	approach_place_direction_reference_frame = "screw_tool_" + screw_type + "_pickup_link";
	approach_place_direction[0] = 1.0;
	approach_place_direction[1] = 0.0;
	approach_place_direction[2] = 0.0;

	// move to screw_tool_pickup pose
	{
	auto stage = std::make_unique<stages::MoveTo>("place_tool_start", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("tool_pick_ready");
		t.add(std::move(stage));
	}

	// place screw tool
	t.add(Modules_Planner::Place(tool, screw_tool_place, "b_bot", true, "", false));

	// move_back
	{
	auto stage = std::make_unique<stages::MoveTo>("place_tool_end", sampling_planner);
		stage->setGroup("b_bot");
		stage->setGoal("tool_pick_ready");
		t.add(std::move(stage));
	}
}

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_modules");

	mtc_modules::Modules_Planner Modules;
	Modules.init();

	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
