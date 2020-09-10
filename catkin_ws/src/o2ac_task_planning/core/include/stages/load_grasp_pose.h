#pragma once

#include <stages/generate_pose.h>

namespace moveit {
namespace task_constructor {
namespace stages {

class LoadGraspPose : public GeneratePose
{
public:
	LoadGraspPose(const std::string& name = "load grasp pose");

	void init(const core::RobotModelConstPtr& robot_model) override;
	void compute() override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }
    void setAssembly(const std::string& assembly) { setProperty("assembly", assembly); }

	void setPreGraspPose(const std::string& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setPreGraspPose(const moveit_msgs::RobotState& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setGraspPose(const std::string& grasp) { properties().set("grasp", grasp); }
	void setGraspPose(const moveit_msgs::RobotState& grasp) { properties().set("grasp", grasp); }

protected:
	void onNewSolution(const SolutionBase& s) override;
};
}
}
}
