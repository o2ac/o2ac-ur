#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/storage.h>
#include <rviz_marker_tools/marker_creation.h>
#include <stages/load_grasp_pose.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <eigen_conversions/eigen_msg.h>

namespace moveit {
namespace task_constructor {
namespace stages {

LoadGraspPose::LoadGraspPose(const std::string &name) : GeneratePose(name) {
  auto &p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object");
  p.declare<std::string>("assembly");

  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");
}

void LoadGraspPose::init(const core::RobotModelConstPtr &robot_model) {
  InitStageException errors;
  try {
    GeneratePose::init(robot_model);
  } catch (InitStageException &e) {
    errors.append(e);
  }

  const auto &props = properties();

  // check availability of object
  props.get<std::string>("object");
  // check availability of eef
  const std::string &eef = props.get<std::string>("eef");
  if (!robot_model->hasEndEffector(eef))
    errors.push_back(*this, "unknown end effector: " + eef);
  else {
    // check availability of eef pose
    const moveit::core::JointModelGroup *jmg = robot_model->getEndEffector(eef);
    const std::string &name = props.get<std::string>("pregrasp");
    std::map<std::string, double> m;
    if (!jmg->getVariableDefaultPositions(name, m))
      errors.push_back(*this, "unknown end effector pose: " + name);
  }

  if (errors)
    throw errors;
}

void LoadGraspPose::onNewSolution(const SolutionBase &s) {
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

  const auto &props = properties();
  const std::string &object = props.get<std::string>("object");
  if (!scene->knowsFrameTransform(object)) {
    const std::string msg = "object '" + object + "' not in scene";
    if (storeFailures()) {
      InterfaceState state(scene);
      SubTrajectory solution;
      solution.markAsFailure();
      solution.setComment(msg);
      spawn(std::move(state), std::move(solution));
    } else
      ROS_WARN_STREAM_NAMED("LoadGraspPose", msg);
    return;
  }

  upstream_solutions_.push(&s);
}

void LoadGraspPose::compute() {
  ros::NodeHandle nh;
  if (upstream_solutions_.empty())
    return;
  planning_scene::PlanningScenePtr scene =
      upstream_solutions_.pop()->end()->scene()->diff();

  std::string grasp_name;
  XmlRpc::XmlRpcValue grasps;

  // set end effector pose
  const auto &props = properties();
  const std::string &eef = props.get<std::string>("eef");
  const moveit::core::JointModelGroup *jmg =
      scene->getRobotModel()->getEndEffector(eef);

  robot_state::RobotState &robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));

  geometry_msgs::PoseStamped target_pose_msg;
  std::string object_name = props.get<std::string>("object");
  std::string assembly_name = props.get<std::string>("assembly");
  target_pose_msg.header.frame_id = object_name;

  std::string param_name = "/" + assembly_name + "/" + object_name;
  const std::string msg = "The requested param '" + param_name +
                          "' cannot be found on the ROS parameter server!";
  if (nh.getParam(param_name, grasps)) { // CHECK IF PARAM EXISTS
    for (int32_t i = 0; i < grasps.size(); ++i) {
      grasp_name = "grasp_" + std::to_string(i + 1);
      target_pose_msg.pose.position.x = grasps[grasp_name]["position"][0];
      target_pose_msg.pose.position.y = grasps[grasp_name]["position"][1];
      target_pose_msg.pose.position.z = grasps[grasp_name]["position"][2];
      target_pose_msg.pose.orientation.x = grasps[grasp_name]["orientation"][0];
      target_pose_msg.pose.orientation.y = grasps[grasp_name]["orientation"][1];
      target_pose_msg.pose.orientation.z = grasps[grasp_name]["orientation"][2];
      target_pose_msg.pose.orientation.w = grasps[grasp_name]["orientation"][3];

      InterfaceState state(scene);
      state.properties().set("target_pose", target_pose_msg);
      props.exposeTo(state.properties(), {"pregrasp", "grasp"});

      SubTrajectory trajectory;
      trajectory.setCost(0.0);
      trajectory.setComment(grasp_name);

      // add frame at target pose
      rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1,
                                     "grasp frame");

      spawn(std::move(state), std::move(trajectory));
    }
  } else if (storeFailures()) {
    InterfaceState state(scene);
    SubTrajectory trajectory;
    trajectory.markAsFailure();
    trajectory.setComment(msg);
    spawn(std::move(state), std::move(trajectory));
  } else {
    ROS_WARN_STREAM_NAMED("LoadGraspPose", msg);
  }
}
} // namespace stages
} // namespace task_constructor
} // namespace moveit
