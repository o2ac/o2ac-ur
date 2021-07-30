#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include "o2ac_pose_distribution_updater/test_tools.hpp"
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv) {
  // load stl file

  char stl_file_path[1000];
  scanf("%999s", stl_file_path);
  std::shared_ptr<moveit_msgs::CollisionObject> object(
      new moveit_msgs::CollisionObject);
  load_CollisionObject_from_file(object, std::string(stl_file_path));
  std::vector<Eigen::Vector3d> vertices;
  std::vector<boost::array<int, 3>> triangles;
  CollisionObject_to_eigen_vectors(*object, vertices, triangles);

  // visualize the result of action plan

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nd;

  // create the client
  actionlib::SimpleActionClient<o2ac_msgs::updateDistributionAction> client(
      "update_distribution", true);
  client.waitForServer();

  // create the visualizer client
  ros::ServiceClient visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");

  // create the visualization marker publisher
  ros::Publisher marker_publisher =
      nd.advertise<visualization_msgs::MarkerArray>("test_marker_array", 1);

  Eigen::Isometry3d initial_mean;
  CovarianceMatrix initial_covariance;
  scan_pose(initial_mean);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      scanf("%lf", &initial_covariance(i, j));
    }
  }
  double support_surface;
  scanf("%lf", &support_surface);

  PoseEstimator estimator;
  estimator.load_config_file(
      "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/launch/"
      "estimator_config.yaml");

  int number_of_actions;
  scanf("%d", &number_of_actions);

  std::vector<UpdateAction> actions(number_of_actions);
  for (int t = 0; t < number_of_actions; t++) {
    int type;
    scanf("%d", &type);
    actions[t].type = (action_type)type;
    scan_pose(actions[t].gripper_pose);
  }

  auto current_time = ros::Time::now();

  Eigen::Isometry3d mean = initial_mean;
  CovarianceMatrix covariance = initial_covariance;

  geometry_msgs::PoseWithCovarianceStamped current_pose;

  double lifetime = 0.0;

  tf::poseEigenToMsg(mean, current_pose.pose.pose);
  current_pose.pose.covariance = matrix_6x6_to_array_36(covariance);
  current_pose.header.frame_id = "world";
  current_pose.header.stamp = current_time;
  send_pose_belief(visualizer_client, *object, 1, lifetime, current_pose);

  std::cout << mean.matrix() << std::endl;
  std::cout << covariance << std::endl;

  for (int t = 0; t < number_of_actions; t++) {
    std::string gripper_frame_id = "gripper-" + std::to_string(t);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(actions[t].gripper_pose, pose);
    pose.position.x += 0.5 * (t + 1);
    broadcast_gripper_pose(gripper_frame_id, current_time, pose);

    UpdateAction &action = actions[t];
    std::cout << action.type << std::endl;
    std::cout << action.gripper_pose.matrix() << std::endl;

    o2ac_msgs::updateDistributionGoal goal;
    goal.observation_type = (action.type == touch_action_type
                                 ? goal.TOUCH_OBSERVATION
                                 : action.type == look_action_type
                                       ? goal.LOOK_OBSERVATION
                                       : action.type == place_action_type
                                             ? goal.PLACE_OBSERVATION
                                             : action.type == grasp_action_type
                                                   ? goal.GRASP_OBSERVATION
                                                   : goal.PUSH_OBSERVATION);
    tf::poseEigenToMsg(action.gripper_pose, goal.gripper_pose.pose);
    if (action.type == touch_action_type) {
      goal.touch_observation.touched_object_id = 0;
    } else if (action.type == look_action_type) {
      cv::Mat mean_image, inv_image, bgr_image;
      boost::array<unsigned int, 4> ROI{0, 1080, 0, 1920};
      estimator.generate_image(mean_image, vertices, triangles,
                               action.gripper_pose * mean, ROI);
      inv_image = cv::Mat::ones(1080, 1920, CV_8UC1) - mean_image;
      cv::imwrite(std::to_string(t) + ".jpg", 255 * inv_image);
      cv::cvtColor(255 * inv_image, bgr_image, cv::COLOR_GRAY2BGR);
      goal.look_observation.looked_image =
          *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image)
                .toImageMsg());
      goal.look_observation.ROI = ROI;
    } else if (action.type == place_action_type) {
      goal.place_observation.support_surface = support_surface;
    }
    goal.distribution_type = 1;
    if (action.type != grasp_action_type && action.type != push_action_type) {
      tf::poseEigenToMsg(mean, goal.distribution.pose.pose);
      goal.distribution.pose.covariance = matrix_6x6_to_array_36(covariance);
    } else {
      tf::poseEigenToMsg(action.gripper_pose.inverse() * mean,
                         goal.distribution.pose.pose);
      goal.distribution.pose.covariance = matrix_6x6_to_array_36(
          transform_covariance(action.gripper_pose.inverse(), covariance));
    }
    goal.distribution.header.frame_id = gripper_frame_id;
    goal.distribution.header.stamp = current_time;

    goal.gripped_object = *object;

    o2ac_msgs::updateDistributionResultConstPtr result;

    while (ros::ok()) {
      // Send a call
      client.sendGoal(goal);
      client.waitForResult();

      result = client.getResult();

      if (result->success) {
        break;
      }
    }

    send_pose_belief(visualizer_client, *object, 1, lifetime,
                     result->distribution);

    static int marker_id = 0;
    double text_size = 0.10;
    std_msgs::ColorRGBA text_color;
    text_color.r = 1.0;
    text_color.g = 1.0;
    text_color.b = 1.0;
    text_color.a = 1.0;
    if (action.type == touch_action_type || action.type == place_action_type) {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = current_time;
      marker.ns = "action_marker";
      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::CUBE;

      marker.action = visualization_msgs::Marker::ADD;

      marker.pose = to_Pose(
          pose.position.x, pose.position.y,
          action.type == place_action_type ? support_surface - 0.01 : -0.01,
          1.0, 0.0, 0.0, 0.0);
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.02;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(lifetime);
      marker_array.markers.push_back(marker);

      if (action.type == place_action_type) {
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = text_size;
        marker.color = text_color;
        marker.text = "Place";
        marker.header.frame_id = "world";
        marker.pose = to_Pose((t + 1) * 0.50, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0);
        marker_array.markers.push_back(marker);
      } else {
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = text_size;
        marker.color = text_color;
        marker.text = "Touch";
        marker.header.frame_id = "world";
        marker.pose = to_Pose((t + 1) * 0.50, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0);
        marker_array.markers.push_back(marker);
      }
      marker_publisher.publish(marker_array);
    } else if (action.type == look_action_type) {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = current_time;
      marker.ns = "action_marker";
      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

      marker.action = visualization_msgs::Marker::ADD;

      double width =
          estimator.image_width / estimator.camera_matrix.at<double>(0, 0);
      double center_x = estimator.camera_matrix.at<double>(0, 2) /
                        estimator.camera_matrix.at<double>(0, 0);
      double height =
          estimator.image_height / estimator.camera_matrix.at<double>(1, 1);
      double center_y = estimator.camera_matrix.at<double>(1, 2) /
                        estimator.camera_matrix.at<double>(1, 1);
      geometry_msgs::Point origin, p00, p01, p10, p11, mp00, mp01, mp10, mp11;
      origin.x = origin.y = origin.y = 0.0;
      p00.x = p01.x = -center_x;
      p10.x = p11.x = width - center_x;
      p00.y = p10.y = -center_y;
      p01.y = p11.y = height - center_y;
      p00.z = p01.z = p10.z = p11.z = 1.0;
      mp00.x = mp01.x = center_x;
      mp10.x = mp11.x = -width + center_x;
      mp00.y = mp10.y = center_y;
      mp01.y = mp11.y = -height + center_y;
      mp00.z = mp01.z = mp10.z = mp11.z = -1.0;

      marker.points = std::vector<geometry_msgs::Point>{
          origin, p00,    p01,    origin, p01,    p11,    origin, p11,
          p10,    origin, p10,    p00,    origin, mp00,   mp01,   origin,
          mp01,   mp11,   origin, mp11,   mp10,   origin, mp10,   mp00};

      Eigen::Isometry3d camera_pose = estimator.get_camera_pose();
      camera_pose.translation()(0) += (t + 1) * 0.5;
      tf::poseEigenToMsg(camera_pose, marker.pose);
      marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(lifetime);
      marker_array.markers.push_back(marker);

      marker.id = marker_id++;

      marker.points.clear();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.z = text_size;
      marker.color = text_color;
      marker.text = "Look";
      marker.header.frame_id = "world";
      marker.pose = to_Pose((t + 1) * 0.50, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0);
      marker_array.markers.push_back(marker);

      marker_publisher.publish(marker_array);
    } else if (action.type == grasp_action_type) {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header.frame_id = gripper_frame_id;
      marker.header.stamp = current_time;
      marker.ns = "action_marker";
      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::CUBE;

      marker.action = visualization_msgs::Marker::ADD;

      double gripper_breadth = 0.10;

      marker.scale.x = 0.040;
      marker.scale.y = 0.006;
      marker.scale.z = 0.020;
      marker.pose =
          to_Pose(-marker.scale.x / 2.0, gripper_breadth + marker.scale.y / 2.0,
                  0.0, 1.0, 0.0, 0.0, 0.0);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(lifetime);
      marker_array.markers.push_back(marker);
      marker.pose = to_Pose(-marker.scale.x / 2.0,
                            -gripper_breadth - marker.scale.y / 2.0, 0.0, 1.0,
                            0.0, 0.0, 0.0);
      marker.id = marker_id++;
      marker_array.markers.push_back(marker);

      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::ARROW;

      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = gripper_breadth;
      marker.scale.y = 0.005;
      marker.scale.z = 0.005;
      marker.pose = to_Pose(-0.020, gripper_breadth, 0.0, 1. / sqrt(2.), 0.0,
                            0.0, -1. / sqrt(2.));
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker_array.markers.push_back(marker);

      marker.id = marker_id++;

      marker.pose = to_Pose(-0.020, -gripper_breadth, 0.0, 1. / sqrt(2.), 0.0,
                            0.0, 1. / sqrt(2.));
      marker_array.markers.push_back(marker);

      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      marker.scale.z = text_size;
      marker.color = text_color;
      marker.text = "Grasp";
      marker.header.frame_id = "world";
      marker.pose = to_Pose((t + 1) * 0.50, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0);
      marker_array.markers.push_back(marker);

      marker_publisher.publish(marker_array);
    } else if (action.type == push_action_type) {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.header.frame_id = gripper_frame_id;
      marker.header.stamp = current_time;
      marker.ns = "action_marker";
      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::CUBE;

      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.040;
      marker.scale.y = 0.006;
      marker.scale.z = 0.020;
      marker.pose = to_Pose(-marker.scale.x / 2.0, marker.scale.y / 2.0, 0.0,
                            1.0, 0.0, 0.0, 0.0);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(lifetime);
      marker_array.markers.push_back(marker);
      marker.pose = to_Pose(-marker.scale.x / 2.0, -marker.scale.y / 2.0, 0.0,
                            1.0, 0.0, 0.0, 0.0);
      marker.id = marker_id++;
      marker_array.markers.push_back(marker);
      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::ARROW;

      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.030;
      marker.scale.y = 0.005;
      marker.scale.z = 0.005;
      marker.pose = to_Pose(-0.020, 0.0, -marker.scale.x - 0.01, 1. / sqrt(2.),
                            0.0, -1. / sqrt(2.), 0.0);
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(lifetime);
      marker_array.markers.push_back(marker);

      marker.id = marker_id++;

      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      marker.scale.z = text_size;
      marker.color = text_color;
      marker.text = "Push";
      marker.header.frame_id = "world";
      marker.pose = to_Pose((t + 1) * 0.50, 0.0, 0.20, 1.0, 0.0, 0.0, 0.0);
      marker_array.markers.push_back(marker);

      marker_publisher.publish(marker_array);
    }

    tf::poseMsgToEigen(result->distribution.pose.pose, mean);
    covariance = array_36_to_matrix_6x6(result->distribution.pose.covariance);
    if (action.type == place_action_type || action.type == push_action_type) {
      mean = action.gripper_pose * mean;
      covariance = transform_covariance(action.gripper_pose, covariance);
    }

    std::cout << mean.matrix() << std::endl;
    std::cout << covariance << std::endl;
  }
  return 0;
}
