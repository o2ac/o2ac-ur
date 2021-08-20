/*
The implementation of the test client

This program creates a test client and executes tests for touch, look and place
actions.
 */

#include "o2ac_pose_distribution_updater/test.hpp"

std::string test_directory;

std::shared_ptr<Client> client;

ros::ServiceClient visualizer_client;

ros::Publisher marker_publisher;

TEST(TouchTest, TouchGearMotor1) {
  touch_test(client, test_directory + "/touch_input_gearmotor_1.txt",
             test_directory + "/CAD/gearmotor.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE);
}

TEST(TouchTest, TouchGearMotor1Lie) {
  touch_test(client, test_directory + "/touch_input_gearmotor_1.txt",
             test_directory + "/CAD/gearmotor.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE);
}

TEST(LookTest, LookGearMotor) {
  look_test(client, test_directory + "/look_gripper_tip.csv",
            test_directory + "/look_action_images",
            test_directory + "/CAD/gearmotor.stl",
            std::vector<int>{50, 660, 1000, 1500}, 2, 1, 2,
            o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE);
}

TEST(LookTest, LookGearMotorLie) {
  look_test(client, test_directory + "/look_gripper_tip.csv",
            test_directory + "/look_action_images",
            test_directory + "/CAD/gearmotor.stl",
            std::vector<int>{50, 660, 1000, 1500}, 2, 1, 2,
            o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE);
}

TEST(PlaceTest, PlaceGearMotor) {
  place_test(client, test_directory + "/place_test_gearmotor.txt",
             test_directory + "/CAD/gearmotor.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, false,
             visualizer_client);
}

TEST(PlaceTest, PlaceTestTetrahedron1) {
  place_test(client, test_directory + "/place_test_tetrahedron_1.txt",
             test_directory + "/CAD/test_tetrahedron_1.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, false,
             visualizer_client);
}

TEST(PlaceTest, PlaceTestTetrahedron2) {
  place_test(client, test_directory + "/place_test_tetrahedron_2.txt",
             test_directory + "/CAD/test_tetrahedron_2.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, false,
             visualizer_client);
}

TEST(PlaceTest, PlaceCones1) {
  place_test(client, test_directory + "/place_test_cones_1.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceCones2) {
  place_test(client, test_directory + "/place_test_cones_2.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceCones3) {
  place_test(client, test_directory + "/place_test_cones_3.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceConesLie1) {
  place_test(client, test_directory + "/place_test_cones_Lie_1.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceConesLie2) {
  place_test(client, test_directory + "/place_test_cones_Lie_2.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceConesLie3) {
  place_test(client, test_directory + "/place_test_cones_Lie_3.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client);
}

TEST(PlaceTest, PlaceCones1Convert) {
  place_test(client, test_directory + "/place_test_cones_1.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::RPY_COVARIANCE, false,
             visualizer_client, true);
}

TEST(PlaceTest, PlaceCones1LieConvert) {
  place_test(client, test_directory + "/place_test_cones_Lie_1.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, false,
             visualizer_client, true);
}

TEST(GraspTest, GraspCones) {
  grasp_test(client, test_directory + "/grasp_test_cones_Lie_1.txt",
             test_directory + "/CAD/cones.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client, marker_publisher);
}

TEST(GraspTest, GraspGearmotor) {
  grasp_test(client, test_directory + "/grasp_test_gearmotor_Lie_1.txt",
             test_directory + "/CAD/gearmotor.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client, marker_publisher);
}

TEST(GraspTest, GraspShaft) {
  grasp_test(client, test_directory + "/grasp_test_shaft_Lie_1.txt",
             test_directory + "/CAD/shaft.stl",
             o2ac_msgs::updateDistributionGoal::LIE_COVARIANCE, true,
             visualizer_client, marker_publisher);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_client");
  ros::NodeHandle nd;
  nd.getParam("test_directory", test_directory);

  // create the client
  client = std::shared_ptr<Client>(new Client("update_distribution", true));
  client->waitForServer();

  // create the visualizer client
  visualizer_client =
      nd.serviceClient<o2ac_msgs::visualizePoseBelief>("visualize_pose_belief");

  // create the visualization marker publisher
  marker_publisher =
      nd.advertise<visualization_msgs::MarkerArray>("test_marker_array", 1);

  ros::Duration(1.0).sleep();
  RUN_ALL_TESTS();
  ros::Duration(1.0)
      .sleep(); // If the program ends too early, td data is not sent
  return 0;
}
