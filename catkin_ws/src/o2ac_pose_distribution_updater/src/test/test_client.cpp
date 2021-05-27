/*
The implementation of the test client

This program creates a test client and executes tests for touch, look and place
actions.
 */

#include "o2ac_pose_distribution_updater/test.hpp"

std::string test_directory;

std::shared_ptr<Client> client;

ros::ServiceClient visualizer_client;

TEST(TouchTest, GearMotor1) {
  touch_test(client, test_directory + "/touch_input_gearmotor_1.txt",
             test_directory + "/CAD/gearmotor.stl");
}

TEST(LookTest, GearMotor) {
  look_test(client, test_directory + "/look_gripper_tip.csv",
            test_directory + "/look_action_images",
            test_directory + "/CAD/gearmotor.stl",
            std::vector<int>{50, 660, 1000, 1500}, 2, 1, 2);
}

TEST(PlaceTest, GearMotor) {
  place_test(client, test_directory + "/place_test_gearmotor.txt",
             test_directory + "/CAD/gearmotor.stl", false, visualizer_client);
}

TEST(PlaceTest, TestTetrahedron1) {
  place_test(client, test_directory + "/place_test_tetrahedron_1.txt",
             test_directory + "/CAD/test_tetrahedron_1.stl", false,
             visualizer_client);
}

TEST(PlaceTest, TestTetrahedron2) {
  place_test(client, test_directory + "/place_test_tetrahedron_2.txt",
             test_directory + "/CAD/test_tetrahedron_2.stl", false,
             visualizer_client);
}

TEST(PlaceTest, Cones1) {
  place_test(client, test_directory + "/place_test_cones_1.txt",
             test_directory + "/CAD/cones.stl", true, visualizer_client);
}

TEST(PlaceTest, Cones2) {
  place_test(client, test_directory + "/place_test_cones_2.txt",
             test_directory + "/CAD/cones.stl", true, visualizer_client);
}

TEST(PlaceTest, Cones3) {
  place_test(client, test_directory + "/place_test_cones_3.txt",
             test_directory + "/CAD/cones.stl", true, visualizer_client);
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

  return RUN_ALL_TESTS();
}
