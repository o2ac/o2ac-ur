# Overview
This package implements an action server for updateDistribution action.

When an object is gripped and its pose has ambiguity, there exist three acts to clarify its pose, "touch", "look" and "place".

- "touch" : To move the gripper and let the object to touch some other object.
- "look" : To take an image of the object from a camera
- "place" : To release the object and place it on a support surface

When the action server receives the current pose of object with ambiguity and information about one of these acts,
it calculates the updated pose with ambiguity of the object.


More specifically, the pose of the object is represented by a 6-dimensional vector (x, y, z, roll, pitch, yaw).
The first three values (x, y, z) represents the translation.
The latter three values (roll, pitch, yaw) represents the rotation.
The pose with ambiguity is represented by a pair of a 6-dimensional vector, which means the mean of the pose, and a 6x6 matrix, which means the covariance matrix of the distribution of the pose.

For the details of imformation about acts, see "Details of message" section and "Details of updateDistribution.action" section.

# Usage

To run the action server for updateDistribution action, run the following command:

```
roslaunch o2ac_pose_distribution_updater distribution_updater.launch
```

## Parameters for action server
This section desribes the parameters for action server.

The parameters are written in `launch/distribution_updater.launch` file.

### For Guassian particle filter

The update of distribution for "touch" act and "look" act is calculated by Gaussian particle filter.

- `number_of_particles` : the number of particle
- `noise_variance`: a 6 dimensional vector, which represents the variance of noise in each step

### For touch act

There exists two objects to touch, "ground" and "box".

- `ground_size`: 3 dimensional vector representing the size of the ground object
- `ground_position`: 3 dimensional vector representing the position of the ground object
- `box_size`: 3 dimensional vector representing the size of the box object
- `box_position`: 3 dimensional vector representing the position of the box object
- `distance_threshold`: If the distance between two objects is less than this value, they are regarded as touching each other.

### For look act
- `look_threshold`: If the value of the grayscaled image at a pixel is less than or equal to this value, the object is regarded as containing the pixel.
- `calibration_object_points`: a vector representing the 3 dimensional coorinates of the points which are used for camera calibration. 
This vector is the concatenation of the coordinates of the points, e.g., if the points are point (x0, y0, z0), point (x1, y1, z1) and point (x2, y2, z2), this vector is (x0, y0, z0, x1, y1, z1, x2, y2, z2).
- `calibration_image_points`: a vector representing the 2 dimensional coordinates of calibration points projected by the camera.
This vector is the concatenation of the coordinates of the points, e.g., if the points are point (x0, y0), point (x1, y1) and point (x2, y2), this vector is (x0, y0, x1, y1, x2, y2).

The following four parameters are the intrinsic parameters of the camera.

- `camera_fx` and  `camera_fy`: focal length in terms of pixel
- `camera_cx` and `camera_cy`: the principal point

# Details of messages
This section describe messages to send information about the three acts.

## `TouchObservation.msg`
- `geometry_msgs/PoseStamped gripper_pose`: the pose of the gripper when the object touches some other object
- `uint8 touched_object_id`: the index of touched object. If this value is 0, the touched object is the ground object. If this value is 1, the touched object is the box object. 

## `LookObservation.msg`
- `geometry_msgs/PoseStamped gripper_pose`: the pose of the gripper when the object is looked by the camera
- `sensor_msgs/Image looked_image`: looked image represented as bgr8 image
- `std_msgs/uint32[4] ROI`: an array of length 4 representing the range of interests of the image. The range of interests is a rectangle and this array is [left boundary, right boundary, top boundary, bottom boundary].

## `PlaceObservation.msg`
- `geometry_msgs/PoseStamped gripper_pose`: the pose of the gripper when the object is released
- `float64 support_surface`: the z coordinate of the support surface where the object is placed

# Details of updateDistribution.action
This section describe updateDistribution action, an action to send current poses and information about acts and receive new poses.
## Goal
- `uint8 observation_type`: the type of the act. If this value is `TOUCH_OBSERVATION` (constant, equal to 0), the act is "touch". If it is `LOOK_OBSERVATION` (constant, equal to 1), the act is "look". If it is `PLACE_OBSERVATION` (constant, equal to 2), the act is "place".
- `TouchObservation touch_observation`: When the type of act is "touch", this represents information about the act.
- `LookObservation look_observation`: When the type of act is "look", this represents information about the act.
- `PlaceObservation place_observation`: When the type of act is "place", this represents information about the act.
- `moveit_msgs/CollisionObject gripped_object`: a CollisionObject representing the gripped object
- `geometry_msgs/PoseWithCovarianceStamped distribution`: the current distribution of the pose of the object

## Result
- `bool success`: If update is calculated successfully, this value is `true`. Otherwise, it is `false`.
- `geometry_msgs/PoseWithCovarianceStamped distribution`: If update is calculated successfully, this stores the updated distribution of the pose of the object
- `std_msgs/String error_message`: If update is not calculated successfully, this string represents the error message.

## Feedback
Nothing

# File Structure
```
o2ac_pose_distribution_updater           # package direcotory
├── CMakeLists.txt
├── include                              # directory containing header files
│   └── o2ac_pose_distribution_updater   # header files for this package
│       ├── conversions.hpp              # conversion functions, implemented in this header
│       ├── estimator.hpp                # class calculating distributions
│       ├── place_action_helpers.hpp     # functions for calculations associated to place action
│       ├── read_stl.hpp                 # function to read stl files
│       ├── ros_converted_estimator.hpp  # class calculating distributions, wrapped for ros message input
│       ├── ros_converters.hpp           # conversion functions associated with ros message
│       └── test.hpp                     # procedures for unit test
├── launch                               # direcotory containing launch files
│   └── distribution_updater.launch      # for launching action server for updateDistribution action
├── package.xml                          # package discription
├── README.md                            # this README
├── src                                  # direcotory containing source files
│   ├── action_server.cpp                # implementation of the action server
│   ├── estimator.cpp                    # implementation of estimator.hpp
│   ├── place_action_helpers.cpp         # implementation of place_action_helpers.hpp
│   ├── read_stl.cpp                     # implementation of read_stl.hpp
│   ├── ros_converted_estimator.cpp      # implementation of ros_converted_estimator.hpp
│   ├── ros_converters.cpp               # implementation of ros_converters.hpp
│   ├── test                             # source associated to unit test
│   │   ├── generate_place_test_case.cpp # program to generate test case for touch action such as test/touch_input_gearmotor_1.txt
│   │   ├── generate_touch_test_case.cpp # program to generate test case for place action such as test/place_test_gearmotor.txt
│   │   ├── look_test.cpp                # implementation of look_test in test.hpp
│   │   ├── place_test.cpp               # implementation of place_test in test.hpp
│   │   ├── test_client.cpp              # test client which executes touch, look and place test
│   │   └── touch_test.cpp               # implementation of touch_test in test.hpp
│   └── tools.cpp
├── test                                 # files used in unit test
│   ├── CAD                              # stl files
│   │   └── ...
│   ├── look_action_images               # jpg files used in look test
│   │   └── ...
│   ├── look_gripper_tip.csv             # csv files used in look test
│   ├── place_test_gearmotor.txt         # test cases for place test
│   ├── place_test_tetrahedron_1.txt
│   ├── place_test_tetrahedron_2.txt
│   ├── test_client.test
│   └── touch_input_gearmotor_1.txt      # test case for touch test
└── test_results
```
