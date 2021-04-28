geometry_msgs/Pose gripper_pose
sensor_msgs/Image looked_image
std_msgs/Float64MultiArray calibration_points # array such that [point1_x, point1_y, point2_x, point2_y,...]
std_msgs/UInt32MultiArray ROI # [left boundary, right boundary, top boundary, bottom boundary]