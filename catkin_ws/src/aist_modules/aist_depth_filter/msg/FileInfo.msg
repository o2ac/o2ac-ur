string                file_path      # path to the PLY file
std_msgs/Header       header         # timestamp and frame_id
bool                  plane_detected # TRUE if dominant plane is detected
geometry_msgs/Vector3 normal         # plane normal directed toward the camera
float64		      distance	     # distance between the plane and the camera