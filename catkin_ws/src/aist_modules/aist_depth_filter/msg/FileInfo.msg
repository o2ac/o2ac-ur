std_msgs/Header       header
string                file_path      # path to the PLY file
bool                  plane_detected # TRUE if dominant plane is detected
geometry_msgs/Vector3 normal         # plane normal directed toward the camera
float64		      distance	      # plane distance from the camera
