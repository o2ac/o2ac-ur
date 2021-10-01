int64   class_id              # part ID detected by SSD
int64[] bbox                  # bounding box [x, y, w, h]
float64 confidence            # confidence of SSD results
geometry_msgs/Pose2D[] poses  # 2D poses within bbox estimated by template matching or graspability detection
bool    upside_down           # True if the item is upside down
