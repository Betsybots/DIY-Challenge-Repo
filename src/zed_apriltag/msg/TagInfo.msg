# One detected AprilTag: numeric ID and its label from config/tag_labels.yaml.
std_msgs/Header header
int32 id
string label
float64 distance  # meters from camera to tag (solvePnP), -1 if unknown
