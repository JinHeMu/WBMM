# Base pose in header.frame_id; optional terminal arm configuration.
# Empty joint arrays leave the final arm posture free.
std_msgs/Header header
geometry_msgs/Pose base_pose
string[] joint_names
float64[] joint_positions
