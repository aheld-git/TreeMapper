# UNUSED: no node publishes or reads this message (the node takes the pose from TF or a
# geometry_msgs/PoseStamped, see treemapper_ros/README.md). Kept only so the interface package stays stable.
# Drone pose in the map frame.
Header header                     # frame_id = map frame
geometry_msgs/Pose pose           # body position and orientation
float64 height_above_ground       # UNUSED, never read: the mapper gates on the pose's world z (pose.position.z)
bool valid                        # false until the pose source is trusted; processing holds until true
