# Drone pose in the map frame.
Header header                     # frame_id = map frame
geometry_msgs/Pose pose           # body position and orientation
float64 height_above_ground       # altitude used for range gating and the sky filter (m)
bool valid                        # false until the pose source is trusted; processing holds until true
