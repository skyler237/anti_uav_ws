# Waypoint.msg
# A single waypoint is defined by a position and velocity relative to node id
# A Path consists of multiple Waypoints relative to a specified node_id
# Note - we may decide to stay with nav_msg/Path

float64 x
float64 y
float64 z
float64 u
float64 v
float64 w
float64 yaw
