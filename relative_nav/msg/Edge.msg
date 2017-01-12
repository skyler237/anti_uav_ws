# Defines an edge between two nodes in the relative navigation graph-based map
# Header time is the time of the new node (not the time of the edge creation)
# Rotation is yaw only (roll and pitch are removed)

Header header
uint32 from_node_id
uint32 to_node_id
geometry_msgs/Transform transform
float64[36] covariance
