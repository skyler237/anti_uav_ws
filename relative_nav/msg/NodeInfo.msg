# Contains information about a node in the relative navigation graph-based map
#   Published by MEKF
#   Provides mapping from MEKF node_id and VO keyframe ID

Header header
uint32 node_id
uint32 keyframe_id
geometry_msgs/Quaternion node_to_body
geometry_msgs/Transform camera_to_body
