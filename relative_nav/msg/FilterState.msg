# This message provides the relative state and covariance that the estimator tracks
# Values are defined by the states in the filter:
#   transform.translation.x:y:Z are relative to the node frame
#   transform.rotation is quaternion of body frame relative to node frame
#   velocity is u,v,w relative to body frame

uint32 node_id
geometry_msgs/Transform transform
geometry_msgs/Vector3 velocity
float64[36] covariance
