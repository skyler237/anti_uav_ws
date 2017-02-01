# This message provides the entire state of the filter and any additionally desired information for plotting and tuning
# Values are defined by the states in the filter:
#   transform.translation.x:y:Z are relative to the node frame
#   transform.rotation is quaternion of body frame relative to node frame
#   velocity is u,v,w relative to body frame

Header header
uint32 node_id
float64[16] state
float64[16] covariance_diagonal
