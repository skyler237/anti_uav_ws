# NodeState.msg
# This message defines the state of craft relative to the node frame
# It is used by the path follower to send the desired state to the controller
# When waypoints are used, acceleration terms are likely zero
# When trajectories are used, velocities and accelerations provide feed forward terms
#   to the controller.
# These values are relative to the node frame indicated by the node_id

uint32 node_id
DesiredVector pose
DesiredVector velocity
DesiredVector acceleration
bool position_valid
bool velocity_valid
bool acceleration_valid
