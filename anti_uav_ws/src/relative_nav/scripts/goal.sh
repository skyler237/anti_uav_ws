#!/bin/bash
rostopic pub -1 /goal relative_nav/Goal "node_id: 1
pose:
  x: $1
  y: $2
  z: $3
  yaw: $4
"
