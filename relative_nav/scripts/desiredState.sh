#!/bin/bash

rostopic pub -1 /desired_state relative_nav/DesiredState "node_id: 1
pose:
  x: $1
  y: $2
  z: $3
  yaw: $4
velocity:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
acceleration:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0" 

