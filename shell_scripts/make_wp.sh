#!/bin/sh
rostopic pub /new_waypoint geometry_msgs/Pose2D "x: $1
y: $2
theta: 1" 
