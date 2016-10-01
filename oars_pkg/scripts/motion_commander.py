#!/usr/bin/env python
import rospy
import math

class MotionCommander

	max_linear_velocity = 1 # 1 meter/second
	max_angular_velocity = 6 # 6 degrees/second

	def calcVelocity(next_waypoint_rel, has_waypoint):
		# Calculate angular velocity
		if next_waypoint_rel.theta > 20:
			angularVel = 25
		else:
			angularVel = next_waypoint_rel.theta * max_angular_velocity / 30
		
		# Calculate linear velocity
		distanceToWaypoint = math.sqrt( (next_waypoint_rel.x)^2 + (next_waypoint_rel.y)^2 )
		if (distanceToWaypoint < 0.1): # TODO May not need
			lvs = 0
		else
			lvs = distanceToWaypoint / 2; # linear velocity scalar TODO Adjust

		# Generate return object
		cmd_vel = Twist( Vector3( lvs*next_waypoint_rel.x, lvs*next_waypoint_rel.y, 0), Vector3( angularVel, 0, 0 ) )

		return cmd_vel
		

