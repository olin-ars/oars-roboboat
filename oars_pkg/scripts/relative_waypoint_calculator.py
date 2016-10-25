#!/usr/bin/env python
import rospy
"""
Owner: Eric Miller (eric.miller@students.olin.edu)

This file reads from:
- location (geometry_msgs/Pose2D)
- next_waypoint (geometry_msgs/Pose2D)

and publishes to:
- next_waypoint_rel (geometry_msgs/Pose2D)
"""

import math

# This line may need to change depending on your message types
from geometry_msgs.msg import Pose2D

class relWaypointCalculator:#needs a topic to read in waypoint angle from
	def __init__(self):
		rospy.init_node('relative_waypoint_calculator')

		# Step 1: Initialize publishers. This will be referred to later whenever data is transmitted.
		self.relPub = rospy.Publisher('next_waypoint_rel', Pose2D, queue_size=10)

		# Step 2: Initialize subscribers. Each subscriber (incomming information) gets a "self.*" variable, a callback, and a subscriber
		self.next_waypoint = Pose2D(0,0,0)
		self.waypointSub = rospy.Subscriber('next_waypoint', Pose2D, self.onWaypoint)

		self.locationSub = rospy.Subscriber('location', Pose2D, self.onLocation)

	def onWaypoint(self, msg):
		self.next_waypoint = msg

	def calculateRelativePosition(self, location, waypoint):

		deltay = waypoint.y - location.y
		deltax = waypoint.x - location.x
		distance = math.sqrt(deltax^2 + deltay^2)
		absAngle = math.atan2(deltay, deltax)

		while absAngle <= -180:
			absAngle += 360

		while absAngle > 180:
			absAngle -= 360

		return Pose2D(distance, 0, absAngle)

	def onLocation(self, msg):
		self.relPub.publish(self.calculateRelativePosition(msg, self.next_waypoint))

if __name__ == '__main__':
	try:
		# Note that publishing is triggered whenever the boat's position changes
		# to minimize latency.
		handler = relWaypointCalculator()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
