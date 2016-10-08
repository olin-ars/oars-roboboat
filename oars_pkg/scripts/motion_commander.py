#!/usr/bin/env python
import rospy
import math

"""
This file reads from:
- next_waypoint_rel (geometry_msgs/Pose2D)
- has_waypoint (std_msgs/Bool)

and publishes to:
- cmd_vel (geometry_msgs/Pose2D)

"""

from std_msgs.msg import Bool, Int16, Float32
from geometry_msgs.msg import Pose2D, Twist, Vector3

class MotionCommander:

	max_linear_velocity = 1 # 1 meter/second
	max_angular_velocity = 6 # 6 degrees/second

	def __init__(self):
		rospy.init_node('motion_commander')

		# Step 1: Initialize publishers. These will be referred to later whenever data is transmitted.
		self.cmd_velPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		# Step 2: Initialize subscribers. Each subscriber (incomming information) gets a "self.*" variable, a callback, and a subscriber
		self.next_waypoint_rel = Pose2D(0,0,0)
		self.nextWaypointRelSub = rospy.Subscriber('next_waypoint_rel', Pose2D, self.onWaypoint)

		self.has_waypoint = False
		self.nextWaypointRelSub = rospy.Subscriber('has_waypoint', Bool, self.onWaypoint)

	def onWaypoint(self, msg):
		self.next_waypoint_rel = msg

	def onHas(self, msg):
		self.has_waypoint = msg.data

	def calcVelocity(self, next_waypoint_rel, has_waypoint):
		# Calculate angular velocity
		if next_waypoint_rel.theta > 20:
			angularVel = 25
		else:
			angularVel = next_waypoint_rel.theta * self.max_angular_velocity / 30
		
		# Calculate linear velocity
		distanceToWaypoint = math.sqrt( (next_waypoint_rel.x)^2 + (next_waypoint_rel.y)^2 )
		if (distanceToWaypoint < 0.1): # TODO May not need
			lvs = 0
		else:
			lvs = distanceToWaypoint / 2; # linear velocity scalar TODO Adjust

		# Generate return object
		cmd_vel = Twist( Vector3( lvs*next_waypoint_rel.x, 0, 0), Vector3( angularVel, 0, 0 ) )

		return cmd_vel

	def run(self):
		rate = rospy.Rate(50)  # 50hz refresh rate
		while not rospy.is_shutdown():
			cmd_vel = self.calcVelocity(self.next_waypoint_rel,self.has_waypoint)
			
			# This is where the actual publishing happens
			self.cmd_velPub.publish(cmd_vel)

			rate.sleep()

if __name__ == '__main__':
	try:
		handler = MotionCommander()
		handler.run()
	except rospy.ROSInterruptException:
		pass
