#!/usr/bin/env python
import rospy
"""
This file reads from:
- heading_err (std_msgs/Int16)

and publishes to:
- auto_mode/rudder/set_point (std_msgs/Int16)

"""

# This line may need to change depending on your message types
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Twist


class autonomousRudderPublisher:#needs a topic to read in waypoint angle from
	def __init__(self):
		rospy.init_node('rudder_publisher')

		# Step 1: Initialize publishers. This will be referred to later whenever data is transmitted.
		self.rudderPub = rospy.Publisher('rudder_angle', Float32)
		self.powerPub = rospy.Publisher('propeller_power', Float32)

		# Step 2: Initialize subscribers. Each subscriber (incomming information) gets a "self.*" variable, a callback, and a subscriber
		self.velocity = 0
		self.anglevelocity = 0
		self.waypointAngleSub = rospy.Subscriber('cmd_vel', Twist, self.onAngle)

	def onAngle(self, msg):
		self.velocity = msg.linear.x
		self.anglevelocity = msg.angular.x
		

	def calculatePropellerPower(self, velocity):
		# Do your main logic in a separate function for cleanliness reasons.

		power = velocity

		return power

	def calculateRudderAngle(self, anglevelocity):

                angle = anglevelocity * 20


	def run(self):
		rate = rospy.Rate(10)  # 10hz refresh rate
		while not rospy.is_shutdown():
			power = self.calculatePropellerPower(self.velocity)
			ang = self.calculateRudderAngle(self.anglevelocity)
			
			# This is where the actual publishing happens
			self.powerPub.publish(power)
			self.rudderPub.publish(ang)

			rate.sleep()

if __name__ == '__main__':
	try:
		handler = autonomousRudderPublisher()
		handler.run()
	except rospy.ROSInterruptException:
		pass
