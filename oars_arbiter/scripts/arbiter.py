#!/usr/bin/env python

import rospy

from request_handler import Arbiter_Request_Handler

#maximum time between requests from a behavior
TIMEOUT = rospy.Duration(1.2)

class Arbiter(Arbiter_Request_Handler):
	"""
	Combines various inputs for heading, speed, and turn rate
	and publishes a final speed/heading request to /cmd_vel
	"""
	def __init__(self):
		""" initializes node and starts the service handler """
		rospy.init_node("arbiter") #initialize node

		# run the request handler's init function
		Arbiter_Request_Handler.__init__(self)

	def check_for_inactivity(self):
		""" cycles through the behaviors to see if any have
		become inactive
		resets request to default for inactive behaviors
		"""
		now = rospy.Time.now() #get current time
		for behavior in self.behaviors.values(): #cycle through behaviors
			time_since_update = now - behavior.last_update
			#check if the behavior is active and if it has been longer than timeout
			if behavior.updating and time_since_update > TIMEOUT:
				behavior.clear() #reset request to default
				behavior.updating=False #indicate behavior is inactive


	def main(self):
		""" main loop for arbiter, continually outputs final
		commands for the boat """
		r = rospy.Rate(1) #runrate in Hz
		while not rospy.is_shutdown():
			self.check_for_inactivity() #check for inactive behaviors
			
			# print things out for testing
			for behavior in self.behaviors.values():
				if behavior.updating:
					print behavior.name
					print behavior.dir[0]
			
			r.sleep() #wait for next loop cycle

if __name__ == "__main__":
	arbiter = Arbiter()
	arbiter.main()