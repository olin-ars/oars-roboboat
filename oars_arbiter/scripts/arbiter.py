#!/usr/bin/env python

import rospy
import numpy as np
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

		self.heading = 50 #initialize heading to forward
		self.speed = 0 #initialize speed to stopped
		self.turn = 25

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

	def scale_votes(self, votes):
		""" function for converting votes from the -1:1 scale 
		to a -inf:inf scale (I promise, this is a real function)
		"""
		pos = np.abs(votes)
		scaled_pos = (1+pos)/(1-pos)-1
		scaled = np.sign(votes)*scaled_pos
		return scaled

	def get_direction(self):
		""" combine direction requests and find the most
		prefered heading """
		dir_total = np.zeros(101) #initialize array for the sum
		dir_total[self.heading] = .01 #give small weight to old heading
		for behavior in self.behaviors.values():
			if behavior.updating:
				#add up scaled votes from all active behaviors
				dir_total += self.scale_votes(behavior.dir)
		self.heading = int(np.argmax(dir_total)) #find index of maximum

	def get_speed(self):
		""" get the minimum speed vote for the current heading """
		min_vote = 101 #initialize to > max speed
		for behavior in self.behaviors.values():
			if behavior.updating:
				#get speed vote for all active behaviors
				speed_vote = behavior.speed[self.heading]
				if speed_vote < min_vote:
					# track minimum
					min_vote = speed_vote
		if min_vote > 100: #if there were no votes, stop
			self.speed = 0
		else: #set speed to min vote
			self.speed = min_vote

	def get_turn_rate(self):
		""" combine turn rate requests and find most
		prefered turn rate """
		turn_total = np.zeros(51) #initialize sum array
		turn_total[self.default_turn()] = .01 #weight turning towards heading slightly
		for behavior in self.behaviors.values():
			if behavior.updating:
				#add turn votes for all active behaviors
				turn_total += self.scale_votes(behavior.turn)
		self.turn = np.argmax(turn_total) #find index of maximum

	def default_turn(self):
		""" find a default turn rate to turn towards the heading 
			This function makes the boat tur sharply if the
			heading is perpendicular to the direction of the boat
			and not at all if the boat is going either forward or
			backward
		"""
		if self.heading > 50:
			x=100-self.heading
			flip=-1
		else:
			x = self.heading
			flip=1
		if x < 25: turn = -(x/25.)**2*25
		else: turn = x-50
		return int(round(turn*flip+25))

	def main(self):
		""" main loop for arbiter, continually outputs final
		commands for the boat """
		r = rospy.Rate(1) #runrate in Hz
		while not rospy.is_shutdown():
			self.check_for_inactivity() #check for inactive behaviors
			self.get_direction() #calculate prefered heading
			self.get_speed() #find speed at that heading
			self.get_turn_rate() #calculate prefered turn rate

			# print things out for testing
			print '\nheading', (self.heading-50)/50.
			print 'speed', self.speed
			print 'turn', (self.turn-25)/25.
			
			r.sleep() #wait for next loop cycle

if __name__ == "__main__":
	arbiter = Arbiter()
	arbiter.main()