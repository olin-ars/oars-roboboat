#!/usr/bin/env python

import rospy

from oars_arbiter.srv import *

class Behavior():
	"""
	Store action requests from a behavior
	"""
	def __init__(self, name):
		""" 
		stores the behavior's name and initializes requests
		"""
		self.name=name
		self.clear()

	def clear(self):
		""" reset requests """
		self.dir = [0]*101
		self.speed = [100]*101
		self.turn = [0]*51

	def update_dir(self, dir_req):
		""" updates the direction request """
		self.dir = dir_req

	def update_speed(self, speed_req):
		""" updates the speed request """
		self.speed = speed_req

	def update_turn(self, turn_req):
		""" updates the turn request """
		self.turn = turn_req

class Arbiter():
	""" 
	Gets inputs from each behavior in the form of service
	requests. 
	These requests contain arrays which represent votes for
	each heading, the speed at those headings, and the
	overall turn rate.
	"""
	def __init__(self):
		""" initializes node and sets up service handlers """
		rospy.init_node("arbiter")

		# Service handlers
		rospy.Service('request_full', request_full, self.handle_full_request)
		rospy.Service('request_dir', request_dir, self.handle_dir_request)

		#dictionary of behaviors
			# {behavior name : behavior class}
		self.behaviors = {}


	def handle_full_request(self, req):
		""" 
		receive an action request containing all possible information
		"""
		sender = req.sender #string, name of sender

		# make sure sender is initialized
		self.sender_check(sender)

		# class holding the sender's request
		behavior = self.behaviors[sender]

		#update votes based on new input
		behavior.update_dir(req.dir) #update direction request
		behavior.update_speed(req.speed) #update speed request
		behavior.update_turn(req.rotation) #update rotation request

		return True #confirm request was received

	def handle_dir_request(self, req):
		""" 
		receive an action request containing only a direction vote
		"""
		sender = req.sender #string, name of sender

		# make sure sender is initialized
		self.sender_check(sender)

		# class holding the sender's request
		behavior = self.behaviors[sender]

		#update votes based on new input
		behavior.update_dir(req.dir) #update direction request

		return True #confirm request was received

	def sender_check(self, sender):
		"""
		takes name of a sender and checks if it is already in
		behaviours dictionary, if not, it adds the new sender
		"""
		if not self.behaviors.get(sender, 0): #check for existance
			self.behaviors[sender] = Behavior(sender) #initialize sender

	def main(self):
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			for key in self.behaviors.keys():
				print self.behaviors[key].name
				print self.behaviors[key].speed[0]
			r.sleep()

if __name__ == "__main__":
	arbiter = Arbiter()
	arbiter.main()