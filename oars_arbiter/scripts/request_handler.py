import rospy

from oars_arbiter.srv import *
from behavior import Behavior

class Arbiter_Request_Handler():
	""" 
	Gets inputs from each behavior in the form of service
	requests. 
	These requests contain arrays which represent votes for
	each heading, the speed at those headings, and the
	overall turn rate.
	"""
	def __init__(self):
		""" sets up service handlers """
		# Service handlers
		rospy.Service('request_full', request_full, self.handle_full_request)
		rospy.Service('request_dir', request_dir, self.handle_dir_request)
		rospy.Service('update_weight', update_weight, self.handle_weight_request)

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
		behavior.reg_update() #indicate an update occurred
		behavior.update_dir(req.dir) #update direction request
		behavior.update_speed(req.speed) #update speed request
		behavior.update_turn(req.rotation) #update rotation request

		#for testing that the correct weight is being updated
		#print behavior.weight

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

		#clear vote
		behavior.clear()
		#update votes based on new input
		behavior.reg_update() #indicate update has occurred
		behavior.update_dir(req.dir) #update direction request

		return True #confirm request was received

	def handle_weight_request(self, req):
		target = req.target
		self.sender_check(target)

		behavior = self.behaviors[target]

		behavior.update_weight(req.weight)
		return True

	def sender_check(self, sender):
		"""
		takes name of a sender and checks if it is already in
		behaviors dictionary, if not, it adds the new sender
		"""
		if not self.behaviors.get(sender, 0): #check for existance
			self.behaviors[sender] = Behavior(sender) #initialize sender
		