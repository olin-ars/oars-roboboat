import rospy
import numpy as np

class Behavior():
	"""
	Store action requests from a behavior
	"""
	def __init__(self, name):
		""" 
		stores the behavior's name and initializes requests
		"""
		self.updating = False
		self.last_update = rospy.Time.now()
		self.name=name
		self.weight=1
		self.clear()

	def clear(self):
		""" reset requests """
		self.dir = np.zeros(101)
		self.speed = np.ones(101)*100
		self.turn = np.zeros(51)

	def reg_update(self):
		""" 
		'register' an update, aka set values to indicate
		that an update has occured 
		"""
		self.updating = True
		self.last_update = rospy.Time.now()

	def update_dir(self, dir_req):
		""" updates the direction request """
		self.dir = np.array(dir_req)

	def update_speed(self, speed_req):
		""" updates the speed request """
		self.speed = np.array(speed_req)

	def update_turn(self, turn_req):
		""" updates the turn request """
		self.turn = np.array(turn_req)

	def update_weight(self, new_weight):
		self.weight=new_weight