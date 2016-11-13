import rospy

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
		self.clear()

	def clear(self):
		""" reset requests """
		self.dir = [0]*101
		self.speed = [100]*101
		self.turn = [0]*51

	def reg_update(self):
		""" 
		'register' an update, aka set values to indicate
		that an update has occured 
		"""
		self.updating = True
		self.last_update = rospy.Time.now()

	def update_dir(self, dir_req):
		""" updates the direction request """
		self.dir = dir_req

	def update_speed(self, speed_req):
		""" updates the speed request """
		self.speed = speed_req

	def update_turn(self, turn_req):
		""" updates the turn request """
		self.turn = turn_req