from tasks import *

class Plan(object):
	"""docstring for Plan"""
	def __init__(self, tasks=[]):
		super(Plan, self).__init__()
		self.tasks = tasks


RCtestPlan = Plan([
					RCTask(),
					NullTask(),
					])