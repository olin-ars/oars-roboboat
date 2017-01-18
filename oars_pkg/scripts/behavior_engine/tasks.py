class Task(object):
	"""Task is the base class for anything that can go in a plan"""
	def __init__(self, name, activationTopic="", doneTopic="", hasData=False):
		super(Task, self).__init__()
		self.activationTopic = activationTopic
		self.hasData = hasData
		self.doneTopic = doneTopic		


class RCTask(Task):
	"""An RC Task triggers the RC code when it starts and never finishes"""
	def __init__(self):
		super(RCTask, self).__init__("Obey RC commands", activationTopic="triggerRC")


class NullTask(Task):
	"""This task will not do anything when it triggers and never finish"""
	def __init__(self):
		super(NullTask, self).__init__("Null Task")


class NavTask(Task):
	"""This task will not do anything when it triggers and never finish"""
	def __init__(self, destinationStr):
		super(NavTask, self).__init__("Navigate to point", activationTopic="/start_navigation", doneTopic="/navigation_finished", hasData=True)
		self.destination = destinationStr
