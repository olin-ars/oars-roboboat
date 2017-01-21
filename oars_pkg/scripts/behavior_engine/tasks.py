#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

class Task(object):
	"""Task is the base class for anything that can go in a plan"""
	def __init__(self, name):
		super(Task, self).__init__()

		self.name = name	

	def start(self):
		print("Error: start() not implemented for task " + self.name)
		raise NotImplementedError

	def stop(self):
		print("Error: stop() not implemented for task " + self.name)
		raise NotImplementedError


class TopicTask(Task):
	"""A TopicTask is a task that writes to ROS topics when it starts and stops"""
	def __init__(self, name, activationTopic="", doneTopic=""):
		super(TopicTask, self).__init__(name)

		self.activationTopic = activationTopic
		self.doneTopic = doneTopic

		self.active = False

		self.activationPub = rospy.Publisher(activationTopic, Bool, queue_size=10, latch=True)

		if doneTopic:
			self.doneSub = rospy.Subscriber(doneTopic, Bool, self.onDoneMessage)

	def onDoneMessage(self, msg):
		if msg.data == True and self.active:
			self.stop()
			self.finishCallback()

	def start(self, finishCallback):
		self.active = True

		self.finishCallback = finishCallback

		self.activationPub.publish(Bool(True))
		print("Starting task ({})".format(self.name))

	def stop(self):
		self.active = False

		self.activationPub.publish(Bool(False))
		print("Stopping task ({})".format(self.name))

		

class RCTask(TopicTask):
	"""An RC Task triggers the RC code when it starts and never finishes"""
	def __init__(self, name="Obey RC commands"):
		super(RCTask, self).__init__(name, activationTopic="/RC_active")
