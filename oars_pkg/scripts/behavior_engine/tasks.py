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

	def start(self):
		self.active = True

		self.activationPub.publish(Bool(True))
		print("Starting task ({})".format(self.name))

	def stop(self):
		self.active = False
		
		self.activationPub.publish(Bool(False))
		print("Stopping task ({})".format(self.name))

		

class RCTask(TopicTask):
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


def test():
	rospy.init_node('task_test_node')
	firsttask = TopicTask("Test task", "/test_task_active", "/test_task_done")

	firsttask.start()

	rospy.spin()


if __name__ == '__main__':
	test()