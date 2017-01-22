#!/usr/bin/python

import rospy
from std_msgs.msg import Bool


class Task(object):
    """Task is the base class for anything that can go in a plan"""

    def __init__(self, name):
        super(Task, self).__init__()

        self.name = name

        self.finishCallback = lambda: None
        self.active = False

    def start(self, finishcallback):
        self.active = True
        self.finishCallback = finishcallback
        print("Task {} starting".format(self.name))

    def stop(self):
        print "Task {} finished".format(self.name)
        if self.active:
            self.finishCallback()
        else:
            print("WARNING: Task {} stop called on inactive task")


class TopicTask(Task):
    """A TopicTask is a task that writes to ROS topics when it starts and stops"""

    def __init__(self, name, activationTopic="", doneTopic=""):
        super(TopicTask, self).__init__(name)

        self.activationTopic = activationTopic
        self.doneTopic = doneTopic

        self.activationPub = rospy.Publisher(activationTopic, Bool, queue_size=10, latch=True)

        if doneTopic:
            self.doneSub = rospy.Subscriber(doneTopic, Bool, self.onDoneMessage)

    def onDoneMessage(self, msg):
        if msg.data == True and self.active:
            self.stop()

    def start(self, finishcallback):
        self.activationPub.publish(Bool(True))

        super(TopicTask, self).start(finishcallback)

    def stop(self):
        self.activationPub.publish(Bool(False))

        super(TopicTask, self).stop()


class RCTask(TopicTask):
    """An RC Task triggers the RC code when it starts and never finishes"""

    def __init__(self, name="Obey RC commands"):
        super(RCTask, self).__init__(name, activationTopic="/RC_active")
