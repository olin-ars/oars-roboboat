#!/usr/bin/python
import threading

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
            self.doneSub = rospy.Subscriber(doneTopic, Bool, self.donecallback)

    def donecallback(self, msg):
        if msg.data is True and self.active:
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


class SampleTask(TopicTask):
    """A SampleTask interacts with the sample_task defined in this directory."""

    def __init__(self, name="Sample Task"):
        super(SampleTask, self).__init__(name, "/test_task_active", "/test_task_done"),


class DelayTask(Task):
    """A DelayTask simply pauses for the provided number of seconds before finishing"""

    def __init__(self, time):
        super(DelayTask, self).__init__('Time delay: {}s'.format(time))

        self.delayduration = time
        self.timer = threading.Timer(self.delayduration, self.stop)

    def start(self, finishcallback):
        super(DelayTask, self).start(finishcallback)

        self.timer.start()

    def stop(self):
        self.timer.cancel()

        super(DelayTask, self).stop()
