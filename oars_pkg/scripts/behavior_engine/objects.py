"""
This file contains the base objects used for in the operation of the Planner,
including Plan, Task, and TopicTask. This file is not intended to be edited by
task developers, only extended.
"""

import rospy
from std_msgs.msg import Bool, String
import smach
import time


class Plan(object):
    """
    A Plan represents a sequence of Tasks that are intended to be exectued in order by the robot.
    """
    def __init__(self, name, sm):
        """
        :type tasklist: list of Task
        """
        self.name = name
        self.sm = sm

class Task(smach.State):
    """Task is the base class for anything that can go in a plan"""

    def __init__(self, name):
        smach.State.__init__(self, outcomes = ['done'])

        self.name = name

        self.finishCallback = lambda: None
        self.active = False
        self.statusPub = rospy.Publisher('/smach', String, queue_size=10)

    def execute(self, userdata): 
    	self.start(None)

    	while self.active and not rospy.is_shutdown():
    		time.sleep(0.01)
            self.statusPub.publish(self.name)

    	return 'done'


    def start(self, finishcallback):
        """
        Starts the task
        :param finishcallback: A function that will be called when self.stop() is called
        """
        self.active = True
        self.finishCallback = finishcallback
        print("\rTask {} starting\r".format(self.name))

    def stop(self):
        """
        Stops the task and calls the function passed to start()
        """
        print "\rTask {} finished\r".format(self.name)
        if self.active:
            self.active = False
            if self.finishCallback:
                self.finishCallback()
        else:
            print("\rWARNING: Task {} stop called on inactive task\r")


class TopicTask(Task):
    """A TopicTask is a task that writes to ROS topics when it starts and stops"""

    def __init__(self, name, activationTopic="", doneTopic=""):
        """
        :param activationTopic: The ROS topic name (of type std_msgs/Bool)
            that this Task will publish to when it starts or stops
        :param doneTopic: The ROS topic name (of type std_msgs/Bool) that this
            task will stop when it recieves a "True" from
        """
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
        """ Sends a True message to activationTopic and starts the task """
        self.activationPub.publish(Bool(True))

        super(TopicTask, self).start(finishcallback)

    def stop(self):
        """ Sends a False message to activationTopic and stops the task"""
        self.activationPub.publish(Bool(False))

        super(TopicTask, self).stop()