"""
This file contains mission-specific Task definitions. Task developers
should modify this file to have a class representing their functionality.
"""
import threading
from time import sleep

from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, Point

from objects import *


class SampleTask(TopicTask):
    """A SampleTask interacts with the sample_task defined in this directory."""

    def __init__(self, name="Sample Task"):
        super(SampleTask, self).__init__(name, "/test_task_active", "/test_task_done"),


class RCTask(TopicTask):
    """An RC Task triggers the RC code when it starts and never finishes"""

    def __init__(self, name="Obey RC commands"):
        super(RCTask, self).__init__(name, activationTopic="/RC_active")

# YOUR TASKS HERE

class GPSNavigationTask(TopicTask):

    def __init__(self, name="Navigation Task", destination=""):
        super(GPSNavigationTask, self).__init__(name, activationTopic="/GPS_navigation_active", doneTopic="/GPS_navigation_done")
        self.destinationPub = rospy.Publisher("/GPS_navigation_destination", String, queue_size=10)
        self.destination = destination

        self.activation_delay = 0.1

    def start(self, finishcallback):
        self.destinationPub.publish(String(self.destination))
        sleep(self.activation_delay)
        super(GPSNavigationTask, self).start(finishcallback)


class WaypointNavigationTask(TopicTask):

    def __init__(self, destination, name="Navigation Task"):
        """
        :type destination: (string, float, float)
        """
        super(WaypointNavigationTask, self).__init__(name, activationTopic="/waypoint_nav_active", doneTopic="/waypoint_nav_done")
        self.destinationPub = rospy.Publisher("/next_waypoint", PointStamped, queue_size=1, latch=True)
        self.destination = self.makePoint(destination)

        self.activation_delay = 0.1

    @staticmethod
    def makePoint(destination):
        frame, x, y = destination
        return PointStamped(header=Header(frame_id=frame), point=Point(x=x, y=y, z=0))

    def start(self, finishcallback):
        super(WaypointNavigationTask, self).start(finishcallback)
        sleep(self.activation_delay)
        self.destinationPub.publish(self.destination)


class DelayTask(Task):
    """A DelayTask simply pauses for the provided number of seconds before finishing"""

    def __init__(self, time):
        task_name = 'Time delay: {}s'.format(time)
        print task_name
        super(DelayTask, self).__init__(task_name)

        self.delayduration = time

    def start(self, finishcallback):
        super(DelayTask, self).start(finishcallback)

        self.timer = threading.Timer(self.delayduration, self.stop)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

        super(DelayTask, self).stop()

