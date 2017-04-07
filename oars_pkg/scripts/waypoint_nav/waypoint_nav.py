#!/usr/bin/env python
"""
This node navigates to a waypoint.
"""
#python imports
import math
#ros imports
import rospy
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
#oars imports
from oars_arbiter.voter import Voter_full
from oars_arbiter import createVote

name = 'waypoint_nav'

rospy.init_node(name+'_node')

boat_frame = 'base_link'
waypoint_topic = '/next_waypoint'
checkoff_distance = 2
P_const = 25

def bound(n, low, high):
    if n<low:
        return low
    if n>high:
        return high
    return n

class Main(object):
    """docstring for Main"""

    def __init__(self, ):
        super(Main, self).__init__()

        self.startSub = rospy.Subscriber('/'+name+'_active', Bool, self.onEnable)
        self.donePub = rospy.Publisher('/'+name+'_done', Bool, queue_size=10)

        self.voter = Voter_full(name)

        self.running = False
        self.has_wpt = False
        self.wpt = PointStamped()
        self.tf = tf.TransformListener()

    def set_wpt(self, point):
        if True: #not self.has_wpt:
            self.has_wpt = True
            self.wpt = point
            print "navigating to waypoint"


    def onEnable(self, msg):
        if msg.data == True:
            self.running = True
            # Initialize values here
            self.waypointSub = rospy.Subscriber(waypoint_topic, PointStamped, self.set_wpt)

            print "waiting for waypoint"
        else:
            self.running = False

    def finishTask(self):
        self.waypointSub.unregister()
        self.has_wpt = False
        self.donePub.publish(Bool(True))
        print 'waypoint reached'

    def to_polar(self, point):
        r = math.sqrt(point.x**2 + point.y**2)
        theta = math.atan2(point.x, point.y)*180/math.pi
        return r, theta

    def run(self):
        r = rospy.Rate(10)  # 5hz refresh rate

        while not rospy.is_shutdown():
            if self.running and self.has_wpt:

                goal = self.tf.transformPoint(boat_frame, self.wpt).point
                distance, angle = self.to_polar(goal)

                if distance < checkoff_distance:
                    self.finishTask()

                speed = bound(distance*P_const, 20, 100)

                self.voter.dir_vote = createVote.directionVoteGauss(angle)
                self.voter.speed_vote = createVote.max_speed(speed)

                self.voter.make_vote()

            r.sleep()


if __name__ == '__main__':
    Main().run()