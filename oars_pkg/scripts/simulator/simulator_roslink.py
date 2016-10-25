#! /usr/bin/python

import simulator_main as sim
import math

import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Pose2D


class ROShandler():
    """docstring for ROShandler"""
    def __init__(self, model):
        rospy.init_node('simulator', anonymous=True)
        self.model = model
        self.registerSubs()
        self.registerPubs()

    def registerSubs(self):
        self.rudderSub = rospy.Subscriber('/rudder_angle',
                                          Float32, self.onRudder)
        self.sailSub = rospy.Subscriber('/propeller_power',
                                        Float32, self.onPower)

    def registerPubs(self):
        self.locationPub = rospy.Publisher('location', Pose2D, queue_size=10)

    def publish(self):
        def angleconvert(valin):
            angle = -valin * 180 / math.pi + 90
            while (angle < 0):
                angle += 360
            while (angle >= 360):
                angle -= 360
            return angle

        b = model.boat1
        self.locationPub.publish(Pose2D(b.xpos, b.ypos, angleconvert(b.heading)))


    def onRudder(self, msg):
        """The ROS network thinks in degrees,
        the sim things in 1/4th rotations"""
        self.model.boat1.RudderSuggestion = msg.data * 1/90.0

    def onPower(self, msg):
        self.model.boat1.motorboat = msg.data * 10

if __name__ == '__main__':
    model = sim.WorldModel(0, 0) # disable wind in the model
    roshandler = ROShandler(model)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        model.update_model()
        roshandler.publish()
        print model.boat1.posStr()
        print model.boat1.RudderPos
        # print model.boat1.forward_speed
        # print "angvel={}".format(model.boat1.angularVelocity)
        # print model.boat1.MainPos
