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
        self.rudderSub = rospy.Subscriber('/rudder/set_point',
                                          Int16, self.onRudder)
        self.sailSub = rospy.Subscriber('/sail/set_point',
                                        Float32, self.onSail)

    def registerPubs(self):
        self.locationPub = rospy.Publisher('location', Pose2D)
        self.velocityPub = rospy.Publisher('velocity', Pose2D)
        self.true_windPub = rospy.Publisher('true_wind', Pose2D)
        self.relative_windPub = rospy.Publisher('relative_wind', Pose2D)
        self.global_windPub = rospy.Publisher('global_wind', Pose2D)

        self.sailPub = rospy.Publisher('/sail/pos', Float32)
        self.rudderPub = rospy.Publisher('/rudder/pos', Int16)

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

        # This nasty math creates a 0...359 degrees CW from North
        vheading = angleconvert(math.atan2(b.vy, b.vx))
        self.velocityPub.publish(Pose2D(math.sqrt(b.vx**2+b.vy**2), 0, vheading))

        w = model.wind
        self.true_windPub.publish(Pose2D(w.windspeed, 0, angleconvert(w.windheading-b.heading-math.pi/2)))
        self.global_windPub.publish(Pose2D(w.windspeed, 0, angleconvert(w.windheading + math.pi)))

        (wvx, wvy) = (w.windspeed * math.cos(w.windheading), w.windspeed * math.sin(w.windheading))
        (wvx, wvy) = (wvx - b.vx, wvy - b.vy)

        self.relative_windPub.publish(Pose2D(math.sqrt(wvx**2 + wvy**2), 0,
                                      angleconvert(math.atan2(wvy, wvx) - b.heading + 3*math.pi/2)))

        self.sailPub.publish(Float32(model.boat1.MainPos*6))
        self.rudderPub.publish(Int16(model.boat1.RudderPos*90))

    def onRudder(self, msg):
        """The ROS network thinks in degrees,
        the sim things in 1/4th rotations"""
        self.model.boat1.RudderSuggestion = msg.data * 1/90.0

    def onSail(self, msg):
        """The ROS network thinks in sensors (0-6),
        the sim things in 1/4th rotations"""
        self.model.boat1.MainSuggestion = msg.data * 1.0/6

if __name__ == '__main__':
    model = sim.WorldModel(20, -math.pi/4)  # initial windspeed, the direction the wind is going (not coming from)
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
