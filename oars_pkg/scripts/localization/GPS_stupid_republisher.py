#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

class repub():
    def __init__(self):
        rospy.init_node('gps_repub')
        self.pub = rospy.Publisher('fix2', NavSatFix, queue_size=10)
        sub = rospy.Subscriber('fix', NavSatFix, self.republish)
        rospy.spin()
    def republish(self, msg):
        msg.header.frame_id = 'gps'
        self.pub.publish(msg)

if __name__ == '__main__':
    repub()
