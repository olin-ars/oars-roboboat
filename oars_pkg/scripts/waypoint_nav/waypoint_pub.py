#!/usr/bin/env python

""" Test waypoint publisher, not for final use """

import sys
import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node('wp_test_pub')
wp_pub = rospy.Publisher('/next_waypoint', PointStamped, queue_size = 10)

x = 0
y = 0
if len(sys.argv) == 3:
	x = float(sys.argv[1])
	y = float(sys.argv[2])
print 'x: ', x, '	y: ', y
msg = PointStamped()
msg.point.x = x
msg.point.y = y

rospy.sleep(0.1)

msg.header.frame_id = "world"

wp_pub.publish(msg)

rospy.spin()