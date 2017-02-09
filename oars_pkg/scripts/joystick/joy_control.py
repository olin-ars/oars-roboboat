#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Joy
from oars_arbiter.voter import Voter_full
from oars_arbiter import createVote

class JoyController(object):
	def __init__(self):
		self.control={'p/s':3 ,'f/a':4, 'spin':1}
		self.switch={'estop':0, 'rc_override':1}

		rospy.init_node('joy_control')
		rospy.Subscriber('/joy', Joy, self.joy_cb)

		self.voter=Voter_full()
		self.direction = 0
		self.speed = 0
		self.rotation = 0

	def joy_cb(self, msg):
		ps_cmd=msg.axes[self.control['p/s']]
		fa_cmd=msg.axes[self.control['f/a']]
		spin_cmd=msg.axes[self.control['spin']]
		speed_01, dir_rad = self.cartesian_to_polar(fa_cmd, ps_cmd)
		self.speed = speed_01*100
		self.direction = round(dir_rad*50/math.pi)
		self.rotation = round(spin_cmd*25)

	def cartesian_to_polar(self, x, y):
		r = math.sqrt(x**2. + y**2.)
		theta = math.atan2(y, x)
		return r, theta

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.voter.dir_vote=createVote.directionVoteGauss(self.direction)
			self.voter.turn_vote=createVote.yawVoteGauss(self.rotation)
			self.voter.speed_vote=createVote.max_speed(self.speed)
			self.voter.make_vote()
			r.sleep()

if __name__=='__main__':
	joy = JoyController()
	joy.run()