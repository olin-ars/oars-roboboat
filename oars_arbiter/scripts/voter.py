#!/usr/bin/env python

import rospy
from oars_arbiter.srv import *
class Voter_full():
    def __init__(self):
        rospy.wait_for_service('request_full')
        self.vote = rospy.ServiceProxy('request_full', request_full)

        self.dir_vote = [0]*101
        self.speed_vote = [1]*101
        self.turn_vote = [0]*51

    def make_vote(self):
        return self.vote(rospy.Time(), 'test', self.dir_vote, self.speed_vote, self.turn_vote)