#!/usr/bin/env python

import rospy

# this imports the Voter_full class from our arbiter package
from oars_arbiter.voter import Voter_full

if __name__ == "__main__":
    voter = Voter_full() #define the voter

    # set vote values
    voter.dir_vote = [.8]*101
    voter.dir_vote[40] = 0.86
    voter.speed_vote = [40]*101
    #voter.turn_vote = [0.1]*51
    #voter.turn_vote[22] = 0.2

    print "Requesting update"

    if voter.make_vote(): #place vote (it returns a boolean)
        print "placed vote"
    else:
        print "failed"