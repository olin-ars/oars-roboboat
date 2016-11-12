#!/usr/bin/env python

import rospy

# this imports the Voter_full class from our arbiter package
from oars_arbiter.voter import Voter_dir

if __name__ == "__main__":
    voter = Voter_dir() #define the voter

    # set vote values
    voter.dir_vote = [1]*101
    #voter.speed_vote = [1.5]*101
    #voter.turn_vote = [0.1]*51

    print "Requesting update"

    if voter.make_vote(): #place vote (it returns a boolean)
        print "placed vote"
    else:
        print "failed"