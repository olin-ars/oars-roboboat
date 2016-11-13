#!/usr/bin/env python

import rospy

from getch import getch

# this imports the Voter_full class from our arbiter package
from oars_arbiter.voter import Voter_full

class Controller():
    """Implements RC control of the vehicle from the terminal"""

    angle = speed = 0;


    def __init__(self):
        self.voter = Voter_full('Terminal RC Controller') #define the voter

    def keyControl(self):
        startTime = time.time()

        forward = turn = pan = tilt = 0

        while True: # time.time() < startTime + 30:
            command = getch()

            if command == '\x03':
                # Ctrl-C cancels
                return

            driveCommands = {
            'w':[1, 0],
            's':[-1,0],
            'a':[0,-1],
            'd':[0, 1],
            ' ':[0, 0],
            }

            if command in driveCommands:
                cmd = driveCommands[command]
                angle = cmd[0]
                speed = cmd[1]

            self.vote(angle, speed)

    def vote(self, angle, speed):
        self.voter.clear()

        pickiness = 1.1;

        self.voter.speed_vote = [speed] * 101

        satisfaction = lambda a: pickiness**(-abs(a-angle))

        self.voter.dir_vote = [satisfaction(a * 360.0/101) for a in range(-50, 51)]

        self.voter.make_vote()
        


if __name__ == "__main__":
    voter = Voter_full('test voter') #define the voter

    # set vote values
    voter.dir_vote = [1]*101
    #voter.speed_vote = [1.5]*101
    #voter.turn_vote = [0.1]*51

    print "Requesting update"

    if voter.make_vote(): #place vote (it returns a boolean)
        print "placed vote"
    else:
        print "failed"