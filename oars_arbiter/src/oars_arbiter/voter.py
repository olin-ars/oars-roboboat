import rospy
from oars_arbiter.srv import *

"""
Helper classes for using the arbiter. Import these into your
scripts and use them to manage how you input values into the
arbiter.
"""

class Voter_full():
    """ use this class if you want to request a specific motion from
    all of the degrees of freedom of the boat (direction, speed, and
        turn rate).
    """
    def __init__(self, name):
        """ initialize voter and set votes to default values """
        rospy.wait_for_service('request_full')
        self.vote = rospy.ServiceProxy('request_full', request_full)

        self.name = name
        self.dir_vote = [0]*101
        self.speed_vote = [100]*101
        self.turn_vote = [0]*51

    def make_vote(self):
        """ make service request to place vote, and return success/failure"""
        return self.vote(rospy.Time(), self.name, self.dir_vote, self.speed_vote, self.turn_vote)

class Voter_dir():
    """ Use this class if you only care about direction """
    def __init__(self, name):
        """ initialize voter and set votes to default values """
        rospy.wait_for_service('request_dir')
        self.vote = rospy.ServiceProxy('request_dir', request_dir)

        self.name = name
        self.dir_vote = [0]*101

    def make_vote(self):
        """ make service request to place vote, and return success/failure"""
        return self.vote(rospy.Time(), self.name, self.dir_vote)