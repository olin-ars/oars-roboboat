"""Waypoint checker. Decides when a waypoint has been reached, based on the location of the boat."""

import rospy
import Pose2D from geometry_msgs.msgs
import Bool   from std_msgs.msgs
import math

rospy.init_node('waypoint_checker')
pubReachedWP =  rospy.Publisher('reached_waypoint',Bool)
subLocation  =  rospy.Subscriber('location',Pose2D,locationCallback)
subNextWP    =  rospy.Subscriber('next_waypoint',Pose2D,nextWPCallback)
subHasWP     =  rospy.Subscriber('has_waypoint',Bool,hasWPCallback)

waypointRadius = 1 #meters

nextWP = None #None if there is no next waypoint. Otherwise contains the target waypoint

def locationCallback(location):
    """Callback for the location topic. Updates the reached_waypoint callback with true if the current target waypoint is within waypointRadius of the location """
    if nextWP is not None:
        pubReadedWP.publish(Bool(getDistance(location,nextWP) < waypointRadius))


def getDistance(location1, location2):
    """returns the euclidian distance between two Pos2D objects"""
    x = location1.x - location2.x
    y = location1.y - location2.y
    return math.sqrt(x**2 + y**2) #pythagoras was a great man

def nextWPCallback(location):
    """Callback for the next_waypoint topic. Updates the current target waypoint with the new one provided in the message"""
    global nextWP
    nextWP = location


def hasWPCallback(exists):
    """Callback for the has_waypoint topic. Sets the target to None if the message is false, otherwise does nothing"""
    global nextWP
    if not exists.data:
        nextWP = None

rospy.spin()
