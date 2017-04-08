#!/usr/bin/env python
"""
Republishes the position estimate from the AR tag tracker
as an Odometry message, including generation of covariance data
"""

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

MARKER_ID = 0
LINEAR_COVARIANCE = 0.1
ANGULAR_COVARIANCE = 0.3
CHILD_FRAME = 'ar_tag'


class ARrepublisher:  # needs a topic to read in waypoint angle from
    def __init__(self):
        rospy.init_node('ar_republisher')

        # Step 1: Initialize publishers. This will be referred to later whenever data is transmitted.
        self.pub = rospy.Publisher('ar_odometry', Odometry, queue_size=1)

        # Step 2: Initialize subscribers. Each subscriber (incomming information) gets a "self.*" variable,
        # a callback, and a subscriber
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.onMarker)

        self.covariance = [0] * 36
        self.covariance[0] = self.covariance[7] = self.covariance[14] = LINEAR_COVARIANCE
        self.covariance[21] = self.covariance[28] = self.covariance[35] = ANGULAR_COVARIANCE

        self.twist = TwistWithCovariance(covariance=[99999] * 36)

    def onMarker(self, msg):
        """
        :type msg: AlvarMarkers
        """
        for marker in msg.markers:
            if marker.id == MARKER_ID:
                self.publish_odom(marker)

    def publish_odom(self, marker):
        pose = marker.pose.pose
        pose_covariance = PoseWithCovariance(pose=pose, covariance=self.covariance)
        odometry = Odometry(header=marker.header, child_frame_id=CHILD_FRAME, pose=pose_covariance, twist=self.twist)
        self.pub.publish(odometry)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        handler = ARrepublisher()
        handler.run()
    except rospy.ROSInterruptException:
        pass
