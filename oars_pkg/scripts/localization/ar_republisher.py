#!/usr/bin/env python
"""
Republishes the position estimate from the AR tag tracker
as an Odometry message, including generation of covariance data
"""

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose
from nav_msgs.msg import Odometry
import tf2_ros

MARKER_ID = 0
LINEAR_COVARIANCE = 0.03
ANGULAR_COVARIANCE = 0.3
CHILD_FRAME = 'ar_tag'
BASE_LINK = 'boat_link'


class ARrepublisher:  # needs a topic to read in waypoint angle from
    def __init__(self):
        rospy.init_node('ar_republisher')

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

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

    def fix_pose_to_base_link(self, pose):
        """

        :type pose: Pose
        """
        transform = self.tfBuffer.lookup_transform(CHILD_FRAME, BASE_LINK, rospy.Time(0)).transform

        pose.position.x += transform.translation.x
        pose.position.y += transform.translation.y
        pose.position.z += transform.translation.z

        posequat = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        adjquat = transform.rotation.x, transform.rotation.y, transform.rotation.z, \
                      transform.rotation.w

        newquat = q_mult(adjquat, posequat)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = newquat

        return pose

    def publish_odom(self, marker):
        pose = marker.pose.pose
        pose = self.fix_pose_to_base_link(pose)
        pose_covariance = PoseWithCovariance(pose=pose, covariance=self.covariance)
        odometry = Odometry(header=marker.header, child_frame_id=CHILD_FRAME, pose=pose_covariance, twist=self.twist)
        self.pub.publish(odometry)

    def run(self):
        rospy.spin()


def q_mult(q1, q2):
    """
    Taken from http://stackoverflow.com/a/4870905
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


if __name__ == '__main__':
    try:
        handler = ARrepublisher()
        handler.run()
    except rospy.ROSInterruptException:
        pass
