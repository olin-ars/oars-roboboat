#!/usr/bin/env python
"""
Republishes the position estimate from the AR tag tracker
as an Odometry message, including generation of covariance data
"""

import rospy
import tf2_geometry_msgs
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

MARKER_ID = 0
LINEAR_COVARIANCE = 0.03
ANGULAR_COVARIANCE = 0.3
CHILD_FRAME = 'ar_tag'  # The frame associated with the physical tag on the robot
WORLD_FRAME = 'world'  # The frame associated with the world


class ARrepublisher:  # needs a topic to read in waypoint angle from
    def __init__(self):
        rospy.init_node('ar_republisher')

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # Step 1: Initialize publishers. This will be referred to later whenever data is transmitted.
        self.imu_pub = rospy.Publisher('ar_imu', Imu, queue_size=0)
        self.odom_pub = rospy.Publisher('ar_odometry', Odometry, queue_size=0)

        # Step 2: Initialize subscribers. Each subscriber (incomming information) gets a "self.*" variable,
        # a callback, and a subscriber
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.onMarker)

        self.pose_covariance = [0] * 36
        self.pose_covariance[0] = self.pose_covariance[7] = self.pose_covariance[14] = LINEAR_COVARIANCE
        self.pose_covariance[21] = self.pose_covariance[28] = self.pose_covariance[35] = 1e6

        self.orientation_covariance = [0] * 9
        self.orientation_covariance[0] = \
            self.orientation_covariance[4] = \
            self.orientation_covariance[8] = ANGULAR_COVARIANCE

        self.fake_twist = TwistWithCovariance(covariance=[99999] * 36)

    def onMarker(self, msg):
        """
        :type msg: AlvarMarkers
        """
        for marker in msg.markers:
            if marker.id == MARKER_ID:
                self.publish_messages(marker)

    def fix_pose_to_base_link(self, pose, header):
        """

        :type pose: Pose
        """
        transform = self.tfBuffer.lookup_transform(
            WORLD_FRAME, header.frame_id, header.stamp, rospy.Duration(1))

        pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=header, pose=pose), transform).pose
        return pose

    def publish_messages(self, marker):
        """
        :type marker: AlvarMarker
        """
        pose = marker.pose.pose
        pose = self.fix_pose_to_base_link(pose, marker.header)

        # Build and publish the odom message with only position information
        pose_covariance = PoseWithCovariance(pose=pose, covariance=self.pose_covariance)
        odometry = Odometry(header=Header(frame_id=WORLD_FRAME, stamp=marker.header.stamp),
                            child_frame_id=CHILD_FRAME, pose=pose_covariance, twist=self.fake_twist)
        self.odom_pub.publish(odometry)

        # Publish the IMU message with only orientation information
        imu = Imu(header=Header(frame_id=CHILD_FRAME, stamp=marker.header.stamp),
                  orientation=pose.orientation, orientation_covariance=self.orientation_covariance,
                  angular_velocity_covariance=[1e5] * 9,
                  linear_acceleration_covariance=[1e5] * 9)
        self.imu_pub.publish(imu)

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
