#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Vector3, Quaternion, TwistWithCovariance

rospy.init_node('probably_still')

pub = rospy.Publisher('/probably_still', Odometry, queue_size=10)

r = rospy.Rate(10)

xy_var = 1.0
z_var = 0.1
theta_xy_var = 0.5
theta_z_var = 100

vel_xy_var = 10.0
vel_z_var = 0.5
vel_theta_xy_var = 0.5
vel_theta_z_var = 100

msg = Odometry(
    header=Header(frame_id='map'),
    pose=PoseWithCovariance(pose=Pose(position=Vector3(x=10), orientation=Quaternion(w=1)),
                            covariance=[xy_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, xy_var, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, z_var, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, theta_xy_var, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, theta_xy_var, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, theta_z_var]),

    twist=TwistWithCovariance(covariance=[vel_xy_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, vel_xy_var, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, vel_z_var, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, vel_theta_xy_var, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, vel_theta_xy_var, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, vel_theta_z_var])
)

while not rospy.is_shutdown():
    r.sleep()
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
