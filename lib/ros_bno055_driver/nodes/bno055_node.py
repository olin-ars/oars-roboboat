#!/usr/bin/env python
from __future__ import print_function, division

import sys
import os
import rospy
import math
import time

from std_msgs.msg import Header
from sensor_msgs.msg import Imu, Temperature
from Adafruit_BNO055.BNO055 import BNO055


class BNO055Driver(object):

    def __init__(self):
        self.init_device()
        calibration_file = rospy.get_param('~calibration_file', 'bno055.json')
        if calibration_file:
            self.load_calibration(calibration_file)
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.temp_pub = rospy.Publisher('temperature', Temperature, queue_size=1)
        self.frame_id = rospy.get_param('~frame_id', '/base_imu')
        self.reset_msgs()

    def init_device(self):
        serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')

        try:
            self.device = BNO055(serial_port=serial_port)
        except:
            rospy.logerr('unable to find IMU at port {}'.format(serial_port))
            sys.exit(-1)

        if not self.device.begin():
            rospy.logerr('unable to initialize IMU at port {}'.format(serial_port))
            sys.exit(-1)

        self.device.set_external_crystal(True)

        status = self.device.get_system_status()
        rospy.loginfo('system status is {} {} {} '.format(*status))

        calibration_file = rospy.get_param('~calibration_file', 'bno055.json')
        if os.path.isfile(calibration_file):
            self.device.load_calibration(calibration_file)
        else:
            rospy.loginfo('unable load calibration')

        time.sleep(1)
        calibration_status = self.device.get_calibration_status()
        rospy.loginfo('calibration status is {} {} {} {} '.format(*calibration_status))

    def load_calibration(self, calibration_file):
        if os.path.isfile(calibration_file):
            rospy.loginfo('loading calibration file from: {}'.format(calibration_file))
            self.device.load_calibration(calibration_file)
        else:
            rospy.logwarn('unable to load calibration file from: {}'.format(calibration_file))
        calibration_status = self.device.get_calibration_status()
        rospy.loginfo('calibration status is system={} gyro={} accel={} mag={} '.format(*calibration_status))

    def reset_msgs(self):
        self.imu_msg = Imu()

        self.imu_msg.header.frame_id = self.frame_id
        self.imu_msg.header.seq = 0

        # ignore the covariance data
        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

        self.temp_msg = Temperature()
        self.temp_msg.header.frame_id = self.frame_id
        self.temp_msg.header.seq = 0
        self.temp_msg.variance = 0

    def publish_data(self):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.seq = self.imu_msg.header.seq + 1

        q = self.device.read_quaternion()
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]

        g = self.device.read_gyroscope()
        # convert from deg/sec to rad/sec
        self.imu_msg.angular_velocity.x = g[0] * math.pi / 180.0
        self.imu_msg.angular_velocity.y = g[1] * math.pi / 180.0
        self.imu_msg.angular_velocity.z = g[2] * math.pi / 180.0

        a = self.device.read_linear_acceleration()
        self.imu_msg.linear_acceleration.x = a[0]
        self.imu_msg.linear_acceleration.y = a[1]
        self.imu_msg.linear_acceleration.z = a[2]

        self.imu_pub.publish(self.imu_msg)

        # ..todo:: temperature sensor supports only 1 Hz
        self.temp_msg.header.stamp = rospy.Time.now()
        self.temp_msg.header.seq = self.temp_msg.header.seq + 1
        self.temp_msg.temperature = self.device.read_temp()
        self.temp_pub.publish(self.temp_msg)



def main():
    rospy.init_node('bno055_driver')
    node = BNO055Driver()
    while not rospy.is_shutdown():
        node.publish_data()

if __name__ == '__main__':
    main()
