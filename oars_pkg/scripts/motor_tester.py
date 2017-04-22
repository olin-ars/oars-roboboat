#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Float32


class MotorTester(object):
    def __init__(self):
        rospy.init_node('motor_tester')
        self.publishers = [rospy.Publisher('/motor{}'.format(i + 1), Float32, queue_size=1) for i in range(4)]

        rospy.on_shutdown(self.stop)

    def run(self):
        assert len(sys.argv) == 5
        pows = map(float, sys.argv[1:5])

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            for pub, pow in zip(self.publishers, pows):
                pub.publish(pow)

            r.sleep()

    def stop(self):
        # Stop the boat
        for pub in self.publishers:
            pub.publish()

if __name__ == '__main__':
    MotorTester().run()
