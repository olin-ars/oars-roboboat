#!/usr/bin/env python
import rospy

"""
This node translates commanded velocities into motor commands.

Omnidirectional boat has four motors:

1    2
|    |
------
|    |
3    4

where positive power indicates forward thrust
"""

# This line may need to change depending on your message types
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

rospy.init_node('motor_commander_omni')


class autonomousRudderPublisher:  # needs a topic to read in waypoint angle from
    def __init__(self):

        self.publishers = [rospy.Publisher('motor{}'.format(i + 1), Float32, queue_size=1) for i in range(4)]

        self.cmdVel = Twist()
        self.velSub = rospy.Subscriber('cmd_vel', Twist, self.onCmdVel)

    def onCmdVel(self, msg):
        self.cmdVel = msg

    @staticmethod
    def transformMotorPower(force):
        """
        Scales the commanded parameter into motor units (-1...1)
        :param force:
        :return:
        """

        return force

    def publish(self, forces):
        """
        Publishes the powers onto the various publishers
        :param forces: list<float>
        :return:
        """
        for power, publisher in zip(forces, self.publishers):
            publisher.publish(Float32(self.transformMotorPower(power)))

    def run(self):
        rate = rospy.Rate(100)  # 100hz refresh rate
        while not rospy.is_shutdown():
            forward_force = self.cmdVel.linear.x / 100
            side_force = self.cmdVel.linear.y / 100  # to the left
            turn_force = self.cmdVel.angular.z  # Counterclockwise

            powers = [0] * 4

            powers[0] = forward_force - side_force - turn_force
            powers[1] = forward_force + side_force + turn_force
            powers[2] = forward_force + side_force - turn_force
            powers[3] = forward_force - side_force + turn_force

            self.publish(powers)

            rate.sleep()


if __name__ == '__main__':
    try:
        handler = autonomousRudderPublisher()
        handler.run()
    except rospy.ROSInterruptException:
        pass
