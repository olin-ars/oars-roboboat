#!/usr/bin/env python
"""
Converts commanded velocities into motor values.
Configurable for tugboat or omni
"""
import rospy

from motor_commander_tugboat import autonomousRudderPublisher as tugboatCommander
from motor_commander_omni import omniMotorController as omniCommander

if __name__ == '__main__':
    rospy.init_node('motor_commander')

boat_type = rospy.get_param('~boat_type', 'tugboat')


if __name__ == '__main__':
    commander = None
    if boat_type == 'tugboat':
        commander = tugboatCommander()
    elif boat_type == 'omni':
        commander = omniCommander()
    else:
        rospy.logerr('Unsupported boat type detected')
        exit()

    commander.run()
