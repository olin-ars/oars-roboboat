#!/usr/bin/python
"""
This is the main ROS node for the planner. It is intended to be run
on the command line or a roslaunch file.
It takes in one optional ROS parameter, plan_name, which specifies
the plan to run. If blank, it asks the user for command line input.
"""
import thread
import time

import rospy

from plans import *
from getch import getch

import sys
import smach_ros

rospy.init_node('planner')
configured_plan = rospy.get_param('~plan_name', 'firstplan')


def killonkey(cleanup):
    while True:
        key = getch()

        if key == 'x' or key == '\x03':
            cleanup()

def main():
    plansm = plans.get(configured_plan, None)

    if not plansm:
        print('No plan found with that name!')
        return

    print('Running state machine "{}", press the "x" key at any time to abort.'.format(configured_plan))

    sis = smach_ros.IntrospectionServer(configured_plan + 'server', plansm, '/SM_ROOT')
    sis.start()

    smach_ros.set_preempt_handler(plansm)

    plansm.execute()

    rospy.spin()

    plansm.request_preempt()

    sis.stop()


if __name__ == '__main__':
    main()
