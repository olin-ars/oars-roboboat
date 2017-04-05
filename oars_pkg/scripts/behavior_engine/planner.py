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

rospy.init_node('planner')
configured_plan = rospy.get_param('~plan_name', '')


def killonkey(cleanup):
    while True:
        key = getch()

        if key == 'x' or key == '\x03':
            cleanup()


def main():
    if configured_plan == '':
        print('Which plan do you want to execute?')
        for i, plan in enumerate(plans):
            print '\t{}.  {}'.format(i+1, plan[0])

        curr_plan = int(raw_input())-1

        plan = plans[curr_plan]

    else:
        plan = None
        for p in plans:
            if p[0] == configured_plan:
                plan = p
        if not plan:
            print('No plan found with that name!')
            return

    print('Running state machine "{}", press the "x" key at any time to abort.'.format(plan[0]))
    sm = plan[1]
    outcome = sm.execute()

    # def cleanup():
    #     getch.cleanup()
    #     if plan.active:
    #         plan.stop()
    #         time.sleep(0.5)
    #     exit(0)

    # rospy.on_shutdown(cleanup)

    # thread.start_new(killonkey, (cleanup,))

    # r = rospy.Rate(1000)
    # while True:
    #     if not plan.active:
    #         break

    #     if rospy.is_shutdown():
    #         break

    #     r.sleep()

    # cleanup()


if __name__ == '__main__':
    main()
