#!/usr/bin/python
import thread
import time

import rospy

from plans import plans
from getch import getch


def killonkey(cleanup):
    while True:
        key = getch()

        if key == 'x' or key == '\x03':
            cleanup()


def main():
    rospy.init_node('planner')

    print('Which plan do you want to execute?')
    for i, plan in enumerate(plans):
        print '\t{}.  {}'.format(i+1, plan.name)
    curr_plan = int(raw_input())-1

    plan = plans[curr_plan]

    plan.execute()

    def cleanup():
        getch.cleanup()
        if plan.active:
            plan.stop()
            time.sleep(0.5)
        exit()

    rospy.on_shutdown(cleanup)

    thread.start_new(killonkey, (cleanup,))

    r = rospy.Rate(1000)
    while True:
        if not plan.active:
            break

        if rospy.is_shutdown():
            break

        r.sleep()

    cleanup()


if __name__ == '__main__':
    main()
