#!/usr/bin/python
"""
This is the main ROS node for the planner. It is intended to be run
on the command line or a roslaunch file.
It takes in one optional ROS parameter, plan_name, which specifies
the plan to run. If blank, it defaults to firstplan.

To run:
start roscore
$ roscore
start sample_task
$ python sample_task.py
start the plan (plans can be found in plans.py)
$ python planner.py _plan_name:=your_plan

To add a plan:
Add to plans.py

To add a task:
Add to tasks.py
"""

import sys

from plans import *

rospy.init_node('planner')

if len(sys.argv) != 2:
    rospy.logerr('Planner must be given an argument to specify the plan to run')
    exit()

configured_plan = sys.argv[1]


def main():
    plansm = plans.get(configured_plan, None)

    if not plansm:
        print('No plan found with that name!')
        return

    print('Running state machine "{}", ctrl c at any time to abort.'.format(configured_plan))

    plansm.execute()

if __name__ == '__main__':
    main()
