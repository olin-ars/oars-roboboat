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
import thread
import time

import rospy

from plans import *

import sys
import smach_ros

rospy.init_node('planner')
configured_plan = rospy.get_param('~plan_name', 'firstplan')


def main():
    plansm = plans.get(configured_plan, None)

    if not plansm:
        print('No plan found with that name!')
        return

    print('Running state machine "{}", ctrl c at any time to abort.'.format(configured_plan))

    plansm.execute();

if __name__ == '__main__':
    main()
