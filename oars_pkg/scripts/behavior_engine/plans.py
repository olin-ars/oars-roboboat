#!/usr/bin/python
from tasks import *

import getch


class Plan():
    """
    A Plan represents a sequence of Tasks that are intended to be exectued in order by the robot.
    """
    def __init__(self, name, tasklist):
        """
        :type tasklist: list of Task
        """
        self.name = name
        self.tasklist = tasklist
        self.numTasksCompleted = 0
        self.currenttask = None
        self.active = False

    def execute(self):
        if not self.active:
            self.numTasksCompleted = 0
            self.active = True
            self.startnexttask()

    def stop(self):
        if self.active:
            self.active = False
            currenttask = self.tasklist[self.numTasksCompleted]

            currenttask.stop()

    def skipcurrenttask(self):
        """
        Aborts the execution of the current task
        """
        if self.active:
            # Note that calling stop() on a task should always trigger the taskcompletioncallback,
            # which increments the active task before continuing
            # TODO: Consider creating an additional method to allow cleaning up a task without triggering the callback
            self.currenttask.stop()

    def startnexttask(self):
        if self.numTasksCompleted >= len(self.tasklist):
            self.active = False
            return
        self.currenttask = self.tasklist[self.numTasksCompleted]
        self.currenttask.start(self.taskcompletioncallback)

    def taskcompletioncallback(self):
        if self.active:
            self.numTasksCompleted += 1
            self.startnexttask()


def test():
    rospy.init_node('test_planner')

    plan = Plan('testplan',[
        SampleTask(),
        DelayTask(2),
        GPSNavigationTask('Navigate home', destination='home'),
        RCTask()
    ])

    plan.execute()

    rospy.spin()


alltheplans = [
    Plan('thefirstplan', [
        SampleTask(),
        DelayTask(2),
        GPSNavigationTask('Navigate home', destination='home'),
        RCTask()
    ]),
    Plan('fastplan', [
        DelayTask(2),
    ]),


]


def main():
    rospy.init_node('planner')

    while True:
        print('Which plan do you want to execute?')
        for i, plan in enumerate(alltheplans):
            print '\t{}.  {}'.format(i+1, plan.name)
        currPlan = int(raw_input())-1

        alltheplans[currPlan].execute()

        r = rospy.Rate(100)

        plan = alltheplans[currPlan]

        while True:
            key = getch.getch()

            if key == 'x':
                plan.stop()
                return

            if key == '\x03':
                plan.stop()
                return

            if plan.active == False:
                break

            if rospy.is_shutdown():
                plan.stop()
                return

            r.sleep()


if __name__ == '__main__':
    main()
