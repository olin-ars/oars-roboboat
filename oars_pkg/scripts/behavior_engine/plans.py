#!/usr/bin/python
from tasks import *


class Plan():
    """
    A Plan represents a sequence of Tasks that are intended to be exectued in order by the robot.
    """
    def __init__(self, tasklist):
        """
        :type tasklist: list of Task
        """
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
    rospy.init_node('task_test_node')

    plan = Plan([
        SampleTask(),
        DelayTask(2),
        SampleTask(),
        RCTask()
    ])

    plan.execute()

    rospy.spin()


if __name__ == '__main__':
    test()
