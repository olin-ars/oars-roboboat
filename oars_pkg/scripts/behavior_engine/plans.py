#!/usr/bin/python
from tasks import *

class Plan():
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
        if self.active:
            self.currenttask.stop()

            self.startnexttask()

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
